
#include <functional>

#include <rclcpp/rclcpp.hpp>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "builtin_interfaces/msg/time.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"
#include "tf2/exceptions.h"

#include "nav_msgs/msg/map_meta_data.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "kurome.h"
#include "occupancy_grid.hpp"
#include "slam_system.hpp"
#include "qeftser_graph_slam.hpp"
#include "builtin_graph_slam.hpp"

/* The SLAM system for Kurome. This is an implimentation of the GraphSLAM
 * algorithm using odometry, LiDAR, Point Cloud, and IMU data. Fixing nodes
 * is also allowed via input from a local beacon                           */

using std::placeholders::_1;

class Pino : public rclcpp::Node {
public:

   Pino() : Node("pino") {

      /* topic to publish the constructed map on */
      this->declare_parameter("map_out","map");
      /* interval at which to publish the full map, in seconds */
      this->declare_parameter("map_publish_interval",1.0);
      /* whether to publish visualization data of the system */
      this->declare_parameter("publish_visualization",true);
      /* topic to publish the visualization on if it is being used */
      this->declare_parameter("visualization_topic","pino/visual");

      /* topic to listen for beacon odometry on */
      this->declare_parameter("beacon_in","beacon");
      /* topic to listen for odometry on */
      this->declare_parameter("odom_in","demo/odom");
      /* topic to listen for incoming laser scans on */
      this->declare_parameter("scan_in","scan");
      /* topic to listen for incoming point cloud data on */
      this->declare_parameter("cloud_in","points");

      /* whether to use the odometry estimate of velocity
       * or another, seperate subscription to a twist topic */
      this->declare_parameter("use_odom_vel",true);
      /* the topic to listen for velocity values on if above
       * parameter is set to false.                         */
      this->declare_parameter("vel_in","cmd_vel");

      /* time difference that excludes an odometry or beacon
       * observation from being used as a link in the 
       * construction of an edge on the pose graph. Time
       * is in seconds.                                     */
      this->declare_parameter("time_error",0.01);
      /* time difference at which we assume that we have lost
       * the beacon and decide to rely on odometry for our
       * current position updates. In seconds               */
      this->declare_parameter("beacon_lost_time",0.25);
      /* If this value is set to true, pass a hint to fix
       * the observation to the slam system if the beacon
       * was used to estimate position. Otherwise the 
       * beacon covariance will be used.                */
      this->declare_parameter("fix_beacon_nodes",true);

      /* whether or not to use the internal motion model to 
       * estimate position up to the exact timestep. Setting
       * this value to false will result in the closest timewise
       * value being used for the position estimates.       */
      this->declare_parameter("estimate_movement_updates",false);

      /* whether to aggregate the sensor data as it comes in
       * and batch observations. If this value is false, each
       * sensor observation will be treated as an individual 
       * measurement.                                       */
      this->declare_parameter("aggregate_sensor_data",true);
      /* How long to delay between sensor data collections 
       * if aggregate_sensor_data is set. Time is in seconds */
      this->declare_parameter("aggregation_interval",0.1);

      /* the slam algorithm to use */
      this->declare_parameter("algorithm","builtin");

      /* parameters for the slam algorithm */
      this->declare_parameter("bin_size",1.0); /* meters */
      this->declare_parameter("linear_update_dist",0.3); /* meters */
      this->declare_parameter("angular_update_dist",0.3); /* radians */
      this->declare_parameter("lidar_acceptance_threshold",0.7); /* probability */
      this->declare_parameter("point_cloud_acceptance_threshold",0.5); /* probability */
      this->declare_parameter("node_association_dist",0.5); /* meters */

      grid_out = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
            this->get_parameter("map_out").as_string(), 10);
      grid_callback = this->create_wall_timer(
            std::chrono::milliseconds((long)(1000.0 *
                  this->get_parameter("map_publish_interval").as_double())),
            std::bind(&Pino::publish_map, this));

      if (this->get_parameter("publish_visualization").as_bool()) {

         visual_out = this->create_publisher<visualization_msgs::msg::MarkerArray>(
               this->get_parameter("visualization_topic").as_string(), 10);
         visual_callback = this->create_wall_timer(
               std::chrono::milliseconds(((long)(1000.0 *
                     this->get_parameter("map_publish_interval").as_double()))),
               std::bind(&Pino::publish_visual, this));
      }

      beacon_in = this->create_subscription<nav_msgs::msg::Odometry>(
            this->get_parameter("beacon_in").as_string(), 10,
            std::bind(&Pino::collect_beacon, this, _1));

      odom_in = this->create_subscription<nav_msgs::msg::Odometry>(
            this->get_parameter("odom_in").as_string(), 10,
            std::bind(&Pino::collect_odom, this, _1));

      scan_in = this->create_subscription<sensor_msgs::msg::LaserScan>(
            this->get_parameter("scan_in").as_string(), 10,
            std::bind(&Pino::collect_scan, this, _1));

      cloud_in = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            this->get_parameter("cloud_in").as_string(), 10,
            std::bind(&Pino::collect_cloud, this, _1));

      if (this->get_parameter("use_odom_vel").as_bool() == false) {
         vel_in = this->create_subscription<geometry_msgs::msg::Twist>(
               this->get_parameter("vel_in").as_string(), 10,
               std::bind(&Pino::collect_vel, this, _1));
      }

      tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());

      tf_listener = std::make_unique<tf2_ros::TransformListener>(*tf_buffer);

      map_frame = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
      map_frame_callback = this->create_wall_timer(
            std::chrono::milliseconds(((long)(1000.0 *
                     this->get_parameter("map_publish_interval").as_double()))),
            std::bind(&Pino::broadcast_map_frame, this));

      if (this->get_parameter("algorithm").as_string() == "qeftser") {
         slam_system = new QeftserGraphSlam(this->get_clock(),
               new CorrelativeLidarMatcher(),
               new DummyPointCloudMatcher(),
               this->get_parameter("bin_size").as_double(),
               this->get_parameter("linear_update_dist").as_double(), 
               this->get_parameter("angular_update_dist").as_double(),
               this->get_parameter("lidar_acceptance_threshold").as_double(), 
               this->get_parameter("point_cloud_acceptance_threshold").as_double(),
               this->get_parameter("node_association_dist").as_double());
      }
      else if (this->get_parameter("algorithm").as_string() == "builtin") {
         slam_system = new BuiltinGraphSlam(this->get_clock(),
               new CorrelativeLidarMatcher(),
               new DummyPointCloudMatcher(),
               this->get_parameter("bin_size").as_double(),
               this->get_parameter("linear_update_dist").as_double(), 
               this->get_parameter("angular_update_dist").as_double(),
               this->get_parameter("lidar_acceptance_threshold").as_double(), 
               this->get_parameter("point_cloud_acceptance_threshold").as_double(),
               this->get_parameter("node_association_dist").as_double());
      }

      if (this->get_parameter("aggregate_sensor_data").as_bool()) {
         observation_callback = this->create_wall_timer(
               std::chrono::milliseconds(((long)(1000.0 * 
                        this->get_parameter("aggregation_interval").as_double()))),
               std::bind(&Pino::flush_observation, this));
      }

   }

private:

   /* =================== publishers =================== */
   /* The current map representation. This will be a rather large
    * message, so will not be sent live with every update, but 
    * rather periodically over a user-controlled interval.          */
   rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_out;
   rclcpp::TimerBase::SharedPtr grid_callback;

   /* Used in publishing the visualization - i.e. all nodes and constraints
    * in the system, as well as relative correlation to the current position. */
   rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr visual_out;
   rclcpp::TimerBase::SharedPtr visual_callback;

   /* Used to periodically send aggregated observation data to the
    * slam system in use if the aggregate_sensor_data flag is set */
   rclcpp::TimerBase::SharedPtr observation_callback;
   /* The most recent aggregated observation. Sent to the slam system
    * by observation_callback if valid and updated by the lidar and
    * point cloud callbacks.                                         */
   std::pair<Observation *,bool> current_observation = std::make_pair(nullptr, false);

   /* =================== subscribers =================== */
   /* Input from our beacon (local positioning system) if we have one.
    * The result is fixed nodes in our pose graph and direct translation
    * of position information.                                      */
   rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr beacon_in;
   /* Input from the various LiDAR scanners on the system. They
    * will need to have different frame_id values to distinguish
    * where they are coming from. These will be used for scan 
    * matching and map generation.                                  */
   rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_in;
   /* Input from any sort of 3d LiDAR or depth camera. Multiple of
    * these will also need different frame_ids. These will be used in
    * an ICP algorithm for scan matching, and also will be used for
    * map generation.                                               */
   rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_in;
   /* Input from an associated odometry system. Used for position
    * estimation when the input from the beacon is not avaliable.   */
   rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_in;
   /* Input from another node that provides this one with the current
    * velocity estimate. This can either be some sort of sensor or
    * just the desired velocity that is being published to cmd_vel.
    * Used for state estimation in place of odometry velocity if
    * the appropriate parameter is set.                           */
   rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr vel_in;

   /* =================== transforms =================== */
   /* Publish the map -> odom transformation as described in REP 105.
    * This is done to allow the nessesary transform math to be 
    * carried out in our other systems.                             */
   std::unique_ptr<tf2_ros::TransformBroadcaster> map_frame;
   rclcpp::TimerBase::SharedPtr map_frame_callback;
   /* Used to transform incoming scan data into the base_link
    * reference frame for the robot.                                */
   std::unique_ptr<tf2_ros::TransformListener> tf_listener;
   /* Performs all nessesary conversions between different 
    * reference frames.                                             */
   std::unique_ptr<tf2_ros::Buffer> tf_buffer;

   /* =================== internal =================== */
   /* The slam system used as the core of this node. */
   SlamSystem * slam_system;
   /* Last timestep we have collected a beacon message at. If
    * this value gets too far away, normal odometry must be
    * considered instead.                                   */
   rclcpp::Time last_beacon_time;
   /* Last timestamp odometry was recieved on. Used for 
    * determining whether or not to use odometry in the
    * construction of a observation in the slam system.  */
   rclcpp::Time last_odom_time;
   /* The current best estimate for the pose of the robot */
   pose_2d beacon_pose;
   Covariance3 beacon_pose_covariance;
   /* The last received odometry input for the pose of
    * the robot. Used for the transform calculation.  */
   pose_2d odom_pose;
   Covariance3 odom_pose_covariance;
   /* the current estimate of the robot velocity */
   velocity_2d curr_vel;
   /* the last computed map -> odom transform stored
    * for local use if needed.                     */
   pose_2d last_map_odom_transform = { {0, 0}, 0 };

   bool get_best_pose_estimate(pose_2d & best_pose, Covariance3 & covariance) {
      double time_diff = 0.0;
      bool is_beacon = false;

      /* first check if we can use our beacon time, as this is
       * the best possible estimate avaliable to us.          */
      if (this->get_parameter("beacon_lost_time").as_double() >
          time_dist(last_beacon_time,this->get_clock()->now())) {
         best_pose = beacon_pose;
         covariance = beacon_pose_covariance;
         is_beacon = true;
         time_diff = time_dist(last_beacon_time,this->get_clock()->now());
      }

      /* if we do not have the beacon, use the odometry transformed
       * into the map frame, with the assumption that this represents
       * the next best estimate avaliable - i.e. the transform is good. */
      else {

         /*
         printf("odom_pose: %f %f %f\n",odom_pose.pos.x,odom_pose.pos.y,odom_pose.theta);
         printf("last_tran: %f %f %f\n",last_map_odom_transform.pos.x,last_map_odom_transform.pos.y,last_map_odom_transform.theta);
         */

         best_pose.pos = transform(odom_pose.pos, last_map_odom_transform);
         best_pose.theta = odom_pose.theta + last_map_odom_transform.theta;
         best_pose.theta = atan2(sin(best_pose.theta),cos(best_pose.theta));

         /*
         printf("best_pose: %f %f %f\n",best_pose.pos.x,best_pose.pos.y,best_pose.theta);
         */

         covariance = odom_pose_covariance;
         is_beacon = false;
         time_diff = time_dist(last_odom_time,this->get_clock()->now());
      }

      if (this->get_parameter("estimate_movement_updates").as_bool())
         best_pose = estimate_movement(best_pose,curr_vel,time_diff);

      best_pose.theta = atan2(sin(best_pose.theta),cos(best_pose.theta));

      return is_beacon;
   }

   void publish_map() {

      nav_msgs::msg::OccupancyGrid msg;

      slam_system->get_map(msg);

      msg.info.map_load_time = this->get_clock()->now();
      msg.header.stamp = this->get_clock()->now();
      msg.header.frame_id = "map";

      grid_out->publish(msg);
   }

   void publish_visual() {

      visualization_msgs::msg::MarkerArray msg;
      int marker_id = 0;

      slam_system->construct_visualization(msg);

      for (visualization_msgs::msg::Marker & m : msg.markers) {
         m.header.frame_id = "map";
         m.header.stamp = this->get_clock()->now();
         m.id = marker_id++;
      }

      visual_out->publish(msg);

   }

   void broadcast_map_frame() {


      /* this is the map -> odom transform */
      geometry_msgs::msg::TransformStamped msg;
      msg.header.frame_id = "map";
      msg.child_frame_id = "odom";

      auto poses = slam_system->get_last_pose();

      pose_2d diff = { {std::get<0>(poses).pos.x - std::get<1>(poses).pos.x,
                        std::get<0>(poses).pos.y - std::get<1>(poses).pos.y},
                       std::get<0>(poses).theta - std::get<1>(poses).theta  };
      diff.theta = atan2(sin(diff.theta),cos(diff.theta));

      diff = {{0,0},0};

      last_map_odom_transform = diff;
      //printf("transform: %f %f %f\n",diff.pos.x,diff.pos.y,diff.theta);

      msg.transform.translation.x = diff.pos.x;
      msg.transform.translation.y = diff.pos.y;
      msg.transform.translation.z = 0.0;
      tf2::Quaternion q; q.setRPY(0.0,0.0,diff.theta);
      msg.transform.rotation.x = q.getX();
      msg.transform.rotation.y = q.getY();
      msg.transform.rotation.z = q.getZ();
      msg.transform.rotation.w = q.getW();

      msg.header.stamp = this->get_clock()->now();
      map_frame->sendTransform(msg);
   }

   void collect_beacon(const nav_msgs::msg::Odometry & msg) {


      /* hard set the pose of the robot. Assume that
       * the beacon is a perfect input source.      */
      beacon_pose = ros2_pose_to_pose_2d(msg.pose.pose);
      last_beacon_time = msg.header.stamp;

   }

   void collect_odom(const nav_msgs::msg::Odometry & msg) {



      odom_pose = ros2_pose_to_pose_2d(msg.pose.pose);
      odom_pose_covariance = Covariance3{msg.pose.covariance[0],   /* xx */
                                         msg.pose.covariance[1],   /* xy */
                                         msg.pose.covariance[5],   /* x(theta) */
                                         msg.pose.covariance[7],   /* yy */
                                         msg.pose.covariance[11],  /* y(theta) */
                                         msg.pose.covariance[35]}; /* (theta)(theta) */
      last_odom_time = msg.header.stamp;

      if (this->get_parameter("use_odom_vel").as_bool()) {
         curr_vel = {msg.twist.twist.linear.x, msg.twist.twist.angular.z};
      }

   }

   void collect_scan(const sensor_msgs::msg::LaserScan & msg) {
      static int fail_count = 0;
      Observation * observation = NULL;


      /* convert the lidar data into the frame of
       * reference for base_link.               */
      try {
         geometry_msgs::msg::PoseStamped pose_in;
         geometry_msgs::msg::PoseStamped pose_out;
         /*
         tf_buffer->transform<geometry_msgs::msg::PoseStamped>(pose_in,pose_out,"base_link",
               tf2::Duration(std::chrono::milliseconds(200)));
               */

         observation = new Observation();

         observation->current_odometry = odom_pose;
         bool is_beacon = get_best_pose_estimate(observation->global_pose_estimate,
                                                 observation->global_pose_covariance);

         if (!this->get_parameter("fix_beacon_nodes").as_bool())
            is_beacon = false;

         observation->laser_scan = LidarData(msg,pose_out.pose);

         if (this->get_parameter("aggregate_sensor_data").as_bool()) {

            if (std::get<0>(current_observation) == NULL)
               current_observation = std::make_pair(observation,is_beacon);
            else {
               std::get<0>(current_observation)->aggregate(*observation);
               delete observation;
            }

         }
         else {
            slam_system->insert_observation(observation,is_beacon);
         }

      }
      catch(const tf2::TransformException & ex) {
         RCLCPP_WARN(this->get_logger(),"transformation of scan failed for the %dth time",fail_count++);
         return;
      }

   }

   void collect_cloud(const sensor_msgs::msg::PointCloud2 & msg) {
      static int fail_count = 0;
      Observation * observation = NULL;


      /* convert the point cloud into the
       * frame of reference for base_link */
      try {
         sensor_msgs::msg::PointCloud2 cloud_out;
         tf_buffer->transform<sensor_msgs::msg::PointCloud2>(msg,cloud_out,"base_link",
               tf2::Duration(std::chrono::milliseconds(200)));

         observation = new Observation();

         observation->current_odometry = odom_pose;
         bool is_beacon = get_best_pose_estimate(observation->global_pose_estimate,
                                                 observation->global_pose_covariance);

         if (!this->get_parameter("fix_beacon_nodes").as_bool())
            is_beacon = false;

         observation->point_cloud = PointCloudData(cloud_out);

         if (this->get_parameter("aggregate_sensor_data").as_bool()) {

            if (std::get<0>(current_observation) == NULL)
               current_observation = std::make_pair(observation,is_beacon);
            else {
               std::get<0>(current_observation)->aggregate(*observation);
               delete observation;
            }

         }
         else {
            slam_system->insert_observation(observation,is_beacon);
         }

      }
      catch(const tf2::TransformException & ex) {
         RCLCPP_WARN(this->get_logger(),"transformation of point cloud failed for the %dth time",fail_count++);
         return;
      }
   }

   void collect_vel(const geometry_msgs::msg::Twist & msg) {
      curr_vel = {msg.linear.x,msg.angular.z};
   }

   void flush_observation() {


      if (std::get<0>(current_observation) == NULL)
         return;

      slam_system->insert_observation(std::get<0>(current_observation),std::get<1>(current_observation));

      current_observation = std::make_pair(nullptr,false);

   }

};


int main(int argc, char ** argv) {
   rclcpp::init(argc,argv);
   rclcpp::spin(std::make_shared<Pino>());
   rclcpp::shutdown();
   return 0;
}
