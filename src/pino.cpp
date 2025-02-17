
#include <functional>

#include <rclcpp/rclcpp.hpp>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "nav_msgs/msg/map_meta_data.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "kurome.h"
#include "occupancy_grid.hpp"
#include "slam_system.hpp"

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
      this->declare_parameter("odom_in","odom");
      /* topic to listen for incoming laser scans on */
      this->declare_parameter("scan_in","scan");
      /* topic to listen for incoming point cloud data on */
      this->declare_parameter("cloud_in","points");

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

   /* =================== transforms =================== */
   /* Publish the map -> odom transformation as described in REP 105.
    * This is done to allow the nessesary transform math to be 
    * carried out in our other systems.                             */
   std::unique_ptr<tf2_ros::TransformBroadcaster> map_frame;
   /* Performs all nessesary conversions between different 
    * reference frames.                                             */
   std::unique_ptr<tf2_ros::Buffer> tf_buffer;

   /* =================== internal =================== */
   /* The best representation of the map of the environment
    * that we currently have. Will be continually updated and 
    * sometimes completely recomputed in the case of a loop 
    * closure event.                                        */
   OccupancyGrid global_map;
   /* The slam system used as the core of this node. */
   SlamSystem * slam_system;
   /* Last timestep we have collected a beacon message at. If
    * this value gets too far away, normal odometry must be
    * considered instead.                                   */
   rclcpp::Time last_beacon_time;

   void publish_map() {

      nav_msgs::msg::OccupancyGrid msg;

      global_map.to_msg(msg);

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

   void collect_beacon(const nav_msgs::msg::Odometry & msg) {
   }

   void collect_odom(const nav_msgs::msg::Odometry & msg) {
   }

   void collect_scan(const sensor_msgs::msg::LaserScan & msg) {
   }

   void collect_cloud(const sensor_msgs::msg::PointCloud2 & msg) {
   }

};


int main(int argc, char ** argv) {
   rclcpp::init(argc,argv);
   rclcpp::spin(std::make_shared<Pino>());
   rclcpp::shutdown();
   return 0;
}
