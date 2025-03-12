
#include <functional>

#include <rclcpp/rclcpp.hpp>

#include "kurome/msg/competition_area.hpp"
#include "kurome/msg/rectangle.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "nav_msgs/msg/map_meta_data.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "commander.hpp"
#include "static_box_digger.hpp"

/* The central control system of Kurome. Coordinate with all of the
 * other nodes to ensure things are functioning correctly and to 
 * set node parameters globally. All systems related to actually completing
 * the objective are executed in part by this node.                         */

using std::placeholders::_1;

class Brain : public rclcpp::Node {
public:

   Brain() : Node("brain") {

      /* set exposed parameters */

      /* the bottom left corner of the arena */
      this->declare_parameter("arena_origin_x",0.00);
      this->declare_parameter("arena_origin_y",0.00);

      /* the width and height of the arena */
      this->declare_parameter("arena_width",6.88);
      this->declare_parameter("arena_height",5.00);

      /* the bottom left corner of the dig area */
      this->declare_parameter("dig_origin_x",3.88);
      this->declare_parameter("dig_origin_y",2.00);

      /* the width and height of the dig area */
      this->declare_parameter("dig_width",3.00);
      this->declare_parameter("dig_height",3.00);

      /* the bottom left corner of the dump area */
      this->declare_parameter("dump_origin_x",4.38);
      this->declare_parameter("dump_origin_y",0.25);

      /* the width and height of the dump area */
      this->declare_parameter("dump_width",2.00);
      this->declare_parameter("dump_height",0.70);

      /* whether to publish the visualization and if 
       * so how often in seconds to publish it. */
      this->declare_parameter("publish_visualization",true);
      this->declare_parameter("visualization_out","brain/visual");
      this->declare_parameter("visualization_publish_interval",1.0);

      /* interval and location to publish the arena configuration */
      this->declare_parameter("arena_out","arena");
      this->declare_parameter("arena_publish_interval",3.0);

      /* interval and location to publish the goal */
      this->declare_parameter("goal_out","goal");
      this->declare_parameter("goal_publish_interval",0.1);

      /* the algorithm or module to use to command
       * the robot.                               */
      this->declare_parameter("algorithm","static_box");

      /* the range in [0 - 100] to consider a square
       * area of the map an obstacle.                */
      this->declare_parameter("obstacle_threshold",30);

      /* the width and height of a sample area */
      this->declare_parameter("sample_width",0.1);
      this->declare_parameter("sample_height",0.1);

      /* the topic to recieve odometry on */
      this->declare_parameter("odom_in","odom");

      /* the topic to recieve the map on */
      this->declare_parameter("map_in","map");

      /* set all callbacks */

      arena_out = this->create_publisher<kurome::msg::CompetitionArea>(
            this->get_parameter("arena_out").as_string(), 10);
      arena_callback = this->create_wall_timer(
            std::chrono::milliseconds((long)(1000.0 *
                  this->get_parameter("arena_publish_interval").as_double())),
            std::bind(&Brain::publish_arena, this));

      goal_out = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            this->get_parameter("goal_out").as_string(), 10);
      goal_callback = this->create_wall_timer(
            std::chrono::milliseconds((long)(1000.0 *
                  this->get_parameter("goal_publish_interval").as_double())),
            std::bind(&Brain::publish_goal, this));

      if (this->get_parameter("publish_visualization").as_bool()) {

         visual_out = this->create_publisher<visualization_msgs::msg::MarkerArray>(
               this->get_parameter("visualization_topic").as_string(), 10);
         visual_callback = this->create_wall_timer(
               std::chrono::milliseconds(((long)(1000.0 *
                     this->get_parameter("visual_publish_interval").as_double()))),
               std::bind(&Brain::publish_visual, this));
      }

      odom_in = this->create_subscription<nav_msgs::msg::Odometry>(
            this->get_parameter("odom_in").as_string(), 10,
            std::bind(&Brain::collect_odometry, this, _1));

      map_in = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            this->get_parameter("map_in").as_string(), 10,
            std::bind(&Brain::collect_map, this, _1));

      tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());

      tf_listener = std::make_unique<tf2_ros::TransformListener>(*tf_buffer);

      /* define the arena */
      arena.arena.origin.x = this->get_parameter("arena_origin_x").as_double();
      arena.arena.origin.y = this->get_parameter("arena_origin_y").as_double();
      arena.arena.width    = this->get_parameter("arena_width").as_double();
      arena.arena.height   = this->get_parameter("arena_height").as_double();
      arena.dig.origin.x   = this->get_parameter("dig_origin_x").as_double();
      arena.dig.origin.y   = this->get_parameter("dig_origin_y").as_double();
      arena.dig.width      = this->get_parameter("dig_width").as_double();
      arena.dig.height     = this->get_parameter("dig_height").as_double();
      arena.dump.origin.x  = this->get_parameter("dump_origin_x").as_double();
      arena.dump.origin.y  = this->get_parameter("dump_origin_y").as_double();
      arena.dump.width     = this->get_parameter("dump_width").as_double();
      arena.dump.height    = this->get_parameter("dump_height").as_double();

      /* setup the commander */
      if (this->get_parameter("algorithm").as_string() == "static_box") {
         commander = new StaticBoxDigger(arena,
                                         this->get_parameter("sample_width").as_double(),
                                         this->get_parameter("sample_height").as_double());
      }

   }

private:

   /* ===== Publishers ===== */

   /* Represents the arena we are competing in */
   kurome::msg::CompetitionArea arena;
   rclcpp::Publisher<kurome::msg::CompetitionArea>::SharedPtr arena_out;
   rclcpp::TimerBase::SharedPtr arena_callback;

   /* visualization of the competition area */
   rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr visual_out;
   rclcpp::TimerBase::SharedPtr visual_callback;

   rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_out;
   rclcpp::TimerBase::SharedPtr goal_callback;

   /* ===== Subscribers ===== */

   /* odometry input */
   rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_in;

   /* map input */
   rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_in;

   /* Used to transform incoming odometry data into the map
    * reference frame for the robot.                                */
   std::unique_ptr<tf2_ros::TransformListener> tf_listener;
   /* Performs all nessesary conversions between different 
    * reference frames.                                             */
   std::unique_ptr<tf2_ros::Buffer> tf_buffer;

   /* ===== Misc. ===== */

   /* where all of the actual work is done */
   CommanderBase * commander;

   /* Last transformation collected */
   geometry_msgs::msg::TransformStamped transformation;

   /* ===== Callback Functions ===== */

   void publish_arena() {

      arena_out->publish(arena);

   }

   void publish_goal() {

      geometry_msgs::msg::PoseStamped msg;
      msg.header.frame_id = "map";
      msg.header.stamp = this->get_clock()->now();

      msg.pose = commander->get_goal();

      goal_out->publish(msg);
   }

   void publish_visual() {

      visualization_msgs::msg::MarkerArray msg;

      commander->construct_visualization(msg);

      /* we want this to display in global coordinates */
      for (visualization_msgs::msg::Marker & marker : msg.markers) {
         marker.header.frame_id = "map";
         marker.header.stamp = this->get_clock()->now();
      }

      visual_out->publish(msg);

   }

   void collect_odometry(const nav_msgs::msg::Odometry & msg) {
      static int fail_count = 0;

      /* transform to map frame */
      geometry_msgs::msg::PoseStamped pose_out;
      geometry_msgs::msg::PoseStamped pose_in;
      pose_in.pose = msg.pose.pose;
      pose_in.header = msg.header;

      try {

         /* get the last avaliable transform */
         transformation = tf_buffer->lookupTransform("map","odom",
                                                     tf2::TimePointZero,
                                                     tf2::Duration(std::chrono::milliseconds(200)));

      }
      catch (const tf2::TransformException & ex) {
         RCLCPP_WARN(this->get_logger(),"transformation of odometry failed for the %dth time",fail_count++);
      }

      /* get the odometry - even if the transform fails just fall back 
       * to the last transform collected under the assumption that it 
       * did not change that much.                                    */
      tf2::doTransform<geometry_msgs::msg::PoseStamped>(pose_in,pose_out,transformation);
      pose_2d current_pose = ros2_pose_to_pose_2d(pose_out.pose);

      commander->update_position(current_pose);
   }

   void collect_map(const nav_msgs::msg::OccupancyGrid & msg) {

      commander->update_map(msg,this->get_parameter("obstacle_threshold").as_int());

   }

};

int main(int argc, char ** argv) {
   rclcpp::init(argc, argv);
   rclcpp::spin(std::make_shared<Brain>());
   rclcpp::shutdown();
   return 0;
}
