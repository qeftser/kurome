
#include <functional>

#include <rclcpp/rclcpp.hpp>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/exceptions.h"
#include "nav_msgs/msg/odometry.hpp"

/* Simple node that publishes the odometry and map -> odom
 * poses, mostly for visualization and debugging purposes */

using std::placeholders::_1;

class PosePublisher : public rclcpp::Node {
public:

   PosePublisher() : Node("pose_publisher") {

      /* Where to receive the input */
      this->declare_parameter("odom_in","odom");

      /* Whether to translate and publish the map odometry */
      this->declare_parameter("produce_map_frame",true);

      /* Topic to publish the odometry pose on */
      this->declare_parameter("odom_pose_out","odom_pose");

      /* Topic to publish the map frame pose on */
      this->declare_parameter("map_pose_out","map_pose");

      /* initialize vars */
      odom_pose = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            this->get_parameter("odom_pose_out").as_string(), 10);

      map_pose = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            this->get_parameter("map_pose_out").as_string(), 10);

      odom_in = this->create_subscription<nav_msgs::msg::Odometry>(
            this->get_parameter("odom_in").as_string(), 10,
            std::bind(&PosePublisher::collect_odometry, this, _1));

      tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());

      tf_listener = std::make_unique<tf2_ros::TransformListener>(*tf_buffer);

   }

private:

   /* The current pose as given by odometry */
   rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr odom_pose;

   /* The current pose translated into the map frame */
   rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr map_pose;

   /* Subscription to the odometry */
   rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_in;

   /* Used for transforming the pose into the map frame */
   std::unique_ptr<tf2_ros::TransformListener> tf_listener;
   std::unique_ptr<tf2_ros::Buffer> tf_buffer;

   void collect_odometry(const nav_msgs::msg::Odometry & msg) {

      geometry_msgs::msg::PoseStamped pose;
      pose.pose = msg.pose.pose;
      pose.header= msg.header;

      if (this->get_parameter("produce_map_frame").as_bool()) {

         try {

            geometry_msgs::msg::PoseStamped m_pose;
            tf_buffer->transform<geometry_msgs::msg::PoseStamped>(pose,m_pose,"map",
                  tf2::Duration(std::chrono::milliseconds(200)));

            m_pose.header.frame_id = "map";
            map_pose->publish(m_pose);

         }
         catch(const tf2::TransformException & ex) {
         }
      }

      pose.header.frame_id = "map"; /* this is so we can see it */
      odom_pose->publish(pose);

   }
};

int main(int argc, char ** argv) {
   rclcpp::init(argc, argv);
   rclcpp::spin(std::make_shared<PosePublisher>());
   rclcpp::shutdown();
   return 0;
}
