
/* A small as possible ros2 node that publishes a 
 * hardcoded path via it's callback message.      */

#include <functional>
#include <chrono>

#include <rclcpp/rclcpp.hpp>

#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose.hpp"

using std::placeholders::_1;

class HardcodedPathPublisher : public rclcpp::Node {
public:

   HardcodedPathPublisher() : Node("hardcoded_path_publisher") {

      /* construct our publisher */
      path_out = this->create_publisher<nav_msgs::msg::Path>("path", 10);

      /* place the publisher on a timer */
      path_callback = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&HardcodedPathPublisher::publish_path, this));

   }

   /* publisher */
   rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_out;

   /* timer the publisher runs on */
   rclcpp::TimerBase::SharedPtr path_callback;

   /* define and send out our path message */
   void publish_path() {

      nav_msgs::msg::Path path;

      path.header.stamp = this->get_clock()->now();
      path.header.frame_id = "base_link";

      geometry_msgs::msg::PoseStamped pose0;
      pose0.pose.position.x = 3;
      pose0.pose.position.y = 1;
      geometry_msgs::msg::PoseStamped pose1;
      pose1.pose.position.x = 5;
      pose1.pose.position.y = 0;
      geometry_msgs::msg::PoseStamped pose2;
      pose2.pose.position.x = 7;
      pose2.pose.position.y = 7;

      path.poses.push_back(pose0);
      path.poses.push_back(pose1);
      path.poses.push_back(pose2);

      path_out->publish(path);

   }

};

int main(int argc, char ** argv) {
   rclcpp::init(argc,argv);
   rclcpp::spin(std::make_shared<HardcodedPathPublisher>());
   rclcpp::shutdown();
   return 0;
}

