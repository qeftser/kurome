
#include <functional>

#include <rclcpp/rclcpp.hpp>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"

/* The navigation system for Kurome. Responsible for planning a route 
 * through the map provided by Pino (the SLAM system) and communicating
 * with the control system to correctly reach the position dictated by
 * the brain                                                            */

using std::placeholders::_1;

class Yoriko : public rclcpp::Node {
public:

   Yoriko() : Node("yoriko") {
   }

private:

   /* =================== publishers =================== */
   /* The path we produce with our pathfinding algorith for use
    * by a controller or for our own use in creating a series of 
    * Twist messages to send directly to something like a 
    * ros2_control controller.                                   */
   rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr       path_out;
   /* Velocity output we send to our control node for processing.
    * This will give the movement we want between two nodes on our
    * path. This value may or may not be used, it depends on how
    * good of a controller we can get our hands on.              */
   rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_out;

   /* =================== subscribers =================== */
   /* Subscription to the goal of the navigation system, presumably
    * published to this node by the system brain. It doesn't have
    * to be though. The system will attempt to navigate towards this
    * goal.                                                      */
   rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr        goal_in;
   /* The current besst estimate of the robot's position. Provided by
    * Pino, our resident GraphSLAM algorithm. Yoriko will use this to aid
    * in following our path, and in constructing it.             */
   rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr position_in;
   /* Subscription to the occupancy grid, which is published
    * periodically by Pino. This system will path through this map,
    * attempting to reach the goal provided.                     */
   rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr    grid_in;

};

int main(int argc, char ** argv) {
   rclcpp::init(argc,argv);
   rclcpp::spin(std::make_shared<Yoriko>());
   rclcpp::shutdown();
   return 0;
}
