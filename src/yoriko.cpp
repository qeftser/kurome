
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

};

int main(int argc, char ** argv) {
   rclcpp::init(argc,argv);
   rclcpp::spin(std::make_shared<Yoriko>());
   rclcpp::shutdown();
   return 0;
}
