
#include <functional>

#include <rclcpp/rclcpp.hpp>

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

/* The central control system of Kurome. Coordinate with all of the
 * other nodes to ensure things are functioning correctly and to 
 * set node parameters globally. All systems related to actually completing
 * the objective are executed in part by this node.                         */

using std::placeholders::_1;

class Brain : public rclcpp::Node {
public:

   Brain() : Node("brain") {
   }

private:

};

int main(int argc, char ** argv) {
   rclcpp::init(argc, argv);
   rclcpp::spin(std::make_shared<Brain>());
   rclcpp::shutdown();
   return 0;
}
