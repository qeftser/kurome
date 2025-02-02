
#include <functional>

#include <rclcpp/rclcpp.hpp>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "nav_msgs/msg/map_meta_data.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

/* The SLAM system for Kurome. This is an implimentation of the GraphSLAM
 * algorithm using odometry, LiDAR, Point Cloud, and IMU data. Fixing nodes
 * is also allowed via input from a local beacon                           */

using std::placeholders::_1;

class Pino : public rclcpp::Node {
public:

   Pino() : Node("pino") {
   }

private:

};


int main(int argc, char ** argv) {
   rclcpp::init(argc,argv);
   rclcpp::spin(std::make_shared<Pino>());
   rclcpp::shutdown();
   return 0;
}
