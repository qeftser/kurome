
#include <functional>

#include <rclcpp/rclcpp.hpp>

#include "builtin_interfaces/msg/time.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/map_meta_data.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

/* A small node in Kurome designed to filter out point cloud or
 * lidar points that are outside of a provided boundary        */

using std::placeholders::_1;

class WallFilter : public rclcpp::Node {
public:

   WallFilter() : Node("wall_filter") {
   }

private:

};

int main(int argc, char ** argv) {
   rclcpp::init(argc,argv);
   rclcpp::spin(std::make_shared<WallFilter>());
   rclcpp::shutdown();
   return 0;
}
