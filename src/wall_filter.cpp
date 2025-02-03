
#include <functional>

#include <rclcpp/rclcpp.hpp>

#include "builtin_interfaces/msg/time.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/map_meta_data.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

/* A small node in Kurome designed to filter out point cloud or
 * LiDAR points that are outside of a provided boundary.
 * Note: This node needs transforms from the given sensors to the
 * map frame in order to be used. This is because it needs to 
 * know the global position of points in order to determine 
 * whether or not they are in bounds.                            */

using std::placeholders::_1;

class WallFilter : public rclcpp::Node {
public:

   WallFilter() : Node("wall_filter") {

   /* =================== parameters =================== */
   /* The distance from the edge of the map to begin allowing
    * points to pass through the filter. A value of zero will
    * cull points past the edge of the map, but not those
    * before it. Values that are very high will result in 
    * culling points that are equiviliantly far from the wall.
    * This value is in meters, set it according to the 
    * accuracy of your sensors.                               */
   this->declare_parameter("filter_inset",0.1);
   /* List of LaserScan topics to subscribe to. Each topic
    * should be seperated by a comma.                         */
   this->declare_parameter("scan_topics","scan");
   /* List of PointCloud2 topics to subscribe to. Each topic
    * should be seperated by a comma,                         */
   this->declare_parameter("cloud_topics","points");
   /* Where is the map info coming from?                      */
   this->declare_parameter("map_info","brain/map_info");

   }

private:

   /* =================== publishers =================== */
   /* The processed point cloud data.                       */
   rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_out;
   /* The processed scan data.                              */
   rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr   scan_out;

   /* =================== subscribers =================== */
   /* The point cloud data before processing.               */
   std::vector<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr> cloud_in;
   /* The scan data before processing.                      */
   std::vector<rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr>   scan_in;
   /* Info on the size of the map, the area that we should 
    * cull points outside of.                               */
   rclcpp::Subscription<nav_msgs::msg::MapMetaData>::SharedPtr    map_info;

   /* =================== transforms =================== */
   /* The buffer and listener we will use to collect the 
    * transforms of the messages we receive and convert 
    * them to the appropriate global coordinate frames.
    * Then, we will filter the data and send it forward,
    * with the frame changed to the new one.                */
   std::unique_ptr<tf2_ros::Buffer>            tf_buffer;
   std::unique_ptr<tf2_ros::TransformListener> tf_listener;


};

int main(int argc, char ** argv) {
   rclcpp::init(argc,argv);
   rclcpp::spin(std::make_shared<WallFilter>());
   rclcpp::shutdown();
   return 0;
}
