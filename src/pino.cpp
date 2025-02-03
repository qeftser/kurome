
#include <functional>

#include <rclcpp/rclcpp.hpp>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
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

   /* =================== publishers =================== */
   /* The best estimate of our current position. This will usually
    * be the last node on the pose graph, but will be the beacon_in
    * pose if it is active.                                         */
   rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr position_out;
   /* The current map representation. This will be a rather large
    * message, so will not be sent live with every update, but 
    * rather periodically over a user-controlled interval.          */
   rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr    grid_out;

   /* =================== subscribers =================== */
   /* Input from our beacon (local positioning system) if we have one.
    * The result is fixed nodes in our pose graph and direct translation
    * of position information.                                      */
   rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr       beacon_in;
   /* Input from the various LiDAR scanners on the system. They
    * will need to have different frame_id values to distinguish
    * where they are coming from. These will be used for scan 
    * matching and map generation.                                  */
   rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr   scan_in;
   /* Input from any sort of 3d LiDAR or depth camera. Multiple of
    * these will also need different frame_ids. These will be used in
    * an ICP algorithm for scan matching, and also will be used for
    * map generation.                                               */
   rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_in;
   /* Input from an associated odometry system. Used for position
    * estimation when the input from the beacon is not avaliable.   */
   rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr         odom_in;

   /* =================== transforms =================== */
   /* Publish the map -> odom transformation as described in REP 105.
    * This is done to allow the nessesary transform math to be 
    * carried out in our other systems.                             */
   std::unique_ptr<tf2_ros::TransformBroadcaster> map_frame;

   /* =================== internal =================== */



};


int main(int argc, char ** argv) {
   rclcpp::init(argc,argv);
   rclcpp::spin(std::make_shared<Pino>());
   rclcpp::shutdown();
   return 0;
}
