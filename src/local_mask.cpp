
#include <functional>

#include <rclcpp/rclcpp.hpp>

#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/exceptions.h"

#include <cassert>

/* Very basic node that culls sensor readings that are off
 * of the robot body.                                    */

using std::placeholders::_1;

class LocalMask : public rclcpp::Node {
public:

   LocalMask() : Node("local_mask") {

      /* incoming scans */
      this->declare_parameter("scan_in","scan");

      /* outgoing scans */
      this->declare_parameter("scan_out","scan_masked");

      /* Distance from zero to cull the data at and below */
      this->declare_parameter("cull_dist",0.3);

      /* setup variables */
      scan_in = this->create_subscription<sensor_msgs::msg::LaserScan>(
            this->get_parameter("scan_in").as_string(), 10,
            std::bind(&LocalMask::collect_scan, this, _1));

      scan_out = this->create_publisher<sensor_msgs::msg::LaserScan>(
            this->get_parameter("scan_out").as_string(), 10);

      tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());

      tf_listener = std::make_unique<tf2_ros::TransformListener>(*tf_buffer);

   }

private:

   /* class variables */

   rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_out;

   rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_in;

   std::unique_ptr<tf2_ros::TransformListener> tf_listener;
   std::unique_ptr<tf2_ros::Buffer> tf_buffer;

   /* scan collection/modification/release */

   void collect_scan(const sensor_msgs::msg::LaserScan & msg) {

      sensor_msgs::msg::LaserScan out(msg);
      
      /* This is kind of crappy to do, but it should work
       * as a solution for now.                          */

      int entries = (out.angle_max - out.angle_min) / out.angle_increment;
      double cutoff = this->get_parameter("cull_dist").as_double();

      for (int i = 0; i < entries; ++i) {

         if (out.ranges[i] < cutoff || out.ranges[i] > out.range_max)
            out.ranges[i] = 1000.0;

         assert(out.ranges[i] > 0.5);

      }

      out.header.stamp = this->get_clock()->now();
      scan_out->publish(out);
      
   }

};

int main(int argc, char ** argv) {
   rclcpp::init(argc, argv);
   rclcpp::spin(std::make_shared<LocalMask>());
   rclcpp::shutdown();
   return 0;
}
