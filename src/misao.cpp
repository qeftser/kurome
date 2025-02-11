
#include <functional>

#include <rclcpp/rclcpp.hpp>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "kurome.h"

/* The smoothing node for Kurome. It will mainly be used to
 * run the elastic band algorithm and forward the constructed
 * path to the controller.                                   */

using std::placeholders::_1;

class Misao : public rclcpp::Node {
public:

   Misao() : Node("misao") {

      /* The topic to recieve the raw path on */
      this->declare_parameter("path_in","path");

      /* The topic to send out the smoothed path on */
      this->declare_parameter("path_out","path_smooth");

      /* The topic to receive the environment map on */
      this->declare_parameter("map","map");

      /* Time interval in seconds to publish on */
      this->declare_parameter("publish_rate",0.1);

      /* instantiate the publisher */
      path_out = this->create_publisher<nav_msgs::msg::Path>(
            this->get_parameter("path_out").as_string(), 10);
      path_callback = this->create_wall_timer(
            std::chrono::milliseconds((long)(1000 * this->get_parameter("publish_rate").as_double())),
            std::bind(&Misao::publish_path, this));

      /* instantiate subscribers */
      path_in = this->create_subscription<nav_msgs::msg::Path>(
            this->get_parameter("path_in").as_string(), 10,
            std::bind(&Misao::collect_path, this, _1));

      map_in = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            this->get_parameter("map").as_string(), 10,
            std::bind(&Misao::collect_map, this, _1));

   }


private:

   /* The path produced by the smoother */
   rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_out;
   rclcpp::TimerBase::SharedPtr path_callback;

   /* The raw path collected to be smoothed */
   rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_in;
   /* The map the path resides in */
   rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_in;

   void collect_path(const nav_msgs::msg::Path & msg) {
   }

   void collect_map(const nav_msgs::msg::OccupancyGrid & msg) {
   }

   void publish_path() {
   }

};

int main(int argc, char ** argv) {
   rclcpp::init(argc,argv);
   rclcpp::spin(std::make_shared<Misao>());
   rclcpp::shutdown();
   return 0;
}

