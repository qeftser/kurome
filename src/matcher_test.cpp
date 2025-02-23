
#include <functional>

#include <rclcpp/rclcpp.hpp>

#include "sensor_msgs/msg/laser_scan.hpp"

#include "kurome.h"
#include "lidar_data.hpp"
#include "lidar_matcher.hpp"

/* Node for testing various matcher types. When a scan
 * is received it will be matched againts itself, a random
 * offset of itself, and the previous scan recieved.     */

using std::placeholders::_1;

class MatcherTest : public rclcpp::Node {
public:

   MatcherTest() : Node("matcher_test") {

      /* the topic to listen for scans on */
      this->declare_parameter("scan_in","scan");

      /* the algorithm to match with */
      this->declare_parameter("matching_algorithm","correlative");

      scan_in = this->create_subscription<sensor_msgs::msg::LaserScan>(
            this->get_parameter("scan_in").as_string(), 10,
            std::bind(&MatcherTest::collect_scan, this, _1));

      if (this->get_parameter("matching_algorithm").as_string() ==
          "correlative") {
         lidar_matcher = new CorrelativeLidarMatcher();
      }

      RCLCPP_INFO(this->get_logger(),"listening on %s\n",this->get_parameter("scan_in").as_string().c_str());
   }

private:

   /* reception area for the scans */
   rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_in;

   /* the previous scan collected */
   LidarData last_scan;

   /* the lidar matcher we are using */
   LidarMatcher * lidar_matcher;

   void collect_scan(const sensor_msgs::msg::LaserScan & msg) {

      pose_2d pose;
      Covariance3 covariance;
      double certainty;
      LidarData data(msg);

      pose_2d rand_move = {{((((double)rand())/RAND_MAX) * 2.0) - 1.0,
                            ((((double)rand())/RAND_MAX) * 2.0) - 1.0},
                            ((((double)rand())/RAND_MAX) * 0.5) - 0.25 };
      LidarData move(msg,rand_move);

      RCLCPP_INFO(this->get_logger(),"matching with self");
      clock_t tStart = clock();

      pose = {{0,0},0};
      certainty = lidar_matcher->match_scan(data,data,pose,covariance);

      RCLCPP_INFO(this->get_logger(),"finished in %.5f sec",((double)(clock() - tStart) / CLOCKS_PER_SEC));
      printf("certainty: %f\n",certainty);
      printf("best pose: %f %f %f\n",pose.pos.x,pose.pos.y,pose.theta);
      printf("covariance: %7e %7e %7e\n",covariance.xx,covariance.xy,covariance.xz);
      printf("            %7e %7e %7e\n",covariance.xy,covariance.yy,covariance.yz);
      printf("            %7e %7e %7e\n",covariance.xz,covariance.yz,covariance.zz);

      /*
      RCLCPP_INFO(this->get_logger(),"matching with movement %f %f %f",rand_move.pos.x,rand_move.pos.y,rand_move.theta);
      tStart = clock();

      pose = {{0,0},0};
      certainty = lidar_matcher->match_scan(data,move,pose,covariance);

      RCLCPP_INFO(this->get_logger(),"finished in %.5f sec",((double)(clock() - tStart) / CLOCKS_PER_SEC));
      printf("certainty: %f\n",certainty);
      printf("best pose: %f %f %f\n",pose.pos.x,pose.pos.y,pose.theta);
      printf("covariance: %7e %7e %7e\n",covariance.xx,covariance.xy,covariance.xz);
      printf("            %7e %7e %7e\n",covariance.xy,covariance.yy,covariance.yz);
      printf("            %7e %7e %7e\n",covariance.xz,covariance.yz,covariance.zz);

      RCLCPP_INFO(this->get_logger(),"matching with previous scan");
      tStart = clock();

      pose = {{0,0},0};
      certainty = lidar_matcher->match_scan(data,last_scan,pose,covariance);

      RCLCPP_INFO(this->get_logger(),"finished in %.5f sec",((double)(clock() - tStart) / CLOCKS_PER_SEC));
      printf("certainty: %f\n",certainty);
      printf("best pose: %f %f %f\n",pose.pos.x,pose.pos.y,pose.theta);
      printf("covariance: %7e %7e %7e\n",covariance.xx,covariance.xy,covariance.xz);
      printf("            %7e %7e %7e\n",covariance.xy,covariance.yy,covariance.yz);
      printf("            %7e %7e %7e\n",covariance.xz,covariance.yz,covariance.zz);
      */

      last_scan = data;
   }

};

int main(int argc, char ** argv) {
   rclcpp::init(argc, argv);
   rclcpp::spin(std::make_shared<MatcherTest>());
   rclcpp::shutdown();
   return 0;
}
