
#include <functional>

#include <rclcpp/rclcpp.hpp>

#include "sensor_msgs/msg/laser_scan.hpp"

#include "kurome.h"
#include "lidar_data.hpp"
#include "lidar_matcher.hpp"
#include <random>

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

      /* construct normal distribution sampler */
      r.gen = std::mt19937{r.r()};
      r.d = std::normal_distribution{0.0,0.025};

      RCLCPP_INFO(this->get_logger(),"listening on %s\n",this->get_parameter("scan_in").as_string().c_str());
   }

private:

   /* reception area for the scans */
   rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_in;

   /* the previous scan collected */
   LidarData last_scan;
   
   /* the previous random movement */
   LidarData last_rand;

   /* the last time a scan was recieved */
   rclcpp::Time last_scan_time;

   /* the lidar matcher we are using */
   LidarMatcher * lidar_matcher;

   /* params for producing gaussian noise */
   struct {
      std::random_device r;
      std::mt19937 gen;
      std::normal_distribution<double> d;
   } r;

   void collect_scan(const sensor_msgs::msg::LaserScan & msg) {

      rclcpp::Time this_time = this->get_clock()->now();

      pose_2d pose;
      Covariance3 covariance;
      /*
      Information3 information;
      */
      double certainty;
      clock_t tStart;
      LidarData data(msg);

      pose_2d rand_move = {{((((double)rand())/RAND_MAX) * 2.0) - 1.0,
                            ((((double)rand())/RAND_MAX) * 2.0) - 1.0},
                            ((((double)rand())/RAND_MAX) * 0.5) - 0.25 };
                            //0.0 };
      LidarData move(msg,rand_move);
      LidarData gauss(msg);
      for (unsigned int i = 0; i < gauss.points.size(); ++i) {
         gauss.points[i].x += r.d(r.gen);
         gauss.points[i].y += r.d(r.gen);
      }

      /*
      RCLCPP_INFO(this->get_logger(),"matching with self");
      tStart = clock();

      pose = {{0,0},0};
      certainty = lidar_matcher->match_scan(data,data,pose,covariance);

      RCLCPP_INFO(this->get_logger(),"finished in %.5f sec",((double)(clock() - tStart) / CLOCKS_PER_SEC));
      printf("certainty: %f\n",certainty);
      printf("best pose: %f %f %f\n",pose.pos.x,pose.pos.y,pose.theta);
      printf("covariance: %7e %7e %7e\n",covariance.xx,covariance.xy,covariance.xz);
      printf("            %7e %7e %7e\n",covariance.xy,covariance.yy,covariance.yz);
      printf("            %7e %7e %7e\n",covariance.xz,covariance.yz,covariance.zz);
      */

      RCLCPP_INFO(this->get_logger(),"matching with adjusted self");
      tStart = clock();

      pose = {{0,0},0};
      certainty = lidar_matcher->match_scan(gauss,data,pose,covariance);

      RCLCPP_INFO(this->get_logger(),"finished in %.5f sec",((double)(clock() - tStart) / CLOCKS_PER_SEC));
      printf("certainty: %f\n",certainty);
      printf("best pose: %f %f %f\n",pose.pos.x,pose.pos.y,pose.theta);

      RCLCPP_INFO(this->get_logger(),"matching with movement %f %f %f",rand_move.pos.x,rand_move.pos.y,rand_move.theta);
      tStart = clock();

      pose = {{0,0},0};
      certainty = lidar_matcher->match_scan(data,move,pose,covariance);

      RCLCPP_INFO(this->get_logger(),"finished in %.5f sec",((double)(clock() - tStart) / CLOCKS_PER_SEC));
      printf("certainty: %f\n",certainty);
      printf("best pose: %f %f %f\n",pose.pos.x,pose.pos.y,pose.theta);
      /*
      printf("covariance: %7e %7e %7e\n",covariance.xx,covariance.xy,covariance.xz);
      printf("            %7e %7e %7e\n",covariance.xy,covariance.yy,covariance.yz);
      printf("            %7e %7e %7e\n",covariance.xz,covariance.yz,covariance.zz);
      */

      RCLCPP_INFO(this->get_logger(),"matching adjusted scan with movement %f %f %f",rand_move.pos.x,rand_move.pos.y,rand_move.theta);
      tStart = clock();

      pose = {{0,0},0};
      certainty = lidar_matcher->match_scan(gauss,move,pose,covariance);

      RCLCPP_INFO(this->get_logger(),"finished in %.5f sec",((double)(clock() - tStart) / CLOCKS_PER_SEC));
      printf("certainty: %f\n",certainty);
      printf("best pose: %f %f %f\n",pose.pos.x,pose.pos.y,pose.theta);

      /*
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

      RCLCPP_INFO(this->get_logger(),"matching with previous movement");
      tStart = clock();

      pose = {{0,0},0};
      certainty = lidar_matcher->match_scan(data,last_rand,pose,covariance);

      RCLCPP_INFO(this->get_logger(),"finished in %.5f sec",((double)(clock() - tStart) / CLOCKS_PER_SEC));
      printf("certainty: %f\n",certainty);
      printf("best pose: %f %f %f\n",pose.pos.x,pose.pos.y,pose.theta);

      last_scan_time = this_time;
      last_scan = data;
      last_rand = move;

   }

};

int main(int argc, char ** argv) {
   rclcpp::init(argc, argv);
   rclcpp::spin(std::make_shared<MatcherTest>());
   rclcpp::shutdown();
   return 0;
}
