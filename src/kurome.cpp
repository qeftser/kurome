
#include "kurome.h"

std::vector<std::string> split(std::string & str, const std::string & delimiter) {
   std::vector<std::string> splits;
   size_t pos = 0;
   std::string token;
   
   while ((pos = str.find(delimiter)) != std::string::npos) {
      token = str.substr(0,pos);
      splits.push_back(token);
      str.erase(0,pos+delimiter.length());
   }
   splits.push_back(str);

   return splits;
}

double time_dist(const builtin_interfaces::msg::Time & t1, const rclcpp::Time & t2) {
   return fabs(rclcpp::Time(t1.sec,t1.nanosec).seconds() - t2.seconds());
}

double time_dist(const builtin_interfaces::msg::Time & t1, const builtin_interfaces::msg::Time & t2) {
   return fabs(rclcpp::Time(t1.sec,t1.nanosec).seconds() - rclcpp::Time(t2.sec,t2.nanosec).seconds());
}
