
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


sensor_msgs::msg::LaserScan transform_scan(const sensor_msgs::msg::LaserScan & scan,
                                           const geometry_msgs::msg::TransformStamped & transform) {
}

