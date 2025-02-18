
#include "kurome.h"
#include <cmath>

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

double quaternion_to_z_rotation(const geometry_msgs::msg::Quaternion & q) {
   double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
   double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
   return atan2(siny_cosp,cosy_cosp);
}

pose_2d ros2_pose_to_pose_2d(const geometry_msgs::msg::Pose & p) {
   return {{p.position.x,p.position.y},quaternion_to_z_rotation(p.orientation)};
}

point transform(const point & target, const pose_2d & trans) {
   return point{ target.x * cos(trans.theta) - target.y * sin(trans.theta) + trans.pos.x,
                 target.x * sin(trans.theta) + target.y * cos(trans.theta) + trans.pos.y };
}

point inv_transform(const point & target, const pose_2d & trans) {
   point translated = { target.x - trans.pos.x, target.y - trans.pos.y };
   return point{ target.x * cos(trans.theta) + target.y * sin(trans.theta),
                 target.y * cos(trans.theta) - target.x * cos(trans.theta) };
}

