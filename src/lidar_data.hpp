
#ifndef __LIDAR_DATA

#define __LIDAR_DATA
#include "kurome.h"
#include <cmath>
#include <cstring>

/* A nice abstraction over the raw LiDAR
 * data that has some nice things that you 
 * don't get when dealing with the ros2 lidar 
 * message type.                          */
class LidarData {
public:

   /* Origin of the points in
    * the laser scan         */
   pose_2d center = {{0,0},0};

   /* The points in the scan */
   std::vector<point> points;

   LidarData() {};
   
   /* Construct a instance of LidarData from a ros2
    * LaserScan message type. Assume center of 0,0 */
   LidarData(const sensor_msgs::msg::LaserScan & msg) {

      points = std::vector<point>();

      int entries = (msg.angle_max - msg.angle_min) / msg.angle_increment;
      double angle = msg.angle_min;
      for (int i = 0; i < entries; ++i) {

         if (msg.ranges[i] < msg.range_min || msg.ranges[i] > msg.range_max)
            continue;

         points.push_back(point{ msg.ranges[i] * cos(angle), msg.ranges[i] * sin(angle) });

         angle += msg.angle_increment;
      }
   }

   /* Construct an instance of LidarData using the provided offset. */
   LidarData(const sensor_msgs::msg::LaserScan & msg, const pose_2d & offset) 
      : LidarData(msg) {
      apply_offset(offset);
   }
   LidarData(const sensor_msgs::msg::LaserScan & msg, const geometry_msgs::msg::Pose & offset) {
      pose_2d as_pose_2d = ros2_pose_to_pose_2d(offset);
      LidarData(msg,as_pose_2d);
   }

   /* Offset the scan data by the given amount. Center does not change.
    * This means that all points in the scan will be rotated and then
    * translated by the provided offset. Useful when working with 
    * robot transforms where the given scanner is not at 0,0 of the robot */
   void apply_offset(const pose_2d & offset) {
      for (size_t i = 0; i < points.size(); ++i)
         points[i] = transform(points[i],offset);
   }
   void apply_offset(const geometry_msgs::msg::Pose & offset) {
      pose_2d as_pose_2d = ros2_pose_to_pose_2d(offset);
      apply_offset(as_pose_2d);
   }

   /* Move the scan and all of it's points to another location on the
    * global map. Like apply_offset but affects and is affected by the center */
   void change_position(const pose_2d & position) {
      static const pose_2d origin_ref = {{0,0},0};

      /* undo the previous transform */
      if (memcmp(&center,&origin_ref,sizeof(pose_2d))) {

         for (size_t i = 0; i < points.size(); ++i)
            points[i] = inv_transform(points[i],center);
      }

      /* applying this as an offset and changing the center will do
       * what we want here.                                        */
      apply_offset(position);
      center = position;
   }
   void change_position(const geometry_msgs::msg::Pose & position) {
      pose_2d as_pose_2d = ros2_pose_to_pose_2d(position);
      change_position(as_pose_2d);
   }

};

#endif
