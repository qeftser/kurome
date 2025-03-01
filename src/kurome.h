
#ifndef __KUROME_GLOBALS

#define __KUROME_GLOBALS
#include <string>
#include <vector>
#include <algorithm>

#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/color_rgba.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "geometry_msgs/msg/transform.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "tf2/LinearMath/Quaternion.h"

/* Represents a position
 * on the map           */
struct point {
   double x;
   double y;
};

/* represents a pose in the
 * world assuming 3 dof    */
struct pose_2d {
   point pos;
   double theta = 0.0;
};

/* represents the relative 
 * velocity information in
 * the 2d case.           */
struct velocity_2d {
   double linear;
   double angular;
};

/* A point with a z axis
 * in adition to the other two */
struct point3 {
   union {
      point pos_2d;
      struct {
         double x;
         double y;
      };
   };
   double z;
};

/* A handy method for use with maps. 
 * We basically or v1 with v2 shifted
 * to the upper 32 bits.             */
#define long_from_ints(v1,v2) \
   (((((long)(v1))&0xffffffff))|((((long)(v2))&0xffffffff)<<32))

/* set two ints given a long value */
#define ints_from_long(l,i1,i2)        \
   do {                                \
      ((i1) = ((l)&0xffffffff));       \
      ((i2) = (((l)>>32)&0xffffffff)); \
   } while (false)

/* a nice abstraction over the very verbose method
 * of removing an element completely from a vector */
#define erase_from_vector(vector,element)                                 \
   do {                                                                   \
      auto result = std::remove((vector).begin(),(vector).end(),element); \
      (vector).erase(result,(vector).end());                              \
   } while (false)

/* a dirty method of getting a
 * value to be seen as a long. 
 * Note: this will only convert
 * the first 8 bytes to a long, as
 * that is all the long holds */
#define value_to_long(val) \
   (*(long *)((void *)&(val)))

/* another dirty method of getting
 * a pointer to be seen as a long.
 * This is useful for storing pointers
 * in sets.                         */
#define ptr_to_long(ptr) \
   ((long)(ptr))

/* Return a vector of strings that are the individual parts
 * of the passed value str, but split along the passed string
 * delimiter. Do not include the delimiter in the returned values. */
std::vector<std::string> split(std::string & str, const std::string & delimiter);

/* return the distance between two ros2 time values */
double time_dist(const builtin_interfaces::msg::Time & t1, const rclcpp::Time & t2);
double time_dist(const builtin_interfaces::msg::Time & t1, const builtin_interfaces::msg::Time & t2);

/* distance squared between two points */
double point_dist2(const point & p1, const point & p2);

/* Produce a z rotation in radians given a
 * quaternion ros2 message. Note that this
 * assumes the 3-2-1 rotation application order */
double quaternion_to_z_rotation(const geometry_msgs::msg::Quaternion & q);

/* Produce a pose_2d value given a pose
 * ros2 message. Note that this assumes the
 * 3-2-1 rotation application order.       */
pose_2d ros2_pose_to_pose_2d(const geometry_msgs::msg::Pose & p);

/* Produce a ros2 pose message type given a pose_2d */
geometry_msgs::msg::Pose pose_2d_to_ros2_pose(const pose_2d & p);

/* Simple transformation in the order rotation -> translation */
point transform(const point & target, const pose_2d & transformation);
pose_2d transform(const pose_2d target, const pose_2d & transformation);

/* Simple inverse transformation in the order inv_translation -> inv_rotation
 * This is such that A = inv_transform(transform(A,X),X) when A is a point and
 * X is a transform.                                                           */
point inv_transform(const point & target, const pose_2d & transformation);

/* Estimate the position of the robot given an initial position, velocity
 * and timestep.                                                         */
pose_2d estimate_movement(pose_2d pose, velocity_2d vel, double timestep);

/* Return the pose that represents the movement to arrive at a
 * from b                                                     */
pose_2d a_from_b(const pose_2d & a, const pose_2d & b);

#endif
