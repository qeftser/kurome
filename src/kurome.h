
#ifndef __KUROME_GLOBALS

#define __KUROME_GLOBALS
#include <string>
#include <vector>
#include <algorithm>

#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/transform.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"

/* A handy method for use with maps. 
 * We basically or v1 with v2 shifted
 * to the upper 32 bits.             */
#define long_from_ints(v1,v2) \
   (((long)(v1))|(((long)(v2))<<32))

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
#define value_to_long(ptr) \
   (*(long *)((void *)&(ptr)))

/* Return a vector of strings that are the individual parts
 * of the passed value str, but split along the passed string
 * delimiter. Do not include the delimiter in the returned values. */
std::vector<std::string> split(std::string & str, const std::string & delimiter);

/* Convert a LaserScan message to the new coordinate system given
 * by the TransformStamped transform. This is done by changing the
 * angle_min and angle_max values according to the Z rotation described
 * in the transformation. Note that this assumes that the LiDAR is 
 * mounted permendicular to the Z axis. The range_min and range_max
 * values, as well as the values in the ranges array, are changed
 * according to the pose difference in the transformation. This
 * results in the effect that when the LaserScan is converted to
 * cartesian coordinates, these values will be the appropriate
 * positions relative to the robot. If this scan is used for other
 * purposes, such as map construction, it has the effect that all
 * scans will appear to come from the origin of the robot, which
 * could be seen as a good thing.                                 */
sensor_msgs::msg::LaserScan transform_scan(const sensor_msgs::msg::LaserScan & scan,
                                           const geometry_msgs::msg::TransformStamped & transform);

#endif
