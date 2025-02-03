
#ifndef __KUROME_GLOBALS

#define __KUROME_GLOBALS
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/transform.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

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
