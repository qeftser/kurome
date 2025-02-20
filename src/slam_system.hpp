
#ifndef __SLAM_SYSTEM

#define __SLAM_SYSTEM
#include "kurome.h"
#include "observation.hpp"
#include "occupancy_grid.hpp"
#include "lidar_matcher.hpp"
#include "point_cloud_matcher.hpp"

/* as it sounds. A wrapper over whatever graph slam
 * backend is used in the final system. Similar to the
 * pathfinder and smoother base classes in that it defines
 * a loose outline of how the system is expected to work
 * and leaves the rest to the child classes.             */

class SlamSystem {
protected:

   rclcpp::Clock::SharedPtr clock;
   LidarMatcher * lidar_matcher;
   PointCloudMatcher * point_cloud_matcher;

public:

   SlamSystem(rclcpp::Clock::SharedPtr clock, 
              LidarMatcher * lidar_matcher, PointCloudMatcher * point_cloud_matcher) 
      : clock(clock), lidar_matcher(lidar_matcher), point_cloud_matcher(point_cloud_matcher) {}

   static const bool FIX_NODE = true;
   static const bool NOFIX_NODE = false;

   /* add an observation that may or may not be fixed to the pose graph */
   virtual void insert_observation(Observation * observation, bool fixed = NOFIX_NODE) = 0;

   /* fill the associated marker array with a visualization of the slam system's
    * current state.                                                            */
   virtual void construct_visualization(visualization_msgs::msg::MarkerArray & msg) = 0;
   
   /* fill the given map with the most up to date map avaliable in
    * the slam system at the current time.                        */
   virtual void get_map(nav_msgs::msg::OccupancyGrid & ret_msg) = 0;

   /* return the best estimate of the current pose of the robot
    * from the slam system, along with the associated odometry value
    * at that timestep. Used for computation of the map -> odom
    * transform.                                               */
   virtual std::pair<pose_2d,pose_2d> get_last_pose() = 0;

};

#endif
