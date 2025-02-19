
#ifndef __SLAM_SYSTEM

#define __SLAM_SYSTEM
#include "kurome.h"
#include "observation.hpp"
#include "occupancy_grid.hpp"

/* as it sounds. A wrapper over whatever graph slam
 * backend is used in the final system. Similar to the
 * pathfinder and smoother base classes in that it defines
 * a loose outline of how the system is expected to work
 * and leaves the rest to the child classes.             */

class SlamSystem {

   rclcpp::Clock::SharedPtr clock;

public:

   SlamSystem(rclcpp::Clock::SharedPtr clock) 
      : clock(clock) {}

   static const bool FIX_NODE = true;
   static const bool NOFIX_NODE = false;

   /* add an observation that may or may not be fixed to the pose graph */
   virtual void insert_observation(Observation * observation, bool fixed = NOFIX_NODE) = 0;

   /* fill the associated marker array with a visualization of the slam system's
    * current state.                                                            */
   virtual void construct_visualization(visualization_msgs::msg::MarkerArray & msg) = 0;
   
   /* Note: this function is expected to allocate the memory and
    * the calling function hold the responsibility to deallocate it
    * using delete                                                */
   virtual void get_map(nav_msgs::msg::OccupancyGrid * ret_msg) = 0;

   /* return the best estimate of the current pose of the robot
    * from the slam system.                                    */
   virtual pose_2d get_pose(const velocity_2d & vel) = 0;




};

#endif
