
#ifndef __LIDAR_SCAN_MATCHER

#define __LIDAR_SCAN_MATCHER
#include "kurome.h"
#include "lidar_data.hpp"
#include "observation.hpp"
#include "occupancy_grid.hpp"

class LidarScanMatcher {
public:

   virtual double match_scan(const LidarData & scan, const OccupancyGrid & map,
                             pose_2d & ret_pose, Covariance3 & ret_covariance) = 0;

};

#endif
