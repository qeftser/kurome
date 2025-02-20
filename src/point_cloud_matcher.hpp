
#ifndef __POINT_CLOUD_MATCHER

#define __POINT_CLOUD_MATCHER
#include "kurome.h"
#include "lidar_data.hpp"
#include "observation.hpp"
#include "occupancy_grid.hpp"

class PointCloudMatcher {
public:

   virtual double match_point_cloud(const PointCloudData & current, 
                                    const PointCloudData & reference,
                                    pose_2d & ret_pose, Covariance3 & ret_covariance) = 0;

};

#endif
