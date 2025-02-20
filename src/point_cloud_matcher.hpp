
#ifndef __POINT_CLOUD_MATCHER

#define __POINT_CLOUD_MATCHER
#include "kurome.h"
#include "lidar_data.hpp"
#include "observation.hpp"
#include "occupancy_grid.hpp"

class PointCloudMatcher {
public:

   /* attempts to compute the best alignment of two point clouds in three
    * dimensional space, filling the ret_pose and ret_covariance values with
    * the two dimensional portions of the result. Returns the certainty in
    * the range [0.0 - 1.0].                                               */
   virtual double match_point_cloud(const PointCloudData & current, 
                                    const PointCloudData & reference,
                                    pose_2d & ret_pose, Covariance3 & ret_covariance) = 0;

};

class DummyPointCloudMatcher : public PointCloudMatcher {

   double match_point_cloud(const PointCloudData & current, 
                            const PointCloudData & reference,
                            pose_2d & ret_pose, Covariance3 & ret_covariance) {

      (void)reference;

      ret_pose = current.center;

      ret_covariance.xx = 1e10;
      ret_covariance.yy = 1e10;
      ret_covariance.zz = 1e10;

      ret_covariance.xy = 1e12;
      ret_covariance.xz = 1e12;
      ret_covariance.yz = 1e12;

      return 0.0;
   }

};

#endif
