
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
    * the range [0.0 - 1.0].                                               
    * current - the point cloud we want to match
    * reference - the point cloud to match to
    * guess_ret_pose - the initial guess of the offset as well as the returned
    *                  estimate for the offset                                
    * ret_covariance - the returned covariance of the measurement */
   virtual double match_point_cloud(const PointCloudData & current, 
                                    const PointCloudData & reference,
                                    pose_2d & guess_ret_pose, Covariance3 & ret_covariance) = 0;

};

class DummyPointCloudMatcher : public PointCloudMatcher {

   double match_point_cloud(const PointCloudData & current, 
                            const PointCloudData & reference,
                            pose_2d & guess_ret_pose, Covariance3 & ret_covariance) {

      (void)current;
      (void)reference;
      (void)guess_ret_pose;

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
