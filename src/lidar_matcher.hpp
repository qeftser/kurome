
#ifndef __LIDAR_MATCHER

#define __LIDAR_MATCHER
#include "kurome.h"
#include "lidar_data.hpp"
#include "observation.hpp"
#include "occupancy_grid.hpp"

class LidarMatcher {
public:

   /* matches a lidar scan to a map, filling the ret_pose and ret_covariance
    * values and returning the certainty of the best match on the range [0.0 - 1.0] 
    * scan - the scan we want to match
    * other - the scan to match to
    * guess_ret_pose - the initial guess of the offset as well as the returned
    *                  estimate for the offset
    * ret_covariance - the returned covariance of the measurement */
   virtual double match_scan(const LidarData & scan, const LidarData & other,
                             pose_2d & guess_ret_pose, Covariance3 & ret_covariance) = 0;

};

/* useless implimentation to get the files to compile */
class DummyLidarMatcher : public LidarMatcher {

   double match_scan(const LidarData & scan, const LidarData & other,
                     pose_2d & guess_ret_pose, Covariance3 & ret_covariance) override {

      (void)other;
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
