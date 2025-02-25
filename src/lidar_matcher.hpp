
#ifndef __LIDAR_MATCHER

#define __LIDAR_MATCHER
#include "kurome.h"
#include "lidar_data.hpp"
#include "observation.hpp"
#include "occupancy_grid.hpp"
#include <queue>


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

      (void)scan;
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

/* based on the paper:
 * Real-Time Correlative Scan Matching */
class CorrelativeLidarMatcher : public LidarMatcher {
private:

   struct coarse_result {
      double certainty;
      double theta;
      int x;
      int y;
   };

   struct point_int {
      int x;
      int y;
   };

   struct pose_2d_int {
      point_int pos;
      double theta;
   };

   struct {
      bool operator()(const coarse_result & l, const coarse_result & r) {
         return l.certainty < r.certainty;
      }
   } cmp_coarse_result;

public:

   double match_scan(const LidarData & scan, const LidarData & other,
                     pose_2d & guess_ret_pose, Covariance3 & ret_covariance) override {
      static const double radian = 0.01745329;
      const point * const last = &scan.points.back();
      point_int * rotated = (point_int *)malloc(sizeof(point_int)*scan.points.size());
      const point_int * const r_end = rotated+(scan.points.size());
      std::vector<coarse_result> results;

      double certainty = 0.0;

      OccupancyGrid high_res = OccupancyGrid(0.03,100,0);
      OccupancyGrid low_res  = OccupancyGrid(0.30,100,0);
      high_res.add_scan(other,guess_ret_pose);
      low_res.add_scan(other,guess_ret_pose);

      /* vars for covariance computation */
      Covariance3 & K = ret_covariance;
      K.xx = K.xy = K.xz = K.yy = K.yz = K.zz = 0.0;
      pose_2d u;
      u.pos.x = u.pos.y = u.theta = 0.0;
      double s = 0.0;

      double cos_t, sin_t;

      /* get the coarse scan matching results */
      double rot_max = radian*24;
      for (double theta = -rot_max; theta <= rot_max; theta += radian) {
         cos_t = cos(theta); sin_t = sin(theta);
         {
            point_int * result = rotated;
            point const * target = &scan.points.front();
            while (target <= last) {
               *result = (point_int{ (int)((target->x * cos_t - target->y * sin_t) / 0.3),
                                     (int)((target->x * sin_t + target->y * cos_t) / 0.3) });
               ++target; ++result;
            }
         }

         for (int x = -7; x <= 7; x += 1) {
            for (int y = -7; y <= 7; y += 1) {

               double local_certainty = 0.0;

               {
                  point_int * element = rotated;
                  while (element < r_end) {
                     local_certainty += low_res(element->x+x,element->y+y);
                     ++element;
                  }
               }

               results.push_back(coarse_result{local_certainty,theta,x*10,y*10});
            }
         }
      }

      std::priority_queue sorted_results (results.begin(),results.end(),cmp_coarse_result);

      pose_2d_int best_pose = {{0,0},0};

      /* find the best result and compute it at a high resolution. */
      while (!sorted_results.empty()) {
         coarse_result best = sorted_results.top();

         if (best.certainty < certainty || best.certainty == 0.0)
            break;

         cos_t = cos(best.theta); sin_t = sin(best.theta);
         {
            point_int * result = rotated;
            point const * target = &scan.points.front();
            while (target <= last) {
               *result = (point_int{ (int)((target->x * cos_t - target->y * sin_t) / 0.03),
                                     (int)((target->x * sin_t + target->y * cos_t) / 0.03) });
               ++target; ++result;
            }
         }

         double best_certainty = 0.0;
         point_int best_move;

         for (int x = -10; x <= 10; x += 1) {
            for (int y = -10; y <= 10; y += 1) {

               double local_certainty = 0.0;

               {
                  point_int * element = rotated;
                  while (element < r_end) {
                     local_certainty += high_res(element->x+best.x+x,element->y+best.y+y);
                     ++element;
                  }
               }

               if (local_certainty > best_certainty) {
                  best_certainty = local_certainty;
                  best_move = point_int{ x, y };
               }
            }
         }

         if (best_certainty > certainty) {
            certainty = best_certainty;
            best_pose = pose_2d_int{{ best.x + best_move.x, best.y + best_move.y}, best.theta};
         }

         sorted_results.pop();
      }

      /* compute covariance given the best value */
      rot_max = 6*radian;
      for (double theta = -rot_max; theta <= rot_max; theta += radian) {

         double r_theta = theta + best_pose.theta;

         cos_t = cos(r_theta); sin_t = sin(r_theta);
         {
            point_int * result = rotated;
            point const * target = &scan.points.front();
            while (target <= last) {
               *result = (point_int{ (int)((target->x * cos_t - target->y * sin_t) / 0.03),
                                     (int)((target->x * sin_t + target->y * cos_t) / 0.03) });
               ++target; ++result;
            }
         }

         for (int x = -10; x <= 10; x += 1) {
            for (int y = 10; y <= 10; y += 1) {

               point_int pos = { x + best_pose.pos.x, y + best_pose.pos.y };

               double prob = 0.0;

               {
                  point_int * element = rotated;
                  while (element < r_end) {
                     prob += high_res(element->x+pos.x,element->y+pos.y);
                     ++element;
                  }
               }

               s += prob;

               /*
               K.xx += ((x*0.03)*(x*0.03)*prob);
               K.xy += ((x*0.03)*(y*0.03)*prob);
               K.yy += ((y*0.03)*(y*0.03)*prob);
               K.zz += ((theta*theta*prob));
               */

            }
         }

      }

      /*
      K.xx /= s;
      K.yy /= s;
      K.xy /= s;
      K.zz /= s;

      K.xz = 1e-10;
      K.yz = 1e-10;
      */
      K.xx = 1e-10;
      K.yy = 1e-10;
      K.zz = 1e-8;

      /* update the guess with the best computed value */
      guess_ret_pose.pos.x += best_pose.pos.x * 0.03;
      guess_ret_pose.pos.y += best_pose.pos.y * 0.03;
      guess_ret_pose.theta += best_pose.theta;
      guess_ret_pose.theta = atan2(sin(guess_ret_pose.theta),
                                   cos(guess_ret_pose.theta));

      return certainty / (scan.points.size() * 100.0);
   }

};

#endif
