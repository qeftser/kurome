
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
      double x;
      double y;
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
      std::vector<point> rotated;
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

      /* get the coarse scan matching results */
      double rot_max = radian*30;
      for (double theta = -rot_max; theta <= rot_max; theta += radian) {
         rotated.clear();
         for (const point & p : scan.points)
            rotated.push_back(point{ p.x * cos(theta) - p.y * sin(theta),
                                     p.x * sin(theta) + p.y * cos(theta) });

         for (double x = -2.4; x <= 2.4; x += 0.3) {
            for (double y = -2.4; y <= 2.4; y += 0.3) {

               double local_certainty = 0.0;

               for (point & p : rotated)
                  local_certainty += low_res(p.x+x,p.y+y);

               results.push_back(coarse_result{local_certainty,theta,x,y});
            }
         }
      }

      std::priority_queue sorted_results (results.begin(),results.end(),cmp_coarse_result);

      pose_2d best_pose = {{0,0},0};

      /* find the best result and compute it at a high resolution. */
      while (!sorted_results.empty()) {
         coarse_result best = sorted_results.top();

         if (best.certainty < certainty)
            break;

         rotated.clear();
         for (const point & p : scan.points)
            rotated.push_back(point{ p.x * cos(best.theta) - p.y * sin(best.theta),
                                     p.x * sin(best.theta) + p.y * cos(best.theta) });

         double best_certainty = 0.0;
         point best_move;

         for (double x = -0.3; x <= 0.3; x += 0.03) {
            for (double y = -0.3; y <= 0.3; y += 0.03) {

               double local_certainty = 0.0;

               for (point & p : rotated)
                  local_certainty += high_res(p.x+best.x+x,p.y+best.y+y);

               if (local_certainty > best_certainty) {
                  best_certainty = local_certainty;
                  best_move = point{ x, y };
               }
            }
         }

         if (best_certainty > certainty) {
            certainty = best_certainty;
            best_pose = pose_2d{{ best.x + best_move.x, best.y + best_move.y}, best.theta};
         }

         sorted_results.pop();
      }

      /* compute covariance given the best value */
      rot_max = 6*radian;
      for (double theta = -rot_max; theta <= rot_max; theta += radian) {

         double r_theta = theta + best_pose.theta;

         rotated.clear();
         for (const point & p : scan.points)
            rotated.push_back(
                  point{ p.x * cos(r_theta) - p.y * sin(r_theta),
                         p.x * sin(r_theta) + p.y * cos(r_theta) });

         for (double x = -0.1; x <= 0.1; x += 0.01) {
            for (double y = -0.1; y <= 0.1; y += 0.01) {

               point pos = { x + best_pose.pos.x, y + best_pose.pos.y };

               double prob = 0.0;

               for (point & p : rotated)
                  prob += high_res(pos.x,pos.y);

               K.xx += pos.x*pos.x*prob;
               K.xy += pos.x*pos.y*prob;
               K.xz += pos.x*r_theta*prob;
               K.yy += pos.y*pos.y*prob;
               K.yz += pos.y*r_theta*prob;
               K.zz += r_theta*r_theta*prob;

               u.pos.x += pos.x*prob;
               u.pos.y += pos.y*prob;
               u.theta += r_theta*prob;

               s += prob;

            }
         }

      }

      double s_2 = s*s;
      ret_covariance.xx = (K.xx / s) - ((u.pos.x*u.pos.x) / s_2);
      ret_covariance.xy = (K.xy / s) - ((u.pos.x*u.pos.y) / s_2);
      ret_covariance.xz = (K.xz / s) - ((u.pos.x*u.theta) / s_2);
      ret_covariance.yy = (K.yy / s) - ((u.pos.y*u.pos.y) / s_2);
      ret_covariance.yz = (K.yz / s) - ((u.pos.y*u.theta) / s_2);
      ret_covariance.zz = (K.zz / s) - ((u.theta*u.theta) / s_2);

      /* update the guess with the best computed value */
      guess_ret_pose.pos.x += best_pose.pos.x;
      guess_ret_pose.pos.y += best_pose.pos.y;
      guess_ret_pose.theta += best_pose.theta;
      guess_ret_pose.theta = atan2(sin(guess_ret_pose.theta),
                                   cos(guess_ret_pose.theta));

      return certainty / (scan.points.size() * 100.0);
   }

};

#endif
