
#ifndef __MI_RRT_X_FN

#define __MI_RRT_X_FN

#include "rrt_x_fn.hpp"

/* 
 * Motion Informed variant of the
 * RRTX-FN algorithm             */

class MI_RRTX_FN : public RRTX_FN {
private:

   /* Minimum radius allowed on a turn
    * between two points by this planner */
   double turning_radius;

   /* here we assume that p1 is the point we are coming from,
    * and p2 is the point we are trying to get to.           */
   inline bool is_collision(const point & p1, const point & p2) override {
      /* collect the minimum turning radius needed to
       * connect these two points                  */
      point in_frame = { p2.x - p1.x, p2.y - p1.y };
      in_frame = point{ in_frame.x * cos(p1.theta) - in_frame.y * sin(p1.theta),
                        in_frame.x * sin(p1.theta) + in_frame.y * cos(p1.theta) };
      double min_rad = sqrt(in_frame.x * in_frame.x + in_frame.y * in_frame.y);
      min_rad /= sin(atan2(in_frame.y,in_frame.x));

      return RRTX_FN::is_collision(p1,p2) || (fabs(min_rad) < turning_radius);
   }

   /* Assume that these values have been deemed in an appropriate turning
    * radius, compute the angle change between then and set the angle
    * of the node relative to it's parent.                               */
   inline void make_relative_to_parent(node * n) override {
      double parent_theta = n->parent->pos.theta;
      point in_frame = { n->pos.x - n->parent->pos.x, 
                         n->pos.y - n->parent->pos.y };
      in_frame = point{ in_frame.x * cos(parent_theta) - in_frame.y * sin(parent_theta),
                        in_frame.x * sin(parent_theta) + in_frame.y * cos(parent_theta) };
      double turn = (M_PI / 2.0) - atan2(in_frame.y,in_frame.x) + parent_theta;
      in_frame.theta = atan2(sin(turn),cos(turn));
   }

public:

   MI_RRTX_FN(double collision_radius, int accept_oob_goal,
              double dominance_region, double cull_range,
              double point_set_divisors, double expansion_length, int node_limit,
              int generation_tick_speed, double turning_radius)
      : RRTX_FN(collision_radius,accept_oob_goal,dominance_region,cull_range,point_set_divisors,
                expansion_length,node_limit,generation_tick_speed), turning_radius(turning_radius) {}
};

#endif
