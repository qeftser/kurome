
#ifndef __MI_A_STAR

#define __MI_A_STAR
#include "pathfinder.hpp"
#include "kurome.h"
#include <cmath>
#include <queue>
#include <vector>
#include <unordered_set>
#include <unordered_map>

class MI_AStar : public PathfinderBase {
private:

   struct node {
      point pos;
      node * next;
      int weight;
      int step;
   };

   /* How many nodes to recompute when the positon
    * of the robot updates. Setting this to a negative
    * value will result in no computation limit.      */
   int backtrack_count;

   /* Maximum size the queue can grow to during the A*
    * search. Prevents system lockup due to memory 
    * running out.                                    */
   int node_limit;

   /* The turning radius allowed to be taken
    * in the algorithm.                     */
   double turning_radius;

   /* At what granularity should the angle be taken?
    * This is needed for binning and for the update
    * step of the A* algorithm. We cannot represent
    * every angle, just like how we only represent blocks 
    * of space in A*. So we will sort of stack angle
    * bins on the 2d bins, using the granularity given here.
    * Value is in radians.                         */
   double angle_slice;

   /* Use a higher granularity than the normal A* to
    * ensure better performance out of the algorithm */
   double position_slice;

   /* What movement forward (or backward) should
    * be taken at each timestep?                */
   double forward_step;

   /* Enusre that the angle on the given 
    * node is between 0 and 2PI         */   
   inline double normalize_angle(double angle) {
      while (angle > 2*M_PI)
         angle -= 2*M_PI;
      while (angle < 0.0)
         angle += 2*M_PI;
      return angle;
   }

   /* represent the position of a node as a long value */
   inline ulong as_long(point & pos) {
      ulong x = (int)(pos.x / position_slice);
      ulong y = (int)(pos.y / position_slice);
      ulong theta = (int)(pos.theta / angle_slice);
      return ((x<<48)|(y<<42)|(theta));
   }

   /* return the position in the grid array that
    * this point falls in.                      */
   inline int point_pos(const point & p) {
      return ((int)floor(p.x) + ((int)floor(p.y) * grid_metadata.width));
   }

   /* returns 1 if the given index is
    * in the bounds of the map, 0 otherwise */
   int in_bounds(const point & p) {
      if (p.x < 0 || p.y < 0 ||
          p.x >= grid_metadata.width ||
          p.y >= grid_metadata.height)
         return 0;
      return 1;
   }

   /* compare two nodes based on their weights. Used as 
    * the priority queue comparison operator.          */
   struct cmp_node {
      bool operator()(const node * n1, const node * n2) const {
         if (n1->weight > n2->weight)
            return true;
         return false;
      }
   };

   /* squared distance between two points */
   inline double point_dist2(const point & p1, const point & p2) {
      return (p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y);
   }

   /* construct a small line at the given point
    * for use in the visualizaton function.
    * This is a bit of a cheeky function as we
    * know that points has two values in it. */
   inline void visualize_motion(sf::Vertex * points, const point & pos) {
      point move = point{ 3.0*cos(pos.theta), 3.0*sin(pos.theta) };
      points[0].position = sf::Vector2f(pos.x*10 - move.x, pos.y*10 - move.y);
      points[1].position = sf::Vector2f(pos.x*10 + move.x, pos.y*10 + move.y);
   }

   /* The last full path produced. Starts
    * from the origin of the robot here  */
   node * last_path = NULL;

   /* The mutex is nessesary for dealing
    * with the constant gui calls to the
    * draw_environment function.        */
   std::mutex mut;

   /* Flags that notify what values have changed
    * so the pathfinder knows what to update,
    * check, and recompute.                      */
   enum updates { MIAS_update_none  =0,
                  MIAS_update_origin=1, 
                  MIAS_update_goal  =2,
                  MIAS_update_map   =4 };
   uint updates_since = 0;

   /* This value is maintained outside of the mapping
    * function for use in the visualization function */
   std::vector<node *> allocated_nodes;


public:

   MI_AStar(double collision_radius, int accept_oob_goal, 
            int backtrack_count, int node_limit,
            double turning_radius, double angle_slice, double position_slice, 
            double forward_step) : PathfinderBase(collision_radius,accept_oob_goal),
      backtrack_count(backtrack_count), node_limit(node_limit), turning_radius(turning_radius),
      angle_slice(angle_slice), position_slice(position_slice), forward_step(forward_step) {}

   void load_map(const nav_msgs::msg::OccupancyGrid & map, int occupant_cutoff) override {

      /* notify of update */
      updates_since |= MIAS_update_map;

      /* load the map in using default behavior */
      PathfinderBase::load_map(map,occupant_cutoff);

   }

   void set_origin(const geometry_msgs::msg::Pose & pose) override {

      /* notify of update */
      updates_since |= MIAS_update_origin;

      /* set the origin as usual */
      PathfinderBase::set_origin(pose);
   }

   void set_goal(const geometry_msgs::msg::Pose & pose) override {
      
      /* notify of update */
      updates_since |= MIAS_update_goal;

      /* set the goal as usual */
      PathfinderBase::set_goal(pose);
   }

   nav_msgs::msg::Path get_path() override {

      if (no_goal)
         return nav_msgs::msg::Path();

      /* declare our start and ending values 
       * Note that we are pathfinding from the
       * goal to the origin here.             */
      point goal_point = origin;
      node * curr,* head = NULL;


      /* prepare the storage structures */
      struct cmp_node cmp;
      std::priority_queue<node *,std::vector<node *>,decltype(cmp)> node_queue;
      std::unordered_set<long> seen_nodes;
      std::unordered_map<ulong,node *> best_at;

      /* if nothing has changed, republish the same path */
      if (updates_since == MIAS_update_none)
         goto transform_path;

      if (in_bounds(goal_point) && grid[point_pos(goal_point)])
         goto no_path;

      /* check if the goal has changed. If so, we are
       * forced to generate an entirely new path.    */
      if (updates_since & MIAS_update_goal) {

         /* lock the memory */
         mut.lock();

         /* free all the nodes on the current path */
         node * to_delete;
         while (last_path) {
            to_delete = last_path;
            last_path = last_path->next;
            delete to_delete;
         }

         /* release the memory */
         mut.unlock();

         /* go to the full recomputation */
         goto from_start;

      }

      /* check if the goal has changed. If so, try
       * and save some computation by only backtracking
       * to the point at which a connection was broken
       * by the map. This will produce a suboptimal path,
       * but it results in a hopefully much faster response */
      if (updates_since & MIAS_update_map) {

         /* lock the memory, we will possibly free some of it */
         mut.lock();

         node * last_safe = last_path;
         node * curr = last_path;
         node * to_delete;
         while (curr) {

            /* if curr is colliding with an obstacle, remove
             * all nodes in front of it in addition to itself */
            if (in_bounds(curr->pos) && grid[point_pos(curr->pos)]) {

               while (last_safe != curr) {
                  to_delete = last_safe;
                  last_safe = last_safe->next;
                  delete to_delete;
               }

               curr = curr->next;
               /* this is the old curr */
               delete last_safe;
               last_safe = curr;
            }
            else
               /* simply move to the next
                * value otherwise        */
               curr = curr->next;

         }

         /* update the last_path value to point
          * to unfreed memory                  */
         last_path = last_safe;

         /* release the memory */
         mut.unlock();

         /* we have lost our entire path.
          * This shouldn't happen but could */
         if (last_safe == NULL)
            goto from_start;
         else
            /* recompute from the last safe
             * value, retaining some of the
             * previously computed path.    */
            head = last_safe;
      }

      /* If the origin of the robot has changed,
       * go back a set number of nodes and restart
       * the search from there. This should
       * give *decent* results as it allows the path
       * to be strung out without losing it's shape */
      if (updates_since & MIAS_update_origin && last_path) {

         /* lock the memory, as we are deleting some of it */
         mut.lock();

         int to_cull = backtrack_count;

         /* clear out everything before the closest value */
         node * to_delete;
         while (last_path && to_cull--) {
            to_delete = last_path;
            last_path = last_path->next;
            delete to_delete;
         }

         /* release the memory */
         mut.unlock();

         /* set the head as the best value we could find */
         head = last_path;
      }

from_start:

      if (head == NULL) {
         /* set the start value as the goal */
         head = new node{};
         head->next = NULL;
         head->pos = goal;
      }

      /* clear the update flags now */
      updates_since = 0;

      /* set the head's weight */
      head->weight = point_dist2(head->pos,goal_point);

      /* seed the algorithm */
      best_at[as_long(head->pos)] = head;
      node_queue.push(head);

      /* perform the A* search 
       * There is a hard cap on nodes to avoid
       * nuking the environment memory wise */
      while (!node_queue.empty() && node_queue.size() < (ulong)node_limit) {

         /* collect the next node to process */
         curr = node_queue.top(); node_queue.pop();
         seen_nodes.erase(ptr_to_long(curr));

         /* check if the goal has been reached */
         if (point_pos(goal_point) == point_pos(curr->pos)) {

            last_path = curr;
            /* mark all values on the path */
            while (curr) {
               curr->weight = -1.0;
               curr = curr->next;
            }

            /* take control of memory - we are going
             * to replace the last_path now.        */
            mut.lock();

            /* cull all unmarked nodes */
            for (node * n : allocated_nodes) {
               if (n->weight >= 0.0)
                  delete n;
}
            allocated_nodes.clear();

            /* release memory */
            mut.unlock();

            /* move to transforming the path into
             * a nav_msgs/Path structure         */
            goto transform_path;
            
         }

         /* generate new motions */
         /* The maximun angle we can turn given
          * the turning radius and the forward step */
         double max_angle = (2.0 * asin((1.0 / turning_radius) * (forward_step / 2.0)));
         printf("max_angle: %f\tangle_slice: %f\n",max_angle,angle_slice);

         /* perform all four directions in a single loop */
         double angle = max_angle;
         /* the motions generated by these computations */
         std::vector<point> motions;
         while (angle > 0.0) {

            /* calculate the movement vectors for
             * the left and right directions */
            double left_angle = curr->pos.theta + angle;
            left_angle = normalize_angle(left_angle);
            double right_angle = curr->pos.theta + angle;
            right_angle = normalize_angle(right_angle);

            point left_move =  { forward_step * cos(left_angle ),
                                 forward_step * sin(left_angle ) };
            point right_move = { forward_step * cos(right_angle),
                                 forward_step * sin(right_angle) };
            /* left forward */
            motions.push_back(point{ curr->pos.x + left_move.x, curr->pos.y + left_move.y, left_angle });
            /* left backward */
            motions.push_back(point{ curr->pos.x - left_move.x, curr->pos.y - left_move.y, left_angle });
            /* right forward */
            motions.push_back(point{ curr->pos.x + right_move.x, curr->pos.y + right_move.y, right_angle });
            /* right backward */
            motions.push_back(point{ curr->pos.x - right_move.x, curr->pos.y - right_move.y, right_angle });

            /* decriment angle */
            angle -= angle_slice;
         }

         /* also do the forward and backward
          * movements.                       */
         point forward_move = point{ forward_step * cos(curr->pos.theta),
                                       forward_step * sin(curr->pos.theta) };
         motions.push_back(point{curr->pos.x + forward_move.x, curr->pos.y + forward_move.y, curr->pos.theta});
         motions.push_back(point{curr->pos.x - forward_move.x, curr->pos.y - forward_move.y, curr->pos.theta});
         
         /* check all values and add them if they are good */
         for (ulong i = 0; i < motions.size(); ++i) {

            /* ignore out of bounds possibilities */
            /*
            if (motions[i].x < 0 || motions[i].x >= (int)grid_metadata.width ||
                motions[i].y < 0 || motions[i].y >= (int)grid_metadata.height)
               continue;
               */

            /* if we are better than the previous value
             * at this position, replace it, otherwise
             * do not add this value. If there is no value
             * here and no obstacle, add the new_pos     */
            if (best_at.count(as_long(motions[i]))) {
               node * old = best_at.at(as_long(motions[i]));
               if (point_dist2(motions[i],goal_point) < old->weight) {

                  old->next = curr;
                  old->weight = point_dist2(motions[i],goal_point);

                  /* add this value to the queue if it is not already in it */
                  if (!seen_nodes.count(ptr_to_long(old))) {
                     node_queue.push(old);
                     seen_nodes.insert(ptr_to_long(old));
                  }
               }
            }
            else if (!in_bounds(motions[i]) || !grid[point_pos(motions[i])]) {
               node * new_node = new node{};
               new_node->next = curr;
               new_node->pos = motions[i];
               new_node->weight = point_dist2(motions[i],goal_point);

               /* add this option to the algorithm */
               seen_nodes.insert(ptr_to_long(new_node));
               node_queue.push(new_node);
               allocated_nodes.push_back(new_node);
               best_at[as_long(motions[i])] = new_node;
            }
         }
      }

no_path:

      /* no viable path could be found. 
       * Remove the old one, it is no longer
       * really useful either.             */
      mut.lock();
      node * to_delete;
      while (last_path) {
         to_delete = last_path;
         last_path = last_path->next;
         delete to_delete;
      }
      mut.unlock();


transform_path:

      /* if there is no path, you get an empty message */
      if (last_path == NULL)
         return nav_msgs::msg::Path();

      /* move through each node, converting it to ros2
       * coordinates and adding it to the path.       */

      nav_msgs::msg::Path ret;

      curr = last_path;
      while (curr) {
         geometry_msgs::msg::PoseStamped pose;

         pose.pose.position.x = ((double)curr->pos.x * grid_metadata.resolution) +
                                 grid_metadata.origin.position.x;
         pose.pose.position.y = ((double)curr->pos.y * grid_metadata.resolution) +
                                 grid_metadata.origin.position.y;

         ret.poses.push_back(pose);

         curr = curr->next;
      }

      return ret;

   }

   void draw_environment(sf::RenderWindow * window) override {
      PathfinderBase::draw_environment(window);

      sf::Vertex points[2];

      /* ensure we have control over the vars */
      mut.lock();

      /* draw enumerated movements */
      points[0].color = (sf::Color(255,255,0,255));
      points[1].color = (sf::Color(255,255,0,255));
      for (node * n : allocated_nodes) {
         visualize_motion(points,n->pos);
         window->draw(points,2,sf::Lines);
      }

      /* draw last path */
      node * curr = last_path;
      if (curr) {
         points[0].color = (sf::Color(0,255,255,255));
         points[1].color = (sf::Color(0,255,255,255));
         while (curr->next) {
            visualize_motion(points,curr->pos);
            window->draw(points,2,sf::Lines);
            curr = curr->next;
         }
      }

      /* release control */
      mut.unlock();

   }


};

#endif
