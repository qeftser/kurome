
#ifndef __A_STAR

#define __A_STAR

#include "kurome.h"
#include "pathfinder.hpp"
#include <cstddef>
#include <cmath>
#include <cstring>
#include <thread>
#include <mutex>
#include <queue>
#include <unordered_set>
#include <unordered_map>
#include <vector>
#include <cassert>

class AStar : public PathfinderBase {
private:

   struct node {
      block pos;
      node * next;
      long weight;
      long dist;
   };

   static const int step_cost = 3.0;

   static inline long cost(const node * n) {
      return n->dist + n->weight;
   }

   /* compare two nodes based on their weights. Used as 
    * the priority queue comparison operator.          */
   struct cmp_node {
      bool operator()(const node * n1, const node * n2) const {
         if (cost(n1) > cost(n2))
            return true;
         return false;
      }
   };

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
   enum updates { AS_update_none  =0,
                  AS_update_origin=1, 
                  AS_update_goal  =2,
                  AS_update_map   =4 };
   uint updates_since = 0;

   /* How many nodes to recompute when the positon
    * of the robot updates. Setting this to a negative
    * value will result in no computation limit.      */
   int backtrack_count;

   /* Maximum size the queue can grow to during the A*
    * search. Prevents system lockup due to memory 
    * running out.                                    */
   int node_limit;

   /* Record of the best values at each map position */
   std::unordered_map<long,node *> best_at;

public:

   AStar(double collision_radius, int accept_oob_goal, int backtrack_count, int node_limit)
      : PathfinderBase(collision_radius,accept_oob_goal), 
        backtrack_count(backtrack_count), node_limit(node_limit) {}


   void load_map(const nav_msgs::msg::OccupancyGrid & map, int occupant_cutoff) override {

      /* notify of update */
      updates_since |= AS_update_map;

      /* load the map in using default behavior */
      PathfinderBase::load_map(map,occupant_cutoff);

   }

   void set_origin(const geometry_msgs::msg::Pose & pose) override {

      /* notify of update */
      updates_since |= AS_update_origin;

      /* set the origin as usual */
      PathfinderBase::set_origin(pose);
   }

   void set_goal(const geometry_msgs::msg::Pose & pose) override {

      /* notify of update */
      updates_since |= AS_update_goal;

      /* set the goal as usual */
      PathfinderBase::set_goal(pose);
   }

   nav_msgs::msg::Path get_path() override {

      bool found_open = false;

      if (no_goal)
         return nav_msgs::msg::Path();

      /* declare our start and ending values 
       * Note that we are pathfinding from the
       * goal to the origin here.             */
      block goal_block = { (int)floor(origin.x), (int)floor(origin.y) };
      node * curr,* head = NULL;

      /* prepare the storage structures */
      struct cmp_node cmp;
      std::priority_queue<node *,std::vector<node *>,decltype(cmp)> node_queue;
      std::vector<node *> allocated_nodes;
      std::unordered_set<long> seen_nodes;

      /* if nothing has changed, republish the same path */
      if (updates_since == AS_update_none)
         goto transform_path;

      if (grid_length && in_bounds(goal_block) && grid[block_pos(goal_block)])
         goto no_path;

      /* check if the goal has changed. If so, we are
       * forced to generate an entirely new path.    */
      if (updates_since & AS_update_goal) {

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
      if (updates_since & AS_update_map) {

         /* lock the memory, we will possibly free some of it */
         mut.lock();

         node * last_safe = last_path;
         node * curr = last_path;
         node * to_delete;
         while (curr) {

            /* if curr is colliding with an obstacle, remove
             * all nodes in front of it in addition to itself */
            if (in_bounds(curr->pos) && grid[block_pos(curr->pos)]) {

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
      if (updates_since & AS_update_origin && last_path) {

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
         head->pos = { (int)floor(goal.x), (int)floor(goal.y) };
      }


      /* clear out old values from the best_at vector */
      mut.lock();
      best_at.clear();
      mut.unlock();

      /* clear the update flags now */
      updates_since = 0;

      /* set the head's weight */
      head->weight = 0.0;
      head->dist = sqrt(block_dist2(head->pos,goal_block));

      /* seed the algorithm */
      best_at[long_from_ints(head->pos.x,head->pos.y)] = head;
      node_queue.push(head);


      /* perform the A* search 
       * There is a hard cap on nodes to avoid
       * nuking the environment memory wise */
      while (!node_queue.empty() && node_queue.size() < (ulong)node_limit) {

         /* collect the next node to process */
         curr = node_queue.top(); node_queue.pop();
         seen_nodes.erase(ptr_to_long(curr));

         /* check if the goal has been reached */
         if (goal_block.x == curr->pos.x && goal_block.y == curr->pos.y) {

            /* take control of memory - we are going
             * to replace the last_path now.        */
            mut.lock();

            last_path = curr;
            /* mark all values on the path */
            while (curr) {
               curr->weight = -1.0;
               curr = curr->next;
            }

            /* cull all unmarked nodes */
            for (node * n : allocated_nodes) {
               if (n->weight >= 0.0)
                  delete n;
            }

            /* release memory */
            mut.unlock();

            /* move to transforming the path into
             * a nav_msgs/Path structure         */
            goto transform_path;
            
         }

         /* generate new motions */
         for (int x = -1; x <= 1; ++x) {
            for (int y = -1; y <= 1; ++y) {

               if (!x && !y)
                  continue;


               block new_pos = block{curr->pos.x + x,
                                     curr->pos.y + y};

               int local_step = ((double)step_cost * (x && y ? 1.4 : 1.0));

               /* if we are better than the previous value
                * at this position, replace it, otherwise
                * do not add this value. If there is no value
                * here and no obstacle, add the new_pos     */
               node * old;
               old = best_at.count(long_from_ints(new_pos.x,new_pos.y)) ? 
                     best_at[long_from_ints(new_pos.x,new_pos.y)] : NULL;
               if (old) {
                   if (sqrt(block_dist2(new_pos,goal_block)) + curr->weight + local_step < cost(old)) {

                     old->next = curr;
                     old->dist = sqrt(block_dist2(new_pos,goal_block));
                     old->weight = curr->weight + local_step;
                     //old->weight = curr->weight + old->dist;

                     /* add this value to the queue if it is not already in it */
                     if (!seen_nodes.count(ptr_to_long(old))) {
                        node_queue.push(old);
                        seen_nodes.insert(ptr_to_long(old));
                     }
                  }
               }
               else if (!in_bounds(new_pos) || (!grid[block_pos(new_pos)] || !found_open)) {
                  node * new_node = new node{};
                  new_node->next = curr;
                  new_node->pos = new_pos;
                  new_node->dist = sqrt(block_dist2(new_pos,goal_block));
                  new_node->weight = curr->weight + local_step;
                  //new_node->weight = curr->weight + new_node->dist;

                  /* add this option to the algorithm */
                  seen_nodes.insert(ptr_to_long(new_node));
                  node_queue.push(new_node);
                  allocated_nodes.push_back(new_node);
                  best_at[long_from_ints(new_pos.x,new_pos.y)] = new_node;
               }

               if (!found_open && in_bounds(new_pos) && !grid[block_pos(new_pos)])
                  found_open = true;
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

         pose.pose.position.x = ((double)curr->pos.x * grid_metadata.resolution);
         pose.pose.position.y = ((double)curr->pos.y * grid_metadata.resolution);

         ret.poses.push_back(pose);

         curr = curr->next;
      }

      return ret;

   }

   void draw_environment(sf::RenderWindow * window) override {
      PathfinderBase::draw_environment(window);

      sf::RectangleShape rect;
      sf::Vertex points[2];

      /* ensure we have control over the vars */
      mut.lock();

      /* draw visited blocks */
      rect.setSize(sf::Vector2f(10,10));
      rect.setOutlineColor(sf::Color(255,255,0,255));
      rect.setFillColor(sf::Color(0,0,0,0));
      rect.setOutlineThickness(2);
      int x, y;
      for (auto & val : best_at) {
         ints_from_long(std::get<0>(val),x,y);
         rect.setPosition(x*10,y*10);
         window->draw(rect);
      }

      /* draw last path */
      node * curr = last_path;
      if (curr) {
         points[0].color = (sf::Color(0,255,255,255));
         points[1].color = (sf::Color(0,255,255,255));
         while (curr->next) {
            points[0].position = sf::Vector2f(curr->pos.x*10,curr->pos.y*10);
            points[1].position = sf::Vector2f(curr->next->pos.x*10,curr->next->pos.y*10);
            window->draw(points,2,sf::Lines);
            curr = curr->next;
         }
      }

      /* release control */
      mut.unlock();

   }

};

#endif

