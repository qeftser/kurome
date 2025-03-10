
#ifndef __PATHFINDER_BASE

#define __PATHFINDER_BASE
#include "kurome.h"
#include <cstddef>
#include <mutex>
#include <cmath>

#include <SFML/Graphics.hpp>

class PathfinderBase {
protected:

   /* Represents a position
    * on our grid.         */
   struct point {
      double x;
      double y;
      double theta = 0.0;
   };

   /* Represents a slot on
    * the grid.            */
   struct block {
      int x;
      int y;
   };

   /* A vector that represents a grid of points
    * in 2d space. We are using uint8_t values 
    * because we will exploit thier ability to
    * hold numbers higher than one in our grid
    * generation / update algorithm.          */
   uint8_t * grid = NULL;
   /* markers for whether or not a value is set */
   uint8_t * grid_flag = NULL;
   /* length of the vector */
   int grid_length = 0;

   /* Information about the grid that is 
    * useful to have.                   */
   nav_msgs::msg::MapMetaData grid_metadata;

   /* info for indexing into the local map */
   struct {
      int x_off;
      int y_off;
      int x_len;
      int y_len;
   } grid_bound;

   /* where the root node is located */
   point origin = { 0.0, 0.0 };
   /* where we are trying to go */
   point goal = { 0.0, 0.0 };
   /* true if there is no valid goal */
   int no_goal = true;

   /* the points on the radius of the circle
    * that represents the boundary of collision
    * around obstacles.                        */
   std::vector<block> circle_vals;

   /* The minimum distance the robot should
    * keep all times from the nearest obstacle */
   double collision_radius;

   /* Whether goals that lie out of bounds should 
    * be accepted by the system.                 */
   int accept_out_of_bounds_goal;

   /* return the distance sqaured between two blocks */
   int block_dist2(const block & b1, const block &  b2) {
      return (b1.x-b2.x)*(b1.x-b2.x) + (b1.y-b2.y)*(b1.y-b2.y);
   }

   /* return the position of a block in
    * the grid or best_at vectors.    */
   int block_pos(const block & b) {
      int off_x = b.x - grid_bound.x_off;
      int off_y = b.y - grid_bound.y_off;
      return off_x + (off_y * grid_bound.x_len);
   }

   /* returns 1 if the given index is
    * in the bounds of the map, 0 otherwise */
   int in_bounds(const block & b) {
      int x = b.x - grid_bound.x_off;
      int y = b.y - grid_bound.y_off;
      if (x < 0 || y < 0 ||
          x >= grid_bound.x_len ||
          y >= grid_bound.y_len)
         return 0;
      return 1;
   }
   int in_bounds(const point & b) {
      int x = (int)b.x - grid_bound.x_off;
      int y = (int)b.y - grid_bound.y_off;
      if (x < 0 || y < 0 ||
          x >= grid_bound.x_len ||
          y >= grid_bound.y_len)
         return 0;
      return 1;
   }

private:

   /* mutex for ensuring that we don't draw the
    * environment while we are deleting it.    */
   std::mutex mut;

   /* compute the points on the radius of the collision
    * boundary circle. This will be used in the map
    * loading stage. This is the midpoint algorithm with
    * 0,0 as the center and collision_radius / grid_size
    * as the radius.                                    */
   void compute_circle(int radius) {
      circle_vals.clear();

      int radius_sqr = radius * radius;

      for (int x = -radius; x < radius; ++x) {
         int hh = (int)sqrt(radius_sqr - x*x);
         int rx = x;
         int ph = hh;

         for (int y = -hh; y < ph; ++y)
            circle_vals.push_back({rx,y});

      }

//      int t1, t2, x, y = 0;
//      x = radius;
//      t1 = radius / 16;
//
//      while (x >= y) {
//            /* we are computing the circle on a vector
//             * so we multiply by x length to get the
//             * y position.                            */
//            circle_vals.push_back({x,y});
//            /* fill out all 8 circle values */
//            circle_vals.push_back({y,x});
//            circle_vals.push_back({-x,y});
//            circle_vals.push_back({-y,x});
//            circle_vals.push_back({x,-y});
//            circle_vals.push_back({y,-x});
//            circle_vals.push_back({-x,-y});
//            circle_vals.push_back({-y,-x});
//
//         /* update values and compute error */
//         t1 = t1 + y + 1;
//         t2 = t1 - x;
//         if (t2 >= 0) {
//            t1 = t2;
//            x = x - 1;
//
//            /* we also need to add more circle values
//             * here, which is a slight deviation from
//             * the normal algorithm, but nessesary to
//             * avoid clipping through walls during the
//             * pathfinding procedures                 */
//            circle_vals.push_back({x,y});
//            circle_vals.push_back({y,x});
//            circle_vals.push_back({-x,y});
//            circle_vals.push_back({-y,x});
//            circle_vals.push_back({x,-y});
//            circle_vals.push_back({y,-x});
//            circle_vals.push_back({-x,-y});
//            circle_vals.push_back({-y,-x});
//         }
//         y = y + 1;
//      }

   }

public:

   PathfinderBase(double collision_radius, int accept_out_of_bounds_goal) 
      : collision_radius(collision_radius), 
        accept_out_of_bounds_goal(accept_out_of_bounds_goal) {

   }

   /* A rather complicated function. Collect the current occupancy
    * grid. If it is the same size, we will simply update based on
    * changes to the previous map. If it a different size, recompute
    * the map entirely. Optionally accept a function to be called
    * when a grid cell is newly set as an obstacle.                */
   virtual void load_map(const nav_msgs::msg::OccupancyGrid & map, int occupant_cutoff) {
      load_map(map,occupant_cutoff,nullptr);
   }
   template <typename T>
   void load_map(const nav_msgs::msg::OccupancyGrid & map, int occupant_cutoff, 
                 const T * notify_obstacle) {
      /* if we are given a 'new' map, not an update of
       * an existing one, we need to update the metadata
       * of the map and recompute the vector that represents it. */
      if (map.info.resolution != grid_metadata.resolution ||
          map.info.width != grid_metadata.width           ||
          map.info.height != grid_metadata.height           ) {

         /* ensure we have full hold of the map */
         mut.lock();

         /* get rid of the old vector */
         free(grid);
         free(grid_flag);

         /* reset the nessesary values */
         grid_metadata = map.info;
         grid_bound.x_len = grid_metadata.width;
         grid_bound.y_len = grid_metadata.height;
         grid_bound.x_off = grid_metadata.origin.position.x / grid_metadata.resolution;
         grid_bound.y_off = grid_metadata.origin.position.y / grid_metadata.resolution;
         compute_circle(collision_radius / grid_metadata.resolution);


         /* allocate our new grid vector */
         grid_length = grid_metadata.width * grid_metadata.height;
         grid = (uint8_t *)(calloc(sizeof(uint8_t),grid_length));
         grid_flag = (uint8_t *)(calloc(sizeof(uint8_t),grid_length));

         /* release the mutex as soon as possible */
         mut.unlock();

         /* fill our vector with the current values */
         for (int i = 0; i < grid_length; ++i) {
            /* check if the value in a map cell is large
             * enough to be considered an obstacle.     */
            if (map.data[i] > occupant_cutoff) {
               /* add a circle if this point is an
                * obstacle                        */
               for (size_t j = 0; j < circle_vals.size(); ++j) {
                  block pos = {circle_vals[j].x + (int)(i%grid_metadata.width), 
                               circle_vals[j].y + (int)(i/grid_metadata.width) };
                  if (pos.x >= 0 && pos.y >= 0 && pos.x < (int)grid_metadata.width && 
                      pos.y < (int)grid_metadata.height) {

                     /* this is done to ensure that we can
                      * remove circles with overlap later. */
                     grid[pos.x + pos.y*grid_metadata.width] += 1;
                     if (grid[pos.x + pos.y*grid_metadata.width] == 1)
                        /* call our notify_obstacle function if the obstacle is new */
                        (*notify_obstacle)(pos.x,pos.y);
                  }
               }
               /* mark the position as having a circle */
               grid_flag[i] = 1;
            }
         }
      }
      /* Our overall map structure is still correct, we
       * do need to update the whole thing, just the
       * nodes that have changed.                      */
      else {
         for (int i = 0; i < grid_length; ++i) {
            /* Record if this is an obstacle and whether it 
             * is already mapped.                          */
            switch(grid_flag[i] | (map.data[i] > occupant_cutoff ? 2 : 0)) {
               /* There is no obstacle and this space is 
                * emoty, this results in no change.     */
               case 0:
               break;
               /* There is no obstacle, but this space
                * is occupied, remove the circle.     */
               case 1:
                  /* unmark the position */
                  grid_flag[i] = 0;
                  /* remove the circle */
                  for (size_t j = 0; j < circle_vals.size(); ++j) {
                     block pos = {circle_vals[j].x + (int)(i%grid_metadata.width), 
                                  circle_vals[j].y + (int)(i/grid_metadata.width) };
                     if (pos.x >= 0 && pos.y >= 0 && pos.x < (int)grid_metadata.width &&
                         pos.y < (int)grid_metadata.height) {

                        /* this should always result in a value
                         * greater than or equal to zero.      */
                        grid[pos.x + pos.y*grid_metadata.width] -= 1;
                     }
                  }
               break;
               /* there is an obstacle but no circle,
                * so draw one here.                  */
               case 2:
                  /* mark the position */
                  grid_flag[i] = 1;
                  /* add the circle */
                  for (size_t j = 0; j < circle_vals.size(); ++j) {
                     block pos = {circle_vals[j].x + (int)(i%grid_metadata.width), 
                                  circle_vals[j].y + (int)(i/grid_metadata.width) };
                     if (pos.x >= 0 && pos.y >= 0 && pos.x < (int)grid_metadata.width &&
                         pos.y < (int)grid_metadata.height) {

                        grid[pos.x + pos.y*grid_metadata.width] += 1;
                        if (grid[pos.x + pos.y*grid_metadata.width] == 1)
                           /* call our notify_obstacle function if the obstacle is new */
                           (*notify_obstacle)(pos.x,pos.y);
                     }
                  }
               break;
               /* There is an obstacle and the position
                * is marked, so take no action.        */
               case 3:
               break;
            }
         }
      }
   }

   /* clone of the previous function with the nullptr */
   void load_map(const nav_msgs::msg::OccupancyGrid & map, int occupant_cutoff, std::nullptr_t) {
      if (map.info.resolution != grid_metadata.resolution ||
          map.info.width != grid_metadata.width           ||
          map.info.height != grid_metadata.height           ) {

         mut.lock();

         free(grid);
         free(grid_flag);

         grid_metadata = map.info;
         grid_bound.x_len = grid_metadata.width;
         grid_bound.y_len = grid_metadata.height;
         grid_bound.x_off = grid_metadata.origin.position.x / grid_metadata.resolution;
         grid_bound.y_off = grid_metadata.origin.position.y / grid_metadata.resolution;
         compute_circle(collision_radius / grid_metadata.resolution);

         grid_length = grid_metadata.width * grid_metadata.height;
         grid = (uint8_t *)(calloc(sizeof(uint8_t),grid_length));
         grid_flag = (uint8_t *)(calloc(sizeof(uint8_t),grid_length));

         mut.unlock();

         for (int i = 0; i < grid_length; ++i) {
            if (map.data[i] > occupant_cutoff) {
               for (size_t j = 0; j < circle_vals.size(); ++j) {
                  block pos = {circle_vals[j].x + (int)(i%grid_metadata.width), 
                               circle_vals[j].y + (int)(i/grid_metadata.width) };
                  if (pos.x >= 0 && pos.y >= 0 && pos.x < (int)grid_metadata.width && 
                      pos.y < (int)grid_metadata.height) {

                     grid[pos.x + pos.y*grid_metadata.width] += 1;
                  }
               }
               grid_flag[i] = 1;
            }
         }
      }
      else {
         for (int i = 0; i < grid_length; ++i) {
            switch(grid_flag[i] | (map.data[i] > occupant_cutoff ? 2 : 0)) {
               case 0:
               break;
               case 1:
                  grid_flag[i] = 0;
                  for (size_t j = 0; j < circle_vals.size(); ++j) {
                     block pos = {circle_vals[j].x + (int)(i%grid_metadata.width), 
                                  circle_vals[j].y + (int)(i/grid_metadata.width) };
                     if (pos.x >= 0 && pos.y >= 0 && pos.x < (int)grid_metadata.width &&
                         pos.y < (int)grid_metadata.height) {
                        grid[pos.x + pos.y*grid_metadata.width] -= 1;
                     }
                  }
               break;
               case 2:
                  grid_flag[i] = 1;
                  for (size_t j = 0; j < circle_vals.size(); ++j) {
                     block pos = {circle_vals[j].x + (int)(i%grid_metadata.width), 
                                  circle_vals[j].y + (int)(i/grid_metadata.width) };
                     if (pos.x >= 0 && pos.y >= 0 && pos.x < (int)grid_metadata.width &&
                         pos.y < (int)grid_metadata.height) {

                        grid[pos.x + pos.y*grid_metadata.width] += 1;
                     }
                  }
               break;
               case 3:
               break;
            }
         }
      }
   }

   /* helper method...
    * produce the full map as a OccupancyGrid message. 
    * Note that this does not fill the header or time slots */
   nav_msgs::msg::OccupancyGrid collect_map() {
      nav_msgs::msg::OccupancyGrid ret;
      ret.info = grid_metadata;
      for (int i = 0; i < grid_length; ++i)
         ret.data.push_back((grid[i] ? 100 : 0));
      return ret;
   }

   /* Set the new origin for our pathfinder */
   virtual void set_origin(const geometry_msgs::msg::Pose & pose) {
      origin.x = (pose.position.x) / grid_metadata.resolution;
      origin.y = (pose.position.y) / grid_metadata.resolution;
   }

   /* Set the new goal for our pathfinder */
   virtual void set_goal(const geometry_msgs::msg::Pose & pose) {
      goal.x = (pose.position.x) / grid_metadata.resolution;
      goal.y = (pose.position.y) / grid_metadata.resolution;

      /* if we are out of bounds or in a colliding area, do not
       * set it as a valid goal for the system.                */
      if ((!accept_out_of_bounds_goal && 
           in_bounds(goal)) ||
         (grid && grid[(int)(goal.x + (goal.y * grid_metadata.width))])) {
         no_goal = true;
      }
      else /* goal is valid */
         no_goal = false;
 
   }

   /* collect the current best path
    * from our pathfinder           */
   virtual nav_msgs::msg::Path get_path() = 0;

   /* draw the environment and current pathing data to
    * a provided window. Using this over rviz2 as it
    * will be faster.                                 */
   virtual void draw_environment(sf::RenderWindow * window) {
      sf::RectangleShape rect;

      /* grab full control of the grid */
      mut.lock();

      double x = grid_bound.x_off * 10.0;
      double y = grid_bound.y_off * 10.0;

      /* draw border */
      rect.setSize(sf::Vector2f(10*grid_metadata.width,10*grid_metadata.height));
      rect.setPosition((x),
                       (y));
      rect.setFillColor(sf::Color(0,0,0,0));
      rect.setOutlineColor(sf::Color(255,255,255,255));
      rect.setOutlineThickness(3);
      window->draw(rect);

      /* draw obstacles */
      rect.setSize(sf::Vector2f(10,10));
      rect.setFillColor(sf::Color(255,255,255,255));
      for (int i = 0; i < grid_length; ++i) {
         if (grid[i]) {
            rect.setPosition(x+((i%grid_metadata.width)*10),y+((i/grid_metadata.width)*10));
            window->draw(rect);
         }
      }

      /* release control */
      mut.unlock();
   }

};

#endif
