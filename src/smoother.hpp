
#ifndef __SMOOTHER_BASE

#define __SMOOTHER_BASE

#include "kurome.h"
#include "spatial_bin.hpp"
#include <cstddef>
#include <mutex>

#include <SFML/Graphics.hpp>

class SmootherBase {
protected:

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
   /* is the current path valid? */
   int valid_path = 0;

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

   /* the points on the radius of the circle
    * that represents the boundary of collision
    * around obstacles.                        */
   std::vector<block> circle_vals;

   /* The minimum distance the robot should
    * keep all times from the nearest obstacle */
   double collision_radius;

   int block_pos(int x_pos, int y_pos) {
      int x = x_pos - grid_bound.x_off;
      int y = y_pos - grid_bound.y_off;
      return x + (y * grid_bound.x_len);
   }

   int in_bounds(int x_pos, int y_pos) {
      int x = x_pos - grid_bound.x_off;
      int y = y_pos - grid_bound.y_off;
      if (x < 0 || x >= grid_bound.x_len ||
          y < 0 || y >= grid_bound.y_len)
         return 0;
      return 1;
   }


private:

   /* needed for managing the shared memory */
   std::mutex mut;

   /* compute the points on the radius of the collision
    * boundary circle. This will be used in the map
    * loading stage. This is the midpoint algorithm with
    * 0,0 as the center and collision_radius / grid_size
    * as the radius.                                    */
   void compute_circle(int radius) {
      circle_vals.clear();

      int t1, t2, x, y = 0;
      x = radius;
      t1 = radius / 16;

      while (x >= y) {
            /* we are computing the circle on a vector
             * so we multiply by x length to get the
             * y position.                            */
            circle_vals.push_back({x,y});
            /* fill out all 8 circle values */
            circle_vals.push_back({y,x});
            circle_vals.push_back({-x,y});
            circle_vals.push_back({-y,x});
            circle_vals.push_back({x,-y});
            circle_vals.push_back({y,-x});
            circle_vals.push_back({-x,-y});
            circle_vals.push_back({-y,-x});

         /* update values and compute error */
         t1 = t1 + y + 1;
         t2 = t1 - x;
         if (t2 >= 0) {
            t1 = t2;
            x = x - 1;

            /* we also need to add more circle values
             * here, which is a slight deviation from
             * the normal algorithm, but nessesary to
             * avoid clipping through walls during the
             * pathfinding procedures                 */
            circle_vals.push_back({x,y});
            circle_vals.push_back({y,x});
            circle_vals.push_back({-x,y});
            circle_vals.push_back({-y,x});
            circle_vals.push_back({x,-y});
            circle_vals.push_back({y,-x});
            circle_vals.push_back({-x,-y});
            circle_vals.push_back({-y,-x});
         }
         y = y + 1;
      }

   }

public:

   SmootherBase(double collision_radius)
      : collision_radius(collision_radius) {}

   virtual bool propose_path(const nav_msgs::msg::Path & msg) = 0;

   virtual void advance_path(const geometry_msgs::msg::PoseStamped & msg) = 0;

   virtual void simulate_path(pose_2d curr_pose, geometry_msgs::msg::PoseArray & ret_msg) = 0;

   virtual geometry_msgs::msg::Twist get_vel() = 0;

   virtual bool is_path_valid() {
      return valid_path;
   }

   virtual void invalidate_path() {
      valid_path = false;
   }

//   virtual void load_map(const nav_msgs::msg::OccupancyGrid & map, int occupant_cutoff) {
//
//      /* if we are given a 'new' map, not an update of
//       * an existing one, we need to update the metadata
//       * of the map and recompute the vector that represents it. */
//      if (map.info.resolution != grid_metadata.resolution ||
//          map.info.width != grid_metadata.width           ||
//          map.info.height != grid_metadata.height           ) {
//
//         /* set the new and updated metadata */
//         grid_metadata = map.info;
//
//         /* get rid of the old vector */
//         free(grid);
//
//         /* allocate a new vector */
//         grid_length = grid_metadata.width * grid_metadata.height;
//         grid = (uint8_t *)(calloc(sizeof(uint8_t),grid_length));
//
//         /* fill the vector accordingly */
//         for (int i = 0; i < grid_length; ++i)
//            grid[i] = (map.data[i] >= occupant_cutoff ? 1 : 0);
//      }
//      /* The map is the same size, simply fill it with the new values */
//      else {
//         for (int i = 0; i < grid_length; ++i)
//            grid[i] = (map.data[i] >= occupant_cutoff ? 1 : 0);
//      }
//   }

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
   virtual void draw_environment(sf::RenderWindow * window) {
      sf::RectangleShape rect;

      /* take control of the grid */
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

      /* release control of the grid */
      mut.unlock();

   }

   virtual visualization_msgs::msg::MarkerArray construct_visualization() {
      return visualization_msgs::msg::MarkerArray();
   }

};

#endif
