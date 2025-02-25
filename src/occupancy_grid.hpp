
#ifndef __OCCUPANCY_GRID

#define __OCCUPANCY_GRID
#include "kurome.h"
#include "lidar_data.hpp"
#include "point_cloud_data.hpp"
#include <unordered_set>
#include <cstring>

/* A class that will hopefully be useful for me. Allows creation
 * of a pose graph using multiple preregistered scans as well as
 * the addition of new scans to an existing pose graph. Values
 * are exposed in case I decide to use this class as an input
 * to a scan matcher.                                           */

class OccupancyGrid {
private:

   static constexpr point zero_ref = {0,0};

   int8_t unallocated_point;

   double resolution;
   int16_t positive_update;
   int16_t negative_update;

   int x_min = 0;
   int y_min = 0;

   int x_len = 0;
   int y_len = 0;

   int8_t * grid = NULL;

   inline int deref_location(const int x, const int y) const {
      return x + (y * x_len);
   }

public:

   OccupancyGrid(double resolution, int8_t positive_update, int8_t negative_update) 
      : resolution(resolution), positive_update(positive_update), negative_update(negative_update) {};

   /* convert the occupancy grid to a ros2 message, filling
    * the provided message variable's relavant fields.      */
   void to_msg(nav_msgs::msg::OccupancyGrid & msg) {

      msg.info.resolution = resolution;
      msg.info.width = x_len;
      msg.info.height = y_len;

      msg.info.origin.position.x = (double)x_min * resolution;
      msg.info.origin.position.y = (double)y_min * resolution;
      msg.info.origin.position.z = 0.0;
      
      msg.info.origin.orientation.x = 0.0;
      msg.info.origin.orientation.y = 0.0;
      msg.info.origin.orientation.z = 0.0;
      msg.info.origin.orientation.w = 1.0;

      for (int i = 0; i < x_len*y_len; ++i)
         msg.data.push_back(grid[i]);
   }

   /* get the value of one of the points on the grid using indicies in meters */
   int8_t & operator()(const double x_index, const double y_index) {
      int canonical_x_index = (x_index / resolution) - x_min;
      int canonical_y_index = (y_index / resolution) - y_min;

      if (canonical_x_index < 0 || canonical_x_index >= x_len ||
          canonical_y_index < 0 || canonical_y_index >= y_len) {

         unallocated_point = 0;
         return unallocated_point;
      }

      return grid[deref_location(canonical_x_index,canonical_y_index)];
   }

   /* get the value of one of the points on the grid using integer indicies
    * based on the resolution of the grid                                  */
   int8_t & operator()(int x_index, int y_index) {
      x_index -= x_min;
      y_index -= y_min;

      if (x_index < 0 || x_index >= x_len ||
          y_index < 0 || y_index >= y_len) {

         unallocated_point = 0;
         return unallocated_point;
      }

      return grid[deref_location(x_index,y_index)];
   }

   /*
    * Based on:
    * https://en.wikipedia.org/wiki/Bresenham's_line_algorithm
    */

   /* insert a lidar scan into this occupancy grid */
   void add_scan(const LidarData & scan, const pose_2d & pose){

      printf("position: %f %f %f\n",pose.pos.x,pose.pos.y,pose.theta);

      std::unordered_set<int> seen;


      int x1 = (pose.pos.x / resolution) - x_min;
      int y1 = (pose.pos.y / resolution) - y_min;


      std::vector<std::pair<int,int>> locations;

      int new_x_min = 0;
      int new_y_min = 0;
      int new_x_len = x_len;
      int new_y_len = y_len;


      bool size_change = false;

      for (const point & p : scan.points) {

         point moved;
         if (memcmp(&zero_ref,&p,sizeof(point)) != 0) 
            moved = point{ p.x * cos(pose.theta) - p.y * sin(pose.theta) + pose.pos.x,
                           p.x * sin(pose.theta) + p.y * cos(pose.theta) + pose.pos.y };
         else
            moved = p;

         int x = (moved.x / resolution) - x_min;

         if (x < new_x_min) {
            new_x_min = x;
            size_change = true;
         }
         else if (x >= new_x_len) {
            new_x_len = x + 1;
            size_change = true;
         }

         int y = (moved.y / resolution) - y_min;

         if (y < new_y_min) {
            new_y_min = y;
            size_change = true;
         }
         else if (y >= new_y_len) {
            new_y_len = y + 1;
            size_change = true;
         }

         locations.push_back(std::make_pair(x + x_min,y + y_min));
      }

      if (size_change) {


         int x_change =  - new_x_min;
         int y_change =  - new_y_min;

         new_x_len += x_change;
         new_y_len += y_change;

         new_x_min = x_min + new_x_min;
         new_y_min = y_min + new_y_min;

         int8_t * new_grid = (int8_t *)calloc(sizeof(int8_t),new_x_len*new_y_len);

         for (int x = 0; x < x_len; ++x) {
            for (int y = 0; y < y_len; ++y) {
               new_grid[ x + x_change + ((y + y_change) * new_x_len)] = grid[x + y * x_len];
            }
         }
         /*
         int new_pos = (x_min - new_x_min) + (new_x_len * (y_min - new_y_min));
         for (int old_pos = 0; old_pos < x_len*y_len; ++old_pos) {

            new_grid[new_pos] = grid[old_pos];

            if (old_pos % x_len == x_len - 1)
               new_pos += (new_x_len - x_len);
            else 
               new_pos += 1;

         }
         */

         free(grid);
         grid = new_grid;


         x_len = new_x_len;
         y_len = new_y_len;
         x_min = new_x_min;
         y_min = new_y_min;
      }

      for (std::pair<int,int> & p : locations) {

         int x0 = std::get<0>(p);
         int y0 = std::get<1>(p);


         int dx = abs(x1 - x0);
         int sx = x0 < x1 ? 1 : -1;
         int dy = -abs(y1 - y0);
         int sy = y0 < y1 ? 1 : -1;
         int error = dx + dy;


         /* update the obstacle position as occupied */
         {
            int target = (*this)(x0,y0);

            target += positive_update;
            target = (target > 100 ? 100 : target);

            (*this)(x0,y0) = target;
            seen.insert(deref_location(x0,y0));
         }


         /* update the points in the obstacle's line 
          * of sight as free space.                 */              
         while (true) {

            int loc = deref_location(x0,y0);
            if (!seen.count(loc)) {
               int target = (*this)(x0,y0);

               target += negative_update;
               target = (target < 0 ? 0 : target);

               (*this)(x0,y0) = target;
               seen.insert(loc);
            }

            if (x0 == x1 && y0 == y1)
               break;

            int e2 = 2 * error;

            if (e2 >= dy) {
               error = error + dy;
               x0 = x0 + sx;
            }

            if (e2 <= dx) {
               error = error + dx;
               y0 = y0 + sy;
            }
         }
      }
   }

   /* insert a point cloud into this occupancy grid, cutting out any values 
    * that appear far enough away from the groud of the robot (0,0).       */
   void add_point_cloud(const PointCloudData & points, const pose_2d & pose, double ground_dist_cutoff = 0.1) {
      (void)points;
      (void)pose;
      (void)ground_dist_cutoff;
   }

};

#endif
