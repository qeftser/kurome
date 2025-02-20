
#ifndef __SMOOTHER_BASE

#define __SMOOTHER_BASE

#include "kurome.h"
#include "spatial_bin.hpp"
#include <cstddef>
#include <mutex>

#include <SFML/Graphics.hpp>

class SmootherBase {
protected:

   /* A vector that represents a grid of points
    * in 2d space. We are using uint8_t values 
    * because we will exploit thier ability to
    * hold numbers higher than one in our grid
    * generation / update algorithm.          */
   uint8_t * grid = NULL;
   /* length of the vector */
   int grid_length = 0;
   /* is the current path valid? */
   int valid_path = 0;

   /* Information about the grid that is 
    * useful to have.                   */
   nav_msgs::msg::MapMetaData grid_metadata;

private:

   /* needed for managing the shared memory */
   std::mutex mut;

public:

   SmootherBase() {}

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

   virtual void load_map(const nav_msgs::msg::OccupancyGrid & map, int occupant_cutoff) {

      /* if we are given a 'new' map, not an update of
       * an existing one, we need to update the metadata
       * of the map and recompute the vector that represents it. */
      if (map.info.resolution != grid_metadata.resolution ||
          map.info.width != grid_metadata.width           ||
          map.info.height != grid_metadata.height           ) {

         /* set the new and updated metadata */
         grid_metadata = map.info;

         /* get rid of the old vector */
         free(grid);

         /* allocate a new vector */
         grid_length = grid_metadata.width * grid_metadata.height;
         grid = (uint8_t *)(calloc(sizeof(uint8_t),grid_length));

         /* fill the vector accordingly */
         for (int i = 0; i < grid_length; ++i)
            grid[i] = (map.data[i] >= occupant_cutoff ? 1 : 0);
      }
      /* The map is the same size, simply fill it with the new values */
      else {
         for (int i = 0; i < grid_length; ++i)
            grid[i] = (map.data[i] >= occupant_cutoff ? 1 : 0);
      }
   }

   virtual void draw_environment(sf::RenderWindow * window) {
      sf::RectangleShape rect;

      /* take control of the grid */
      mut.lock();

      /* draw border */
      rect.setSize(sf::Vector2f(10*grid_metadata.width,10*grid_metadata.height));
      rect.setPosition((0.0),
                       (0.0));
      rect.setFillColor(sf::Color(0,0,0,0));
      rect.setOutlineColor(sf::Color(255,255,255,255));
      rect.setOutlineThickness(3);
      window->draw(rect);

      /* draw obstacles */
      rect.setSize(sf::Vector2f(10,10));
      rect.setFillColor(sf::Color(255,255,255,255));
      for (int i = 0; i < grid_length; ++i) {
         if (grid[i]) {
            rect.setPosition((i%grid_metadata.width)*10,(i/grid_metadata.width)*10);
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
