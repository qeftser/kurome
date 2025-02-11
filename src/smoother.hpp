
#ifndef __SMOOTHER_BASE

#define __SMOOTHER_BASE

#include "kurome.h"
#include <cstddef>
#include <atomic>

#include <SFML/Graphics.hpp>

class SmootherBase {
protected:

   /* Represents a position
    * on the map           */
   struct point {
      double x;
      double y;
   };

   /* A vector that represents a grid of points
    * in 2d space. We are using uint8_t values 
    * because we will exploit thier ability to
    * hold numbers higher than one in our grid
    * generation / update algorithm.          */
   uint8_t * grid = NULL;
   /* length of the vector */
   int grid_length = 0;

   /* Information about the grid that is 
    * useful to have.                   */
   nav_msgs::msg::MapMetaData grid_metadata;

public:

   SmootherBase() {}

   virtual void propose_path(const nav_msgs::msg::Path & msg) = 0;

   virtual nav_msgs::msg::Path get_path() = 0;

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
         for (size_t i = 0; i < grid_length; ++i)
            grid[i] = map.data[i];
      }
      /* The map is the same size, simply fill it with the new values */
      else {
         for (size_t i = 0; i < grid_length; ++i)
            grid[i] = map.data[i];
      }
   }

};

#endif
