
#ifndef __OCCUPANCY_GRID

#define __OCCUPANCY_GRID
#include "kurome.h"

/* A class that will hopefully be useful for me. Allows creation
 * of a pose graph using multiple preregistered scans as well as
 * the addition of new scans to an existing pose graph. Values
 * are exposed in case I decide to use this class as an input
 * to a scan matcher.                                           */

class OccupancyGrid {
public:

   void to_msg(const nav_msgs::msg::OccupancyGrid & msg) {
   }

};

#endif
