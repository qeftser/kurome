
#ifndef GENERATOR_CLASS

#define GENERATOR_CLASS

/*! \class Generator
 * Set of static methods for generating random
 * full grids for simulation. This helps to
 * preserve randomness in evaluation and saves
 * me some time from making a large set of 
 * simulation environments.
 */

#include "Grid.h"

class Generator {
public:

   /**
    * Generate a set of sparse obstacles over the
    * given Grid. Provide the number and maximum +
    * average width/length of the elements.
    */
   static Grid * sparse(Grid *,int,double,double);

   /**
    * Generate a maze of obstacles over the given Grid.
    * Provide a density (how many paths) for the maze.
    * Provide a minwid (minimum size of passages).
    */
   static Grid * maze(Grid *,int,double);

   /**
    * Generate a map with varying heights similar to 
    * a sand-dune topology for navigation. Same as
    * sparse but with smoothing and some other actions
    * to ensure good behavior.
    */
   static Grid * dune(Grid *,int,double,double);
};

#endif
