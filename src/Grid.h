
#ifndef GRID_OBJECT

#define GRID_OBJECT
#include "Entity.h"
#include <vector>
#include <cmath>
#include <cstdint>

/* Default weight for adding an entity
 * to the grid */
#define GRID_NEW_VALUE_WEIGHT   0.6
/* Number of slices we consider when drawing a circle.
 * can be increased somewhat if we want adding circles
 * to be faster */
#define GRID_CIRCLE_GRANULARITY 0.025
/* How many children to create from a node in the 
 * A* mapping. It is roughly this value * 4. */
#define GRID_MAP_FRAME_SLICES   8
/* Distance moved by each subsequent step in the
 * mapping algoritm */
#define GRID_MAP_FRAME_STEP     0.25
/* Granularity to view different routes at in the
 * mapping algorithm */
#define GRID_MAP_ANGLE_SLICING  20

/* pi/2 */
#define PI_1l2 1.57079632679
/* 3pi/2 */
#define PI_3l2 4.71238898038
/* yes there is a 
 * predefined pi but
 * I am too lazy to 
 * figure out how to use it...
 */
#define PI 3.14159265358

/* Sanity check on my angle math.
 * trigonometry is still bad */
double quickCalc(double a, double b);

class Grid {
private:
   /* all entities that have been added to the grid */
   std::vector<Entity> entities;
   /* how wide is each grid cell? */
   double unitSize;
   /* total x axis grid cells */
   int sizeX;
   /* total y axis grid cells */
   int sizeY;
   /* grid weights */
   int ** grid;
   /* used for displaying the mapping */
   int ** marks;

   /* Expands floating point values to match their appropriate
    * grid squares. */
   void roundOut(const double, const double, int &, int &);
   
   /* Expands a floating point value to match the closest grid
    * square above it */
   int roundOutTop(const double);

   /* Expands a floating point value to match the closest grid
    * square below it */
   int roundOutBottom(const double);

   /* Update the grid by adding an element. This is a general add
    * function that we plug entities into */
   void updateGrid(int, int, double, double, double, double, double);

   /* Compute the new weight of a grid square given the old weight and
    * the danger of the new entity */
   int avgWeights(const int, const int, double);

   /* get the cost of moving an entity to a position based on the weights
    * of the grid squares it would step on */
   int getCost(const Entity * const, struct FRAME_STRUCT *);

   /* used in the mapping algorithm */
   struct FRAME_STRUCT * 
      childFrame(struct FRAME_STRUCT *, const Entity * const, double, double, double);

   /* hash a frame to a unique value */
   uint64_t getFrameId(struct FRAME_STRUCT *);
   
   /* is this frame in bounds? */
   bool inBounds(double,double);
public:
   Grid(double, double, double);
   ~Grid();

   /* Insert an entity into the grid */
   void addEntity(const Entity * const);
   void addEntity(const WeightedEntity * const);

   /* map an entity over the grid and display the mapping */
   void mapEntity(const MovingEntity * const, double destX, double destY);
   
   /* print the grid */
   void print(void);

};

#endif
