
#ifndef GRID_OBJECT

#define GRID_OBJECT
#include "Entity.h"
#include <vector>
#include <cmath>
#include <cstdint>

#define GRID_NEW_VALUE_WEIGHT   0.6
#define GRID_CIRCLE_GRANULARITY 0.025
#define GRID_MAP_FRAME_SLICES   8
#define GRID_MAP_FRAME_STEP     0.25
#define GRID_MAP_ANGLE_SLICING  20

#define PI_1l2 1.57079632679
#define PI_3l2 4.71238898038
/* yes there is a 
 * predefined pi but
 * I am too lazy to 
 * figure out how to use it...
 */
#define PI 3.14159265358

double quickCalc(double a, double b);

class Grid {
private:
   std::vector<Entity> entities;
   double unitSize;
   int sizeX;
   int sizeY;
   int ** grid;
   int ** marks;
   void roundOut(const double, const double, int &, int &);
   int roundOutTop(const double);
   int roundOutBottom(const double);
   void updateGrid(int, int, double, double, double, double, double);
   int avgWeights(const int, const int, double);
   int getCost(const Entity * const, struct FRAME_STRUCT *);
   struct FRAME_STRUCT * 
      childFrame(struct FRAME_STRUCT *, const Entity * const, double, double, double);
   uint64_t getFrameId(struct FRAME_STRUCT *);
   bool inBounds(double,double);
public:
   Grid(double, double, double);
   ~Grid();
   void addEntity(const Entity * const);
   void addEntity(const WeightedEntity * const);
   void mapEntity(const MovingEntity * const, double destX, double destY);
   void print(void);

};

#endif
