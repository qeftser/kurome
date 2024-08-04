
#ifndef GRID_OBJECT

#define GRID_OBJECT
#include "Entity.h"
#include <vector>
#include <cmath>


#define GRID_NEW_VALUE_WEIGHT   0.5
#define GRID_CIRCLE_GRANULARITY 0.025

#define PI_1l2 1.57079632679
#define PI_3l2 4.71238898038
/* yes there is a 
 * predefined pi but
 * I am too lazy to 
 * figure out how to use it...
 */
#define PI 3.14159265358

class Grid {
private:
   std::vector<Entity> entities;
   double unitSize;
   int sizeX;
   int sizeY;
   int ** grid;
   void roundOut(const double, const double, int &, int &);
   int roundOutTop(const double);
   int roundOutBottom(const double);
   void updateGrid(int, int, double, double, double, double, double);
   int avgWeights(const int, const int, double);
public:
   Grid(double, double, double);
   void addEntity(const Entity * const);
   void addEntity(const WeightedEntity * const);
   void print(void);

};

#endif
