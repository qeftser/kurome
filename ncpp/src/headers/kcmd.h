
#ifndef KCMD_HELPER_FUNCTIONS

#define KCMD_HELPER_FUNCTIONS
#include "Entity.hpp"
#include "Sample.hpp"
#include "Waiter.hpp"
#include "Mapper.h"
#include "Grid.h"

class kcmd {
public:
   static void addEntity(Entity &, khandle *);
   static void chgSelf(Entity &, khandle *);
   static void chgGoal(Entity &, khandle *);
   static void chgFlags(int, khandle *);
   static void mapCallback(int, khandle *);
   static void setIdx(double, double, int, khandle *);
   static void clear(khandle *);
   static void allSamples(khandle *);
   static void allEntities(khandle *);
   static void chgUnits(double, khandle *);
   static void chgX(double, khandle *);
   static void chgY(double, khandle *);
   static void getGrid(khandle *);
   static void getFullGrid(khandle *);
   static void entity(Entity &, khandle *);
   static void self(Entity &, khandle *);
   static void goal(Entity &, khandle *);
   static void sample(Sample &, khandle *);
   static void waiterInfo(Waiter &, khandle *);
   static void mapperInfo(Mapper &, khandle *);
   static void grid(Grid &, khandle *);
   static void fullGrid(Grid &, khandle *);
   static void fAddEntity(Entity &, khandle *);
   static void fChgEntity(Entity &, khandle *);
   static void fRemEntity(Entity &, khandle *);
   static void fClear(khandle *);
   static void clense(khandle *);
   static void fClense(khandle *);
   static void getSelf(khandle *);
   static void getGoal(khandle *);
   static void getMapperInfo(khandle *);
   static void getWaiterInfo(khandle *);
};

#endif
