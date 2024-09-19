
#ifndef KCMD_HELPER_FUNCTIONS

#define KCMD_HELPER_FUNCTIONS
#include "Entity.hpp"
#include "Sample.hpp"
#include "Waiter.hpp"
#include "Mapper.h"
#include "Grid.h"

class kcmd {
public:
   static void addEntity(Entity &, ll_queue<KB *> *);
   static void chgSelf(Entity &, ll_queue<KB *> *);
   static void chgGoal(Entity &, ll_queue<KB *> *);
   static void chgFlags(int, ll_queue<KB *> *);
   static void mapCallback(int, ll_queue<KB *> *);
   static void setIdx(double, double, int, ll_queue<KB *> *);
   static void clear(ll_queue<KB *> *);
   static void allSamples(ll_queue<KB *> *);
   static void allEntities(ll_queue<KB *> *);
   static void chgUnits(double, ll_queue<KB *> *);
   static void chgX(double, ll_queue<KB *> *);
   static void chgY(double, ll_queue<KB *> *);
   static void getGrid(ll_queue<KB *> *);
   static void getFullGrid(ll_queue<KB *> *);
   static void entity(Entity &, ll_queue<KB *> *);
   static void self(Entity &, ll_queue<KB *> *);
   static void goal(Entity &, ll_queue<KB *> *);
   static void sample(Sample &, ll_queue<KB *> *);
   static void waiterInfo(Waiter &, ll_queue<KB *> *);
   static void mapperInfo(Mapper &, ll_queue<KB *> *);
   static void grid(Grid &, ll_queue<KB *> *);
   static void fullGrid(Grid &, ll_queue<KB *> *);
};

#endif
