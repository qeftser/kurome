
#ifndef KCMD_HELPER_FUNCTIONS

#define KCMD_HELPER_FUNCTIONS
#include "Entity.hpp"
#include "Sample.hpp"
#include "Waiter.hpp"
#include "Mapper.h"
#include "Grid.h"

/*! \class kcmd
 * set of static methods for passing messages between
 * Reporter and Agent objects without exposing the underlying
 * system
 */
class kcmd {
public:
   /**
    * add a message of type KUROME_MSG_ADD_ENTITY with data
    * from the provided Entity to the provided queue */
   static void addEntity(Entity &, khandle *);
   /**
    * add a message of type KUROME_MSG_CHGSELF with data
    * from the provided Entity to the provided queue */
   static void chgSelf(Entity &, khandle *);
   /**
    * add a message of type KUROME_MSG_CHGGOAL with data
    * from the provided Entity to the provided queue */
   static void chgGoal(Entity &, khandle *);
   /**
    * add a message of type KUROME_MSG_CHGFLAGS with data
    * from the provided int to the provided queue */
   static void chgFlags(int, khandle *);
   /**
    * add a message of type KUROME_MSG_MAPCALLBACK with data
    * from the provided int to the provided queue */
   static void mapCallback(int, khandle *);
   /**
    * add a message of type KUROME_MSG_SET_IDX with data
    * from the provided values to the provided queue */
   static void setIdx(double, double, int, khandle *);
   /**
    * add a message of type KUROME_MSG_CLEAR to the provided queue */
   static void clear(khandle *);
   /**
    * add a message of type KUROME_MSG_ALLSAMPLES to the provided queue */
   static void allSamples(khandle *);
   /**
    * add a message of type KUROME_MSG_ALLENTITIES to the provided queue */
   static void allEntities(khandle *);
   /**
    * add a message of type KUROME_MSG_FALLSAMPLES to the provided queue */
   static void fAllSamples(khandle *);
   /**
    * add a message of type KUROME_MSG_FALLENTITIES to the provided queue */
   static void fAllEntities(khandle *);
   /**
    * add a message of type KUROME_MSG_CHG_UNITSIZE to the provided queue */
   static void chgUnits(double, khandle *);
   /**
    * add a message of type KUROME_MSG_CHG_XBLOCKS to the provided queue */
   static void chgX(double, khandle *);
   /**
    * add a message of type KUROME_MSG_CHG_YBLOCKS to the provided queue */
   static void chgY(double, khandle *);
   /**
    * add a message of type KUROME_MSG_GET_GRID to the provided queue */
   static void getGrid(khandle *);
   /**
    * add a message of type KUROME_MSG_GET_FULLGRID to the provided queue */
   static void getFullGrid(khandle *);
   /**
    * add a message of type KUROME_MSG_ENTITY to the provided queue
    * The message will contain the data from the provided entity */
   static void entity(Entity &, khandle *);
   /**
    * add a message of type KUROME_MSG_FENTITY to the provided queue
    * The message will contain the data from the provided entity */
   static void fEntity(Entity &, khandle *);
   /**
    * add a message of type KUROME_MSG_SELF to the provided queue
    * The message will contain the data from the provided entity */
   static void self(Entity &, khandle *);
   /**
    * add a message of type KUROME_MSG_GOAL to the provided queue
    * The message will contain the data from the provided entity */
   static void goal(Entity &, khandle *);
   /**
    * add a message of type KUROME_MSG_SAMPLE to the provided queue
    * The message will contain the data from the provided entity */
   static void sample(Sample &, khandle *);
   /**
    * add a message of type KUROME_MSG_FSAMPLE to the provided queue
    * The message will contain the data from the provided entity */
   static void fSample(Sample &, khandle *);
   /**
    * add a message of type KUROME_MSG_WAITERINFO to the provided queue
    * The message will contain the data from the provided entity */
   static void waiterInfo(Waiter &, khandle *);
   /**
    * add a message of type KUROME_MSG_MAPPERINFO to the provided queue
    * The message will contain the data from the provided entity */
   static void mapperInfo(Mapper &, khandle *);
   /**
    * add a message of type KUROME_MSG_GRID to the provided queue
    * The message will contain the data from the provided grid */
   static void grid(Grid &, khandle *);
   /**
    * add a message of type KUROME_MSG_FULLGRID to the provided queue
    * The message will contain the data from the provided grid */
   static void fullGrid(Grid &, khandle *);
   /**
    * add a message of type KUROME_MSG_FADD_ENTITY to the provided queue
    * The message will contain the data from the provided entity */
   static void fAddEntity(Entity &, khandle *);
   /**
    * add a message of type KUROME_MSG_FCHGENTITY to the provided queue
    * The message will contain the data from the provided entity */
   static void fChgEntity(Entity &, khandle *);
   /**
    * add a message of type KUROME_MSG_FREMENTITY to the provided queue
    * The message will contain the data from the provided entity */
   static void fRemEntity(Entity &, khandle *);
   /**
    * add a message of type KUROME_MSG_FCLEAR to the provided queue */
   static void fClear(khandle *);
   /**
    * add a message of type KUROME_MSG_CLENSE to the provided queue */
   static void clense(khandle *);
   /**
    * add a message of type KUROME_MSG_FCLENSE to the provided queue */
   static void fClense(khandle *);
   /**
    * add a message of type KUROME_MSG_GETSELF to the provided queue */
   static void getSelf(khandle *);
   /**
    * add a message of type KUROME_MSG_GETGOAL to the provided queue */
   static void getGoal(khandle *);
   /**
    * add a message of type KUROME_MSG_GETMAPPER to the provided queue */
   static void getMapperInfo(khandle *);
   /**
    * add a message of type KUROME_MSG_GETWAITERS to the provided queue */
   static void getWaiterInfo(khandle *);
   /**
    * add a message of type KUROME_MSG_START to the provided queue */
   static void start(khandle *);
   /**
    * add a message of type KUROME_MSG_STOP to the provided queue */
   static void stop(khandle *);
   /**
    * add a message of type KUROME_MSG_PAUSE to the provided queue */
   static void pause(khandle *);
   /**
    * add a message of type KUROME_MSG_STATE to the provided queue */
   static void state(khandle *);
};

#endif
