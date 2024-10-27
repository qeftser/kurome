
#ifndef KUROME_AGENT_CLASS

#define KUROME_AGENT_CLASS
#include <vector>
#include <thread>
#include <set>
#include "Entity.hpp"
#include "Waiter.hpp"
#include "kmsgs.h"

class Mapper;
class Grid;

/*! \class Agent
 * The Agent class is a collection of all the elements that should be nessesary to run a robot. 
 * It also has the perk of having a built-in server setup avaliable that makes it really easy
 * to interface with some other tools. Technically you don't need to use a Agent to run a robot,
 * but it makes things easier.
 */
class Agent {
public:
   int flags;                                                           /**< define what the Agent can do - for use with reporters */
   Entity & self;                                                       /**< define the shape and location of the robot */
   Entity & goal;                                                       /**< define the shape and location of the goal */
   std::vector<Waiter *> waiters;                                       /**< collection of all waiters in the robot */
   Mapper * mapper;                                                     /**< the current mapping algorithm in use */
   Grid   & environment;                                                /**< the current representation of the environment */
   Grid   * full_env;                                                   /**< optionally defined 
                                                                         * - holds the given simulation environment if there is one */
   std::thread serv;                                                    /**< holds the thread the server is running on */
   std::mutex  lock;                                                    /**< protect actions that may affect multiple threads */
   ll_queue<std::tuple<KB *,khandle *>> reqs;                           /**< holds the queued requests that need to be processed */
   std::set<khandle *>conns;                                            /**< holds the currently established connections */
   std::unordered_map<int,void (*)(KB *, khandle *, void *)> handlers; /**< holds the handlers used to process each message. 
                                                                        * Handlers accept the message, where it was from, and an 
                                                                        * arbitrary extra value that defaults to the Agent */
   std::unordered_map<int,void *> handlerData;                         /**< holds any special data to be associated with a handler */

   Agent(Entity & self, Entity & goal, Mapper * mapper, Grid & env)
      : self(self), goal(goal), mapper(mapper), environment(env), full_env(NULL) { 
         setDefaultHandlers();
      }

   /**
    * Return the euclidean between self and goal
    */
   double goalDist();

   /** Start the associated server at the given port 
    * with the specified flags */
   void   launchServer(short,int);
   /** Start the associated server at the given port 
    * with the specified name and flags */
   void   launchServer(short,std::string,int);
   /** Dequeue and handle the provided number of messages */
   void   updateFromServer(int updates=12);
   /** Register a message with a message handler */
   void   registerHandler(int, void (*)(KB *, khandle *, void *));
   /** Register a specific pointer with a message handler */
   void   registerHandlerData(int, void *);

   /** Set the provided default handlers. Will override 
    * previous values */
   void setDefaultHandlers();
   /** Send all connections a message with the given entity */
   void sendAll(Entity &);
   /** Send all connections a message of the given type with
    * the given entity */
   void sendAll(Entity &, int);
   /** Send all connections a sample */
   void sendAll(Sample &);
   /** Send all connections the Grid */
   void sendAll(Grid &);
   /** Send all connecitons a message of the given type
    * with a Grid attached */
   void sendAll(Grid &, int);
};

#endif

