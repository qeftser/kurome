
#ifndef KUROME_AGENT_CLASS

#define KUROME_AGENT_CLASS
#include <vector>
#include <thread>
#include "Entity.hpp"
#include "Waiter.hpp"
#include "Mapper.h"

class Agent {
public:
   int flags;
   Entity & self;
   Entity & goal;
   std::vector<Waiter *> waiters;
   Mapper & mapper;
   Grid   & environment;
   Grid   * full_env;
   std::thread serv;
   ll_queue<std::tuple<KB *,khandle *>> reqs;
   std::set<khandle *>conns;
   std::unordered_map<int,void (*)(KB *, khandle *, Agent *)> handlers;

   Agent(Entity & self, Entity & goal, Mapper & mapper, Grid & env)
      : self(self), goal(goal), mapper(mapper), environment(env), full_env(NULL) { 
         mapper.self = this; 
         setDefaultHandlers();
      }

   double goalDist();

   void   launchServer(int,int);
   void   launchServer(int,std::string,int);
   void   updateFromServer(int updates=12);
   void   registerHandler(int, void (*)(KB *, khandle *, Agent *));

   void setDefaultHandlers();
   void sendAll(Entity &);
   void sendAll(Entity &, int);
   void sendAll(Sample &);
   void sendAll(Grid &);
   void sendAll(Grid &, int);
};

#endif

