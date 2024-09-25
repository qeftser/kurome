
#ifndef KUROME_CLASS_REPORTER

#define KUROME_CLASS_REPORTER
#include "Grid.h"
#include <thread>
#include <memory>
#include "kmsgs.h"

class Reporter {
public:
   std::shared_ptr<Grid> environment;
   std::shared_ptr<Grid> full_env;
   std::shared_ptr<Entity> self;
   std::shared_ptr<Entity> goal;
   std::unordered_map<int,void (*)(struct kurome_basemsg *, Reporter *)> handlers;
   std::thread cli;
   std::atomic<uint64_t> recved;
   ll_queue<KB *> reqs;
   std::vector<struct waiter_info> waiters;
   struct mapper_info mapper;
   struct agent_values * avaliable;
   struct agent_values * conn;

   Reporter() 
      : environment(NULL), full_env(NULL), self(NULL), 
        goal(NULL), recved(0L), avaliable(NULL), conn(NULL)
      { setDefaultHandlers(); };

   void launchClient();
   void clientSend(KB *);
   void registerHandler(int, void (*)(struct kurome_basemsg *, Reporter *));
   void setDefaultHandlers();

   int connect(long naddr);
   int connect(char * name);
   void connectFirst();
   void disconnectClient();

   void wait();
   void wait(uint64_t curr);

};


#endif
