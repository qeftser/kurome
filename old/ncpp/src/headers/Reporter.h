
#ifndef KUROME_CLASS_REPORTER

#define KUROME_CLASS_REPORTER
#include "Grid.h"
#include <thread>
#include <memory>
#include "kmsgs.h"

/*! \class Reporter
 * this class is designed to be used as the back end for a client application
 * attached to an Agent class server. It has default callbacks designed to generate
 * a representation of what the Agent is seeing and doing based on the data it receives.
 */
class Reporter {
public:
   std::shared_ptr<Grid> environment; /**< the reporter's representation of the Agent environment */
   std::shared_ptr<Grid> full_env;    /**< the reporter's representation of the Agent full_env */
   std::shared_ptr<Entity> self;      /**< the reporter's representation of the Agent self */
   std::shared_ptr<Entity> goal;      /**< the reporter's representation of the Agent goal */
   std::unordered_map<int,void (*)(struct kurome_basemsg *, void *)> handlers; /**< assigned message handlers */
   std::unordered_map<int,void *> handlerData; /**< assigned message handler data */
   std::thread cli; /**< the thread the client is running on */
   std::atomic<uint64_t> recved; /**< a count of the total received messages */
   khandle reqs; /**< a queue of messages to send */
   std::vector<struct waiter_info> waiters; /**< info about the waiters the Agent is using */
   struct mapper_info mapper; /**< info about the mapper the Agent is using */
   struct agent_values * avaliable; /**< info about the avaliable Agent servers to connect to */
   struct agent_values * conn; /**< the current active connection */
   struct partial_grid_struct  ginfo; /**< info about the current environment Grid representation */
   struct partial_grid_struct fginfo; /**< info about the current full_env Grid */

   /**
    * construct a reporter. Set the default handlers as well */
   Reporter() 
      : environment(NULL), full_env(NULL), self(NULL), 
        goal(NULL), recved(0L), avaliable(NULL), conn(NULL)
      { setDefaultHandlers(); };

   /* message passing */

   /**
    * startup the reporter client */
   void launchClient();
   /**
    * send the given message through the reporter */
   void clientSend(KB *);
   /**
    * register a new handler for the given message type */
   void registerHandler(int, void (*)(struct kurome_basemsg *, void *));
   /**
    * register handler data to be used by the associated handler when it is
    * called on the specified message type */
   void registerHandlerData(int, void *);
   /**
    * set the default message handlers for the Reporter class. Will override any previously set handlers */
   void setDefaultHandlers();

   /* handle connections */

   /**
    * connect to an avaliable connection based on the numerical ip address */
   int connect(long naddr);
   /**
    * connect to an avaliable connection based on the connection name */
   int connect(char * name);
   /**
    * connect to the first avaliable connection */
   void connectFirst();
   /**
    * disconnect the client from whatever Agent server it was connected to */
   void disconnectClient();

   /* wait on server */

   /**
    * wait for any new message to be recieved */
   void wait();
   /**
    * wait until the message count changes from the given one */
   void wait(uint64_t curr);

};


#endif
