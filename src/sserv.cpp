
#include "Kurome.h"
#include "waiters/SimWaiter.hpp"
#include "mappers/NILRotAStarMapper.hpp"
#include "mappers/SimpleAnyRotStaticPotentialFieldMapper.hpp"
#include <unistd.h>

void kurome_sserv_pause_handler(KB * msg, khandle * from, void * flag) {
   (void)msg;
   (void)from;
   *((bool *)flag) = false;
}

void kurome_sserv_start_handler(KB * msg, khandle * from, void * flag) {
   (void)msg;
   (void)from;
   *((bool *)flag) = true;
}

void kurome_sserv_state_handler(KB * msg, khandle * from, void * flag) {
   if (*((bool *)flag)) 
      kcmd::start(from);
   else
      kcmd::pause(from);
}

static int maxWeightSelect(int w1, int w2) {
   return (w1>w2?w1:w2);
   //return (w1+w2)/2;
}

int main(void) {

   srand(time(NULL)*clock());

   Grid g{0.5,200,200}; 

   Grid env{0.5,200,200};
   
   env.clear();
   Entity tst{100,100,12,12,KUROME_TYPE_ELPS,20};

   Entity fov{0,0,16,16,KUROME_TYPE_RECT,0};
   Entity me{10,10,8,3,KUROME_TYPE_ELPS,20};
   Entity goal{190,190,10,10,KUROME_TYPE_PONT,0};

   Waiter * w = new SimWaiter(env,fov,me);

   Mapper * m = NULL;
   
   (void)Generator::sparse(&g,60,5,7);
   Agent OO7 = Agent(me,goal,m,g);
   OO7.waiters.push_back(w);
   OO7.full_env = &env;

   m = new SimpleAnyRotStaticPotentialFieldMapper(100,&OO7);
   OO7.mapper = m;
   OO7.mapper->callback(0);

   bool shouldMove = false;
   OO7.registerHandler(KUROME_MSG_START,kurome_sserv_start_handler);
   OO7.registerHandler(KUROME_MSG_PAUSE,kurome_sserv_pause_handler);
   OO7.registerHandlerData(KUROME_MSG_START,&shouldMove);
   OO7.registerHandlerData(KUROME_MSG_PAUSE,&shouldMove);
   OO7.launchServer(9132,"a* demo",0xffffffff);
   printf("user: server launched\n");


   bool good;
   int step = 0;
   for ( ; ; ) {
      step++;
      OO7.updateFromServer();
      for (Waiter * w : OO7.waiters) {
         w->prepare();
         if (w->poll()) {
            Sample * next = w->serve();
            OO7.environment.apply(next,maxWeightSelect);
            Sample appli = OO7.environment.application(next);
            OO7.sendAll(appli);
            //delete next;
         }
      }
      usleep(70000);
      if (step%5 == 0 && shouldMove) {
         OO7.mapper->callback(0);
         Frame next = OO7.mapper->nextPoint(good);
         if (!good) {
            me.posx = next.posx;
            me.posy = next.posy;
            me.rot = next.rot;
            OO7.sendAll(me,KUROME_MSG_SELF);
         }
      }
   }

   pause();

   return 0;

}
