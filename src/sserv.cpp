
#include "Kurome.h"
#include "waiters/SimWaiter.hpp"
#include "mappers/NILRotAStarMapper.hpp"
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

int main(void) {

   Grid g{0.5,200,200};

   Grid env{0.5,200,200};
   
   env.clear();
   Entity tst{100,100,12,12,KUROME_TYPE_ELPS,20};
   //env.addEntity(&tst);
   //g.addEntity(&tst);

   Entity fov{0,0,8,8,KUROME_TYPE_RECT,0};
   Entity me{10,10,8,3,KUROME_TYPE_ELPS,20};
   Entity goal{190,190,10,10,KUROME_TYPE_PONT,0};

   Waiter * w = new SimWaiter(env,fov,me);

   Mapper * m = new NILRotAStarMapper();
   
   Agent OO7 = Agent(me,goal,*m,g);
   m->env = &g;
   m->self = &me;
   m->goal = &goal;
   OO7.waiters.push_back(w);
   OO7.full_env = &env;

   bool shouldMove = false;
   OO7.registerHandler(KUROME_MSG_START,kurome_sserv_start_handler);
   OO7.registerHandler(KUROME_MSG_PAUSE,kurome_sserv_pause_handler);
   OO7.registerHandlerData(KUROME_MSG_START,&shouldMove);
   OO7.registerHandlerData(KUROME_MSG_PAUSE,&shouldMove);
   OO7.launchServer(9132,"brainless",0xffffffff);
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
            OO7.sendAll(*next);
            OO7.environment.apply(next);
         }
      }
      usleep(10000);
      if (step%50 == 0 && shouldMove) {
         OO7.mapper.callback(0);
         me.rot = atan2(me.posx-goal.posx,me.posy-goal.posy);
         Frame next = OO7.mapper.nextPoint(good);
         if (!good) {
            me.posx = next.posx;
            me.posy = next.posy;
            printf("%f %f\n",me.posx,me.posy);
            me.rot = next.rot;
            OO7.sendAll(me,KUROME_MSG_SELF);
         }
      }
   }

   pause();

   return 0;

}
