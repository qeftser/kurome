
#include "Kurome.h"
#include "waiters/SimWaiter.hpp"
#include "mappers/NILRotAStarMapper.hpp"
#include <unistd.h>

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
   OO7.waiters.push_back(w);
   OO7.full_env = &env;

   OO7.launchServer(9132,"brainless",0xffffffff);
   printf("user: server launched\n");

   for ( ; ; ) {
      OO7.updateFromServer();
      usleep(10000);
   }

   sleep(40);

   OO7.updateFromServer();
   OO7.environment.print();

   pause();

   return 0;

}
