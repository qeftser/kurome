
#include "Kurome.h"
#include "SimWaiter.hpp"
#include "NILRotAStarMapper.hpp"
#include <unistd.h>

int main(void) {

   Grid g{1,200,200};

   Grid env{0.5,200,200};
   env.clear();
   Entity fov{0,0,8,8,KUROME_TYPE_RECT,0};
   Entity me{10,10,3,3,KUROME_TYPE_ELPS,20};
   Entity goal{190,190,1,1,KUROME_TYPE_PONT,0};

   Waiter * w = new SimWaiter(env,fov,me);

   Mapper * m = new NILRotAStarMapper();
   
   Agent OO7 = Agent(me,goal,*m,g);
   OO7.waiters.push_back(w);

   OO7.launchServer(0xffffffff,"brainless",12333);
   printf("user: server launched\n");

   for ( ; ; ) {
      printf("cycle\n");
      OO7.updateFromServer();
      usleep(10000);
   }

   sleep(40);

   OO7.updateFromServer();
   OO7.environment.print();

   pause();

   return 0;

}
