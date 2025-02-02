
#include "Kurome.h"
#include "SimWaiter.hpp"
#include "NILRotAStarMapper.hpp"

#include <stdlib.h>
#include <unistd.h>

int main(void) {
   
   Grid g{1,200,200};

   Grid env{0.5,200,200};
   env.clear();
   Entity fov{0,0,8,8,KUROME_TYPE_RECT,0};
   Entity me{10,10,3,3,KUROME_TYPE_ELPS,20};
   Entity goal{190,190,1,1,KUROME_TYPE_PONT,0};

   env.print();

   Waiter * w = new SimWaiter(env,fov,me);

   Mapper * m = new NILRotAStarMapper();
   
   Agent OO7 = Agent(me,goal,*m,g);
   OO7.waiters.push_back(w);

   getchar();
   for (int i = 0; i < 1000; ++i) {
      for (Waiter * w : OO7.waiters) {
         w->prepare();
         OO7.environment.apply(w->serve());
      }
      OO7.mapper.callback(0);
      Frame f = OO7.mapper.nextPoint();
      OO7.self.posx = f.posx;
      OO7.self.posy = f.posy;

      if (((OO7.self.posx-OO7.goal.posx)*(OO7.self.posx-OO7.goal.posy))+
          ((OO7.self.posy-OO7.goal.posy)*(OO7.self.posy-OO7.goal.posy)) < 5)
         return 0;

      OO7.environment.addEntity(&OO7.self);
      OO7.environment.print();
      OO7.environment.remEntity(&OO7.self);

      getchar();
   }

   return 0;
}
