
#include "Kurome.h"

int main(void) {

   Grid g = Grid(0.075,0,10,0,10);

   Entity e0 = Entity(5,5,2,4,0,KUROME_TYPE_ELPS,40);
   /*
   Entity e1 = Entity(0,0,2,2,KUROME_TYPE_ELPS,40);
   Entity e2 = Entity(-5,-5,2,2,KUROME_TYPE_ELPS,40);
   Entity e3 = Entity(-10,-10,2,2,KUROME_TYPE_ELPS,40);
   */
   g.addEntity(&e0);
   /*
   g.addEntity(&e1);
   g.addEntity(&e2);
   g.addEntity(&e3);
   */

   /*
   g.info();
   g.print();

   g.changeSizeXmin(-15);

   g.info();
   g.print();

   g.changeSizeYmin(-5);

   g.info();
   g.print();

   g.changeSizeYmin(5);

   g.info();
   g.print();

   g.changeSizeXmin(5);

   g.info();
   g.print();
   
   g.changeSizeYmin(-20);

   g.info();
   g.print();

   g.changeSizeXmin(-20);

   g.info();
   g.print();

   g.changeUnitSize(0.1);
   g.info();
   g.print();

   g.changeUnitSize(5);
   */
   for ( ; ; ) {
      e0.rot += 1;
      g.changeUnitSize(0.075);
      g.print();
      usleep(10000);
   }

   return 0;
}
