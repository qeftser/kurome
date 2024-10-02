
#include "../src/Grid.h"

int main(void) {

   Grid * g = new Grid(0.10,30.0,20.0);

   MovingEntity * e3 = new MovingEntity(0xa3,ENTITY_TYPE_RECT,1.0,1.0,2.0,2.0,1.00);

   Entity * e = new Entity(0x2f,ENTITY_TYPE_RECT,1.5,1.5,6.0,6.0);
   Entity * e2 = new Entity(0x2f,ENTITY_TYPE_RECT,4.0,4.0,6.0,6.0);
   Entity * e4 = new Entity(0x2f,ENTITY_TYPE_CIRCLE,4.0,1.0,10.0,10.0);
   Entity * e5 = new Entity(0x2f,ENTITY_TYPE_RECT,2.0,8.0,10.0,1.0);
   Entity * e6 = new Entity(0x2f,ENTITY_TYPE_RECT,2.0,4.5,13.0,18.0);
   g->addEntity(e);
   g->addEntity(e2);
   g->addEntity(e4);
   g->addEntity(e5);
   g->addEntity(e6);

   g->mapEntity(e3,18.0,4.0);

   return 0;
}
