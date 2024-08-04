
#include "Grid.h"
#include "Entity.h"
#include <cstdio>

int main(void) {

   Grid * g = new Grid(0.1,20.0,20.0);

   g->print();

   Entity * e = new Entity(0x5f,ENTITY_TYPE_CIRCLE,2.0,0.0,6.0,6.0);
   Entity * e2 = new Entity(0x0f,ENTITY_TYPE_CIRCLE,1.0,0.0,6.0,6.0);
   Entity * e3 = new Entity(0x07,ENTITY_TYPE_CIRCLE,3.0,0.0,6.0,6.0);
   g->addEntity(e);
   g->addEntity(e2);
   g->addEntity(e3);

   g->print();

   /*
   Entity * e2 = new Entity(4,ENTITY_TYPE_RECT,4.0,4.0,1.0,1.0);
   g->addEntity(e2);
   g->print();
   */


   return 0;
}
