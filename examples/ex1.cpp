
#include "../src/Grid.h"

/* ex1.cpp */

int main(void) {


   Grid * g = new Grid(2,10,10);

   Entity * e = new Entity(0x2f,ENTITY_TYPE_RECT,2,2,5,5);

   g->addEntity(e);
   g->print();

   return 0;
}
