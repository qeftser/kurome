
#include "../src/Grid.h"

int main(void) {


   Grid * g = new Grid(0.1,10,10);

   Entity * e = new Entity(0x2f,ENTITY_TYPE_RECT,2,2,5,5);

   g->addEntity(e);
   g->print();

   return 0;
}
