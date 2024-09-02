
#include "Kurome.h"

int main(void) {
   
   Grid * g = new Grid(0.5,100,100);

   g->info();

   RectIterator iter = RectIterator(00.0,30.0,00.0,30.0,g);
   while (!iter.done) {
      *iter = 9;
      ++iter;
   }

   g->print();

   return 0;
}
