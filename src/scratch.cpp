
#include "Kurome.h"

int main(void) {

   srand(time(NULL)*clock());

   Grid g = Grid(5,0,100,0,100);

   (void)Generator::dune(&g,20,5,7);

   g.print();




   return 0;
}
