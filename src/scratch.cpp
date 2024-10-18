
#include "Kurome.h"

int main(void) {

   srand(time(NULL)*clock());

   Grid g = Grid(0.5,0,230,0,120);

   (void)Generator::dune(&g,60,5,7);

   g.print();




   return 0;
}
