
#include "Kurome.h"

int main(void) {

   Grid g = Grid(0.25,0,10,0,10);

   //Entity e0 = Entity(5,5,2,4,0,KUROME_TYPE_ELPS,40);

   Sample s(Entity(5,5,2,4,45,KUROME_TYPE_RECT,0),Eigen::MatrixXi(3,5),1);
   s.values(0,0) = 1;
   s.values(0,1) = 2;
   s.values(0,2) = 3;
   s.values(0,3) = 4;
   s.values(0,4) = 5;
   s.values(1,0) = 2;
   s.values(2,0) = 3;
   
   g.print();
   putchar('\n');
   g.apply(&s);
   g.print();




   return 0;
}
