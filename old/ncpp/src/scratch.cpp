
#include "Kurome.h"
#include "mappers/SimpleAnyRotStaticPotentialFieldMapper.hpp"
#include "mappers/NullMapper.hpp"

#define add(xi,yi) \
        res = nn_add(&n,(double)xi,(double)yi); \
        res2 = add2((double)xi,(double)yi,&test); \
        pass = (res->x == res2.x && res->y == res2.y); \
        if (pass) printf("\033[32m"); else printf("\033[31m"); \
        printf("[%f,%f](%f,%f):(%f,%f)\033[0m\n",res2.x,res2.y,(double)xi,(double)yi,res->x,res->y)

struct point { double x; double y; };
double dist2(struct point * x, struct point * y) {
   return (((x->x-y->x)*(x->x-y->x)) + ((x->y-y->y)*(x->y-y->y)));
}

struct point add2(double x, double y, std::vector<point> * z) {
   double closest = 10000000000;
   struct point ret = {0,0};
   struct point nev = {.x = x, .y = y};
   for (point p : *z) {
      //printf("[%f,%f]:{%f}",p.x,p.y,dist2(&p,&nev));
      if (dist2(&p,&nev) < closest) {
         //printf("!");
         closest = dist2(&p,&nev);
         ret = p;
      }
   }
   z->push_back(nev);
   return ret;
}

int main(void) {

   std::vector<point> test;
   test.push_back({5,5});
   
   struct nn_base n;
   nn_init(&n,1000,10,10,5,5,5);

   struct nn_node * res;
   struct point res2;
   int pass;

   add(6.11,6);

   add(14.51,14);

   add(1.13,12);

   add(18.21,15);

   add(7.11,5);
   
   add(6.14,4);
   add(4.15,4);

   add(6.1,4.9);

   add(6.13,5.1);

   add(3.13,2);
   add(3.12,1);

   add(5.31,4);
   /*
   */






   return 0;
}
