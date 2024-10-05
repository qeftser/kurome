#pragma once

#include "../Kurome.h"

class SimWaiter : public Waiter {
private:
   Grid & fullenv;
   Entity & fov;
   Entity & self;
public:

   SimWaiter(Grid & fullenv, Entity & fov, Entity & self)
      : Waiter(), fullenv(fullenv), fov(fov), self(self) {};

   void prepare() {
      RectIterator ri;
      EllipseIterator ei;
      Sample * ret = new Sample(Entity(self.posx+fov.posx,
                                       self.posy+fov.posy,
                                       fov.xwid,
                                       fov.ywid,
                                       fov.type,
                                       0),
                                Eigen::MatrixXi((int)std::ceil((fov.xwid*2)/fullenv.getUnitSize()),
                                                (int)std::ceil((fov.ywid*2)/fullenv.getUnitSize())),
                                fullenv.getUnitSize());
      switch (fov.type) {
         case KUROME_TYPE_RECT:
            ri = RectIterator(&ret->orgin,&fullenv);
            while (!ri.done) {
               if (*ri) {
                  printf("val!\n");
               }
               ret->localVal(ri.locinfo().posx,ri.locinfo().posy) = *ri;
               ++ri;
            }
            dish(ret);
         break;
         case KUROME_TYPE_ELPS:
            ei = EllipseIterator(&ret->orgin,&fullenv);
            while (!ei.done) {
               ret->localVal(ei.locinfo().posx,ei.locinfo().posy) = *ei;
               ++ei;
            }
            dish(ret);
         break;
         default:
            errnok = KUROME_ETYPE;
            return;
         break;
      }
   }

};
