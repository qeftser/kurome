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
                                       self.rot),
                                Eigen::MatrixXi((int)std::ceil((fov.xwid)/fullenv.getUnitSize()),
                                                (int)std::ceil((fov.ywid)/fullenv.getUnitSize())),
                                fullenv.getUnitSize());
      switch (fov.type) {
         case KUROME_TYPE_RECT:
            ri = RectIterator(&ret->orgin,&fullenv);
            while (!ri.done) {
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
      /*
      putchar('\n');
      for (int i = 0; i < ret->values.rows(); ++i) {
         for (int j = 0; j < ret->values.cols(); ++j)
            printf("%02x ",ret->values(i,j));
         putchar('\n');
      }
      */
   }

};
