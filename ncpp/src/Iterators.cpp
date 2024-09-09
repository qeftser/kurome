
#include "Kurome.h"

int errno_kurome;

RectIterator::RectIterator(double srtx, double endx, double srty, double endy, Grid * g) {
   this->g    = g;
   this->srtx = g->roob(srtx);
   this->endx = g->root(endx);
   this->srty = g->roob(srty);
   this->endy = g->root(endy);
   this->posx = srtx;
   this->posy = srty-1;
   memset(&info,0,sizeof(struct ShapeIteratorInfo));
   info.posy = (posy*g->getUnitSize())+(g->getUnitSize()/2.0000001);
   info.posx = (posx*g->getUnitSize())+(g->getUnitSize()/2.0000001);
   done       = false;
   operator++();
}

RectIterator::RectIterator(const RectIterator & other) {
   this->g    = other.g;
   this->srtx = other.srtx;
   this->endx = other.endx;
   this->srty = other.srty;
   this->endy = other.endy;
   this->posx = other.srtx;
   this->posy = other.srty;
   memcpy(&this->info,&other.info,sizeof(struct ShapeIteratorInfo));
   info.posy = (posy*g->getUnitSize())+(g->getUnitSize()/2.0000001);
   info.posx = (posx*g->getUnitSize())+(g->getUnitSize()/2.0000001);
   this->done = other.done;
}

RectIterator::RectIterator(Entity * e, Grid * g) {
#ifndef KUROME_NOCHECK
   if (e->type != KUROME_TYPE_RECT) {
      errnok = KUROME_ETYPE;
      info.val = NULL;
      done = true;
      return;
   }
#endif
   this->g    = g;
   this->srtx = g->roob(e->posx-(e->xwid/2.0));
   this->endx = g->root(e->posx+(e->xwid/2.0));
   this->srty = g->roob(e->posy-(e->ywid/2.0));
   this->endy = g->root(e->posy+(e->ywid/2.0));
   this->posx = srtx;
   this->posy = srty-1;
   memset(&info,0,sizeof(struct ShapeIteratorInfo));
   info.posy = (posy*g->getUnitSize())+(g->getUnitSize()/2.0000001);
   info.posx = (posx*g->getUnitSize())+(g->getUnitSize()/2.0000001);
   done       = false;
   operator++();
}

RectIterator::RectIterator(double offx, double offy, Entity * e, Grid * g) {
#ifndef KUROME_NOCHECK
   if (e->type != KUROME_TYPE_RECT) {
      errnok = KUROME_ETYPE;
      info.val = NULL;
      done = true;
      return;
   }
#endif
   this->g    = g;
   this->srtx = g->roob((offx-(e->xwid/2.0)));
   this->endx = g->root((offx+(e->xwid/2.0)));
   this->srty = g->roob((offy-(e->ywid/2.0)));
   this->endy = g->root((offy+(e->ywid/2.0)));
   this->posx = srtx;
   this->posy = srty-1;
   memset(&info,0,sizeof(struct ShapeIteratorInfo));
   info.posy = (posy*g->getUnitSize())+(g->getUnitSize()/2.0000001);
   info.posx = (posx*g->getUnitSize())+(g->getUnitSize()/2.0000001);
   done       = false;
   operator++();
}


RectIterator & RectIterator::operator++() {
   double full = g->getUnitSize();
   double half = full/2.00001;
   do {
      ++posy;
      info.posy = (posy*full)+half;
      if (posy > endy) {
         ++posx;
         info.posx = (posx*full)+half;
         posy = srty;
         info.posy = (posy*full)+half;
         if (posx > endx) {
            done = true;
            info.val = NULL;
            return *this;
         }
      }
   } while (!g->inBounds(posx,posy));
   info.val = g->getIdxPtr(posx,posy);
   return *this;
}

RectIterator & RectIterator::operator++(int assign) {
   (void)assign;
   operator++();
   return *this;
}

bool RectIterator::operator==(const RectIterator & other) {
   if (g == other.g &&
       srtx == other.srtx &&
       srty == other.srty &&
       posx == other.posx &&
       posy == other.posy)
      return true;
   return false;
}

bool RectIterator::operator!=(const RectIterator & other) {
   return !operator==(other);
}

int & RectIterator::operator*() {
   return *info.val;
}

struct ShapeIteratorInfo & RectIterator::locinfo() {
   return info;
}

EllipseIterator::EllipseIterator(double wrad, double hrad, double x, double y, Grid * g) {
   this->g           = g;
   this->granularity = g->getUnitSize()/g->getXBlocks();
   this->k           = -granularity;
   this->wrad        = wrad;
   this->hrad        = hrad;
   this->x           = x;
   this->y           = y;
   this->xP          = -999;
   memset(&this->info,0,sizeof(struct ShapeIteratorInfo));
   this->done        = false;
   this->xC          = -999;
   this->yG          = 0;
   this->yL          = 0;
   operator++();
}

EllipseIterator::EllipseIterator(const EllipseIterator & other) {
   this->g           = other.g;
   this->granularity = other.granularity;
   this->k           = other.k ;
   this->wrad        = other.wrad;
   this->hrad        = other.hrad;
   this->x           = other.x;
   this->y           = other.y;
   this->xP          = other.xP;
   memcpy(&this->info,&other.info,sizeof(struct ShapeIteratorInfo));
   this->done        = other.done;
   this->xC          = other.xC;
   this->yG          = other.yG;
   this->yL          = other.yL;
}

EllipseIterator::EllipseIterator(Entity * e, Grid * g) {
#ifndef KUROME_NOCHECK
   if (e->type != KUROME_TYPE_ELPS) {
      errnok = KUROME_ETYPE;
      done = true;
      info.val = NULL;
      return;
   }
#endif
   this->g           = g;
   this->granularity = g->getUnitSize()/g->getXBlocks();
   this->k           = -granularity;
   this->wrad        = e->xwid;
   this->hrad        = e->ywid;
   this->x           = e->posx;
   this->y           = e->posy;
   this->xP          = -999;
   memset(&this->info,0,sizeof(struct ShapeIteratorInfo));
   this->done        = false;
   this->xC          = -999;
   this->yG          = 0;
   this->yL          = 0;
   operator++();
}

EllipseIterator::EllipseIterator(double offx, double offy, Entity * e, Grid * g) {
#ifndef KUROME_NOCHECK
   if (e->type != KUROME_TYPE_ELPS) {
      errnok = KUROME_ETYPE;
      done = true;
      info.val = NULL;
      return;
   }
#endif
   this->g           = g;
   this->granularity = g->getUnitSize()/g->getXBlocks();
   this->k           = -granularity;
   this->wrad        = e->xwid;
   this->hrad        = e->ywid;
   this->x           = offx;
   this->y           = offy;
   this->xP          = -999;
   memset(&this->info,0,sizeof(struct ShapeIteratorInfo));
   this->done        = false;
   this->xC          = -999;
   this->yG          = 0;
   this->yL          = 0;
   operator++();
}

EllipseIterator & EllipseIterator::operator++() {
   do {
      ++yL;
      info.posy += g->getUnitSize();
      if (yL > yG) {
         do {
            xP = xC;
            k+=granularity;
            if (k > PI) {
               done = true;
               info.val = NULL;
               return *this;
            }
            info.posx = (x + (cos(k) * wrad));
            info.posy = (y + (sin(k+PI) * hrad));
            xC = g->roor(info.posx);
            yG = g->roor(y + (sin(k) * hrad));
            yL = g->roor(info.posy);
         } while (xC == xP);
      }
   } while (!g->inBounds(xC,yL));
   info.val = g->getIdxPtr(xC,yL);
   return *this;
}

EllipseIterator & EllipseIterator::operator++(int assign) {
   (void)assign;
   operator++();
   return *this;
}

bool EllipseIterator::operator==(const EllipseIterator & other) {
   if (g           == other.g           &&
       granularity == other.granularity &&
       k           == other.k           &&
       wrad        == other.wrad        &&
       hrad        == other.hrad        &&
       x           == other.x           &&
       y           == other.y           &&
       xP          == other.xP          &&
       done        == other.done        &&
       xC          == other.xC          &&
       yG          == other.yG          &&
       yL          == other.yL          &&
       memcmp(&this->info,&other.info,sizeof(struct ShapeIteratorInfo)) == 0)
      return true;
   return false;
}

bool EllipseIterator::operator!=(const EllipseIterator & other) {
   return !operator==(other);
}

int & EllipseIterator::operator*() {
   return *info.val;
}

struct ShapeIteratorInfo & EllipseIterator::locinfo() {
   return info;
}
