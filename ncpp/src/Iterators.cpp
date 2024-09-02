
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
   val        = NULL;
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
   this->val  = other.val;
   this->done = other.done;
}

RectIterator::RectIterator(Entity * e, Grid * g) {
#ifndef KUROME_NOCHECK
   if (e->type != KUROME_TYPE_RECT) {
      errnok = KUROME_ETYPE;
      val = NULL;
      done = true;
      return;
   }
#endif
   this->g    = g;
   this->srtx = g->roob(e->xpos-(e->xwid/2.0));
   this->endx = g->root(e->xpos+(e->xwid/2.0));
   this->srty = g->roob(e->ypos-(e->ywid/2.0));
   this->endy = g->root(e->ypos+(e->ywid/2.0));
   this->posx = srtx;
   this->posy = srty-1;
   val        = NULL;
   done       = false;
   operator++();
}

RectIterator & RectIterator::operator++() {
   do {
      ++posy;
      if (posy > endy) {
         ++posx;
         posy = srty;
         if (posx > endx) {
            done = true;
            val = NULL;
            return *this;
         }
      }
   } while (!g->inBounds(posx,posy));
   val = g->getIdxPtr(posx,posy);
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
   return *val;
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
   this->val         = NULL;
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
   this->val         = other.val;
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
      val = NULL;
      return;
   }
#endif
   this->g           = g;
   this->granularity = g->getUnitSize()/g->getXBlocks();
   this->k           = -granularity;
   this->wrad        = e->xwid;
   this->hrad        = e->ywid;
   this->x           = e->xpos;
   this->y           = e->ypos;
   this->xP          = -999;
   this->val         = NULL;
   this->done        = false;
   this->xC          = -999;
   this->yG          = 0;
   this->yL          = 0;
}

EllipseIterator & EllipseIterator::operator++() {
   do {
      ++yL;
      if (yL > yG) {
         do {
            xP = xC;
            k+=granularity;
            if (k > PI) {
               done = true;
               val = NULL;
               return *this;
            }
            xC = g->roor(x + (cos(k) * wrad));
            yG = g->root(y + (sin(k) * hrad));
            yL = g->roob(y + (sin(k+PI) * hrad))-1;
         } while (xC == xP);
      }
   } while (!g->inBounds(xC,yL));
   val = g->getIdxPtr(xC,yL);
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
       val         == other.val         &&
       done        == other.done        &&
       xC          == other.xC          &&
       yG          == other.yG          &&
       yL          == other.yL)
      return true;
   return false;
}

bool EllipseIterator::operator!=(const EllipseIterator & other) {
   return !operator==(other);
}

int & EllipseIterator::operator*() {
   return *val;
}

