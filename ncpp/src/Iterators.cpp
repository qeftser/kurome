
#include "Kurome.h"

int errno_kurome;

#define KUROME_NOROTATION_RECT 0

#if !KUROME_NOROTATION_RECT && !KUROME_NOROTATION

RectIterator::RectIterator(double sx, double ex, double sy, double ey, Grid * g) {
   this->g = g;
   double xwid2 = (ex-sx)/2.0;
   double ywid2 = (ey-sy)/2.0;
   steps = std::ceil((xwid2>ywid2?(xwid2*2):(ywid2*2))/(g->getUnitSize()/2));
   this->srtx = sx + xwid2;
   this->srty = sy + ywid2;
   shiftyx = 0; shiftyy = (ey - sy)/(double)steps;
   tshiftxy = shiftxy = 0; 
   tshiftxx = shiftxx = (ex - sx)/(double)steps;
   posx = srtx;
   posy = srty - shiftyy;
   stepx = 0;
   stepy = -1;
   done = false;
   operator++();
}

void RectIterator::setupVars(double xwid, double ywid, double offx, double offy, double rot) {
   double xwid2 = xwid/2.0;
   double ywid2 = ywid/2.0;
   double RAD_rot = rot*DEG_RAD;
   double srr = sinl(RAD_rot);
   double crr = cosl(RAD_rot);
   srtx = ((-xwid2)*crr-ywid2*srr);
   srty = ((-xwid2)*srr+ywid2*crr);
   endx = (xwid2*crr+ywid2*srr);
   endx = (xwid2*srr-ywid2*crr);
   shiftxx = (xwid2*crr-ywid2*srr);
   shiftxy = (xwid2*srr+ywid2*crr);
   shiftyx = ((-xwid2)*crr+ywid2*srr);
   shiftyy = ((-xwid2)*srr-ywid2*crr);
   steps = std::ceil((xwid>ywid?xwid:ywid)/(g->getUnitSize()/2));
   tshiftxx = shiftxx = (shiftxx - srtx)/(double)steps;
   tshiftxy = shiftxy = (shiftxy - srty)/(double)steps;
   shiftyx = (shiftyx - srtx)/(double)steps;
   shiftyy = (shiftyy - srty)/(double)steps;
   srtx += (offx);
   srty += (offy);
   posx = srtx - shiftyx;
   posy = srty - shiftyy;
   printf("xx: %f xy: %f yx: %f yy: %f\n",shiftxx,shiftxy,shiftyx,shiftyy); 
}

RectIterator::RectIterator(Entity * e, Grid * g) 
   : stepx(0), stepy(-1), g(g), done(false) {
#ifndef KUROME_NOCHECK
   if (e->type != KUROME_TYPE_RECT) {
      errnok = KUROME_ETYPE;
      info.val = NULL;
      done = true;
      return;
   }
#endif
   setupVars(e->xwid,e->ywid,e->posx,e->posy,e->rot);
   operator++();
}

RectIterator::RectIterator(double offx, double offy, Entity * e, Grid * g) 
   : stepx(0), stepy(-1), g(g), done(false) {
#ifndef KUROME_NOCHECK
   if (e->type != KUROME_TYPE_RECT) {
      errnok = KUROME_ETYPE;
      info.val = NULL;
      done = true;
      return;
   }
#endif
   setupVars(e->xwid,e->ywid,offx,offy,e->rot);
   operator++();
}

RectIterator::RectIterator(double offx, double offy, double rot, Entity * e, Grid * g)
   : stepx(0), stepy(-1), g(g), done(false) {
#ifndef KUROME_NOCHECK
   if (e->type != KUROME_TYPE_RECT) {
      errnok = KUROME_ETYPE;
      info.val = NULL;
      done = true;
      return;
   }
#endif
   setupVars(e->xwid,e->ywid,offx,offy,rot);
   operator++();
}

RectIterator::RectIterator(const RectIterator & other)
   : shiftxx(other.shiftxx), shiftxy(other.shiftxy), shiftyx(other.shiftyx),
     shiftyy(other.shiftyy), tshiftxx(other.tshiftxx), tshiftxy(other.tshiftxy),
     srtx(other.srtx), srty(other.srty), endx(other.endx), endy(other.endy), 
     posx(other.posx), posy(other.posy), steps(other.steps), stepx(other.stepx), 
     stepy(other.stepy), g(other.g), seen(other.seen), info(other.info) {}

RectIterator & RectIterator::operator++() {
   do {
      ++stepy;
      posx += shiftyx;
      posy += shiftyy;
      if (stepy > steps) {
         stepy = 0;
         ++stepx;
         posx = (srtx + tshiftxx);
         posy = (srty + tshiftxy);
         tshiftxx += shiftxx;
         tshiftxy += shiftxy;
         if (stepx > steps) { 
            done = true;
            info.val = NULL;
            return *this;
         }
      }
   } while (!g->inBounds(posx,posy) || seen.count((((uint64_t)g->roor(posx))<<32)|(g->roor(posy)&UINT_MAX)));
   info.posy = (g->roor(posy)*g->getUnitSize())+(g->getUnitSize()/2.0000001);
   info.posx = (g->roor(posx)*g->getUnitSize())+(g->getUnitSize()/2.0000001);
   info.val = g->getIdxPtr(posx,posy);
   seen.insert((((uint64_t)g->roor(posx))<<32)|(g->roor(posy)&UINT_MAX));
   return *this;
}

#else 

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

RectIterator::RectIterator(double offx, double offy, double rot, Entity * e, Grid * g) {
   (void)rot;
   RectIterator(offx,offy,e,g);
}

#endif

bool RectIterator::operator==(const RectIterator & other) {
   if (g == other.g &&
         !memcmp(&info,&other.info,sizeof(struct ShapeIteratorInfo)))
      return true;
   return false;
}

RectIterator & RectIterator::operator++(int assign) {
   (void)assign;
   operator++();
   return *this;
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

#if !KUROME_NOROTATION && !KUROME_NOROTATION_ELPS 

void EllipseIterator::setupVars(double xwid, double ywid, double offx, double offy, double rot) {
   wrad = xwid/2.0;
   hrad = ywid/2.0;
   this->rot = rot*DEG_RAD;
   srr = sinl(this->rot);
   crr = cosl(this->rot);
   xs = ((-wrad)*crr-hrad*srr);
   ys = ((-wrad)*srr+hrad*crr);
   shiftx = ((-wrad)*crr+hrad*srr);
   shifty = ((-wrad)*srr-hrad*crr);
   double steps = std::ceil((xwid>ywid?xwid:ywid)/(g->getUnitSize()/2));
   shiftx = (shiftx - xs)/steps;
   shifty = (shifty - ys)/steps;
   granularity = g->getUnitSize();
   k = -granularity;
   xs = xe = 0.0;
   ys = ye = 0.0;
   this->offx = offx;
   this->offy = offy;
}

EllipseIterator::EllipseIterator(double wrad, double hrad, double x, double y, Grid * g)
   : g(g), done(false) {
   setupVars(wrad,hrad,x,y,0.0);
   operator++();
}

EllipseIterator::EllipseIterator(Entity * e, Grid * g) 
   : g(g), done(false) {
   setupVars(e->xwid,e->ywid,e->posx,e->posy,e->rot);
   operator++();
}

EllipseIterator::EllipseIterator(double offx, double offy, Entity * e, Grid * g) 
   : g(g), done(false) {
   setupVars(e->xwid,e->ywid,offx,offy,e->rot);
   operator++();
}

EllipseIterator::EllipseIterator(double offx, double offy, double rot, Entity * e, Grid * g) 
   : g(g), done(false) {
   setupVars(e->xwid,e->ywid,offx,offy,rot);
   operator++();
}

EllipseIterator & EllipseIterator::operator++(void) {
   do {
      ys -= shifty;
      xs -= shiftx;
      if ((ye - ys) < 0 || (xe - xs) < 0) {
         k+=granularity;
         if (k > PI) {
            done = true;
            info.val = NULL;
            return *this;
         }
         double temp = 0;
         xs = ((cosl(k) * wrad));
         temp = ((sinl(k+PI) * hrad));
         ye = ((sinl(k) * hrad));
         xe = (xs * crr - ye * srr) + offx;
         ye = (xs * srr + ye * crr) + offy;
         ys = (xs * srr + temp * crr) + offy;
         xs = (xs * crr - temp * srr) + offx;
      }
   } while (!g->inBounds(xs,ys) || seen.count((((uint64_t)g->roor(xs))<<32|(g->roor(ys)&UINT_MAX))));
   info.posy = (g->roor(ys)*g->getUnitSize())+(g->getUnitSize()/2.0000001);
   info.posx = (g->roor(xs)*g->getUnitSize())+(g->getUnitSize()/2.0000001);
   info.val = g->getIdxPtr(xs,ys);
   seen.insert((((uint64_t)g->roor(xs))<<32)|(g->roor(ys)&UINT_MAX));
   return *this;
}

#else

EllipseIterator::EllipseIterator(double wrad, double hrad, double x, double y, Grid * g) {
   this->g           = g;
   this->granularity = g->getUnitSize()/g->getHighBlocks();
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
   this->granularity = g->getUnitSize()/g->getHighBlocks();
   this->k           = -granularity;
   this->wrad        = e->xwid/2.0;
   this->hrad        = e->ywid/2.0;
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
   this->granularity = g->getUnitSize()/g->getHighBlocks();
   this->k           = -granularity;
   this->wrad        = e->xwid/2.0;
   this->hrad        = e->ywid/2.0;
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

EllipseIterator::EllipseIterator(double offx, double offy, double rot, Entity * e, Grid * g) {
   (void)rot;
   EllipseIterator(offx,offy,e,g);
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

#endif

EllipseIterator & EllipseIterator::operator++(int assign) {
   (void)assign;
   operator++();
   return *this;
}

bool EllipseIterator::operator==(const EllipseIterator & other) {
   if (g           == other.g           &&
       !memcmp(&this->info,&other.info,sizeof(struct ShapeIteratorInfo)))
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

