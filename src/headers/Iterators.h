
#ifndef KUROME_SHAPE_ITERATORS

#define KUROME_SHAPE_ITERATORS
#include "Grid.h"

struct ShapeIteratorInfo { double posx; double posy; int * val; };

#if !KUROME_NOROTATION_RECT && !KUROME_NOROTATION

class RectIterator {
private:
   double shiftxx, shiftxy, shiftyx, shiftyy, tshiftxx, tshiftxy;
   double srtx, srty, endx, endy, posx, posy;
   int steps, stepx, stepy;
   Grid * g;
   std::set<uint64_t> seen;
   struct ShapeIteratorInfo info;
   void setupVars(double, double, double, double, double);
public:
   bool done;
   RectIterator() {};
   RectIterator(double, double, double, double, Grid *);
   RectIterator(Entity *, Grid *);
   RectIterator(double, double, Entity *, Grid *);
   RectIterator(double, double, double, Entity *, Grid *);
   RectIterator(const RectIterator &);
   RectIterator& operator++();
   RectIterator& operator++(int);
   bool operator==(const RectIterator &);
   bool operator!=(const RectIterator &);
   int & operator*();
   struct ShapeIteratorInfo & locinfo();
};

#else

class RectIterator {
private:
   Grid * g;
   int srtx, srty;
   int posx, endx;
   int posy, endy;
   struct ShapeIteratorInfo info;
public:
   bool done;
   RectIterator() {};
   RectIterator(double, double, double, double, Grid *);
   RectIterator(Entity *, Grid *);
   RectIterator(double, double, Entity *, Grid *);
   RectIterator(double, double, double, Entity *, Grid *);
   RectIterator(const RectIterator &);
   RectIterator& operator++();
   RectIterator& operator++(int);
   bool operator==(const RectIterator &);
   bool operator!=(const RectIterator &);
   int & operator*();
   struct ShapeIteratorInfo & locinfo();
};

#endif

#if !KUROME_NOROTATION && !KUROME_NOROTATION_ELPS 

class EllipseIterator {
private:
   double shiftx, shifty, granularity;
   double k, wrad, hrad, xs, ys, xe, ye;
   double rot, srr, crr, offx, offy;
   Grid * g;
   std::set<uint64_t> seen;
   struct ShapeIteratorInfo info;
   void setupVars(double, double, double, double, double);
public:
   bool done;
   EllipseIterator() {};
   EllipseIterator(double, double, double, double, Grid *);
   EllipseIterator(Entity *, Grid *);
   EllipseIterator(double, double, Entity *, Grid *);
   EllipseIterator(double, double, double, Entity *, Grid *);
   EllipseIterator(const EllipseIterator &);
   EllipseIterator& operator++();
   EllipseIterator& operator++(int);
   bool operator==(const EllipseIterator &);
   bool operator!=(const EllipseIterator &);
   int & operator*();
   struct ShapeIteratorInfo & locinfo();
};

#else 

class EllipseIterator {
private:
   Grid * g;
   double granularity;
   double k, wrad, hrad, x, y;
   int xP, xC, yG, yL;
   struct ShapeIteratorInfo info;
public:
   bool done;
   EllipseIterator() {};
   EllipseIterator(double, double, double, double, Grid *);
   EllipseIterator(Entity *, Grid *);
   EllipseIterator(double, double, Entity *, Grid *);
   EllipseIterator(double, double, double, Entity *, Grid *);
   EllipseIterator(const EllipseIterator &);
   EllipseIterator& operator++();
   EllipseIterator& operator++(int);
   bool operator==(const EllipseIterator &);
   bool operator!=(const EllipseIterator &);
   int & operator*();
   struct ShapeIteratorInfo & locinfo();
};

#endif

#endif
