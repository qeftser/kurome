
#ifndef KUROME_SHAPE_ITERATORS

#define KUROME_SHAPE_ITERATORS
#include "Grid.h"

struct ShapeIteratorInfo { double posx; double posy; int * val; };

class RectIterator : public std::iterator<std::input_iterator_tag,int> {
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
   RectIterator(const RectIterator &);
   RectIterator& operator++();
   RectIterator& operator++(int);
   bool operator==(const RectIterator &);
   bool operator!=(const RectIterator &);
   int & operator*();
   struct ShapeIteratorInfo & locinfo();
};

class EllipseIterator : public std::iterator<std::input_iterator_tag,int> {
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
   EllipseIterator(const EllipseIterator &);
   EllipseIterator& operator++();
   EllipseIterator& operator++(int);
   bool operator==(const EllipseIterator &);
   bool operator!=(const EllipseIterator &);
   int & operator*();
   struct ShapeIteratorInfo & locinfo();
};

#endif
