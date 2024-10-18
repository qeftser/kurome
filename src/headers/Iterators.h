
#ifndef KUROME_SHAPE_ITERATORS

#define KUROME_SHAPE_ITERATORS
#include "Grid.h"

/**
 * Information about the current state of the given iterator
 */
struct ShapeIteratorInfo { double posx; double posy; int * val; };

#if !KUROME_NOROTATION_RECT && !KUROME_NOROTATION

/*! \class RectIterator
 * An iterator that traces over the area of a rectangle in the provided Grid.
 * Will skip over any out of bounds data when iterating.
 */
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
   bool done; /**< indicates whether the iterator has finished */
   /** construct a new RectIterator */
   RectIterator() {}; 
   /** construct a new RectIterator over the given bounds in the
    * provided Grid.*/
   RectIterator(double, double, double, double, Grid *);
   /** construct a new RectIterator over the area and position of the
    * provided Entity on the given Grid. */
   RectIterator(Entity *, Grid *);
   /** construct a new RectIterator over the area of the provided Entity on the
    * given Grid. The Entity position is overriden by the provided values */
   RectIterator(double, double, Entity *, Grid *);
   /** construct a new RectIterator over the area of the provided Entity on the
    * given Grid. The Entity position and rotation is overriden by the provided values */
   RectIterator(double, double, double, Entity *, Grid *);
   /** construct a RectIterator as a copy of another one */
   RectIterator(const RectIterator &);
   /** incriment the RectIterator to the next value */
   RectIterator& operator++();
   /** incriment the RectIterator to the next value */
   RectIterator& operator++(int);
   /** check the equality of two RectIterators */
   bool operator==(const RectIterator &);
   /** check the equality of two RectIterators */
   bool operator!=(const RectIterator &);
   /** return a reference to the value at the current position of
    * the RectIterator. The iterator will never pass over the same
    * value twice. */
   int & operator*();
   /** return the ShapeIteratorInfo struct associated with this RectIterator.
    * This struct has a lot of useful stuff */
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
   double shiftx, shifty, granularity, ussq;
   double k, wrad, hrad, xs, ys, xe, ye;
   double rot, srr, crr, offx, offy;
   Grid * g;
   std::set<uint64_t> seen;
   struct ShapeIteratorInfo info;
   void setupVars(double, double, double, double, double);
public:
   bool done; /**< indicates whether the iterator has finished */
   /** construct a new EllipseIterator */
   EllipseIterator() {};
   /** construct a new EllipseIterator over the given bounds in the
    * provided Grid. */
   EllipseIterator(double, double, double, double, Grid *);
   /** construct a new EllipseIterator over the area and position of the
    * provided Entity on the given Grid. */
   EllipseIterator(Entity *, Grid *);
   /** construct a new EllipseIterator over the area of the provided Entity on the
    * given Grid. The Entity position is overriden by the provided values */
   EllipseIterator(double, double, Entity *, Grid *);
   /** construct a new EllipseIterator over the area of the provided Entity on the
    * given Grid. The Entity position and rotation is overriden by the provided values */
   EllipseIterator(double, double, double, Entity *, Grid *);
   /** construct a EllipseIterator as a copy of another one */
   EllipseIterator(const EllipseIterator &);
   /** incriment the EllipseIterator to the next value */
   EllipseIterator& operator++();
   /** incriment the EllipseIterator to the next value */
   EllipseIterator& operator++(int);
   /** check the equality of two EllipseIterators */
   bool operator==(const EllipseIterator &);
   /** check the equality of two EllipseIterators */
   bool operator!=(const EllipseIterator &);
   /** return a reference to the value at the current position of
    * the EllipseIterator. The iterator will never pass over the same
    * value twice. */
   int & operator*();
   /** return the ShapeIteratorInfo struct associated with this EllipseIterator.
    * This struct has a lot of useful stuff */
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
