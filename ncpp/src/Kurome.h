
#include <iostream>
#include <cmath>
#include <random>
#include <set>
#include <iterator>
#include <eigen-3.4.0/Eigen/Dense>

class Entity;

extern int errno_kurome;
#define errnok errno_kurome

/* pi/2 */
#define PI_1l2 1.57079632679
/* 3pi/2 */
#define PI_3l2 4.71238898038
/* yes there is a 
 * predefined pi but
 * I am too lazy to 
 * figure out how to use it...
 */
#define PI 3.14159265358

/* All good */
#define KUROME_EOK      0;
/* Indexing outside of grid bounds */
#define KUROME_ERANGE   1;
/* Percent value not used */
#define KUROME_EPERCENT 2;
/* You know what you did */
#define KUROME_EBADNUM  3;
/* Incorrect shape */
#define KUROME_ETYPE    4;

/* rectangle */
#define KUROME_TYPE_RECT 1
/* ellipse */
#define KUROME_TYPE_ELPS 2
/* point */
#define KUROME_TYPE_PONT 3

class Grid {
private:
   Eigen::MatrixXi            blocks;
   int32_t                    blocksX;
   int32_t                    blocksY;
   double                     sizeX;
   double                     sizeY;
   double                     unitSize;
   std::set<Entity *>         entities;
   std::default_random_engine generator;
   

public:

   Grid(double,double,double);

   /* normal operations */

   bool inBounds(double,double);
   bool inBounds(int,int);

   int  getIdx(double,double);
   int  setIdx(double,double,int);
   int  setIdx(double,double,int,double);

   int  addEntity(Entity *);
   int  remEntity(Entity *);

   int  avgWeights(int,int);
   int  avgWeights(int,int,double);

   void clear();
   void smooth();
   int  changeUnitSize(double);
   int  changeSizeX(double);
   int  changeSizeY(double);

   void  info();
   void  print();

   /* simulation/alt/helper operations */

   int             getIdx(double,double,double);
   Eigen::MatrixXi slice(int,int,int,int);
   Eigen::MatrixXi slice(double,double,double,double);
   
   double          getUnitSize();
   int *           getIdxPtr(int,int);
   int *           getIdxPtr(double,double);
   int             getXBlocks();

   int root(double);
   int roob(double);
   int roor(double);
};

class RectIterator : public std::iterator<std::input_iterator_tag,int> {
private:
   Grid * g;
   int srtx, srty;
   int posx, endx;
   int posy, endy;
   int * val;
public:
   bool done;
   RectIterator(double, double, double, double, Grid *);
   RectIterator(Entity *, Grid *);
   RectIterator(const RectIterator &);
   RectIterator& operator++();
   RectIterator& operator++(int);
   bool operator==(const RectIterator &);
   bool operator!=(const RectIterator &);
   int & operator*();
};

class EllipseIterator : public std::iterator<std::input_iterator_tag,int> {
private:
   Grid * g;
   double granularity;
   double k, wrad, hrad, x, y;
   int xP, xC, yG, yL;
   int * val;
public:
   bool done;
   EllipseIterator(double, double, double, double, Grid *);
   EllipseIterator(Entity *, Grid *);
   EllipseIterator(const EllipseIterator &);
   EllipseIterator& operator++();
   EllipseIterator& operator++(int);
   bool operator==(const EllipseIterator &);
   bool operator!=(const EllipseIterator &);
   int & operator*();
};

class Entity {
public:
   double posx;
   double posy;
   double xwid;
   double ywid;
   int    type;
   int    val;
   
   Entity(double, double, double, double, int, int);
   Entity(double, double, int);
};

