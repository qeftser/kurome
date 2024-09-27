

#ifndef KUROME_GRID_CLASS

#define KUROME_GRID_CLASS

#include <Eigen/Dense>
#include <random>
#include <set>

class Entity;
class Agent;
class Sample;

class Grid {
private:
   int32_t                    blocksXmax;
   int32_t                    blocksYmax;
   int32_t                    blocksXmin;
   int32_t                    blocksYmin;
   double                     sizeXmax;
   double                     sizeYmax;
   double                     sizeXmin;
   double                     sizeYmin;
   double                     unitSize;
   Eigen::MatrixXi            blocks;
   std::default_random_engine generator;
   std::set<Entity *>         entities;
   std::set<Sample *>         samples;

public:

   Grid(double,double,double);
   Grid(double,double,double,double,double);
   Grid(struct grid_struct *);

   /* normal operations */

   bool inBounds(double,double);
   bool inBounds(int,int);

   int  getIdx(double,double);
   int  setIdx(double,double,int);
   int  setIdx(double,double,int,double);

   int  addEntity(Entity *);
   int  chgEntity(Entity *);
   int  remEntity(Entity *);
   int  apply(Sample *);

   int  avgWeights(int,int);
   int  avgWeights(int,int,double);

   void clear();
   void smooth();
   void redraw();
   void clense();
   int  changeUnitSize(double);
   int  changeSizeXmax(double);
   int  changeSizeYmax(double);
   int  changeSizeXmin(double);
   int  changeSizeYmin(double);

   void  info();
   void  print();

   /* simulation/alt/helper operations */

   int             getIdx(double,double,double);
   Eigen::MatrixXi slice(int,int,int,int);
   Eigen::MatrixXi slice(double,double,double,double);
   
   double getUnitSize();
   int *  getIdxPtr(int,int);
   int *  getIdxPtr(double,double);
   int    getHighBlocks();

   int root(double);
   int roob(double);
   int roor(double);

   int toStruct(struct grid_struct **);

   friend class Reporter;
};

#endif

