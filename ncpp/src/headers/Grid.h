

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
   Eigen::MatrixXi            blocks;
   int32_t                    blocksX;
   int32_t                    blocksY;
   double                     sizeX;
   double                     sizeY;
   double                     unitSize;
   std::default_random_engine generator;
   std::set<Entity *>         entities;
   std::set<Sample *>         samples;

public:

   Grid(double,double,double);
   Grid(struct grid_struct *);

   /* normal operations */

   bool inBounds(double,double);
   bool inBounds(int,int);

   int  getIdx(double,double);
   int  setIdx(double,double,int);
   int  setIdx(double,double,int,double);

   int  addEntity(Entity *);
   int  remEntity(Entity *);
   int  apply(Sample *);

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

   int toStruct(struct grid_struct **);
};

#endif

