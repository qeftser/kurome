

#ifndef KUROME_GRID_CLASS

#define KUROME_GRID_CLASS

#include <Eigen/Dense>
#include <random>
#include <set>

class Entity;
class Agent;
class Sample;

/*! \class Grid
 * Class for representation of the enviroment. Accepts an arbitrary resolution and 
 * supports positive and negative positions. Allows Entity and Sample classes to
 * be added to it.
 */
class Grid {
private:
   int32_t                    blocksXmax; /**< the largest X block that can be indexed, plus one */
   int32_t                    blocksYmax; /**< the largest Y block that can be indexed, plus one */
   int32_t                    blocksXmin; /**< the smallest X block that can be indexed */
   int32_t                    blocksYmin; /**< the smallest Y block that can be indexed */
   double                     sizeXmax;   /**< the floating point max X size of the grid */
   double                     sizeYmax;   /**< the floating point max Y size of the grid */
   double                     sizeXmin;   /**< the floating point min X size of the grid */
   double                     sizeYmin;   /**< the floating point min Y size of the grid */
   double                     unitSize;   /**< the resolution of the Grid. The size of the length/width 
                                               of one Grid block */
   std::default_random_engine generator;  /**< a random generator that gets used in the noisy getIdx */

public:
   Eigen::MatrixXi            blocks;     /**< 2d Integer Matrix of weights at each position on the Grid */
   std::set<Entity *>         entities;   /**< set of all Entity objects currently applied to the Grid */
   std::set<Sample *>         samples;    /**< set of all Sample objects currently applied to the Grid */

   /** 
    * Construct a Grid with the given xsize, ysize, and unit size
    */
   Grid(double,double,double);
   /**
    * Construct a Grid with the given minimum and maximum x and y sizes, and the given unit size
    */
   Grid(double,double,double,double,double);
   /**
    * Construct a Grid with the provided grid_struct
    */
   Grid(struct grid_struct *);

   /* normal operations */

   /**
    * Check if the provided index is in the Grid bounds
    */
   bool inBounds(double,double);
   /**
    * Check if the provided block is in the Grid bounds
    */
   bool inBounds(int,int);

   /**
    * Return the value at the given position
    */
   int  getIdx(double,double);
   /**
    * Set the value at the given position
    */
   int  setIdx(double,double,int);
   /**
    * Set the value at the given position based on
    * the previous position weighted with the new one
    */
   int  setIdx(double,double,int,double);

   /**
    * Add the given Entity object to the Grid
    */
   int  addEntity(Entity *);
   /**
    * Note that the provided Entity has changed
    * in the Grid
    */
   int  chgEntity(Entity *);
   /**
    * Remove the given Entity from the Grid
    */
   int  remEntity(Entity *);
   /**
    * Apply the given Sample to the grid. This
    * uses the default weight function, which happens
    * to be whatever I felt like the last time I commited :)
    */
   int  apply(Sample *);
   /**
    * Apply the given Sample to the grid, using
    * the provided function to produce the new value
    * given the Sample one and the previous one.
    */
   int  apply(Sample *, int(*)(int,int));

   /**
    * Collect the result of a Sample's application and
    * return it in a new sample. This should be called
    * after the given sample has been applied
    */
   Sample application(Sample *);

   /**
    * The default averaging function
    */
   static int  avgWeights(int,int);
   /**
    * Averages two values based on the given
    * weighting. The equation is (w1 * weight) + (w2 * (1 - weight))
    */
   int  avgWeights(int,int,double);

   /**
    * clear the Grid
    */
   void clear();
   /**
    * set each Grid value as the average of
    * itself and the values surrounding it
    */
   void smooth();
   /**
    * clear the grid and reapply all Entity and Sample
    * objects to it
    */
   void redraw();
   /**
    * clear the Grid and remove all held Sample and 
    * Entity objects
    */
   void clense();
   /**
    * change the unitSize private variable, adjusting
    * the size of the array and reapplying all Entity and
    * Sample objects in the process
    */
   int  changeUnitSize(double);
   /**
    * change the max X size. Results in a change to the 
    * underlying array size and a redraw
    */
   int  changeSizeXmax(double);
   /**
    * change the max Y size. Results in a change to the 
    * underlying array size and a redraw
    */
   int  changeSizeYmax(double);
   /**
    * change the min X size. Results in a change to the 
    * underlying array size and a redraw
    */
   int  changeSizeXmin(double);
   /**
    * change the min Y size. Results in a change to the 
    * underlying array size and a redraw
    */
   int  changeSizeYmin(double);

   /**
    * print some information about the Grid
    */
   void  info();
   /**
    * print the values of the Grid matrix
    */
   void  print();

   /* simulation/alt/helper operations */

   /**
    * Return the value at the given position with
    * the given error added on
    */
   int             getIdx(double,double,double);
   /**
    * Return a porition of the Grid as a new matrix.
    * This still isn't implimented I think
    */
   Eigen::MatrixXi slice(int,int,int,int);
   /**
    * Return a porition of the Grid as a new matrix.
    * This still isn't implimented I think
    */
   Eigen::MatrixXi slice(double,double,double,double);
   
   /**
    * return the unit size of the Grid
    */
   double getUnitSize();
   /**
    * return a pointer to the value at a
    * given Grid index.
    */
   int *  getIdxPtr(int,int);
   /**
    * return a pointer to the value at a
    * given Grid index.
    */
   int *  getIdxPtr(double,double);
   /**
    * Get the largest block number, either the X or Y amount
    */
   int    getHighBlocks();

   /**
    * Return the ceiling of the given value divided by the unit size */
   int root(double);
   /**
    * Return the floor of the given value divided by the unit size */
   int roob(double);
   /**
    * Return the rounding of the given value divided by the unit size */
   int roor(double);

   /**
    * reallocate the pointer of the provided struct and fill it with
    * the data from this Grid. The passed pointer reference must be 
    * to a malloc'ed pointer otherwise the program will crash.
    */
   int toStruct(struct grid_struct **);
};

#endif

