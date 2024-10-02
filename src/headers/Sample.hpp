
#ifndef KUROME_CLASS_SAMPLE

#define KUROME_CLASS_SAMPLE
#include "Entity.hpp"
#include <cmath>
#include <Eigen/Dense>

class Sample {
public:
   Entity          orgin;
   Eigen::MatrixXi values;
   double unitSize;
   int crr, srr;
   int ssx, ssy;
   
   Sample(Entity e, Eigen::MatrixXi m, double d) 
      : orgin(e), values(m), unitSize(d) { 
         values.setZero(); 
         srr = sin((-e.rot)*(180.0/3.14159265358));
         crr = cos((-e.rot)*(180.0/3.14159265358));
         ssx = (e.posx-(e.xwid/2));
         ssy = (e.posy-(e.ywid/2));
      };
   Sample() 
      : orgin(Entity(0,0,0,0,0,0)), values(Eigen::MatrixXi(0,0)), unitSize(0) {}
   Sample(struct sample_struct * s) 
   : orgin(&s->orgin), values(Eigen::MatrixXi(s->blocksX,s->blocksY)), unitSize(s->unitSize) {
      for (int i = 0; i < s->blocksX*s->blocksY; ++i)
         values(i/s->blocksX,i%s->blocksX) = s->matrix[i];
   }

   /* 
    * Map 'realspace' coordinates into the matrix of 
    * Sample data.
    */
   int & localVal(double x, double y) {
      static int null = 0;
      null = -1;
      x -= ssx;
      y -= ssy;
      x = (x * crr) - (y * srr);
      y = (x * srr) + (y * crr);
      int xp = (int)std::round(x/unitSize);
      int yp = (int)std::round(y/unitSize);
      if (xp >= values.rows() || xp < 0 ||
          yp >= values.cols() || yp < 0)
         return null;
      return values(xp,yp);
   }

   /*
    * Reallocate and populate the given
    * sample_struct with the data from
    * me, the Sample.
    */
   int toStruct(struct sample_struct ** s) {
      (*s)->unitSize = unitSize;
      orgin.toStruct(&(*s)->orgin);
      (*s)->blocksX = values.rows();
      (*s)->blocksY = values.cols();
      int size = sizeof(sample_struct)+(sizeof(int)*(*s)->blocksX*(*s)->blocksY);
      *s = (struct sample_struct *)realloc(*s,size);
      for (int i = 0; i < (*s)->blocksX*(*s)->blocksY; ++i)
         (*s)->matrix[i] = values(i/(*s)->blocksX,i%(*s)->blocksX);
      return size;
   }
};

#endif
