
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
   double crr, srr;
   double ssx, ssy;
   
   Sample(Entity e, Eigen::MatrixXi m, double d) 
      : orgin(e), values(m), unitSize(d) { 
         values.setZero(); 
         srr = sinl((e.rot)*(3.14159265358/180.0));
         crr = cosl((e.rot)*(3.14159265358/180.0));
         ssx = (e.xwid/2);
         ssy = (e.ywid/2);
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
      null = 0;
      x -= orgin.posx;
      y -= orgin.posy;
      printf("ssx: %f ssy: %f\n",ssx,ssy);
      double temp;
      temp = ((x * crr) + (y * srr)) + ssx;
      y = ((y * crr) - (x * srr)) + ssy;
      x = temp;
      printf("\033[32m%f %f\033[0m\n",x,y);
      int xp = (int)std::round(x/unitSize);
      int yp = (int)std::round(y/unitSize);
      printf("\033[33m%d %d\033[0m\n",xp,yp);
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

