
#include "Kurome.h"
#include <random>

Grid * Generator::sparse(Grid * tomap, int elements, double avgSize, double maxSize) {
   struct partial_grid_struct pgs;
   tomap->collectInfo(&pgs);
   double hxval = pgs.sizeXmax-pgs.sizeXmin;
   double hyval = pgs.sizeYmax-pgs.sizeYmin;
   
   double minSize = avgSize-(maxSize-avgSize);
   std::uniform_real_distribution<double> unif(minSize,maxSize);
   std::default_random_engine re;

   int val = KUROME_NOGO_COST;
   for (int i = 0; i < elements; ++i) {
      int type = 1+random()%2;
      double xpos = std::fmod(random()*pgs.unitSize,hxval);
      double ypos = std::fmod(random()*pgs.unitSize,hyval);
      double xwid = unif(re);
      double ywid = unif(re);
      tomap->addEntity(new Entity(xpos,ypos,xwid,ywid,type,val));
   }

   return tomap;
}


Grid * Generator::dune(Grid * tomap, int elements, double avgSize, double maxSize) {
   struct partial_grid_struct pgs;
   tomap->collectInfo(&pgs);
   double hxval = pgs.sizeXmax-pgs.sizeXmin;
   double hyval = pgs.sizeYmax-pgs.sizeYmin;
   
   double minSize = avgSize-(maxSize-avgSize);
   std::uniform_real_distribution<double> unif(minSize,maxSize);
   std::default_random_engine re;

   int val = KUROME_NOGO_COST;
   for (int i = 0; i < elements; ++i) {
      int type = 1+random()%2;
      double xpos = std::fmod(random()*pgs.unitSize,hxval);
      double ypos = std::fmod(random()*pgs.unitSize,hyval);
      double xwid = unif(re);
      double ywid = unif(re);
      tomap->addEntity(new Entity(xpos,ypos,xwid,ywid,type,val));
   }

   for (int i = 0; i < tomap->getHighBlocks()/10; ++i)
      tomap->smooth();

   return tomap;
}

/**
 * Not currently supported :/
 */
Grid * Generator::maze(Grid * tomap, int density, double minwid) {
   return tomap;
}
