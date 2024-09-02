
#include "Kurome.h"
#include <stdio.h>

extern int errno_kurome;

Grid::Grid(double unitSize, double sizeX, double sizeY) {
   this->unitSize = unitSize;
   this->sizeX    = sizeX;
   this->sizeY    = sizeY;
   this->blocksX  = root(sizeX);
   this->blocksY  = root(sizeX);
   this->blocks = Eigen::MatrixXi(this->blocksX,this->blocksY);
   this->entities = std::vector<Entity *>();
   this->generator = std::default_random_engine(clock());
   clear();
}

void Grid::clear() {
   for (int i = 0; i < blocksX; ++i)
      for (int j = 0; j < blocksY; ++j)
         blocks(i,j) = 0;
}

/* kinda slow ... */
void Grid::smooth() {
   for (int i = 1; i < blocksX-1; ++i)
      for (int j = 1; j < blocksY-1; ++j)
         blocks(i,j) = (blocks(i,j)+blocks(i+1,j)+blocks(i-1,j)+blocks(i,j+1)+blocks(i,j-1)+
                        blocks(i+1,j+1)+blocks(i-1,j+1)+blocks(i+1,j-1)+blocks(i-1,j-1)) / 9;
}

bool Grid::inBounds(double xpos, double ypos) {
   if (roor(xpos) >= blocksX || roor(ypos) >= blocksY ||
       roor(xpos) <  0       || roor(ypos) <  0)
      return false;
   return true;
}

bool Grid::inBounds(int xpos, int ypos) {
   if (xpos >= blocksX || xpos < 0 ||
       ypos >= blocksY || ypos < 0)
      return false;
   return true;
}

int Grid::getIdx(double xpos, double ypos) {
   int ax = roor(xpos);
   int ay = roor(ypos);
#ifndef KUROME_NOCHECK
   if (!inBounds(ax,ay)) {
      errnok = KUROME_ERANGE;
      return -1;
   }
#endif
   return blocks(ax,ay);
}

int Grid::getIdx(double xpos, double ypos, double error) {
   int ax = roor(xpos);
   int ay = roor(ypos);
   std::normal_distribution r = std::normal_distribution(-error,error);
#ifndef KUROME_NOCHECK
   if (!inBounds(ax,ay)) {
      errnok = KUROME_ERANGE;
      return -1;
   }
#endif
   return blocks(ax,ay) + r(generator);
}

int Grid::setIdx(double xpos, double ypos, int val) {
   int ax = roor(xpos);
   int ay = roor(ypos);
#ifndef KUROME_NOCHECK
   if (!inBounds(ax,ay)) {
      errnok = KUROME_ERANGE;
      return -1;
   }
#endif
   blocks(ax,ay) = avgWeights(val,blocks(ax,ay));
   return 0;
}

int Grid::setIdx(double xpos, double ypos, int val, double weight) {
   int ax = roor(xpos);
   int ay = roor(ypos);
#ifndef KUROME_NOCHECK
   if (!inBounds(ax,ay)) {
      errnok = KUROME_ERANGE;
      return -1;
   }
#endif
   blocks(ax,ay) = (val * weight) + (blocks(ax,ay) * (1.0 - weight));
   return 0;
}

int Grid::avgWeights(int w1, int w2) {
   return w1;
}

int Grid::avgWeights(int w1, int w2, double w1Weight) {
#ifndef KUROME_NOCHECK
   if (w1Weight < 0.0 || w1Weight > 1.0) {
      errnok = KUROME_EPERCENT;
      return w1;
   }
#endif
   return (w1 * w1Weight) + (w2 * (1.0 - w1Weight));
}

int Grid::root(double val) {
   return (int)std::ceil(val/unitSize);
}

int Grid::roob(double val) {
   return (int)std::floor(val/unitSize);
}

int Grid::roor(double val) {
   return (int)std::round(val/unitSize);
}

void Grid::info() {
   printf("GRID %p :\n",this);
   printf("\tUNIT SIZE: %lf\n",unitSize);
   printf("\tSIZE X   : %lf\n",sizeX);
   printf("\tBLOCKS X : %d\n",blocksX);
   printf("\tSIZE Y   : %lf\n",sizeY);
   printf("\tBLOCKS Y : %d\n",blocksY);
   printf("\tENTITIES : %lu\n",entities.size());
}

void Grid::print() {
   for (int j = 0; j < blocksY; ++j) {
      for (int i = 0; i < blocksX; ++i) {
         if (blocks(i,j))
            printf("\033[31m");
         printf("%02x\033[0m",blocks(i,j));
      }
      printf("\n");
   }
}

int Grid::changeUnitSize(double newSize) {
#ifndef KUROME_NOCHECK
   if (newSize <= 0.0) {
      errnok = KUROME_EBADNUM;
      return -1;
   }
#endif
   this->unitSize = newSize;
   this->blocksX = root(this->sizeX);
   this->blocksY = root(this->sizeY);
   this->blocks = Eigen::MatrixXi(this->blocksX,this->blocksY);
   for (Entity * e : entities) 
      addEntity(e);
   return 0;
}

int Grid::changeSizeX(double newX) {
#ifndef KUROME_NOCHECK
   if (newX <= 0.0) {
      errnok = KUROME_EBADNUM;
      return -1;
   }
#endif
   int oldBlocks = this->blocksX;
   this->sizeX = newX;
   this->blocksX = root(this->sizeX);
   blocks.conservativeResize(blocksX,Eigen::NoChange_t());
   if (blocksX > oldBlocks) {
      for (int i = oldBlocks; i < blocksX; ++i)
         for (int j = 0; j < blocksY; ++j)
            blocks(i,j) = 0;
   }
   return 0;
}

int Grid::changeSizeY(double newY) {
#ifndef KUROME_NOCHECK
   if (newY <= 0.0) {
      errnok = KUROME_EBADNUM;
      return -1;
   }
#endif
   int oldBlocks = this->blocksY;
   this->sizeY = newY;
   this->blocksY = root(this->sizeY);
   blocks.conservativeResize(Eigen::NoChange_t(),blocksY);
   if (blocksY > oldBlocks) {
      for (int i = 0; i < blocksX; ++i)
         for (int j = oldBlocks; j < blocksY; ++j)
            blocks(i,j) = 0;
   }
   return 0;
}

double Grid::getUnitSize() {
   return unitSize;
}

int * Grid::getIdxPtr(double xpos, double ypos) {
   int ax = roor(xpos);
   int ay = roor(ypos);
#ifndef KUROME_NOCHECK
   if (!inBounds(ax,ay)) {
      errnok = KUROME_ERANGE;
      return NULL;
   }
#endif
   return &blocks(ax,ay);
}

int * Grid::getIdxPtr(int xpos, int ypos) {
#ifndef KUROME_NOCHECK
   if (!inBounds(xpos,ypos)) {
      errnok = KUROME_ERANGE;
      return NULL;
   }
#endif
   return &blocks(xpos,ypos);
}

int Grid::addEntity(Entity * e) {
   return 0;
}

int Grid::remEntity(Entity * e) {
   return 0;
}

int Grid::getXBlocks() {
   return blocksX;
}
