
#include "Kurome.h"
#include <stdio.h>

#define oblocks(x,y) blocks(x-blocksXmin,y-blocksYmin)

Grid::Grid(double unitSize, double sizeX, double sizeY) {
   this->unitSize = unitSize;
   blocksXmax = root(sizeX); blocksYmax = root(sizeY);
   blocksXmin = blocksYmin = 0;
   sizeXmax = sizeX; sizeYmax = sizeY;
   sizeXmin = sizeYmin = 0.0;
   this->blocks = Eigen::MatrixXi(blocksXmax-blocksXmin,blocksYmax-blocksYmin);
   this->entities = std::set<Entity *>();
   this->samples = std::set<Sample *>();
   this->generator = std::default_random_engine(clock());
   clear();
}

Grid::Grid(double unitSize, double sizeXmin, double sizeXmax, double sizeYmin, double sizeYmax) {
   this->unitSize = unitSize;
   this->sizeXmin = sizeXmin; this->sizeXmax = sizeXmax;
   this->sizeYmin = sizeYmin; this->sizeYmax = sizeYmax;
   blocksXmax = root(sizeXmax); blocksYmax = root(sizeYmax);
   blocksXmin = roob(sizeXmin); blocksYmin = roob(sizeYmin);
   this->blocks = Eigen::MatrixXi(blocksXmax-blocksXmin,blocksYmax-blocksYmin);
   this->entities = std::set<Entity *>();
   this->samples = std::set<Sample *>();
   this->generator = std::default_random_engine(clock());
   clear();
}

void Grid::clear() {
   for (int i = 0; i < blocksXmax-blocksXmin; ++i)
      for (int j = 0; j < blocksYmax-blocksYmin; ++j)
         blocks(i,j) = KUROME_UNDETERMINED;
}

void Grid::clense() {
   clear();
   for (Entity * e : entities)
      delete e;
   for (Sample * s : samples)
      delete s;
   entities.clear();
   samples.clear();
}

/* kinda slow ... */
void Grid::smooth() {
   Eigen::MatrixXi cpy = Eigen::MatrixXi(blocks.rows(),blocks.cols());
   for (int i = 1; i < (blocksXmax-blocksXmin)-1; ++i) {
      for (int j = 1; j < (blocksYmax-blocksYmin)-1; ++j) {
         cpy(i,j) = (blocks(i,j)+blocks(i+1,j)+blocks(i-1,j)+blocks(i,j+1)+blocks(i,j-1)+
                        blocks(i+1,j+1)+blocks(i-1,j+1)+blocks(i+1,j-1)+blocks(i-1,j-1)) / 9;
      }
   }
   for (int i = 0; i < (blocksXmax-blocksXmin); ++i)
      for (int j = 0; j < (blocksYmax-blocksYmin); ++j)
         blocks(i,j) = cpy(i,j);
}

void Grid::redraw() {
   clear();
   for (Entity * e : entities) 
      addEntity(e);
   for (Sample * s : samples)
      apply(s);
}

bool Grid::inBounds(double xpos, double ypos) {
   if (roor(xpos) >= blocksXmax || roor(ypos) >= blocksYmax ||
       roor(xpos) <  blocksXmin || roor(ypos) <  blocksYmin)
      return false;
   return true;
}

bool Grid::inBounds(int xpos, int ypos) {
   if (xpos >= blocksXmax || xpos < blocksXmin ||
       ypos >= blocksYmax || ypos < blocksYmin)
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
   return oblocks(ax,ay);
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
   return oblocks(ax,ay) + r(generator);
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
   oblocks(ax,ay) = avgWeights(val,blocks(ax,ay));
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
   oblocks(ax,ay) = (val * weight) + (oblocks(ax,ay) * (1.0 - weight));
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
   printf("\tSIZE X   : %lf\n",sizeXmax-sizeXmin);
   printf("\tBLOCKS X : %d\n",blocksXmax-blocksXmin);
   printf("\tSIZE Y   : %lf\n",sizeYmax-sizeYmin);
   printf("\tBLOCKS Y : %d\n",blocksYmax-blocksYmin);
   printf("\tRANGE X  : (%lf, %lf)\n",sizeXmin,sizeXmax);
   printf("\tRANGE BX : (%d, %d)\n",blocksXmin,blocksXmax);
   printf("\tRANGE Y  : (%lf, %lf)\n",sizeYmin,sizeYmax);
   printf("\tRANGE BY : (%d, %d)\n",blocksYmin,blocksYmax);
   printf("\tENTITIES : %lu\n",entities.size());
}

void Grid::print() {
   int maxv = 0;
   for (int j = 0; j < blocksYmax-blocksYmin; ++j) {
      for (int i = 0; i < blocksXmax-blocksXmin; ++i) {
         if (blocks(i,j) > maxv)
            maxv = blocks(i,j);
      }
   }
   int stepv = maxv/5;
   int scalev = maxv/255;
   for (int j = 0; j < blocksYmax-blocksYmin; ++j) {
      for (int i = 0; i < blocksXmax-blocksXmin; ++i) {
         if (blocks(i,j) > stepv*4)
            printf("\033[35m");
         else if (blocks(i,j) > stepv*3)
            printf("\033[31m");
         else if (blocks(i,j) > stepv*2)
            printf("\033[33m");
         else if (blocks(i,j) > stepv)
            printf("\033[32m");
         else if (blocks(i,j))
            printf("\033[36m");
         printf("%02x\033[0m",std::abs(blocks(i,j)/scalev));
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
   this->blocksXmax = root(this->sizeXmax);
   this->blocksYmax = root(this->sizeYmax);
   this->blocksXmin = roob(this->sizeXmin);
   this->blocksYmin = roob(this->sizeYmin);
   this->blocks = Eigen::MatrixXi(blocksXmax-blocksXmin,blocksYmax-blocksYmin);
   redraw();
   return 0;
}

int Grid::changeSizeXmax(double newX) {
#ifndef KUROME_NOCHECK
   if (newX <= sizeXmin) {
      errnok = KUROME_EBADNUM;
      return -1;
   }
#endif
   int oldBlocks = this->blocksXmax;
   this->sizeXmax = newX;
   this->blocksXmax = root(this->sizeXmax);
   blocks.conservativeResize(blocksXmax-blocksXmin,Eigen::NoChange_t());
   redraw();
   return 0;
}

int Grid::changeSizeYmax(double newY) {
#ifndef KUROME_NOCHECK
   if (newY <= sizeYmin) {
      errnok = KUROME_EBADNUM;
      return -1;
   }
#endif
   int oldBlocks = this->blocksYmax;
   this->sizeYmax = newY;
   this->blocksYmax = root(this->sizeYmax);
   blocks.conservativeResize(Eigen::NoChange_t(),blocksYmax-blocksYmin);
   redraw();
   return 0;
}

int Grid::changeSizeXmin(double newX) {
#ifndef KUROME_NOCHECK
   if (newX >= sizeXmax) {
      errnok = KUROME_EBADNUM;
      return -1;
   }
#endif
   this->sizeXmin = newX;
   this->blocksXmin = roob(this->sizeXmin);
   blocks.conservativeResize(blocksXmax-blocksXmin,Eigen::NoChange_t());
   redraw();
   return 0;
}

int Grid::changeSizeYmin(double newY) {
#ifndef KUROME_NOCHECK
   if (newY >= sizeYmax) {
      errnok = KUROME_EBADNUM;
      return -1;
   }
#endif
   this->sizeYmin = newY;
   this->blocksYmin = roob(this->sizeYmin);
   blocks.conservativeResize(Eigen::NoChange_t(),blocksYmax-blocksYmin);
   redraw();
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
   return &oblocks(ax,ay);
}

int * Grid::getIdxPtr(int xpos, int ypos) {
#ifndef KUROME_NOCHECK
   if (!inBounds(xpos,ypos)) {
      errnok = KUROME_ERANGE;
      return NULL;
   }
#endif
   return &oblocks(xpos,ypos);
}

int Grid::addEntity(Entity * e) {
   EllipseIterator ei = EllipseIterator(e,this);
   RectIterator ri = RectIterator(e,this);
   switch (e->type) {
      case KUROME_TYPE_PONT:
         setIdx(e->posx,e->posy,e->val);
         break;
      case KUROME_TYPE_ELPS:
         while (!ei.done) {
            *ei = avgWeights(e->val,*ei);
            ++ei;
         }
         break;
      case KUROME_TYPE_RECT:
         while (!ri.done) {
            *ri = avgWeights(e->val,*ri);
            ++ri;
         }
         break;
      default:
         errnok = KUROME_ETYPE;
         return -1;
         break;
   }
   if (!entities.count(e))
      entities.insert(e);
   return 0;
}

int Grid::remEntity(Entity * e) {
   for (Entity * en : entities) {
      if (en->id == e->id) {
         entities.erase(en);
         redraw();
         break;
      }
   }
   return 0;
}

int Grid::chgEntity(Entity * e) {
   for (Entity * en : entities) {
      if (en->id == e->id) {
         *en = *e;
         redraw();
         break;
      }
   }
   return 0;
}

int Grid::getHighBlocks() {
   return (blocksXmax-blocksXmin>blocksYmax-blocksYmin
          ?blocksXmax-blocksXmin:blocksYmax-blocksYmin);
}

int Grid::apply(Sample * sample, int (* func) (int, int)) {
   if (!sample)
      return 0;
   EllipseIterator ei = EllipseIterator(&sample->orgin,this);
   RectIterator ri = RectIterator(&sample->orgin,this);
   switch(sample->orgin.type) {
      case KUROME_TYPE_ELPS:
         while (!ei.done) {
            *ei = func(sample->localVal(ei.locinfo().posx,ei.locinfo().posy),*ei);
            ++ei;
         }
         break;
      case KUROME_TYPE_RECT:
         while (!ri.done) {
            *ri = func(sample->localVal(ri.locinfo().posx,ri.locinfo().posy),*ri);
            ++ri;
         }
         break;
      default:
         errnok = KUROME_ETYPE;
         return -1;
         break;
   }
   if (!samples.count(sample))
      samples.insert(sample);
   return 0;
}

int Grid::apply(Sample * sample) {
   return apply(sample,avgWeights);
}

Sample Grid::application(Sample * s) {
   Sample nev = Sample(s->orgin,Eigen::MatrixXi((int)root(s->orgin.xwid),(int)root(s->orgin.ywid)),unitSize);
   EllipseIterator ei = EllipseIterator(&nev.orgin,this);
   RectIterator ri = RectIterator(&nev.orgin,this);
   switch(nev.orgin.type) {
      case KUROME_TYPE_ELPS:
         while (!ei.done) {
            nev.localVal(ei.locinfo().posx,ei.locinfo().posy) = *ei;
            ++ei;
         }
         break;
      case KUROME_TYPE_RECT:
         while (!ri.done) {
            nev.localVal(ri.locinfo().posx,ri.locinfo().posy) = *ri;
            ++ri;
         }
         break;
   }
   return nev;
}

int Grid::toStruct(struct grid_struct ** gs) {
   int size = sizeof(grid_struct)+(sizeof(int)*(blocksXmax-blocksXmin)*(blocksYmax-blocksYmin));
   *gs = (struct grid_struct *)realloc(*gs,size);
   (*gs)->unitSize = unitSize;
   (*gs)->blocksXmax = blocksXmax;
   (*gs)->blocksYmax = blocksYmax;
   (*gs)->blocksXmin = blocksXmin;
   (*gs)->blocksYmin = blocksYmin;
   (*gs)->sizeXmax = sizeXmax;
   (*gs)->sizeYmax = sizeYmax;
   (*gs)->sizeXmin = sizeXmin;
   (*gs)->sizeYmin = sizeYmin;
   for (int i = 0; i < (blocksXmax-blocksXmin)*(blocksYmax-blocksYmin); ++i)
      (*gs)->matrix[i] = blocks(i/(blocksXmax-blocksXmin),i%(blocksXmax-blocksXmin));
   return size;
}

int Grid::collectInfo(struct partial_grid_struct * pgs) {
   pgs->unitSize   = unitSize;
   pgs->blocksXmax = blocksXmax;
   pgs->blocksYmax = blocksYmax;
   pgs->blocksXmin = blocksXmin;
   pgs->blocksYmin = blocksYmin;
   pgs->sizeXmax   = sizeXmax;
   pgs->sizeYmax   = sizeYmax;
   pgs->sizeXmin   = sizeXmin;
   pgs->sizeYmin   = sizeYmin;
   return 1;
}

Grid::Grid(struct grid_struct * gs)
   : blocksXmax(gs->blocksXmax), blocksYmax(gs->blocksYmax), 
     blocksXmin(gs->blocksXmin), blocksYmin(gs->blocksYmin),
     sizeXmax(gs->sizeXmax), sizeYmax(gs->sizeYmax), unitSize(gs->unitSize) {
   this->blocks = Eigen::MatrixXi(gs->blocksXmax-gs->blocksXmin,gs->blocksYmax-gs->blocksYmin);
   this->entities = std::set<Entity *>();
   this->samples = std::set<Sample *>();
   this->generator = std::default_random_engine(clock());
   for (int i = 0; i < (gs->blocksXmax-gs->blocksXmin)*(gs->blocksYmax-gs->blocksYmin); ++i) {
      blocks(i/(gs->blocksXmax-blocksXmin),i%(gs->blocksXmax-gs->blocksXmin)) = gs->matrix[i];
   }
}

