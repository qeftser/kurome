
#include "Grid.h"
#include <cmath>
#include <cstdlib>
#include <cstdio>
#include <queue>
#include <unordered_set>
#include <cstdint>
#include <cstring>

Grid::Grid(double unitSize, double sizeX, double sizeY) : unitSize(unitSize) {
   this->sizeX = roundOutTop(sizeX);
   this->sizeY = roundOutTop(sizeY);
   entities = std::vector<Entity>();
   printf("sizeX: %d, sizeY: %d\n",this->sizeX,this->sizeY);
   grid = (int **)malloc(sizeof(int *)*(this->sizeX+1));
   for (int i = 0; i <= this->sizeX; ++i)
      grid[i] = (int *)calloc(sizeof(int)*(this->sizeY+1),1);
   marks = (int **)malloc(sizeof(int *)*(this->sizeX+1));
   for (int i = 0; i <= this->sizeX; ++i)
      marks[i] = (int *)calloc(sizeof(int)*(this->sizeY+1),1);
}

int Grid::roundOutTop(double val) {
   return (int)std::ceil(val/unitSize);
}

int Grid::roundOutBottom(double val) {
   return (int)std::floor(val/unitSize);
}

void Grid::roundOut(double vlow, double vhigh, int & nlow, int & nhigh) {
   nlow = std::floor(vlow/unitSize);
   nhigh = std::ceil(vhigh/unitSize);
}

int Grid::avgWeights(const int old, const int nev, double newWeight) {
   if (newWeight > 1.0) {
      fprintf(stderr,"Weight over 1.0! : %f",newWeight);
      newWeight = 1.0;
   }
   else if (newWeight < 0.0) {
      fprintf(stderr,"Weight under 0.0! : %f",newWeight);
      newWeight = 0.0;
   }
   return ceil(((double)old * (1.0 - newWeight)) + ((double)nev * newWeight));
}

Grid::~Grid(void) {
   for (int i = 0; i < this->sizeX; ++i) {
      free(grid[i]);
      free(marks[i]);
   }
   free(grid);
   free(marks);
}

void Grid::updateGrid(int type, int danger, double dist1, double dist2, double posX, double posY, double weight) {
   int width, height, rad, x, y;
   switch (type) {
      case ENTITY_TYPE_RECT:
         roundOut(posX,dist1,x,width);
         roundOut(posY,dist2,y,height);
         x -= (width/2);
         y -= (height/2);
         for (int i = x; i <= width+x; ++i) {
            if (i < 0 || i > sizeX) continue;
            for (int j = y; j <= height+y; ++j) {
               if (j < 0 || j > sizeY) continue;
               grid[i][j] = (avgWeights(grid[i][j],danger,weight));
            }
         }
      break;
      case ENTITY_TYPE_POINT:
         x = roundOutBottom(posX);
         y = roundOutBottom(posY);
         if (x < 0 || x > sizeX || y < 0 || y > sizeY) break;
         grid[x][y] = avgWeights(grid[x][y],danger,weight);
      break;
      case ENTITY_TYPE_CIRCLE:
         rad = roundOutTop(dist1);
         x = roundOutBottom(posX);
         y = roundOutBottom(posY);
         double xGP = -100, xLP = -100;
         for (double k = 0.0; k <= PI; k += GRID_CIRCLE_GRANULARITY) {
            int xG = ceil (x + (cos(k) * rad)),
                xL = floor(x + (cos(k+PI) * rad)),
                yG = ceil (y + (sin(k) * rad)),
                yL = floor(y + (sin(k+PI) * rad));
            if (xG != xGP) {
               if (xG < 0 || xG > sizeX) continue;
               for (int i = yL; i <= yG; ++i) {
                  if (i < 0 || i > sizeY) continue;
                  grid[xG][i] = avgWeights(grid[xG][i],danger,weight);
               }
            }
            if (xL != xLP && xL != xG) {
               if (xL < 0 || xL > sizeX) continue;
               for (int i = yL; i <= yG; ++i) {
                  if (i < 0 || i > sizeY) continue;
                  grid[xL][i] = avgWeights(grid[xL][i],danger,weight);
               }
            }
            xGP = xG; xLP = xL;
         }
      break;
   }
}


void Grid::addEntity(const Entity * const e) {
   entities.push_back(*e);
   updateGrid(e->type,e->danger,e->dist1,e->dist2,e->posX,e->posY,GRID_NEW_VALUE_WEIGHT);
}

void Grid::addEntity(const WeightedEntity * const e) {
   entities.push_back(*e);
   updateGrid(e->type,e->danger,e->dist1,e->dist2,e->posX,e->posY,e->weight);
}

void Grid::print(void) {
   for (int j = sizeY-1; j >= 0; --j)
      printf("---");
   printf("\n");
   for (int j = sizeY-1; j >= 0; --j) {
      for (int i = 0; i < sizeX; ++i) {
         if (marks[i][j] == 1)
            printf("\033[43m");
         else if (marks[i][j] == 2)
            printf("\033[45m");
         if (grid[i][j] > 0x7f)
            printf("\033[36m");
         else if (grid[i][j] > 0x5f)
            printf("\033[35m");
         else if (grid[i][j] > 0x2f)
            printf("\033[34m");
         else if (grid[i][j] > 0x0f)
            printf("\033[31m");
         else if (grid[i][j] > 0x07)
            printf("\033[33m");
         else if (grid[i][j] > 0x00)
            printf("\033[32m");
         printf(" %2x\033[0m",grid[i][j]);
      }
      printf("\n");
   }
   for (int j = sizeY-1; j >= 0; --j)
      printf("---");
   printf("\n");
}

typedef struct FRAME_STRUCT { double angle; int weight; double posX; double posY; int step; struct FRAME_STRUCT * prev; } Frame;
typedef uint64_t FrameID;

FrameID Grid::getFrameId(Frame * f) {
   uint64_t ret;
   ret = ((((uint64_t)floor((180/PI)*f->angle)/GRID_MAP_ANGLE_SLICING)&0xff)<<47)
         | (roundOutBottom(f->posX)<<23)
         | (roundOutBottom(f->posY));
   return ret;
}

int Grid::getCost(const Entity * const e, Frame * f) {
   int width, height, rad, x, y, ret = 0;
   switch (e->type) {
      case ENTITY_TYPE_RECT:
         roundOut(f->posX,e->dist1,x,width);
         roundOut(f->posY,e->dist2,y,height);
         x -= (width/2);
         y -= (height/2);
         for (int i = x; i <= width+x; ++i) {
            if (i < 0 || i > sizeX) continue;
            for (int j = y; j <= height+y; ++j) {
               if (j < 0 || j > sizeY) continue;
               ret += grid[i][j];
            }
         }
      break;
      case ENTITY_TYPE_POINT:
         x = roundOutBottom(f->posX);
         y = roundOutBottom(f->posY);
         if (x < 0 || x > sizeX || y < 0 || y > sizeY) break;
         ret += grid[x][y];
      break;
      case ENTITY_TYPE_CIRCLE:
         rad = roundOutTop(e->dist1);
         x = roundOutBottom(f->posX);
         y = roundOutBottom(f->posY);
         double xGP = -100, xLP = -100;
         for (double k = 0.0; k <= PI; k += GRID_CIRCLE_GRANULARITY) {
            int xG = ceil (x + (cos(k) * rad)),
                xL = floor(x + (cos(k+PI) * rad)),
                yG = ceil (y + (sin(k) * rad)),
                yL = floor(y + (sin(k+PI) * rad));
            if (xG != xGP) {
               if (xG < 0 || xG > sizeX) continue;
               for (int i = yL; i <= yG; ++i) {
                  if (i < 0 || i > sizeY) continue;
                  ret += grid[xG][i];
               }
            }
            if (xL != xLP && xL != xG) {
               if (xL < 0 || xL > sizeX) continue;
               for (int i = yL; i <= yG; ++i) {
                  if (i < 0 || i > sizeY) continue;
                  ret += grid[xL][i];
               }
            }
         }
      break;
   }
   return ret;
}

struct {
   bool operator()(const Frame * f1, const Frame * f2) const {
      if (f1->weight > f2->weight)
         return true;
      return false;
   }
} cmpFrames;

double quickCalc(double a, double b) {
   double hHeight = sqrtf((a * a) - ((b * b)/4));
   if (0.00001 < fabs(a - (sqrt(((b/2)*(b/2))+(hHeight*hHeight) - 0))))
      fprintf(stderr,"dang... :(\n");
   printf("check: %f\n",(sqrt(((b/2)*(b/2))+(hHeight*hHeight) - 0)));
   printf("intermin: %f\n",((1/a)*(b/2)));
   double bAngle = asin((1/a)*(b/2))*(180/PI);
   printf("bAngle: %f\n",bAngle);
   double cAngle = 180 - (bAngle+90);
   return cAngle*(PI/180);
}

void newFrameSet(Frame * old, Frame * nev, double enterAngle) {
   nev->angle = fmod(old->angle + enterAngle,PI);
   nev->step = old->step + 1;
   nev->posX = old->posX + (cos(nev->angle)*GRID_MAP_FRAME_STEP);
   nev->posY = old->posY + (sin(nev->angle)*GRID_MAP_FRAME_STEP);
}

int dist2(double x1, double y1, double x2, double y2) {
   return ceil(((x2-x1)*(x2-x1))+((y2-y1)*(y2-y1)));
}

Frame * Grid::childFrame(Frame * old, const Entity * const e, double step, double destX, double destY) {
   Frame * nFrame = (Frame *)calloc(sizeof(Frame),1);
   nFrame->prev = old;
   newFrameSet(old,nFrame,step);
   nFrame->weight =  (int)((nFrame->step*1.00) + (0.5 * dist2(nFrame->posX,nFrame->posY,destX,destY)) + (1.00 * getCost(e,nFrame)));
   return nFrame;
}

bool Grid::inBounds(double x, double y) {
   if (x < 0 || y < 0 || x > (unitSize*(sizeX-1)) || y > (unitSize*(sizeY-1)))
      return 0;
   return 1;
}

// A*
void Grid::mapEntity(const MovingEntity * const e, double destX, double destY) {
   std::priority_queue<Frame *,std::vector<Frame *>,decltype(cmpFrames)> currPaths;
   std::unordered_set<FrameID> visited;
   std::vector<Frame *> allocated;
   int destXG = roundOutTop   (destX),
       destYG = roundOutTop   (destY),
       destXL = roundOutBottom(destX),
       destYL = roundOutBottom(destY);

   Frame start = { e->angle, 0, e->posX, e->posY, 0, NULL };
   currPaths.push(&start);

   double maxAChange = (PI/90.0L)*(((asinl((1.0L/e->turnRad)*(GRID_MAP_FRAME_STEP/2.0))*(180/PI))));
   double aStep = maxAChange / GRID_MAP_FRAME_SLICES;
   printf("maxAChange: %f\naStep: %f\n",maxAChange,aStep);

   while (!currPaths.empty()) {
      Frame * nFrame, * curr = currPaths.top();
      currPaths.pop();
      //printf("size: %lu weight: %d angle: %f curr pos: (%f,%f)\n",currPaths.size(),curr->weight,curr->angle,curr->posX,curr->posY);

      if ((roundOutTop(curr->posX) == destXG || roundOutBottom(curr->posX) == destXL)
       && (roundOutTop(curr->posY) == destYG || roundOutBottom(curr->posY) == destYL)) {
         printf("path found\n");
         while (curr) {
            marks[roundOutBottom(curr->posX)][roundOutBottom(curr->posY)] = 2;
            curr = curr->prev;
         }
         for (Frame * f : allocated)
            free(f);
         print();
         for (int i = 0; i <= sizeX; ++i)
            memset(&marks[i],0,sizeof(int)*(sizeY+1));
         return;
      }

      // up left
      for (int i = 0; i < GRID_MAP_FRAME_SLICES; ++i) {
         nFrame = childFrame(curr,e,aStep*i,destX,destY);
         if (inBounds(nFrame->posX,nFrame->posY) && visited.insert(getFrameId(nFrame)).second) {
            marks[roundOutBottom(nFrame->posX)][roundOutBottom(nFrame->posY)] = 1;
            //printf("Child: %f,(%f,%f)\n",nFrame->angle,nFrame->posX,nFrame->posY);
            currPaths.push(nFrame);
            allocated.push_back(nFrame);
         }
         else;
            //free(nFrame);
      }
      // up right
      for (int i = 1; i < GRID_MAP_FRAME_SLICES; ++i) {
         nFrame = childFrame(curr,e,aStep*i*-1,destX,destY);
         if (inBounds(nFrame->posX,nFrame->posY) && visited.insert(getFrameId(nFrame)).second) {
            //printf("Child: %f,(%f,%f)\n",nFrame->angle,nFrame->posX,nFrame->posY);
            currPaths.push(nFrame);
            allocated.push_back(nFrame);
         }
         else;
            //free(nFrame);
      }
      // down right
      for (int i = 0; i < GRID_MAP_FRAME_SLICES; ++i) {
         nFrame = childFrame(curr,e,(aStep*i)+PI,destX,destY);
         if (inBounds(nFrame->posX,nFrame->posY) && visited.insert(getFrameId(nFrame)).second) {
            //printf("Child: %f,(%f,%f)\n",nFrame->angle,nFrame->posX,nFrame->posY);
            currPaths.push(nFrame);
            allocated.push_back(nFrame);
         }
         else;
            //free(nFrame);
      }
      // down left
      for (int i = 1; i < GRID_MAP_FRAME_SLICES; ++i) {
         nFrame = childFrame(curr,e,(aStep*i*-1)+PI,destX,destY);
         if (inBounds(nFrame->posX,nFrame->posY) && visited.insert(getFrameId(nFrame)).second) {
            //printf("Child: %f,(%f,%f)\n",nFrame->angle,nFrame->posX,nFrame->posY);
            currPaths.push(nFrame);
            allocated.push_back(nFrame);
         }
         else;
            //free(nFrame);
      }

      //print();
      //getc(stdin);

   }
   printf("no path exists\n");
   for (Frame * f : allocated)
      free(f);
}
