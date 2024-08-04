
#include "Grid.h"
#include <cmath>
#include <cstdlib>
#include <cstdio>

Grid::Grid(double unitSize, double sizeX, double sizeY) : unitSize(unitSize) {
   this->sizeX = roundOutTop(sizeX);
   this->sizeY = roundOutTop(sizeY);
   entities = std::vector<Entity>();
   printf("sizeX: %d, sizeY: %d\n",this->sizeX,this->sizeY);
   grid = (int **)malloc(sizeof(int *)*(this->sizeX));
   for (int i = 0; i < this->sizeX; ++i)
      grid[i] = (int *)calloc(sizeof(int)*(this->sizeY),1);
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

void Grid::updateGrid(int type, int danger, double dist1, double dist2, double posX, double posY, double weight) {
   int width, height, rad, x, y;
   switch (type) {
      case ENTITY_TYPE_RECT:
         roundOut(posX,dist1,x,width);
         roundOut(posY,dist2,y,height);
         for (int i = x; i <= width; ++i)
            for (int j = y; j <= height; ++j)
               grid[i][j] = (avgWeights(grid[i][j],danger,weight));
      break;
      case ENTITY_TYPE_POINT:
         x = roundOutBottom(posX);
         y = roundOutBottom(posY);
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
               for (int i = yL; i <= yG; ++i)
                  grid[xG][i] = avgWeights(grid[xG][i],danger,weight);
            }
            if (xL != xLP && xL != xG) {
               for (int i = yL; i <= yG; ++i)
                  grid[xL][i] = avgWeights(grid[xL][i],danger,weight);
            }
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
         if (grid[i][j] > 0x7f)
            printf("\033[37m");
         else if (grid[i][j] > 0x5f)
            printf("\033[36m");
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
