
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
   entities = std::vector<Entity *>();
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


void Grid::addEntity(Entity * e) {
   entities.push_back(e);
   updateGrid(e->type,e->danger,e->dist1,e->dist2,e->posX,e->posY,GRID_NEW_VALUE_WEIGHT);
}

void Grid::addEntity(WeightedEntity * e) {
   entities.push_back(e);
   updateGrid(e->type,e->danger,e->dist1,e->dist2,e->posX,e->posY,e->weight);
}

void Grid::raddEntity(Entity * e) {
   updateGrid(e->type,e->danger,e->dist1,e->dist2,e->posX,e->posY,GRID_NEW_VALUE_WEIGHT);
}

void Grid::raddEntity(WeightedEntity * e) {
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

void Grid::display(sf::RenderWindow & window) {
   int maxX = window.getSize().x;
   int maxY = window.getSize().y;
   int sepX = maxX / sizeX;
   int sepY = maxY / sizeY;
   lenW = (sepX < sepY ? sepX : sepY);
   double lenWoff = lenW/2.0;
   for (int i = 0; i < sizeX; ++i) {
      for (int j = 0; j < sizeY; ++j) {
         sf::RectangleShape r = sf::RectangleShape(sf::Vector2f(lenW,lenW));
         r.setPosition(i*lenW,j*lenW);
         r.setOutlineColor(sf::Color(155,155,155));
         r.setFillColor(sf::Color(0,0,0,0));
         r.setOutlineThickness(0.75);
         if (marks[i][j] == 1) {
            r.setFillColor(sf::Color(255,255,0));
            if (grid[i][j])
               r.setOutlineColor(sf::Color::Red);
         }
         else if (marks[i][j] == 2) {
            r.setFillColor(sf::Color::Red);
            if (grid[i][j])
               r.setOutlineColor(sf::Color::Red);
         }
         else if (grid[i][j])
            r.setFillColor(sf::Color::Red);
         window.draw(r);
      }
   }
   for (Entity * e : entities) {
      double lenX, lenY, radX;
      sf::RectangleShape r;
      sf::CircleShape c;
      switch (e->type) {
         case ENTITY_TYPE_POINT:
            lenX = 1;
            lenY = 1;
            r = sf::RectangleShape(sf::Vector2f(lenX,lenY));
            r.setPosition(((e->posX/unitSize)*lenW)-(lenX/2-lenWoff),((((e->posY/unitSize)*lenW))-(lenY/2-lenWoff)));
            r.setOutlineColor(sf::Color(10,255,10));
            r.setFillColor(sf::Color::Green);
            r.setOutlineThickness(1);
            window.draw(r);
            break;
         case ENTITY_TYPE_CIRCLE:
            radX = lenW*(e->dist1/unitSize);
            c = sf::CircleShape(radX);
            c.setPosition(((e->posX/unitSize)*lenW)-(radX-lenWoff),((e->posY/unitSize)*lenW)-(radX-lenWoff));
            c.setOutlineColor(sf::Color(10,255,10));
            c.setFillColor(sf::Color::Green);
            c.setOutlineThickness(1);
            window.draw(c);
         break;
         case ENTITY_TYPE_RECT:
            lenX = lenW*(e->dist1/unitSize);
            lenY = lenW*(e->dist2/unitSize);
            r = sf::RectangleShape(sf::Vector2f(lenX,lenY));
            r.setPosition(((e->posX/unitSize)*lenW)-(lenX/2-lenWoff),((((e->posY/unitSize)*lenW))-(lenY/2-lenWoff)));
            r.setOutlineColor(sf::Color(10,255,10));
            r.setFillColor(sf::Color::Green);
            r.setOutlineThickness(1);
            window.draw(r);
            break;
      }
   }
}

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
   nFrame->weight =  (int)((nFrame->step*0.30) + (0.5 * dist2(nFrame->posX,nFrame->posY,destX,destY)) + (1.00 * getCost(e,nFrame)));
   return nFrame;
}

bool Grid::inBounds(double x, double y) {
   if (x < 0 || y < 0 || x > (unitSize*(sizeX-1)) || y > (unitSize*(sizeY-1)))
      return 0;
   return 1;
}

void init_MappingState(struct MappingState * state) {
   state->currPaths = std::priority_queue<Frame *,std::vector<Frame *>,decltype(cmpFrames)>();
   state->visited = std::unordered_set<FrameID>();
   state->allocated = std::vector<Frame *>();
};

#define add_line()                                                                                           \
   do {                                                                                                      \
      if (window) {                                                                                          \
         sf::Vertex line[] =                                                                                 \
         {                                                                                                   \
            sf::Vertex(sf::Vector2f((nFrame->prev->posX/unitSize)*lenW,(nFrame->prev->posY/unitSize)*lenW)), \
            sf::Vertex(sf::Vector2f((nFrame->posX/unitSize)*lenW,(nFrame->posY/unitSize)*lenW))              \
         };                                                                                                  \
         window->draw(line,2,sf::Lines);                                                                     \
      }                                                                                                      \
   } while (0)                                                                                               


// A*
int Grid::mapEntity(sf::RenderWindow * window, struct MappingState * state, int steps, const MovingEntity * const e, double destX, double destY) {
   int destXG = roundOutTop   (destX),
       destYG = roundOutTop   (destY),
       destXL = roundOutBottom(destX),
       destYL = roundOutBottom(destY);

   if (state->currPaths.empty()) {
      Frame * start = (Frame *)calloc(sizeof(Frame),1);
      start->angle = e->angle;
      start->weight = 0;
      start->posX = e->posX;
      start->posY = e->posY;
      start->step = 0;
      start->prev = NULL;
      state->currPaths.push(start);
   }

   double maxAChange = (PI/90.0L)*(((asinl((1.0L/e->turnRad)*(GRID_MAP_FRAME_STEP/2.0))*(180/PI))));
   double aStep = maxAChange / GRID_MAP_FRAME_SLICES;

   while (!state->currPaths.empty() && --steps) {
      Frame * nFrame, * curr = state->currPaths.top();
      state->currPaths.pop();

      if ((roundOutTop(curr->posX) == destXG || roundOutBottom(curr->posX) == destXL)
       && (roundOutTop(curr->posY) == destYG || roundOutBottom(curr->posY) == destYL)) {
         while (curr) {
            marks[roundOutBottom(curr->posX)][roundOutBottom(curr->posY)] = 2;
            curr = curr->prev;
         }
         return 1;
      }

      // up left
      for (int i = 0; i < GRID_MAP_FRAME_SLICES; ++i) {
         nFrame = childFrame(curr,e,aStep*i,destX,destY);
         if (inBounds(nFrame->posX,nFrame->posY) && state->visited.insert(getFrameId(nFrame)).second) {
            marks[roundOutBottom(nFrame->posX)][roundOutBottom(nFrame->posY)] = 1;
            state->currPaths.push(nFrame);
            state->allocated.push_back(nFrame);
            //add_line();
         }
         else
            free(nFrame);
      }
      // up right
      for (int i = 1; i < GRID_MAP_FRAME_SLICES; ++i) {
         nFrame = childFrame(curr,e,aStep*i*-1,destX,destY);
         if (inBounds(nFrame->posX,nFrame->posY) && state->visited.insert(getFrameId(nFrame)).second) {
            marks[roundOutBottom(nFrame->posX)][roundOutBottom(nFrame->posY)] = 1;
            state->currPaths.push(nFrame);
            state->allocated.push_back(nFrame);
            //add_line();
         }
         else
            free(nFrame);
      }
      // down right
      for (int i = 0; i < GRID_MAP_FRAME_SLICES; ++i) {
         nFrame = childFrame(curr,e,(aStep*i)+PI,destX,destY);
         if (inBounds(nFrame->posX,nFrame->posY) && state->visited.insert(getFrameId(nFrame)).second) {
            marks[roundOutBottom(nFrame->posX)][roundOutBottom(nFrame->posY)] = 1;
            state->currPaths.push(nFrame);
            state->allocated.push_back(nFrame);
            //add_line();
         }
         else
            free(nFrame);
      }
      // down left
      for (int i = 1; i < GRID_MAP_FRAME_SLICES; ++i) {
         nFrame = childFrame(curr,e,(aStep*i*-1)+PI,destX,destY);
         if (inBounds(nFrame->posX,nFrame->posY) && state->visited.insert(getFrameId(nFrame)).second) {
            marks[roundOutBottom(nFrame->posX)][roundOutBottom(nFrame->posY)] = 1;
            state->currPaths.push(nFrame);
            state->allocated.push_back(nFrame);
            //add_line();
         }
         else
            free(nFrame);
      }

   }
   /*
   for (Frame * f : state->allocated)
      free(f);
      */
   return 0;
}

void Grid::clear() {
   for (int i = 0; i < sizeX; ++i)
      memset(marks[i],0,sizeof(int)*(sizeY+1));
   for (int i = 0; i < sizeX; ++i)
      memset(grid[i],0,sizeof(int)*(sizeY+1));
   entities.clear();
}

void Grid::remap() {
   for (int i = 0; i < sizeX; ++i)
      memset(marks[i],0,sizeof(int)*(sizeY));
   for (int i = 0; i < sizeX; ++i)
      memset(grid[i],0,sizeof(int)*(sizeY));
   for (Entity * e : entities)
      raddEntity(e);
}

Entity * Grid::match(double x, double y) {
   printf(" x: %f,  y: %f\n",x,y);
   double offX, offY;
   for (Entity * e : entities) {
   printf("px: %f, py: %f\n",e->posX,e->posY);
      switch (e->type) {
         case ENTITY_TYPE_RECT:
            offX = e->dist1/2;
            offY = e->dist2/2;
            if (x > e->posX-offX && x < e->posX+offX &&
                y > e->posY-offY && y < e->posY+offY)
               return e;
            break;
         case ENTITY_TYPE_CIRCLE:
            if (dist2(x,y,e->posX,e->posY) <= e->dist1)
               return e;
            break;
         case ENTITY_TYPE_POINT:
            // we aren't even going to try...
            break;
      }
   }
   return NULL;
}

double Grid::getUSize(void) {
   return unitSize;
}

double Grid::getLenW(void) {
   return lenW;
}
