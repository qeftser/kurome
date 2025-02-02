
#ifndef SIMPLE_ANY_ROT_STATIC_POTENTIAL_FIELD_MAPPER

#define SIMPLE_ANY_ROT_STATIC_POTENTIAL_FIELD_MAPPER

#include "../Kurome.h"
#include <unordered_set>
#include <stack>

typedef uint64_t sarspfm_point;

#define sarspfm_x(v) ((short)(v&0xffff))
#define sarspfm_y(v) ((short)((v>>16)&0xffff))
#define sarspfm_i(v) ((v&0xffffffff00000000))
#define sarspfm_v(v) ((v&0xffffffff00000000)>>32)
#define sarspfm_t(v) ((v&0xffff000000000000)>>48)
#define sarspfm_c(v) ((v&0x0000ffff00000000)>>32)

#define KUROME_SARSPFM_NOGO_VALUE (SHRT_MAX+SHRT_MAX)


class SimpleAnyRotStaticPotentialFieldMapper : public Mapper {
public:
   std::stack<Grid *> grids;
   std::set<sarspfm_point> edge_points;
   std::queue<sarspfm_point> points;
   std::set<sarspfm_point> seen;
   Grid * base;
   Entity max_space;
   long cutoff_cost;
   int mapped;
   int gx;
   int gy;
   double step;
   struct partial_grid_struct binfo;

   SimpleAnyRotStaticPotentialFieldMapper(int cutoff_cost, Agent * me)
      : Mapper(me), max_space(Entity(0,0,0,0,KUROME_TYPE_RECT,0)) {
         init(cutoff_cost);
      }

   SimpleAnyRotStaticPotentialFieldMapper(int cutoff_cost, Reporter * me)
      : Mapper(me), max_space(Entity(0,0,0,0,KUROME_TYPE_RECT,0)) {
         init(cutoff_cost);
      }
   
//#define SARSPFM_DEBUG

   void callback(int flags) {
      (void)flags;
      if (mapped)
         return;
      do {
         delete base;
         base = grids.top(); grids.pop();
         step = base->getUnitSize();
         double hstep = step/2;
         RectIterator ri;
         for (int i = 0; i < base->blocks.rows(); ++i) {
            for (int j = 0; j < base->blocks.cols(); ++j) {
               ri = RectIterator(i*step,j*step,&max_space,base);
               long total = 0;
               while (!ri.done) {
                  total += *ri;
                  ri++;
               }
               if (total >= cutoff_cost) {
                  base->blocks(i,j) = 0;
               }
               else {
                  base->blocks(i,j) = -1;
               }
            }
         }
#ifdef SARSPFM_DEBUG
         base->print();
#endif
         int id = 0;
         for (int i = 0; i < base->blocks.rows(); ++i) {
            for (int j = 0; j < base->blocks.cols(); ++j) {
               if (!base->blocks(i,j))
                  propagate_id(++id,i,j);

            }
         }
#ifdef SARSPFM_DEBUG
         base->print();
#endif
         while (!points.empty()) {
            sarspfm_point p = points.front(); points.pop();
            int y = sarspfm_y(p);
            int x = sarspfm_x(p);
            if (x+1 < base->blocks.rows()) {
               int v = base->blocks(x+1,y);
               if (v == -1) {
                  base->blocks(x+1,y) = sarspfm_v(p)+1;
                  points.push((sarspfm_i(p)+(1L<<32))|(y<<16)|(x+1));
               }
               else if (((v>>16) - sarspfm_t(p)) && sarspfm_c(p) > 1) {
                  edge_points.insert((y<<16)|(x+1));
               }
            }
            if (x-1 >= 0) {
               int v = base->blocks(x-1,y);
               if (v == -1) {
                  base->blocks(x-1,y) = sarspfm_v(p)+1;
                  points.push((sarspfm_i(p)+(1L<<32))|(y<<16)|(x-1));
               }
               else if (((v>>16) - sarspfm_t(p)) && sarspfm_c(p) > 1) {
                  edge_points.insert((y<<16)|(x-1));
               }
            }
            if (y+1 < base->blocks.cols()) {
               int v = base->blocks(x,y+1);
               if (v == -1) {
                  base->blocks(x,y+1) = sarspfm_v(p)+1;
                  points.push((sarspfm_i(p)+(1L<<32))|((y+1)<<16)|(x));
               }
               else if (((v>>16) - sarspfm_t(p)) && sarspfm_c(p) > 1) {
                  edge_points.insert(((y+1)<<16)|(x));
               }
            }
            if (y-1 >= 0) {
               int v = base->blocks(x,y-1);
               if (v == -1) {
                  base->blocks(x,y-1) = sarspfm_v(p)+1;
                  points.push((sarspfm_i(p)+(1L<<32))|((y-1)<<16)|(x));
               }
               else if (((v>>16) - sarspfm_t(p)) && sarspfm_c(p) > 1) {
                  edge_points.insert(((y-1)<<16)|(x));
               }
            }
         }
#ifdef SARSPFM_DEBUG
         base->print();
#endif
         gx = base->roor(goal->posx);
         gy = base->roor(goal->posy);
         sarspfm_point gp = ((gy<<16)|gx);
         sarspfm_point gpi = gp;
         while (!edge_points.count(gpi)) {
            edge_points.insert(gpi);
            gpi = max_surrounding(gpi);
         }
         for (int i = 0; i < base->blocks.rows(); ++i) {
            for (int j = 0; j < base->blocks.cols(); ++j) {
               if ((base->blocks(i,j)&0xffff) != 0)
                  base->blocks(i,j) = -1;
               else
                  base->blocks(i,j) = KUROME_SARSPFM_NOGO_VALUE;
            }
         }
#ifdef SARSPFM_DEBUG
         printf("%ld\n",edge_points.size());
         for (sarspfm_point s : edge_points)
            base->blocks(sarspfm_x(s),sarspfm_y(s)) = KUROME_SARSPFM_NOGO_VALUE;
         base->print();
#endif
         for (sarspfm_point s : edge_points)
            base->blocks(sarspfm_x(s),sarspfm_y(s)) = -1;
         base->blocks(gx,gy) = 0;
         points.push(gp);
         while (!points.empty()) {
            gpi = points.front(); points.pop();
            int y = sarspfm_y(gpi);
            int x = sarspfm_x(gpi);
            if (x+1 < base->blocks.rows()) {
               int v = base->blocks(x+1,y);
               if (v == -1 && edge_points.count((y<<16)|(x+1))) {
                  base->blocks(x+1,y) = sarspfm_v(gpi)+1;
                  points.push((sarspfm_i(gpi)+(1L<<32))|(y<<16)|(x+1));
               }
            }
            if (x-1 >= 0) {
               int v = base->blocks(x-1,y);
               if (v == -1 && edge_points.count((y<<16)|(x-1))) {
                  base->blocks(x-1,y) = sarspfm_v(gpi)+1;
                  points.push((sarspfm_i(gpi)+(1L<<32))|(y<<16)|(x-1));
               }
            }
            if (y+1 < base->blocks.cols()) {
               int v = base->blocks(x,y+1);
               if (v == -1 && edge_points.count(((y+1)<<16)|x)) {
                  base->blocks(x,y+1) = sarspfm_v(gpi)+1;
                  points.push((sarspfm_i(gpi)+(1L<<32))|((y+1)<<16)|(x));
               }
            }
            if (y-1 >= 0) {
               int v = base->blocks(x,y-1);
               if (v == -1 && edge_points.count(((y-1)<<16)|x)) {
                  base->blocks(x,y-1) = sarspfm_v(gpi)+1;
                  points.push((sarspfm_i(gpi)+(1L<<32))|((y-1)<<16)|(x));
               }
            }
         }
         for (int i = 0; i < base->blocks.rows(); ++i) {
            for (int j = 0; j < base->blocks.cols(); ++j) {
               if (base->blocks(i,j) == KUROME_SARSPFM_NOGO_VALUE)
                  base->blocks(i,j) = -2;
            }
         }
#ifdef SARSPFM_DEBUG
         base->print();
#endif
         for (int i = 0; i < base->blocks.rows(); ++i) {
            for (int j = 0; j < base->blocks.cols(); ++j) {
               if (base->blocks(i,j) == -2)
                  base->blocks(i,j) = KUROME_SARSPFM_NOGO_VALUE;
            }
         }
         for (sarspfm_point s : edge_points) {
            int y = sarspfm_y(s);
            int x = sarspfm_x(s);
            if (base->blocks(x,y) != -1)
               points.push((((long)base->blocks(x,y))<<32)|s);
         }
         while (!points.empty()) {
            gpi = points.front(); points.pop();
            int y = sarspfm_y(gpi);
            int x = sarspfm_x(gpi);
            if (x+1 < base->blocks.rows()) {
               int v = base->blocks(x+1,y);
               if (v == -1) {
                  base->blocks(x+1,y) = sarspfm_v(gpi)+1;
                  points.push((sarspfm_i(gpi)+(1L<<32))|(y<<16)|(x+1));
               }
            }
            if (x-1 >= 0) {
               int v = base->blocks(x-1,y);
               if (v == -1) {
                  base->blocks(x-1,y) = sarspfm_v(gpi)+1;
                  points.push((sarspfm_i(gpi)+(1L<<32))|(y<<16)|(x-1));
               }
            }
            if (y+1 < base->blocks.cols()) {
               int v = base->blocks(x,y+1);
               if (v == -1) {
                  base->blocks(x,y+1) = sarspfm_v(gpi)+1;
                  points.push((sarspfm_i(gpi)+(1L<<32))|((y+1)<<16)|(x));
               }
            }
            if (y-1 >= 0) {
               int v = base->blocks(x,y-1);
               if (v == -1) {
                  base->blocks(x,y-1) = sarspfm_v(gpi)+1;
                  points.push((sarspfm_i(gpi)+(1L<<32))|((y-1)<<16)|(x));
               }
            }
         }
      } while ((base->getIdx(self->posx,self->posy) == -1 || 
                base->getIdx(self->posx,self->posy) == KUROME_SARSPFM_NOGO_VALUE) &&
               !grids.empty());
      while (!grids.empty()) {
         Grid * g = grids.top(); grids.pop();
         delete g;
      }
      for (int i = 0; i < base->blocks.rows(); ++i) {
         for (int j = 0; j < base->blocks.cols(); ++j) {
            if (base->blocks(i,j) == KUROME_SARSPFM_NOGO_VALUE)
               base->blocks(i,j) = -2;
         }
      }
      base->print();
      for (int i = 0; i < base->blocks.rows(); ++i) {
         for (int j = 0; j < base->blocks.cols(); ++j) {
            if (base->blocks(i,j) == -2)
               base->blocks(i,j) = KUROME_SARSPFM_NOGO_VALUE;
         }
      }
      base->collectInfo(&binfo);
      mapped = 1;
   }

   Frame nextPoint(bool & done) {
      int sx = base->roob(self->posx);
      int sy = base->roob(self->posy);
      if (sx == gx && sy == gy) {
         done = true;
         return Frame(0,0,0,0,0,0);
      }
      sarspfm_point next = min_surrounding((sy<<16)|sx);
      return Frame(sarspfm_x(next)*step,sarspfm_y(next)*step,std::atan2(gx-sx,gy-sy),0,0,0);
   }

   int mstat(struct mapper_info * mi) {
      static char buf[40] = "SARSPFM Mapper";
      strncpy(mi->name,buf,39);
      mi->state = cutoff_cost;
      timespec_get(&mi->last,TIME_UTC);
      return 1;
   }

private:

   void init(int cutoff_cost) {
         edge_points = std::set<sarspfm_point>();
         points = std::queue<sarspfm_point>();
         grids = std::stack<Grid *>();
         seen = std::set<sarspfm_point>();
         this->cutoff_cost = cutoff_cost;
         base = new Grid(*env);
         double maxLen = (self->xwid > self->ywid ? self->xwid : self->ywid)*1.5;
         max_space.xwid = maxLen;
         max_space.ywid = maxLen;
         step = base->getUnitSize();
         do {
            grids.push(base);
            step *= 2;
            base = base->compress(step);
         } while(base->getHighBlocks()/step > 30);
         grids.push(base);
         base = NULL;
         mapped = 0;
   }

   void propagate_id(int id,int posx, int posy) {
      if (posx >= 0 && posx < base->blocks.rows() &&
          posy >= 0 && posy < base->blocks.cols()) {
         if (!base->blocks(posx,posy)) {
            points.push(((sarspfm_point)id<<48)|(posy<<16)|(posx));
            base->blocks(posx,posy) = id<<16; 
            propagate_id(id,posx-1,posy);
            propagate_id(id,posx+1,posy);
            propagate_id(id,posx,posy-1);
            propagate_id(id,posx,posy+1);
         }
      }
   }


   sarspfm_point max_surrounding(sarspfm_point x) {
      int grtst = 1;
      int grtsv = (sarspfm_x(x)-1 >= 0 ? base->blocks(sarspfm_x(x)-1,sarspfm_y(x)) : -1);
      if (grtsv < (sarspfm_x(x)+1 < base->blocks.rows() ? base->blocks(sarspfm_x(x)+1,sarspfm_y(x)) : -1)) {
         grtst = 2;
         grtsv = base->blocks(sarspfm_x(x)+1,sarspfm_y(x));
      }
      if (grtsv < (sarspfm_y(x)-1 >= 0 ? base->blocks(sarspfm_x(x),sarspfm_y(x)-1) : -1)) {
         grtst = 3;
         grtsv = base->blocks(sarspfm_x(x),sarspfm_y(x)-1);
      }
      if (grtsv < (sarspfm_y(x)+1 < base->blocks.cols() ? base->blocks(sarspfm_x(x),sarspfm_y(x)+1) : -1)) {
         grtst = 4;
         grtsv = base->blocks(sarspfm_x(x),sarspfm_y(x)+1);
      }
      switch(grtst) {
         case 1:
            return (x-1);
         case 2:
            return (x+1);
         case 3:
            return (x-(1L<<16));
         case 4:
            return (x+(1L<<16));
      }
      return (x-1);
   }

   sarspfm_point min_surrounding(sarspfm_point x) {
      int grtst = 1;
      int grtsv = (sarspfm_x(x)-1 >= 0 ? base->blocks(sarspfm_x(x)-1,sarspfm_y(x)) : INT_MAX);
      if (grtsv > (sarspfm_x(x)+1 < base->blocks.rows() ? base->blocks(sarspfm_x(x)+1,sarspfm_y(x)) : INT_MAX)) {
         grtst = 2;
         grtsv = base->blocks(sarspfm_x(x)+1,sarspfm_y(x));
      }
      if (grtsv > (sarspfm_y(x)-1 >= 0 ? base->blocks(sarspfm_x(x),sarspfm_y(x)-1) : INT_MAX)) {
         grtst = 3;
         grtsv = base->blocks(sarspfm_x(x),sarspfm_y(x)-1);
      }
      if (grtsv > (sarspfm_y(x)+1 < base->blocks.cols() ? base->blocks(sarspfm_x(x),sarspfm_y(x)+1) : INT_MAX)) {
         grtst = 4;
         grtsv = base->blocks(sarspfm_x(x),sarspfm_y(x)+1);
      }
      switch(grtst) {
         case 1:
            return (x-1);
         case 2:
            return (x+1);
         case 3:
            return (x-(1L<<16));
         case 4:
            return (x+(1L<<16));
      }
      return (x-1);
   }
};

#endif
