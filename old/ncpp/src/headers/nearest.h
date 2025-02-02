
#ifndef NEAREST_NEIGHBOR

#define NEAREST_NEIGHBOR
#include <cmath>

struct nn_base {
   void * memory;
   int nextfree;
   unsigned int memalloced;
   int * bins;
   int binx;
   int biny;
   double divisor;
};

struct nn_node {
   double x;
   double y;
   int l, r, u, d;
};

#define nn_deref(n,val) (((nn_node *)n->memory)+val)
#define nn_toref(n,val) ((int)((((long)val-(long)(n->memory))/sizeof(nn_node))))
#define nn_bin(divisor,binx,posx,posy) (((int)(posy/divisor)*binx+(int)(posx/divisor)))
#define nn_alloc(n) (struct nn_node *)((nn_node *)n->memory+n->memalloced++)
#define nn_dist(x1,y1,x2,y2) (((x1-x2)*(x1-x2))+((y1-y2)*(y1-y2)))

void nn_init(struct nn_base * n, unsigned int maxn, int binx, int biny, double divisor, double initx, double inity);
struct nn_node * nn_add(struct nn_base *, double, double);
struct nn_node * nn_get(struct nn_base *, double, double);

/* not supported right now... */
void nn_del(struct nn_base *, struct nn_node *);

void nn_destroy(struct nn_base *);

#endif
