
#include "headers/nearest.h"
#include <cstdlib>
#include <climits>
#include <cfloat>
#include <cstring>
#include <cstdio>

/**
 * Initialize a nearest node struct. 
 * Provide (in order):
 *    - pointer to fill
 *    - number of nodes to preallocate
 *    - number of bins in the x direction
 *    - number of bins in the y direction
 *    - bin size (double divisor)
 *    - x value for the seed point
 *    - y value for the seed point
 */
void nn_init(struct nn_base * n, unsigned int maxn, int binx, int biny, double divisor, double initx, double inity) {
   n->memory = calloc(sizeof(struct nn_node),maxn);
   n->binx = binx;
   n->biny = biny;
   n->bins = (int *)calloc(sizeof(int),binx*biny);
   n->divisor = divisor;
   n->memalloced = 0;
   n->nextfree = -1;
   struct nn_node * init = nn_alloc(n);
   struct nn_node * high_bound = nn_alloc(n);
   struct nn_node * low__bound = nn_alloc(n);
   init->u = init->r = nn_toref(n,high_bound);
   init->d = init->l = nn_toref(n,low__bound);
   high_bound->x = high_bound->y = 10000;
   low__bound->x = low__bound->y = -10000;
   high_bound->r = high_bound->u = 1;
   low__bound->l = low__bound->d = 2;
   init->x = initx;
   init->y = inity;
}

struct nn_node * nn_add_propagate(int found, struct nn_node * new_ptr, struct nn_node * guess, struct nn_base * n) {
   struct nn_node * l = nn_deref(n,guess->l);
   struct nn_node * r = nn_deref(n,guess->r);
   struct nn_node * u = nn_deref(n,guess->u);
   struct nn_node * d = nn_deref(n,guess->d);

   printf("z\n");
   printf("a %f %f\n",guess->x,guess->y);
   if (!found) {

   printf("L{%f %f} ",l->x,l->y);
   printf("R{%f %f} ",r->x,r->y);
   printf("U{%f %f} ",u->x,u->y);
   printf("D{%f %f}\n",d->x,d->y);
      printf("b\n");
      struct nn_node * best = guess;
      double best_dist = nn_dist(guess->x,guess->y,new_ptr->x,new_ptr->y);
      printf("c\n");
      printf("%p\n",l);
      if (nn_dist(new_ptr->x,new_ptr->y,l->x,l->y) < best_dist) {
      printf("d\n");
         best_dist = nn_dist(new_ptr->x,new_ptr->y,l->x,l->y);
         best = l;
      }
      if (nn_dist(new_ptr->x,new_ptr->y,r->x,r->y) < best_dist) {
      printf("e\n");
         best_dist = nn_dist(new_ptr->x,new_ptr->y,r->x,r->y);
         best = r;
      }
      if (nn_dist(new_ptr->x,new_ptr->y,u->x,u->y) < best_dist) {
      printf("f\n");
         best_dist = nn_dist(new_ptr->x,new_ptr->y,u->x,u->y);
         best = u;
      }
      if (nn_dist(new_ptr->x,new_ptr->y,d->x,d->y) < best_dist) {
      printf("g\n");
         best_dist = nn_dist(new_ptr->x,new_ptr->y,d->x,d->y);
         best = d;
      }
      if (best == guess) {
      printf("h\n");
         found = 1;
         new_ptr->l = guess->l;
         new_ptr->r = guess->r;
         new_ptr->u = guess->u;
         new_ptr->d = guess->d;
      }
      else 
         return nn_add_propagate(found,new_ptr,best,n);
   }

   switch((new_ptr->x > guess->x ? 1 : 0)|(new_ptr->y > guess->y ? 2 : 0)
         |(fabsl(new_ptr->x-guess->x)>fabsl(new_ptr->y-guess->y) ? 4 : 0)) {
      case 0:
      case 1:
         printf("1\n");
         if (nn_dist(new_ptr->x,new_ptr->y,guess->x,guess->y) < 
             nn_dist(guess->x,guess->y,d->x,d->y)) {
            printf("2\n");
            new_ptr->u = nn_toref(n,guess);
            guess->d    = nn_toref(n,new_ptr);
            nn_add_propagate(found,new_ptr,l,n);
            nn_add_propagate(found,new_ptr,r,n);
            nn_add_propagate(found,new_ptr,d,n);
         }
         break;
      case 2:
      case 3:
         printf("3\n");
         if (nn_dist(new_ptr->x,new_ptr->y,guess->x,guess->y) < 
             nn_dist(guess->x,guess->y,u->x,u->y)) {
            printf("4\n");
            new_ptr->d = nn_toref(n,guess);
            guess->u    = nn_toref(n,new_ptr);
            nn_add_propagate(found,new_ptr,l,n);
            nn_add_propagate(found,new_ptr,r,n);
            nn_add_propagate(found,new_ptr,u,n);
         }
         break;
      case 4:
      case 6:
         printf("5\n");
         if (nn_dist(new_ptr->x,new_ptr->y,guess->x,guess->y) < 
             nn_dist(guess->x,guess->y,l->x,l->y)) {
            printf("6\n");
            new_ptr->r = nn_toref(n,guess);
            guess->l    = nn_toref(n,new_ptr);
            nn_add_propagate(found,new_ptr,l,n);
            nn_add_propagate(found,new_ptr,u,n);
            nn_add_propagate(found,new_ptr,d,n);
         }
         break;
      case 5:
      case 7:
         printf("7\n");
         if (nn_dist(new_ptr->x,new_ptr->y,guess->x,guess->y) < 
             nn_dist(guess->x,guess->y,r->x,r->y)) {
            printf("8\n");
            new_ptr->l = nn_toref(n,guess);
            guess->r    = nn_toref(n,new_ptr);
            nn_add_propagate(found,new_ptr,r,n);
            nn_add_propagate(found,new_ptr,u,n);
            nn_add_propagate(found,new_ptr,d,n);
         }
         break;
   }
   return guess;
}

/**
 * Add a node to the nn_node structure. Return the nearest node
 * to the one added.
 */
struct nn_node * nn_add(struct nn_base * n, double x, double y) {
   struct nn_node * new_node = nn_alloc(n);
   int bin = nn_bin(n->divisor,n->binx,y,y);
   struct nn_node * guess = nn_deref(n,n->bins[bin]);
   new_node->x = x; new_node->y = y;

   return nn_add_propagate(0,new_node,guess,n);

   if (fmod(x,n->divisor)+fmod(y,n->divisor)<fmod(guess->x,n->divisor)+fmod(guess->y,n->divisor)) {
      n->bins[bin] = nn_toref(n,new_node);
   }
}

/**
 * Get the nearest node to the given point.
 */
struct nn_node * nn_get(struct nn_base * n, double x, double y) {
   struct nn_node * guess, * better_guess = nn_deref(n,n->bins[nn_bin(n->divisor,n->binx,y,y)]);
   double dist = nn_dist(guess->x,guess->y,x,y);
   do {
      guess = better_guess;
      if (nn_dist(nn_deref(n,guess->l)->x,nn_deref(n,guess->l)->y,x,y) < dist) {
         better_guess = nn_deref(n,guess->l);
         dist = nn_dist(better_guess->x,better_guess->y,x,y);
      }
      if (nn_dist(nn_deref(n,guess->r)->x,nn_deref(n,guess->r)->y,x,y) < dist) {
         better_guess = nn_deref(n,guess->r);
         dist = nn_dist(better_guess->x,better_guess->y,x,y);
      }
      if (nn_dist(nn_deref(n,guess->u)->x,nn_deref(n,guess->u)->y,x,y) < dist) {
         better_guess = nn_deref(n,guess->u);
         dist = nn_dist(better_guess->x,better_guess->y,x,y);
      }
      if (nn_dist(nn_deref(n,guess->d)->x,nn_deref(n,guess->d)->y,x,y) < dist) {
         better_guess = nn_deref(n,guess->d);
         dist = nn_dist(better_guess->x,better_guess->y,x,y);
      }
   } while (guess != better_guess);
   return guess;
}

/**
 * Delete the given node from the nn_base structure.
 */
/*
void nn_del(struct nn_base * n, struct nn_node * no) {
   struct nn_node * guess = nn_deref(n,no->l);
   double dist = nn_dist(guess->x,guess->y,no->x,no->y);
   if (nn_dist(nn_deref(n,no->r)->x,nn_deref(n,no->r)->y,no->x,no->y) < dist) {
      guess = nn_deref(n,guess->r);
      dist = nn_dist(guess->x,guess->y,no->x,no->y);
   }
   if (nn_dist(nn_deref(n,no->u)->x,nn_deref(n,no->u)->y,no->x,no->y) < dist) {
      guess = nn_deref(n,guess->u);
      dist = nn_dist(guess->x,guess->y,no->x,no->y);
   }
   if (nn_dist(nn_deref(n,no->d)->x,nn_deref(n,no->d)->y,no->x,no->y) < dist) {
      guess = nn_deref(n,guess->d);
      dist = nn_dist(guess->x,guess->y,no->x,no->y);
   }
}
*/

/**
 * Deallocate all memory associated with the nn_base structure.
 */
void nn_destroy(struct nn_base * n) {
   free(n->memory);
   free(n->bins);
   bzero(n,sizeof(nn_base));
}
