
#ifndef KUROME_LIFELONG_A_STAR_MAPPER

#define KUROME_LIFELONG_A_STAR_MAPPER

#include "../Kurome.h"
#include <map>
#include <algorithm>

class LPAFrame : public Frame {
public:
   uint64_t rhs;
   uint64_t g;

   uint64_t calculate_key(void) const {
      return ((g < rhs ? g : rhs)+weight);
   }
};

struct LPAFrameCmp {
   bool operator()(const LPAFrame * f1, const LPAFrame * f2) const {
      if (f1->calculate_key() > f2->calculate_key())
         return true;
      return false;
   }
};

class LifelongAStarMapper : public Mapper {
public:

   struct LPAFrameCmp frameCmp;

   void callback(int flags) {

      if ((flags&KUROME_MFLAG_DATA)) {
         /* react to new data */
      }

      if ((flags&KUROME_MFLAG_MOVE) || (flags&KUROME_MFLAG_CMOVE)) {
         /* react to movement */
      }

      if ((flags&KUROME_MFLAG_GOAL) || (flags&KUROME_MFLAG_EXEC)) {
         /* react to goal change */
         reset();
      }

   }

private:

   inline void rem_queue(LPAFrame * f) {
   }

   inline void add_queue(LPAFrame * f) {
   }

   void reset(void) {
      /* reset the pathing */
   }

   void compute_shortest_path(void) {
      /* no need to explain */
   }

   void update_vertex(LPAFrame * u) {
      /* fix the cost of a frame */
   }


};

#endif
