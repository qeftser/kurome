
#ifndef KUROME_LIFELONG_A_STAR_MAPPER

#define KUROME_LIFELONG_A_STAR_MAPPER

#include "../Kurome.h"
#include <map>
#include <unordered_map>
#include <algorithm>
#include <cfloat>

class LPAFrame {
public:
   uint64_t rhs;
   uint64_t g;
   int64_t h;
   int32_t x;
   int32_t y;
   int32_t r;
   LPAFrame * nextptr;

   uint64_t calculate_key(void) const {
      return ((g < rhs ? g : rhs)+h);
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
   binary_heap<LPAFrame *> Q;
   std::unordered_map<FrameId,LPAFrame *> M;
   LPAFrame * start;
   LPAFrame * goal;

   void callback(int flags) {

      if ((flags&KUROME_MFLAG_DATA)) {
         /* react to new data */
      }

      if ((flags&KUROME_MFLAG_MOVE) || (flags&KUROME_MFLAG_CMOVE)) {
         /* react to movement */
      }

      if ((flags&KUROME_MFLAG_GOAL) || (flags&KUROME_MFLAG_EXEC)) {
         reset();
      }

   }

private:

   inline LPAFrame * rem_queue(LPAFrame * f) {
      if (Q.contains(f->calculate_key()))
         return Q.remove(f->calculate_key());
   }

   void reset(void) {
      /* reset the pathing */
   }

   void compute_shortest_path(void) {
      /* no need to explain */
   }

   void update_vertex(LPAFrame * u) {
      if (u == start)
         return;
      uint64_t old = u->calculate_key();
      u->rhs = DBL_MAX;
   }


};

#endif
