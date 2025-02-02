
#ifndef KUROME_GOOD_A_STAR_MAPPER 

#define KUROME_GOOD_A_STAR_MAPPER
#include "../Kurome.h"

class BasicFrame {
public:
   uint64_t val;
   BasicFrame * nextptr;
   BasicFrame * prevptr;
   int32_t x;
   int32_t y;
   int32_t r;
};

struct BasicFrameCmp {
   bool operator()(const BasicFrame * f1, const BasicFrame * f2) const {
      if (f1->val < f2->val)
         return true;
      return false;
   }
};

class GoodAStarMapper : public Mapper {
public:

   std::unordered_map<uint64_t,BasicFrame *> M;
   BasicFrame * start;
   BasicFrame * end;

   void callback(int flags) {

      if ((flags&KUROME_MFLAG_DATA)) {
         /* react to new data */
      }

      if ((flags&KUROME_MFLAG_MOVE) || (flags&KUROME_MFLAG_CMOVE)) {
         /* react to movement */
      }

      if ((flags&KUROME_MFLAG_GOAL) || (flags&KUROME_MFLAG_EXEC)) {
         /* react to flags */
      }

   }


};

#endif
