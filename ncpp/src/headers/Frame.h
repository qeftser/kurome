
#ifndef KUROME_FRAME_CLASS

#define KUROME_FRAME_CLASS
#include <cstdint>
#include "Grid.h"
#include "kmsgs.h"

typedef uint64_t FrameId;
class Frame {
public:
   double posx;
   double posy;
   double rot;
   uint64_t num;
   uint64_t weight;
   Frame * nextptr;

   Frame() {};
   Frame(double posx, double posy, double rot, uint64_t num, uint64_t weight, Frame * nextptr) 
      : posx(posx), posy(posy), rot(rot), num(num), weight(weight), nextptr(nextptr) {}
   Frame(Frame *);
   Frame(struct frame_struct * s)
      : posx(s->posx), posy(s->posy), rot(s->rot), num(s->num), weight(s->weight), nextptr(NULL) {}

   FrameId id() const;
   bool operator=(const Frame &);
   Frame * operator+(const Frame &);
   void operator+=(Frame &);
   int cost(Entity &, Grid &);
   double dist2(const Frame &);
   void toStruct(struct frame_struct *);
};

struct FrameCmp {
   bool operator()(const Frame * f1, const Frame * f2) const {
      if (f1->weight > f2->weight)
         return true;
      return false;
   }
};

#endif
