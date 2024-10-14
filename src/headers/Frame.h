
#ifndef KUROME_FRAME_CLASS

#define KUROME_FRAME_CLASS
#include <cstdint>
#include "Grid.h"
#include "kmsgs.h"

/**
 * Type for the result of the Frame method id
 */
typedef uint64_t FrameId;
/*! \class Frame
 * Holds a step in the pathfinding process for A* based pathfinders. 
 * I figured A* algorithms are similar enough that some of this
 * functionality would get reused, so I flushed it out in a full
 * class.
 */
class Frame {
public:
   double posx;      /**< x position of the Frame */
   double posy;      /**< y position of the Frame */
   double rot;       /**< rotation of the Frame */
   uint64_t num;     /**< the step in the process this Frame is at */
   uint64_t weight;  /**< computed weight of this Frame */
   Frame * nextptr;  /**< the next Frame in the chain */

   /** 
    * Construct a Frame with no values set
    */
   Frame() : nextptr(NULL) {};
   /** 
    * Construct a Frame by setting all variables 
    */
   Frame(double posx, double posy, double rot, uint64_t num, uint64_t weight, Frame * nextptr) 
      : posx(posx), posy(posy), rot(rot), num(num), weight(weight), nextptr(nextptr) {}
   /**
    * Construct a Frame using another as a template. Copies all data, but incriments the num and
    * sets the nextptr to the provided Frame by default.
    */
   Frame(Frame *);
   /**
    * Construct a Frame from a given frame_struct
    */
   Frame(struct frame_struct * s)
      : posx(s->posx), posy(s->posy), rot(s->rot), num(s->num), weight(s->weight), nextptr(NULL) {}

   /**
    * Return a FrameId for the given frame based on it's position and rotation 
    */
   FrameId id() const;
   /**
    * Check if two Frames have the same FrameId
    */
   bool operator==(const Frame &);
   /**
    * Add two frames to produce a new Frame. Increments the num
    * and sets nextptr to the left-hand Frame
    */
   Frame * operator+(const Frame &);
   /**
    * Add the values of the passed frame to this Frame.
    * Incriments the num and sets nextptr to the provided Frame
    */
   void operator+=(Frame &);
   /**
    * Return the sum of the weights inside the provided hitbox at the Frame
    * position in the provided environment
    */
   int cost(Entity &, Grid &);
   /**
    * Return the distance squared between this Frame and the provided one
    */
   double dist2(const Frame &);
   /**
    * Copy this Frame's values into the provided frame_struct
    */
   void toStruct(struct frame_struct *);
};

/**
 * Struct just for Frame comparison in stl classes. Compares
 * based on Frame weight
 */
struct FrameCmp {
   bool operator()(const Frame * f1, const Frame * f2) const {
      if (f1->weight > f2->weight)
         return true;
      return false;
   }
};

#endif
