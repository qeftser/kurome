
#ifndef ENTITY_OBJECT

#define ENTITY_OBJECT

#define ENTITY_TYPE_RECT   0
#define ENTITY_TYPE_POINT  1
#define ENTITY_TYPE_CIRCLE 2
#include <cstdlib>

/* General entity class */
class Entity {
public:
   /* one of:
    *    ENTITY_TYPE_RECT
    *    ENTITY_TYPE_POINT
    *    ENTITY_TYPE_CIRCLE
    */
   int type;
   /* How much do we want to avoid this entity? */
   int danger;
   /* for rect this is width, for circle this is radius */
   double dist1;
   /* for rect this is height */
   double dist2;
   /* x position of the entity's center */
   double posX;
   /* y position of the entity's center */
   double posY;
   Entity(int danger, int type, double d1, double d2, double px, double py) :
       danger(danger), type(type), dist1(d1), dist2(d2), posX(px), posY(py) {}
};

class WeightedEntity : public Entity {
public:
   /* How much weight should we give the new addition? */
   double weight;
   WeightedEntity(int danger, int type, double d1, double d2, double px, double py, double w) :
      Entity(danger,type,d1,d2,px,py), weight(w) {}
};

class MovingEntity : public Entity {
public:
   /* What is the turning radius of this entity? */
   double turnRad;
   /* current angle. Starts at 90 degrees, facing up */
   double angle;
   MovingEntity(int danger, int type, double d1, double d2, double px, double py, double r) :
      Entity(danger,type,d1,d2,px,py), turnRad(r), angle(1.579) {}
};

#endif
