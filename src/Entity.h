
#ifndef ENTITY_OBJECT

#define ENTITY_OBJECT

#define ENTITY_TYPE_RECT   0
#define ENTITY_TYPE_POINT  1
#define ENTITY_TYPE_CIRCLE 2
#include <cstdlib>

class Entity {
public:
   int type;
   int danger;
   double dist1;
   double dist2;
   double posX;
   double posY;
   Entity(int danger, int type, double d1, double d2, double px, double py) :
       danger(danger), type(type), dist1(d1), dist2(d2), posX(px), posY(py) {}
};

class WeightedEntity : public Entity {
public:
   double weight;
   WeightedEntity(int danger, int type, double d1, double d2, double px, double py, double w) :
      Entity(danger,type,d1,d2,px,py), weight(w) {}
};

class MovingEntity : public Entity {
public:
   double turnRad;
   double angle;
   MovingEntity(int danger, int type, double d1, double d2, double px, double py, double r) :
      Entity(danger,type,d1,d2,px,py), turnRad(r), angle(1.579) {}
};

#endif
