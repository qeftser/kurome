
#ifndef KUROME_ENTITY_CLASS

#define KUROME_ENTITY_CLASS
#include "kmsgs.h"

class Entity {
public:
   double posx;
   double posy;
   double xwid;
   double ywid;
   double rot;
   int    type;
   int    val;
   
   Entity(double posx, double posy, double xwid, double ywid, int type, int val) 
      : posx(posx), posy(posy), xwid(xwid), ywid(ywid), rot(0.0), type(type), val(val) {}

   Entity(double posx, double posy, double xwid, double ywid, double rot, int type, int val) 
      : posx(posx), posy(posy), xwid(xwid), ywid(ywid), rot(rot), type(type), val(val) {}

   Entity(double xwid, double ywid, int type) 
      : posx(0), posy(0), xwid(xwid), ywid(ywid), rot(0.0), type(type), val(0) {}

   Entity(struct entity_struct * es)
      : posx(es->posx), posy(es->posy), xwid(es->xwid), ywid(es->ywid), rot(es->rot), type(es->type), val(es->val) {}

   void toStruct(struct entity_struct * es) {
      es->posx = posx;
      es->posy = posy;
      es->xwid = xwid;
      es->ywid = ywid;
      es->rot  = rot;
      es->type = type;
      es->val  = val;
   }

   double dist2(const Entity & other) {
      return (((other.posx-posx)*(other.posx-posx))+((other.posy-posy)*(other.posy-posy)));
   }

   void operator=(const Entity & other) {
      posx = other.posx;
      posy = other.posy;
      xwid = other.xwid;
      ywid = other.ywid;
      rot  = other.rot;
      type = other.type;
      val  = other.val;
   }
};

#endif
