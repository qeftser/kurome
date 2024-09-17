
#include "Kurome.h"

Entity::Entity(double posx, double posy, double xwid, double ywid, int type, int val) 
   : posx(posx), posy(posy), xwid(xwid), ywid(ywid), type(type), val(val) {}

Entity::Entity(double xwid, double ywid, int type) 
   : posx(0), posy(0), xwid(xwid), ywid(ywid), type(type), val(0) {}

Entity::Entity(struct entity_struct * es)
   : posx(es->posx), posy(es->posy), xwid(es->xwid), ywid(es->ywid), type(es->type), val(es->val) {}

void Entity::toStruct(struct entity_struct * es) {
   es->posx = posx;
   es->posy = posy;
   es->xwid = xwid;
   es->ywid = ywid;
   es->type = type;
   es->val  = val;
}

double Entity::dist2(const Entity & other) {
   return (((other.posx-posx)*(other.posx-posx))+((other.posy-posy)*(other.posy-posy)));
}

void Entity::operator=(const Entity & other) {
   posx = other.posx;
   posy = other.posy;
   xwid = other.xwid;
   ywid = other.ywid;
   type = other.type;
   val  = other.val;
}
