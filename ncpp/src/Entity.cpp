
#include "Kurome.h"

Entity::Entity(double posx, double posy, double xwid, double ywid, int type, int val) {
   this->posx = posx;
   this->posy = posy;
   this->xwid = xwid;
   this->ywid = ywid;
   this->type = type;
   this->val  = val;
}

Entity::Entity(double xwid, double ywid, int type) {
   this->xwid = xwid;
   this->ywid = ywid;
   this->type = type;
}
