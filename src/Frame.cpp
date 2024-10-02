
#include "Kurome.h"

Frame::Frame(Frame * other) {
   memcpy(this,other,sizeof(Frame));
   this->num++;
   this->nextptr = other;
}

FrameId Frame::id() const {
   return (((FrameId)posx<<48)|((FrameId)posy<<24)|((FrameId)rot));
}

bool Frame::operator=(const Frame & other) {
   if (id() == other.id())
      return true;
   return false;
}

Frame * Frame::operator+(const Frame & other) {
   Frame * ret = new Frame(this);
   ret->posx = posx+other.posx;
   ret->posy = posy+other.posy;
   ret->rot = rot+other.rot;
   ret->num = num+1;
   ret->weight = weight+other.weight;
   ret->nextptr = this;
   return ret;
}

void Frame::operator+=(Frame & other) {
   posx += other.posx;
   posy += other.posy;
   rot += other.rot;
   num = other.num+1;
   nextptr = &other;
}

double Frame::dist2(const Frame & other) {
   return (((posx-other.posx)*(posx-other.posx))+((posy-other.posy)*(posy-other.posy)));
}

int Frame::cost(Entity & hitbox, Grid & env) {
   uint sum = 0;
   RectIterator ri;
   EllipseIterator ei;
   switch(hitbox.type) {
      case KUROME_TYPE_RECT:
         ri = RectIterator(posx,posy,&hitbox,&env);
         while (!ri.done) {
            sum += (uint)*ri;
            ++ri;
         }
         break;
      case KUROME_TYPE_ELPS:
         ei = EllipseIterator(posx,posy,&hitbox,&env);
         while (!ei.done) {
            sum += (uint)*ei;
            ++ei;
         }
         break;
      default:
         errnok = KUROME_ETYPE;
         return -1;
         break;
   }
   return sum;
}

void Frame::toStruct(struct frame_struct * s) {
   s->posx   = posx;
   s->posy   = posy;
   s->rot    = rot;
   s->num    = num;
   s->weight = weight;
}
