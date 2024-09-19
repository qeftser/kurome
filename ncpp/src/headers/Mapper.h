
#ifndef KUROME_CLASS_MAPPER

#define KUROME_CLASS_MAPPER
#include "Frame.h"

class Mapper {
public:
   Agent * self;

   Mapper(Agent * me)
      : self(me) {}
   Mapper() {}

   virtual Frame nextPoint() = 0;
   virtual void callback(int) = 0;
   virtual int  mstat(struct mapper_info *) { return 0; };
};

#endif
