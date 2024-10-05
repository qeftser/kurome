
#ifndef KUROME_CLASS_MAPPER

#define KUROME_CLASS_MAPPER
#include "Frame.h"
#include "Agent.h"
#include "Reporter.h"

class Mapper {
public:
   Grid   * env;
   Entity * self;
   Entity * goal;

   Mapper(Grid * e, Entity * s, Entity * g)
      : env(e), self(s), goal(g) {}
   Mapper(Agent * self) 
      : env(&self->environment), self(&self->self), goal(&self->goal) {}
   Mapper(Reporter * self) 
      : env(&(*self->environment)), self(&(*self->self)), goal(&(*self->goal)) {}
   Mapper() {}

   virtual Frame nextPoint(bool &) = 0;
   virtual void callback(int) = 0;
   virtual int  mstat(struct mapper_info *) { return 0; };
};

#endif
