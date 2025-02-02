
#ifndef KUROME_CLASS_MAPPER

#define KUROME_CLASS_MAPPER
#include "Frame.h"
#include "Agent.h"
#include "Reporter.h"

/*! \class Mapper
 * abstract class for all mappers */
class Mapper {
public:
   Grid   * env;  /**< the environment to map over */
   Entity * self; /**< our self, where we are and what our hitbox is */
   Entity * goal; /**< our goal, where we want to go */

   /**
    * construct a mapper with the provided enviroment, goal, and self */
   Mapper(Grid * e, Entity * s, Entity * g)
      : env(e), self(s), goal(g) {}
   /**
    * construct a mapper using the provided Agent */
   Mapper(Agent * self) 
      : env(&self->environment), self(&self->self), goal(&self->goal) {}
   /** 
    * construct a mapper using the provided Reporter */
   Mapper(Reporter * self) 
      : env(&(*self->environment)), self(&(*self->self)), goal(&(*self->goal)) {}
   /**
    * construct an empty mapper */
   Mapper() {}

   /**
    * return the next point in the path. Set the bool to true
    * if more points exist and false if the path is exhausted */
   virtual Frame nextPoint(bool &) = 0;
   /**
    * inform the mapper to remap based on changes to the environment,
    * self, or goal parameters */
   virtual void callback(int) = 0;
   /**
    * optional to define. Provide information about this mapper
    * and return one if supported */
   virtual int  mstat(struct mapper_info *) { return 0; };
};

#endif
