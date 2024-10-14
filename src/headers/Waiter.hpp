
#ifndef KUROME_WAITER_CLASS

#define KUROME_WAITER_CLASS
#include <mutex>
#include "Sample.hpp"

/*! \class Waiter
 * Waiter base class. All waiters will need to prepare, dish, and serve Sample objects
 */
class Waiter {
private:
   std::mutex m;
   Sample * s; /**< The most recent Sample collected */
public:
   Waiter() : m(std::mutex()) {};

   /**
    * Set a new Sample as the most recent one. Should be called inside of the overriden prepare
    * method in the child class
    */
   void dish(Sample * news) { m.lock(); s = news; m.unlock(); };
   /**
    * Return and clear the newest Sample from the Waiter's store
    */
   Sample * serve(void) { m.lock(); Sample * ret = s; s = NULL; m.unlock(); return ret; };
   /**
    * Check if a new Sample is avaliable
    */
   int poll(void) { return s != NULL; };

   /**
    * Prepare a new Sample for useage by the main process
    */
   virtual void prepare() = 0;
   /**
    * Optional override. Provide info about the Waiter in use.
    * If overriding, set the return to 1 to indicate the method 
    * is returning meaningful output.
    */
   virtual int  wstat(struct waiter_info *) { return 0; };
};

#endif
