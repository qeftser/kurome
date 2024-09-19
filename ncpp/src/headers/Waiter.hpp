
#ifndef KUROME_WAITER_CLASS

#define KUROME_WAITER_CLASS
#include <mutex>
#include "Sample.hpp"

class Waiter {
private:
   std::mutex m;
   Sample * s;
public:
   Waiter() : m(std::mutex()) {};

   void dish(Sample * news) { m.lock(); s = news; m.unlock(); };
   Sample * serve(void) { m.lock(); Sample * ret = s; s = NULL; m.unlock(); return ret; };
   int poll(void) { return s != NULL; };

   virtual void prepare() = 0;
   virtual int  wstat(struct waiter_info *) { return 0; };
};

#endif
