
#include "Kurome.h"
#include <unistd.h>

int main(void) {

   Reporter r;

   r.launchClient();

   sleep(10);

   r.kcmdConnect(r.avaliable->naddr);

   kcmd::getGrid(&r.reqs);

   sleep(30);

   r.environment->print();

   kcmd::setIdx(5,5,50,&r.reqs);

   sleep(60);

   r.environment->print();

   pause();

   return 0;
}
