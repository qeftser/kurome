
#include "../Kurome.h"

class NullMapper : public Mapper {

   public:

      NullMapper() :
         Mapper() {}

      void callback(int flags) {
         (void)flags;
      }

      Frame nextPoint(bool & done) {
         done = true;
         return Frame(0,0,0,0,0,0);
      }

      ~NullMapper() {}

};
