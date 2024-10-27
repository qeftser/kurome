
#include "Kurome.h"
#include "mappers/SimpleAnyRotStaticPotentialFieldMapper.hpp"
#include "mappers/NullMapper.hpp"

int main(void) {


   Grid g = Grid(0.5,0,100,0,100);
   Entity me = Entity(10,10,4,4,KUROME_TYPE_ELPS,0);
   Entity goal = Entity(90,90,0,0,KUROME_TYPE_PONT,0);

   (void)Generator::sparse(&g,130,4,7);
   g.print();

   Mapper * m = NULL;
   Agent agent = Agent(me,goal,m,g);

   m = new SimpleAnyRotStaticPotentialFieldMapper(10,&agent);
   agent.mapper = m;

   m->callback(0);


   return 0;
}
