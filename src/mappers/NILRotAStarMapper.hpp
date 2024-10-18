#pragma once

#include "../Kurome.h"
#include <unordered_set>
#include <cstring>

class NILRotAStarMapper : public Mapper {
public:
   struct FrameCmp cmpFrames;
   std::priority_queue<Frame *,std::vector<Frame *>,decltype(cmpFrames)> currPaths;
   std::unordered_set<FrameId> visited;
   std::vector<Frame *> allocated;
   Frame best;

   NILRotAStarMapper(Agent * me) 
      : Mapper(me) {}; 
   NILRotAStarMapper(Reporter * me)
      : Mapper(me) {};
   NILRotAStarMapper() : Mapper() {};

   void callback(int flags) {
      (void)flags;
      for (Frame * f : allocated) 
         delete f;
      allocated.clear();
      while (!currPaths.empty())
         currPaths.pop();
      visited.clear();

      double destX = goal->posx;
      double destY = goal->posy;

      double startX = self->posx;
      double startY = self->posy;

      double step = env->getUnitSize();

      Frame dest{ destX, destY, 0.0, 0, 0, NULL };
      Frame start{ startX, startY, self->rot, 0, 0, NULL };
      currPaths.push(&start);

      while (!currPaths.empty()) {
         Frame * nFrame, * curr = currPaths.top();
         currPaths.pop();

         if (curr->dist2(dest) < step*3) {
            best = Frame(curr);
            return;
         }

         nFrame = new Frame(curr);
         nFrame->posx = curr->posx-step;
         nFrame->posy = curr->posy-step;
         if (env->inBounds(nFrame->posx,nFrame->posy) && !visited.count(nFrame->id())) {
            nFrame->nextptr = curr;
            nFrame->weight = 0.1*nFrame->dist2(dest) + nFrame->cost(*self,*env);
            currPaths.push(nFrame);
            allocated.push_back(nFrame);
            visited.emplace(nFrame->id());
         }
         else {
            delete (nFrame);
         }

         nFrame = new Frame(curr);
         nFrame->posx = curr->posx+step;
         nFrame->posy = curr->posy-step;
         if (env->inBounds(nFrame->posx,nFrame->posy) && !visited.count(nFrame->id())) {
            nFrame->nextptr = curr;
            nFrame->weight = 0.1*nFrame->dist2(dest) + nFrame->cost(*self,*env);
            currPaths.push(nFrame);
            allocated.push_back(nFrame);
            visited.emplace(nFrame->id());
         }
         else {
            delete (nFrame);
         }

         nFrame = new Frame(curr);
         nFrame->posx = curr->posx-step;
         nFrame->posy = curr->posy+step;
         if (env->inBounds(nFrame->posx,nFrame->posy) && !visited.count(nFrame->id())) {
            nFrame->nextptr = curr;
            nFrame->weight = 0.1*nFrame->dist2(dest) + nFrame->cost(*self,*env);
            currPaths.push(nFrame);
            allocated.push_back(nFrame);
            visited.emplace(nFrame->id());
         }
         else {
            delete (nFrame);
         }

         nFrame = new Frame(curr);
         nFrame->posx = curr->posx+step;
         nFrame->posy = curr->posy+step;
         if (env->inBounds(nFrame->posx,nFrame->posy) && !visited.count(nFrame->id())) {
            nFrame->nextptr = curr;
            nFrame->weight = 0.1*nFrame->dist2(dest) + nFrame->cost(*self,*env);
            currPaths.push(nFrame);
            allocated.push_back(nFrame);
            visited.emplace(nFrame->id());
         }
         else {
            delete (nFrame);
         }
      }
   }

   Frame nextPoint(bool & done) {
      if (!best.nextptr) {
         done = true;
         return best;
      }
      done = false;

      Frame * curr = &best;
      while (curr->nextptr->nextptr != NULL) {
         curr = curr->nextptr;
      }
      curr->nextptr = NULL;
      return curr;
   };

   int mstat(struct mapper_info * mi) {
      static char buf[40] = "NILRotAStarMapper";
      strncpy(mi->name,buf,39);
      mi->state = 0;
      timespec_get(&mi->last,TIME_UTC);
      return 1;
   }


};
