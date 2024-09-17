#pragma once

#include "Kurome.h"
#include <unordered_set>

class NILRotAStarMapper : public Mapper {
public:
   struct FrameCmp cmpFrames;
   std::priority_queue<Frame *,std::vector<Frame *>,decltype(cmpFrames)> currPaths;
   std::unordered_set<FrameId> visited;
   std::vector<Frame *> allocated;
   Frame best;

   NILRotAStarMapper(Agent * me) 
      : Mapper(me) {}; 
   NILRotAStarMapper() : Mapper() {};

   void callback(int flags) {
      (void)flags;
      /*
      for (Frame * f : allocated) 
         free(f);
         */
      allocated.clear();
      while (!currPaths.empty())
         currPaths.pop();
      visited.clear();

      double destX = self->goal.posx;
      double destY = self->goal.posy;

      double startX = self->self.posx;
      double startY = self->self.posy;

      double step = self->environment.getUnitSize();

      Frame dest{ destX, destY, 0.0, 0, 0, NULL };
      Frame start{ startX, startY, 0.0, 0, 0, NULL };
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
         if (self->environment.inBounds(nFrame->posx,nFrame->posy) && visited.insert(nFrame->id()).second) {
            nFrame->nextptr = curr;
            nFrame->weight = nFrame->dist2(dest) + nFrame->cost(self->self,self->environment);
            currPaths.push(nFrame);
            allocated.push_back(nFrame);
         }
         else {
            //free(nFrame);
         }

         nFrame = new Frame(curr);
         nFrame->posx = curr->posx+step;
         nFrame->posy = curr->posy-step;
         if (self->environment.inBounds(nFrame->posx,nFrame->posy) && visited.insert(nFrame->id()).second) {
            nFrame->nextptr = curr;
            nFrame->weight = nFrame->dist2(dest) + nFrame->cost(self->self,self->environment);
            currPaths.push(nFrame);
            allocated.push_back(nFrame);
         }
         else {
            //free(nFrame);
         }

         nFrame = new Frame(curr);
         nFrame->posx = curr->posx-step;
         nFrame->posy = curr->posy+step;
         if (self->environment.inBounds(nFrame->posx,nFrame->posy) && visited.insert(nFrame->id()).second) {
            nFrame->nextptr = curr;
            nFrame->weight = nFrame->dist2(dest) + nFrame->cost(self->self,self->environment);
            currPaths.push(nFrame);
            allocated.push_back(nFrame);
         }
         else {
            //free(nFrame);
         }

         nFrame = new Frame(curr);
         nFrame->posx = curr->posx+step;
         nFrame->posy = curr->posy+step;
         if (self->environment.inBounds(nFrame->posx,nFrame->posy) && visited.insert(nFrame->id()).second) {
            nFrame->nextptr = curr;
            nFrame->weight = nFrame->dist2(dest) + nFrame->cost(self->self,self->environment);
            currPaths.push(nFrame);
            allocated.push_back(nFrame);
         }
         else {
            //free(nFrame);
         }
      }
   }

   Frame nextPoint() {
      if (!best.nextptr)
         return best;

      Frame * curr = &best;
      while (curr->nextptr->nextptr != NULL) {
         curr = curr->nextptr;
      }
      curr->nextptr = NULL;
      return curr;
   };


};
