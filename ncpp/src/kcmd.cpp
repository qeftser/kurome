
#include "Kurome.h"
#include "kmsgs.h"

struct kurome_basemsg * wrap_basemsg(void * ptr, int len) {
   struct kurome_basemsg * ret = (KB *)calloc(len+sizeof(KB),1);
   memcpy(&ret->more,ptr,len);
   return ret;
}

void kurome_entity_req_base(Entity * e, ll_queue<KB *> * reqs, int mtype) {
   struct entity_struct em;
   e->toStruct(&em);
   struct kurome_entitymsg * msg = (struct kurome_entitymsg *)wrap_basemsg(&em,sizeof(struct entity_struct));
   msg->type = mtype;
   msg->size = sizeof(struct entity_struct);
   reqs->enqueue((KB *)msg);
}

void kurome_flags_req_base(int flags, ll_queue<KB *> * reqs, int mtype) {
   struct kurome_flagmsg * fm = (struct kurome_flagmsg *)malloc(sizeof(struct kurome_flagmsg));
   fm->type = mtype;
   fm->size = sizeof(struct kurome_flagmsg);
   fm->flag = flags;
   reqs->enqueue((KB *)fm);
}

void kurome_base_req_base(ll_queue<KB *> * reqs, int mtype) {
   struct kurome_basemsg * bm = (KB *)malloc(sizeof(struct kurome_basemsg));
   bm->type = mtype;
   bm->size = sizeof(struct kurome_basemsg);
   reqs->enqueue(bm);
}

void kurome_chg_req_base(double val, ll_queue<KB *> * reqs, int mtype) {
   struct kurome_chgsizemsg * csm = (struct kurome_chgsizemsg *)malloc(sizeof(struct kurome_chgsizemsg));
   csm->type = mtype;
   csm->size = sizeof(struct kurome_chgsizemsg);
   csm->unitSize = val;
   reqs->enqueue((KB *)csm);
}

void kurome_grid_req_base(Grid * g, ll_queue<KB *> * reqs, int mtype) {
   struct grid_struct * gs = (struct grid_struct *)malloc(sizeof(struct grid_struct));
   struct kurome_gridmsg * msg = (struct kurome_gridmsg *)malloc(sizeof(struct kurome_gridmsg));
   msg->type = mtype;
   msg->size = sizeof(KB)+g->toStruct(&gs);
   msg = (struct kurome_gridmsg *)realloc(msg,msg->size);
   memcpy(&msg->gs,gs,msg->size-sizeof(KB));
   free(gs);
   reqs->enqueue((KB *)msg);
}

void kcmd::addEntity(Entity * e, ll_queue<KB *> * reqs) {
   kurome_entity_req_base(e,reqs,KUROME_MSG_ADD_ENTITY);
}

void kcmd::entity(Entity * e, ll_queue<KB *> * reqs) {
   kurome_entity_req_base(e,reqs,KUROME_MSG_ENTITY);
}

void kcmd::goal(Entity * e, ll_queue<KB *> * reqs) {
   kurome_entity_req_base(e,reqs,KUROME_MSG_GOAL);
}

void kcmd::self(Entity * e, ll_queue<KB *> * reqs) {
   kurome_entity_req_base(e,reqs,KUROME_MSG_SELF);
}

void kcmd::chgSelf(Entity * e, ll_queue<KB *> * reqs) {
   kurome_entity_req_base(e,reqs,KUROME_MSG_CHGSELF);
}

void kcmd::chgGoal(Entity * e, ll_queue<KB *> * reqs) {
   kurome_entity_req_base(e,reqs,KUROME_MSG_CHGGOAL);
}

void kcmd::chgFlags(int flags, ll_queue<KB *> * reqs) {
   kurome_flags_req_base(flags,reqs,KUROME_MSG_CHGFLAGS);
}

void kcmd::mapCallback(int flags, ll_queue<KB *> * reqs) {
   kurome_flags_req_base(flags,reqs,KUROME_MSG_MAPCALLBACK);
}

void kcmd::setIdx(double xpos, double ypos, int weight, ll_queue<KB *> * reqs) {
   struct kurome_setidxmsg * sim = (struct kurome_setidxmsg *)malloc(sizeof(struct kurome_setidxmsg));
   sim->type = KUROME_MSG_SET_IDX;
   sim->size = sizeof(struct kurome_setidxmsg);
   sim->posx = xpos;
   sim->posy = ypos;
   sim->weight = weight;
   reqs->enqueue((KB *)sim);
}

void kcmd::clear(ll_queue<KB *> * reqs) {
   kurome_base_req_base(reqs,KUROME_MSG_CLEAR);
}

void kcmd::allSamples(ll_queue<KB *> * reqs) {
   kurome_base_req_base(reqs,KUROME_MSG_ALLSAMPLES);
}

void kcmd::allEntities(ll_queue<KB *> * reqs) {
   kurome_base_req_base(reqs,KUROME_MSG_ALLENTITIES);
}

void kcmd::chgUnits(double units, ll_queue<KB *> * reqs) {
   kurome_chg_req_base(units,reqs,KUROME_MSG_CHG_UNITSIZE);
}

void kcmd::chgX(double x, ll_queue<KB *> * reqs) {
   kurome_chg_req_base(x,reqs,KUROME_MSG_CHG_XBLOCKS);
}

void kcmd::chgY(double y, ll_queue<KB *> * reqs) {
   kurome_chg_req_base(y,reqs,KUROME_MSG_CHG_YBLOCKS);
}

void kcmd::getGrid(ll_queue<KB *> * reqs) {
   kurome_base_req_base(reqs,KUROME_MSG_GET_GRID);
}

void kcmd::getFullGrid(ll_queue<KB *> * reqs) {
   kurome_base_req_base(reqs,KUROME_MSG_GET_FULLGRID);
}

void kcmd::sample(Sample * s, ll_queue<KB *> * reqs) {
   struct sample_struct * ss = (struct sample_struct *)malloc(sizeof(struct sample_struct));
   struct kurome_samplemsg * msg = (struct kurome_samplemsg *)malloc(sizeof(struct kurome_samplemsg));
   msg->type = KUROME_MSG_SAMPLE;
   msg->size = sizeof(KB)+s->toStruct(&ss);
   msg = (struct kurome_samplemsg *)realloc(msg,msg->size);
   memcpy(&msg->s,ss,msg->size-sizeof(KB));
   free(ss);
   reqs->enqueue((KB *)msg);
}

void kcmd::waiterInfo(Waiter * w, ll_queue<KB *> * reqs) {
   struct waiter_info wf;
   if (w->wstat(&wf)) {
      struct kurome_waiterinfomsg * msg = (struct kurome_waiterinfomsg *)malloc(sizeof(struct kurome_waiterinfomsg));
      msg->size = (sizeof(struct kurome_waiterinfomsg));
      msg->type = KUROME_MSG_WAITERINFO;
      memcpy(&msg->wi,&wf,sizeof(struct waiter_info));
      reqs->enqueue((KB *)msg);
   }
}

void kcmd::mapperInfo(Mapper * m, ll_queue<KB *> * reqs) {
   struct mapper_info mf;
   if (m->mstat(&mf)) {
      struct kurome_mapperinfomsg * msg = (struct kurome_mapperinfomsg *)malloc(sizeof(struct kurome_mapperinfomsg));
      msg->size = (sizeof(struct kurome_mapperinfomsg));
      msg->type = KUROME_MSG_MAPPERINFO;
      memcpy(&msg->mi,&mf,sizeof(struct mapper_info));
      reqs->enqueue((KB *)msg);
   }
}

void kcmd::grid(Grid * g, ll_queue<KB *> * reqs) {
   kurome_grid_req_base(g,reqs,KUROME_MSG_GRID);
}

void kcmd::fullGrid(Grid * g, ll_queue<KB *> * reqs) {
   kurome_grid_req_base(g,reqs,KUROME_MSG_FULLGRID);
}

