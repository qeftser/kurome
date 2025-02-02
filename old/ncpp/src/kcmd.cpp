
#include "Kurome.h"

struct kurome_basemsg * wrap_basemsg(void * ptr, int len) {
   struct kurome_basemsg * ret = (KB *)calloc(len+sizeof(KB),1);
   memcpy(&ret->more,ptr,len);
   return ret;
}

void kurome_entity_req_base(Entity & e, khandle * reqs, int mtype) {
   struct entity_struct em;
   e.toStruct(&em);
   struct kurome_entitymsg * msg = (struct kurome_entitymsg *)wrap_basemsg(&em,sizeof(struct entity_struct));
   msg->type = mtype;
   msg->size = sizeof(struct kurome_entitymsg);
   reqs->enqueue((KB *)msg);
}

void kurome_int_req_base(int val, khandle * reqs, int mtype) {
   struct kurome_intmsg * im = (struct kurome_intmsg *)malloc(sizeof(struct kurome_intmsg));
   im->type = mtype;
   im->size = sizeof(struct kurome_intmsg);
   im->val = val;
   reqs->enqueue((KB *)im);
}

void kurome_base_req_base(khandle * reqs, int mtype) {
   struct kurome_basemsg * bm = (KB *)malloc(sizeof(struct kurome_basemsg));
   bm->type = mtype;
   bm->size = sizeof(struct kurome_basemsg);
   reqs->enqueue(bm);
}

void kurome_double_req_base(double val, khandle * reqs, int mtype) {
   struct kurome_doublemsg * dsm = (struct kurome_doublemsg *)malloc(sizeof(struct kurome_doublemsg));
   dsm->type = mtype;
   dsm->size = sizeof(struct kurome_doublemsg);
   dsm->val = val;
   reqs->enqueue((KB *)dsm);
}

void kurome_grid_req_base(Grid & g, khandle * reqs, int mtype) {
   struct grid_struct * gs = (struct grid_struct *)malloc(sizeof(struct grid_struct));
   struct kurome_gridmsg * msg = (struct kurome_gridmsg *)malloc(sizeof(struct kurome_gridmsg));
   msg->type = mtype;
   msg->size = sizeof(KB)+g.toStruct(&gs);
   msg = (struct kurome_gridmsg *)realloc(msg,msg->size);
   memcpy(&msg->gs,gs,msg->size-sizeof(KB));
   free(gs);
   reqs->enqueue((KB *)msg);
}

void kcmd::addEntity(Entity & e, khandle * reqs) {
   kurome_entity_req_base(e,reqs,KUROME_MSG_ADD_ENTITY);
}

void kcmd::fAddEntity(Entity & e, khandle * reqs) {
   kurome_entity_req_base(e,reqs,KUROME_MSG_FADD_ENTITY);
}

void kcmd::entity(Entity & e, khandle * reqs) {
   kurome_entity_req_base(e,reqs,KUROME_MSG_ENTITY);
}

void kcmd::fEntity(Entity & e, khandle * reqs) {
   kurome_entity_req_base(e,reqs,KUROME_MSG_FENTITY);
}

void kcmd::fChgEntity(Entity & e, khandle * reqs) {
   kurome_entity_req_base(e,reqs,KUROME_MSG_FCHGENTITY);
}

void kcmd::fRemEntity(Entity & e, khandle * reqs) {
   kurome_entity_req_base(e,reqs,KUROME_MSG_FREMENTITY);
}

void kcmd::goal(Entity & e, khandle * reqs) {
   kurome_entity_req_base(e,reqs,KUROME_MSG_GOAL);
}

void kcmd::self(Entity & e, khandle * reqs) {
   kurome_entity_req_base(e,reqs,KUROME_MSG_SELF);
}

void kcmd::chgSelf(Entity & e, khandle * reqs) {
   kurome_entity_req_base(e,reqs,KUROME_MSG_CHGSELF);
}

void kcmd::chgGoal(Entity & e, khandle * reqs) {
   kurome_entity_req_base(e,reqs,KUROME_MSG_CHGGOAL);
}

void kcmd::chgFlags(int flags, khandle * reqs) {
   kurome_int_req_base(flags,reqs,KUROME_MSG_CHGFLAGS);
}

void kcmd::mapCallback(int flags, khandle * reqs) {
   kurome_int_req_base(flags,reqs,KUROME_MSG_MAPCALLBACK);
}

void kcmd::setIdx(double xpos, double ypos, int weight, khandle * reqs) {
   struct kurome_setidxmsg * sim = (struct kurome_setidxmsg *)malloc(sizeof(struct kurome_setidxmsg));
   sim->type = KUROME_MSG_SET_IDX;
   sim->size = sizeof(struct kurome_setidxmsg);
   sim->posx = xpos;
   sim->posy = ypos;
   sim->weight = weight;
   reqs->enqueue((KB *)sim);
}

void kcmd::clear(khandle * reqs) {
   kurome_base_req_base(reqs,KUROME_MSG_CLEAR);
}

void kcmd::fClear(khandle * reqs) {
   kurome_base_req_base(reqs,KUROME_MSG_FCLEAR);
}

void kcmd::clense(khandle * reqs) {
   kurome_base_req_base(reqs,KUROME_MSG_CLENSE);
}

void kcmd::fClense(khandle * reqs) {
   kurome_base_req_base(reqs,KUROME_MSG_FCLENSE);
}

void kcmd::getSelf(khandle * reqs) {
   kurome_base_req_base(reqs,KUROME_MSG_GETSELF);
}

void kcmd::getGoal(khandle * reqs) {
   kurome_base_req_base(reqs,KUROME_MSG_GETGOAL);
}

void kcmd::getMapperInfo(khandle * reqs) {
   kurome_base_req_base(reqs,KUROME_MSG_GETMAPPER);
}

void kcmd::getWaiterInfo(khandle * reqs) {
   kurome_base_req_base(reqs,KUROME_MSG_GETWAITERS);
}

void kcmd::allSamples(khandle * reqs) {
   kurome_base_req_base(reqs,KUROME_MSG_ALLSAMPLES);
}

void kcmd::allEntities(khandle * reqs) {
   kurome_base_req_base(reqs,KUROME_MSG_ALLENTITIES);
}

void kcmd::fAllSamples(khandle * reqs) {
   kurome_base_req_base(reqs,KUROME_MSG_FALLSAMPLES);
}

void kcmd::fAllEntities(khandle * reqs) {
   kurome_base_req_base(reqs,KUROME_MSG_FALLENTITIES);
}

void kcmd::chgUnits(double units, khandle * reqs) {
   kurome_double_req_base(units,reqs,KUROME_MSG_CHG_UNITSIZE);
}

void kcmd::chgX(double x, khandle * reqs) {
   kurome_double_req_base(x,reqs,KUROME_MSG_CHG_XBLOCKS);
}

void kcmd::chgY(double y, khandle * reqs) {
   kurome_double_req_base(y,reqs,KUROME_MSG_CHG_YBLOCKS);
}

void kcmd::getGrid(khandle * reqs) {
   kurome_base_req_base(reqs,KUROME_MSG_GET_GRID);
}

void kcmd::getFullGrid(khandle * reqs) {
   kurome_base_req_base(reqs,KUROME_MSG_GET_FULLGRID);
}

void kcmd::start(khandle * reqs) {
   kurome_base_req_base(reqs,KUROME_MSG_START);
}

void kcmd::stop(khandle * reqs) {
   kurome_base_req_base(reqs,KUROME_MSG_STOP);
}

void kcmd::pause(khandle * reqs) {
   kurome_base_req_base(reqs,KUROME_MSG_PAUSE);
}

void kcmd::state(khandle * reqs) {
   kurome_base_req_base(reqs,KUROME_MSG_STATE);
}

void kcmd::sample(Sample & s, khandle * reqs) {
   struct sample_struct * ss = (struct sample_struct *)malloc(sizeof(struct sample_struct));
   struct kurome_samplemsg * msg = (struct kurome_samplemsg *)malloc(sizeof(struct kurome_samplemsg));
   msg->type = KUROME_MSG_SAMPLE;
   msg->size = sizeof(KB)+s.toStruct(&ss);
   msg = (struct kurome_samplemsg *)realloc(msg,msg->size);
   memcpy(&msg->s,ss,msg->size-sizeof(KB));
   free(ss);
   reqs->enqueue((KB *)msg);
}

void kcmd::fSample(Sample & s, khandle * reqs) {
   struct sample_struct * ss = (struct sample_struct *)malloc(sizeof(struct sample_struct));
   struct kurome_samplemsg * msg = (struct kurome_samplemsg *)malloc(sizeof(struct kurome_samplemsg));
   msg->type = KUROME_MSG_FSAMPLE;
   msg->size = sizeof(KB)+s.toStruct(&ss);
   msg = (struct kurome_samplemsg *)realloc(msg,msg->size);
   memcpy(&msg->s,ss,msg->size-sizeof(KB));
   free(ss);
   reqs->enqueue((KB *)msg);
}

void kcmd::waiterInfo(Waiter & w, khandle * reqs) {
   struct waiter_info wf;
   if (w.wstat(&wf)) {
      struct kurome_waiterinfomsg * msg = (struct kurome_waiterinfomsg *)malloc(sizeof(struct kurome_waiterinfomsg));
      msg->size = (sizeof(struct kurome_waiterinfomsg));
      msg->type = KUROME_MSG_WAITERINFO;
      memcpy(&msg->wi,&wf,sizeof(struct waiter_info));
      reqs->enqueue((KB *)msg);
   }
}

void kcmd::mapperInfo(Mapper & m, khandle * reqs) {
   struct mapper_info mf;
   if (m.mstat(&mf)) {
      struct kurome_mapperinfomsg * msg = (struct kurome_mapperinfomsg *)malloc(sizeof(struct kurome_mapperinfomsg));
      msg->size = (sizeof(struct kurome_mapperinfomsg));
      msg->type = KUROME_MSG_MAPPERINFO;
      memcpy(&msg->mi,&mf,sizeof(struct mapper_info));
      reqs->enqueue((KB *)msg);
   }
}

void kcmd::grid(Grid & g, khandle * reqs) {
   kurome_grid_req_base(g,reqs,KUROME_MSG_GRID);
}

void kcmd::fullGrid(Grid & g, khandle * reqs) {
   kurome_grid_req_base(g,reqs,KUROME_MSG_FULLGRID);
}

