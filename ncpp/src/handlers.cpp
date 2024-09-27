
#include "Kurome.h"

/* default agent handlers */

void kurome_agent_default_MSG_ADD_ENTITY_handler(KB * msg, khandle * from, void * me) {
   struct kurome_entitymsg * emsg = (struct kurome_entitymsg *)msg;
   Entity * e = new Entity(&emsg->e);
   ((Agent *)me)->environment.addEntity(e);
   kcmd::entity(*e,from);
}

void kurome_agent_default_MSG_MAPCALLBACK_handler(KB * msg, khandle * from, void * me) {
   struct kurome_intmsg * fm = (struct kurome_intmsg *)msg;
   ((Agent *)me)->mapper.callback(fm->val);
   kcmd::mapperInfo(((Agent *)me)->mapper,from);
}

void kurome_agent_default_MSG_SET_IDX_handler(KB * msg, khandle * from, void * me) {
   struct kurome_setidxmsg * simg = (struct kurome_setidxmsg *)msg;
   ((Agent *)me)->environment.setIdx(simg->posx,simg->posy,simg->weight);
   kcmd::setIdx(simg->posx,simg->posy,simg->weight,from);
}

void kurome_agent_default_MSG_CLEAR_handler(KB * msg, khandle * from, void * me) {
   (void)msg;
   ((Agent *)me)->environment.clear();
   kcmd::clear(from);
}

void kurome_agent_default_MSG_CHGUNITSIZE_handler(KB * msg, khandle * from, void * me) {
   struct kurome_doublemsg * csm = (struct kurome_doublemsg *)msg;
   ((Agent *)me)->environment.changeUnitSize(csm->val);
   kcmd::chgUnits(csm->val,from);
}

void kurome_agent_default_MSG_CHG_XBLOCKS_handler(KB * msg, khandle * from, void * me) {
   struct kurome_doublemsg * csm = (struct kurome_doublemsg *)msg;
   ((Agent *)me)->environment.changeSizeXmax(csm->val);
   kcmd::chgX(csm->val,from);
}

void kurome_agent_default_MSG_CHG_YBLOCKS_handler(KB * msg, khandle * from, void * me) {
   struct kurome_doublemsg * csm = (struct kurome_doublemsg *)msg;
   ((Agent *)me)->environment.changeSizeYmax(csm->val);
   kcmd::chgY(csm->val,from);
}

void kurome_agent_default_MSG_GET_GRID_handler(KB * msg, khandle * from, void * me) {
   (void)msg;
   kcmd::grid(((Agent *)me)->environment,from);
}

void kurome_agent_default_MSG_GETMAPPER_handler(KB * msg, khandle * from, void * me) {
   (void)msg;
   kcmd::mapperInfo(((Agent *)me)->mapper,from);
}

void kurome_agent_default_MSG_GETWAITERS_handler(KB * msg, khandle * from, void * me) {
   (void)msg;
   for (Waiter * w : ((Agent *)me)->waiters) {
      kcmd::waiterInfo(*w,from);
   }
}

void kurome_agent_default_MSG_GET_FULLGRID_handler(KB * msg, khandle * from, void * me) {
   (void)msg;
   if (((Agent *)me)->full_env)
      kcmd::fullGrid(*((Agent *)me)->full_env,from);
}

void kurome_agent_default_MSG_CHGSELF_handler(KB * msg, khandle * from, void * me) {
   struct kurome_entitymsg * emsg = (struct kurome_entitymsg *)msg;
   Entity e = Entity(&emsg->e);
   ((Agent *)me)->self = e;
   kcmd::self(e,from);
}

void kurome_agent_default_MSG_CHGGOAL_handler(KB * msg, khandle * from, void * me) {
   struct kurome_entitymsg * emsg = (struct kurome_entitymsg *)msg;
   Entity e = Entity(&emsg->e);
   ((Agent *)me)->goal = e;
   kcmd::goal(e,from);
}

void kurome_agent_default_MSG_CHGFLAGS_handler(KB * msg, khandle * from, void * me) {
   (void)from;
   struct kurome_intmsg * fm = (struct kurome_intmsg *)msg;
   ((Agent *)me)->flags = fm->val;
}

/* not sure how to do these at the moment */
/*
void kurome_agent_default_MSG_ALLSAMPLES_handler(KB * msg, khandle * from, Agent * me) {
}

void kurome_agent_default_MSG_ALLENTITIES_handler(KB * msg, khandle * from, Agent * me) {
}
*/

void kurome_agent_default_MSG_FADD_ENTITY_handler(KB * msg, khandle * from, void * me) {
   struct kurome_entitymsg * em = (struct kurome_entitymsg *)msg;
   if (((Agent *)me)->full_env)
      ((Agent *)me)->full_env->addEntity(new Entity(&em->e));
}

void kurome_agent_default_MSG_FCLEAR_handler(KB * msg, khandle * from, void * me) {
   if (((Agent *)me)->full_env)
      ((Agent *)me)->full_env->clear();
}

void kurome_agent_default_MSG_FCLENSE_handler(KB * msg, khandle * from, void * me) {
   if (((Agent *)me)->full_env)
      ((Agent *)me)->full_env->clense();
}

void kurome_agent_default_MSG_FCHGENTITY_handler(KB * msg, khandle * from, void * me) {
   struct kurome_entitymsg * em = (struct kurome_entitymsg *)msg;
   if (((Agent *)me)->full_env) {
      Entity e = Entity(&em->e);
      ((Agent *)me)->full_env->chgEntity(&e);
   }
}

void kurome_agent_default_MSG_FREMENTITY_handler(KB * msg, khandle * from, void * me) {
   struct kurome_entitymsg * em = (struct kurome_entitymsg *)msg;
   if (((Agent *)me)->full_env) {
      Entity e = Entity(&em->e);
      ((Agent *)me)->full_env->remEntity(&e);
   }
}

void kurome_agent_default_MSG_CLENSE_handler(KB * msg, khandle * from, void * me) {
   ((Agent *)me)->environment.clense();
   kcmd::clense(from);
}

void kurome_agent_default_MSG_GETSELF_handler(KB * msg, khandle * from, void * me) {
   kcmd::self(((Agent *)me)->self,from);
}

void kurome_agent_default_MSG_GETGOAL_handler(KB * msg, khandle * from, void * me) {
   kcmd::goal(((Agent *)me)->goal,from);
}


/* default reporter handlers */

void kurome_reporter_default_MSG_ADD_ENTITY_handler(KB * msg, void * me) {
   if (((Reporter *)me)->environment) {
      struct kurome_entitymsg * emsg = (struct kurome_entitymsg *)msg;
      ((Reporter *)me)->environment->addEntity(new Entity(&emsg->e));
   }
}
/*
void kurome_reporter_default_MSG_ENTITY_handler(KB * msg, void * me) {
}
*/

/* don't care */
/*
void kurome_reporter_default_MSG_MAPCALLBACK_handler(KB * msg, void * me) {
}
*/

void kurome_reporter_default_MSG_SET_IDX_handler(KB * msg, void * me) {
   if (((Reporter *)me)->environment) {
      struct kurome_setidxmsg * simg = (struct kurome_setidxmsg *)msg;
      ((Reporter *)me)->environment->setIdx(simg->posx,simg->posy,simg->weight);
   }
}

void kurome_reporter_default_MSG_CLEAR_handler(KB * msg, void * me) {
   (void)msg;
   if (((Reporter *)me)->environment) {
      ((Reporter *)me)->environment->clear();
   }
}

void kurome_reporter_default_MSG_CLENSE_handler(KB * msg, void * me) {
   (void)msg;
   if (((Reporter *)me)->environment) {
      ((Reporter *)me)->environment->clense();
   }
}

void kurome_reporter_default_MSG_CHGUNITSIZE_handler(KB * msg, void * me) {
   if (((Reporter *)me)->environment) {
      struct kurome_doublemsg * csm = (struct kurome_doublemsg *)msg;
      ((Reporter *)me)->environment->changeUnitSize(csm->val);
   }
}

void kurome_reporter_default_MSG_CHG_XBLOCKS_handler(KB * msg, void * me) {
   if (((Reporter *)me)->environment) {
      struct kurome_doublemsg * csm = (struct kurome_doublemsg *)msg;
      ((Reporter *)me)->environment->changeSizeXmax(csm->val);
   }
}

void kurome_reporter_default_MSG_CHG_YBLOCKS_handler(KB * msg, void * me) {
   if (((Reporter *)me)->environment) {
      struct kurome_doublemsg * csm = (struct kurome_doublemsg *)msg;
      ((Reporter *)me)->environment->changeSizeYmax(csm->val);
   }
}

/* we don't want to handle these */
/*
void kurome_reporter_default_MSG_GET_GRID_handler(KB * msg, void * me) {
}

void kurome_reporter_default_MSG_GET_FULLGRID_handler(KB * msg, void * me) {
}
*/

void kurome_reporter_default_MSG_CHGSELF_handler(KB * msg, void * me) {
   struct kurome_entitymsg * em = (struct kurome_entitymsg *)msg;
   ((Reporter *)me)->self = std::make_shared<Entity>(&em->e);
}
/*
void kurome_reporter_default_MSG_SELF_handler(KB * msg, void * me) {
}
*/

void kurome_reporter_default_MSG_CHGGOAL_handler(KB * msg, void * me) {
   struct kurome_entitymsg * em = (struct kurome_entitymsg *)msg;
   ((Reporter *)me)->goal = std::make_shared<Entity>(&em->e);
}
/*
void kurome_reporter_default_MSG_GOAL_handler(KB * msg, void * me) {
}
*/

/* don't care? */
/*
void kurome_reporter_default_MSG_CHGFLAGS_handler(KB * msg, void * me) {
}
*/

/* do we even want to get these as the client? */
/*
void kurome_reporter_default_MSG_ALLSAMPLES_handler(KB * msg, void * me) {
}

void kurome_reporter_default_MSG_ALLENTITIES_handler(KB * msg, void * me) {
}
*/

void kurome_reporter_default_MSG_SAMPLE_handler(KB * msg, void * me) {
   if (((Reporter *)me)->environment) {
      struct kurome_samplemsg * sm = (struct kurome_samplemsg *)msg;
      ((Reporter *)me)->environment->apply(new Sample(&sm->s));
   }
}

void kurome_reporter_default_MSG_WAITERINFO_handler(KB * msg, void * me) {
   struct kurome_waiterinfomsg * wim = (struct kurome_waiterinfomsg *)msg;
   int found = 0;
   for (struct waiter_info & w : ((Reporter *)me)->waiters) {
      if (w.id == wim->wi.id) {
         memcpy(&w,&wim->wi,sizeof(waiter_info));
         found = 1;
         break;
      }
   }
   if (!found)
      ((Reporter *)me)->waiters.push_back(wim->wi);
}

void kurome_reporter_default_MSG_MAPPERINFO_handler(KB * msg, void * me) {
   /* 
    * back in high school there was a class you could take called mims, or 
    * math in modern socity. It got clowned on though for being easy and a
    * lazy class choice. It is kind of funny looking back, as it implies that
    * not much math is needed in modern socity. 
    */
   struct kurome_mapperinfomsg * mims = (struct kurome_mapperinfomsg *)msg;
   memcpy(&((Reporter *)me)->mapper,&mims->mi,sizeof(struct mapper_info));
}

void kurome_reporter_default_MSG_GRID_handler(KB * msg, void * me) {
   struct kurome_gridmsg * gmsg = (struct kurome_gridmsg *)msg;
   ((Reporter *)me)->environment = std::make_shared<Grid>(&gmsg->gs);
}

void kurome_reporter_default_MSG_FULLGRID_handler(KB * msg, void * me) {
   struct kurome_gridmsg * gmsg = (struct kurome_gridmsg *)msg;
   ((Reporter *)me)->full_env = std::make_shared<Grid>(&gmsg->gs);
}

