
#include "Kurome.h"

/* default agent handlers */

void kurome_agent_default_MSG_ADD_ENTITY_handler(KB * msg, khandle * from, Agent * me) {
   struct kurome_entitymsg * emsg = (struct kurome_entitymsg *)msg;
   Entity * e = new Entity(&emsg->e);
   me->environment.addEntity(e);
   kcmd::entity(*e,from);
}

void kurome_agent_default_MSG_MAPCALLBACK_handler(KB * msg, khandle * from, Agent * me) {
   struct kurome_intmsg * fm = (struct kurome_intmsg *)msg;
   me->mapper.callback(fm->val);
   kcmd::mapperInfo(me->mapper,from);
}

void kurome_agent_default_MSG_SET_IDX_handler(KB * msg, khandle * from, Agent * me) {
   struct kurome_setidxmsg * simg = (struct kurome_setidxmsg *)msg;
   me->environment.setIdx(simg->posx,simg->posy,simg->weight);
   kcmd::setIdx(simg->posx,simg->posy,simg->weight,from);
}

void kurome_agent_default_MSG_CLEAR_handler(KB * msg, khandle * from, Agent * me) {
   (void)msg;
   me->environment.clear();
   kcmd::clear(from);
}

void kurome_agent_default_MSG_CHGUNITSIZE_handler(KB * msg, khandle * from, Agent * me) {
   struct kurome_doublemsg * csm = (struct kurome_doublemsg *)msg;
   me->environment.changeUnitSize(csm->val);
   kcmd::chgUnits(csm->val,from);
}

void kurome_agent_default_MSG_CHG_XBLOCKS_handler(KB * msg, khandle * from, Agent * me) {
   struct kurome_doublemsg * csm = (struct kurome_doublemsg *)msg;
   me->environment.changeSizeXmax(csm->val);
   kcmd::chgX(csm->val,from);
}

void kurome_agent_default_MSG_CHG_YBLOCKS_handler(KB * msg, khandle * from, Agent * me) {
   struct kurome_doublemsg * csm = (struct kurome_doublemsg *)msg;
   me->environment.changeSizeYmax(csm->val);
   kcmd::chgY(csm->val,from);
}

void kurome_agent_default_MSG_GET_GRID_handler(KB * msg, khandle * from, Agent * me) {
   (void)msg;
   kcmd::grid(me->environment,from);
}

void kurome_agent_default_MSG_GET_FULLGRID_handler(KB * msg, khandle * from, Agent * me) {
   (void)msg;
   if (me->full_env)
      kcmd::fullGrid(*me->full_env,from);
}

void kurome_agent_default_MSG_CHGSELF_handler(KB * msg, khandle * from, Agent * me) {
   struct kurome_entitymsg * emsg = (struct kurome_entitymsg *)msg;
   Entity e = Entity(&emsg->e);
   me->self = e;
   kcmd::self(e,from);
}

void kurome_agent_default_MSG_CHGGOAL_handler(KB * msg, khandle * from, Agent * me) {
   struct kurome_entitymsg * emsg = (struct kurome_entitymsg *)msg;
   Entity e = Entity(&emsg->e);
   me->goal = e;
   kcmd::goal(e,from);
}

void kurome_agent_default_MSG_CHGFLAGS_handler(KB * msg, khandle * from, Agent * me) {
   (void)from;
   struct kurome_intmsg * fm = (struct kurome_intmsg *)msg;
   me->flags = fm->val;
}

/* not sure how to do these at the moment */
/*
void kurome_agent_default_MSG_ALLSAMPLES_handler(KB * msg, khandle * from, Agent * me) {
}

void kurome_agent_default_MSG_ALLENTITIES_handler(KB * msg, khandle * from, Agent * me) {
}
*/

void kurome_agent_default_MSG_FADD_ENTITY_handler(KB * msg, khandle * from, Agent * me) {
   struct kurome_entitymsg * em = (struct kurome_entitymsg *)msg;
   if (me->full_env)
      me->full_env->addEntity(new Entity(&em->e));
}

void kurome_agent_default_MSG_FCLEAR_handler(KB * msg, khandle * from, Agent * me) {
   if (me->full_env)
      me->full_env->clear();
}

void kurome_agent_default_MSG_FCLENSE_handler(KB * msg, khandle * from, Agent * me) {
   if (me->full_env)
      me->full_env->clense();
}

void kurome_agent_default_MSG_FCHGENTITY_handler(KB * msg, khandle * from, Agent * me) {
   struct kurome_entitymsg * em = (struct kurome_entitymsg *)msg;
   if (me->full_env) {
      Entity e = Entity(&em->e);
      me->full_env->chgEntity(&e);
   }
}

void kurome_agent_default_MSG_FREMENTITY_handler(KB * msg, khandle * from, Agent * me) {
   struct kurome_entitymsg * em = (struct kurome_entitymsg *)msg;
   if (me->full_env) {
      Entity e = Entity(&em->e);
      me->full_env->remEntity(&e);
   }
}

void kurome_agent_default_MSG_CLENSE_handler(KB * msg, khandle * from, Agent * me) {
   me->environment.clense();
   kcmd::clense(from);
}

void kurome_agent_default_MSG_GETSELF_handler(KB * msg, khandle * from, Agent * me) {
   kcmd::self(me->self,from);
}

void kurome_agent_default_MSG_GETGOAL_handler(KB * msg, khandle * from, Agent * me) {
   kcmd::goal(me->goal,from);
}


/* default reporter handlers */

void kurome_reporter_default_MSG_ADD_ENTITY_handler(KB * msg, Reporter * me) {
   if (me->environment) {
      struct kurome_entitymsg * emsg = (struct kurome_entitymsg *)msg;
      me->environment->addEntity(new Entity(&emsg->e));
   }
}
/*
void kurome_reporter_default_MSG_ENTITY_handler(KB * msg, Reporter * me) {
}
*/

/* don't care */
/*
void kurome_reporter_default_MSG_MAPCALLBACK_handler(KB * msg, Reporter * me) {
}
*/

void kurome_reporter_default_MSG_SET_IDX_handler(KB * msg, Reporter * me) {
   if (me->environment) {
      struct kurome_setidxmsg * simg = (struct kurome_setidxmsg *)msg;
      me->environment->setIdx(simg->posx,simg->posy,simg->weight);
   }
}

void kurome_reporter_default_MSG_CLEAR_handler(KB * msg, Reporter * me) {
   (void)msg;
   if (me->environment) {
      me->environment->clear();
   }
}

void kurome_reporter_default_MSG_CLENSE_handler(KB * msg, Reporter * me) {
   (void)msg;
   if (me->environment) {
      me->environment->clense();
   }
}

void kurome_reporter_default_MSG_CHGUNITSIZE_handler(KB * msg, Reporter * me) {
   if (me->environment) {
      struct kurome_doublemsg * csm = (struct kurome_doublemsg *)msg;
      me->environment->changeUnitSize(csm->val);
   }
}

void kurome_reporter_default_MSG_CHG_XBLOCKS_handler(KB * msg, Reporter * me) {
   if (me->environment) {
      struct kurome_doublemsg * csm = (struct kurome_doublemsg *)msg;
      me->environment->changeSizeXmax(csm->val);
   }
}

void kurome_reporter_default_MSG_CHG_YBLOCKS_handler(KB * msg, Reporter * me) {
   if (me->environment) {
      struct kurome_doublemsg * csm = (struct kurome_doublemsg *)msg;
      me->environment->changeSizeYmax(csm->val);
   }
}

/* we don't want to handle these */
/*
void kurome_reporter_default_MSG_GET_GRID_handler(KB * msg, Reporter * me) {
}

void kurome_reporter_default_MSG_GET_FULLGRID_handler(KB * msg, Reporter * me) {
}
*/

void kurome_reporter_default_MSG_CHGSELF_handler(KB * msg, Reporter * me) {
   struct kurome_entitymsg * em = (struct kurome_entitymsg *)msg;
   me->self = std::make_shared<Entity>(&em->e);
}
/*
void kurome_reporter_default_MSG_SELF_handler(KB * msg, Reporter * me) {
}
*/

void kurome_reporter_default_MSG_CHGGOAL_handler(KB * msg, Reporter * me) {
   struct kurome_entitymsg * em = (struct kurome_entitymsg *)msg;
   me->goal = std::make_shared<Entity>(&em->e);
}
/*
void kurome_reporter_default_MSG_GOAL_handler(KB * msg, Reporter * me) {
}
*/

/* don't care? */
/*
void kurome_reporter_default_MSG_CHGFLAGS_handler(KB * msg, Reporter * me) {
}
*/

/* do we even want to get these as the client? */
/*
void kurome_reporter_default_MSG_ALLSAMPLES_handler(KB * msg, Reporter * me) {
}

void kurome_reporter_default_MSG_ALLENTITIES_handler(KB * msg, Reporter * me) {
}
*/

void kurome_reporter_default_MSG_SAMPLE_handler(KB * msg, Reporter * me) {
   if (me->environment) {
      struct kurome_samplemsg * sm = (struct kurome_samplemsg *)msg;
      me->environment->apply(new Sample(&sm->s));
   }
}

void kurome_reporter_default_MSG_WAITERINFO_handler(KB * msg, Reporter * me) {
   struct kurome_waiterinfomsg * wim = (struct kurome_waiterinfomsg *)msg;
   int found = 0;
   for (struct waiter_info & w : me->waiters) {
      if (w.id == wim->wi.id) {
         memcpy(&w,&wim->wi,sizeof(waiter_info));
         found = 1;
         break;
      }
   }
   if (!found)
      me->waiters.push_back(wim->wi);
}

void kurome_reporter_default_MSG_MAPPERINFO_handler(KB * msg, Reporter * me) {
   /* 
    * back in high school there was a class you could take called mims, or 
    * math in modern socity. It got clowned on though for being easy and a
    * lazy class choice. It is kind of funny looking back, as it implies that
    * not much math is needed in modern socity. 
    */
   struct kurome_mapperinfomsg * mims = (struct kurome_mapperinfomsg *)msg;
   memcpy(&me->mapper,&mims->mi,sizeof(struct mapper_info));
}

void kurome_reporter_default_MSG_GRID_handler(KB * msg, Reporter * me) {
   struct kurome_gridmsg * gmsg = (struct kurome_gridmsg *)msg;
   me->environment = std::make_shared<Grid>(&gmsg->gs);
}

void kurome_reporter_default_MSG_FULLGRID_handler(KB * msg, Reporter * me) {
   struct kurome_gridmsg * gmsg = (struct kurome_gridmsg *)msg;
   me->full_env = std::make_shared<Grid>(&gmsg->gs);
}

