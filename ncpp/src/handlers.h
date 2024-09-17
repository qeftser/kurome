
#ifndef DEFAULT_MESSAGE_HANDLERS

#define DEFAULT_MESSAGE_HANDLERS

#include "kmsgs.h"
#include "Kurome.h"

/* default agent handlers */

void kurome_agent_default_MSG_ADD_ENTITY_handler(KB * msg, ll_queue<KB *> * from, Agent * me);
void kurome_agent_default_MSG_MAPCALLBACK_handler(KB * msg, ll_queue<KB *> * from, Agent * me);
void kurome_agent_default_MSG_SET_IDX_handler(KB * msg, ll_queue<KB *> * from, Agent * me);
void kurome_agent_default_MSG_CLEAR_handler(KB * msg, ll_queue<KB *> * from, Agent * me);
void kurome_agent_default_MSG_CHGUNITSIZE_handler(KB * msg, ll_queue<KB *> * from, Agent * me);
void kurome_agent_default_MSG_CHG_XBLOCKS_handler(KB * msg, ll_queue<KB *> * from, Agent * me);
void kurome_agent_default_MSG_CHG_YBLOCKS_handler(KB * msg, ll_queue<KB *> * from, Agent * me);
void kurome_agent_default_MSG_GET_GRID_handler(KB * msg, ll_queue<KB *> * from, Agent * me);
void kurome_agent_default_MSG_GET_FULLGRID_handler(KB * msg, ll_queue<KB *> * from, Agent * me);
void kurome_agent_default_MSG_CHGSELF_handler(KB * msg, ll_queue<KB *> * from, Agent * me);
void kurome_agent_default_MSG_CHGGOAL_handler(KB * msg, ll_queue<KB *> * from, Agent * me);
void kurome_agent_default_MSG_CHGFLAGS_handler(KB * msg, ll_queue<KB *> * from, Agent * me);
/*
void kurome_agent_default_MSG_ALLSAMPLES_handler(KB * msg, ll_queue<KB *> * from, Agent * me);
void kurome_agent_default_MSG_ALLENTITIES_handler(KB * msg, ll_queue<KB *> * from, Agent * me);
*/

/* default reporter handlers */

void kurome_reporter_default_MSG_ADD_ENTITY_handler(KB * msg, Reporter * me);
void kurome_reporter_default_MSG_MAPCALLBACK_handler(KB * msg, Reporter * me);
void kurome_reporter_default_MSG_SET_IDX_handler(KB * msg, Reporter * me);
void kurome_reporter_default_MSG_CLEAR_handler(KB * msg, Reporter * me);
void kurome_reporter_default_MSG_CHGUNITSIZE_handler(KB * msg, Reporter * me);
void kurome_reporter_default_MSG_CHG_XBLOCKS_handler(KB * msg, Reporter * me);
void kurome_reporter_default_MSG_CHG_YBLOCKS_handler(KB * msg, Reporter * me);
void kurome_reporter_default_MSG_GET_GRID_handler(KB * msg, Reporter * me);
void kurome_reporter_default_MSG_GET_FULLGRID_handler(KB * msg, Reporter * me);
void kurome_reporter_default_MSG_CHGSELF_handler(KB * msg, Reporter * me);
void kurome_reporter_default_MSG_CHGGOAL_handler(KB * msg, Reporter * me);
/*
void kurome_reporter_default_MSG_CHGFLAGS_handler(KB * msg, Reporter * me);
void kurome_reporter_default_MSG_ALLSAMPLES_handler(KB * msg, Reporter * me);
void kurome_reporter_default_MSG_ALLENTITIES_handler(KB * msg, Reporter * me);
void kurome_reporter_default_MSG_ENTITY_handler(KB * msg, Reporter * me);
void kurome_reporter_default_MSG_SELF_handler(KB * msg, Reporter * me);
void kurome_reporter_default_MSG_GOAL_handler(KB * msg, Reporter * me);
*/
void kurome_reporter_default_MSG_SAMPLE_handler(KB * msg, Reporter * me);
void kurome_reporter_default_MSG_WAITERINFO_handler(KB * msg, Reporter * me);
void kurome_reporter_default_MSG_MAPPERINFO_handler(KB * msg, Reporter * me);
void kurome_reporter_default_MSG_GRID_handler(KB * msg, Reporter * me);
void kurome_reporter_default_MSG_FULLGRID_handler(KB * msg, Reporter * me);

#endif
