
#ifndef DEFAULT_MESSAGE_HANDLERS

#define DEFAULT_MESSAGE_HANDLERS

#include "kmsgs.h"
#include "Agent.h"
#include "Reporter.h"

/* default agent handlers */

void kurome_agent_default_MSG_ADD_ENTITY_handler(KB * msg, khandle * from, void * me);
void kurome_agent_default_MSG_MAPCALLBACK_handler(KB * msg, khandle * from, void * me);
void kurome_agent_default_MSG_SET_IDX_handler(KB * msg, khandle * from, void * me);
void kurome_agent_default_MSG_CLEAR_handler(KB * msg, khandle * from, void * me);
void kurome_agent_default_MSG_CHGUNITSIZE_handler(KB * msg, khandle * from, void * me);
void kurome_agent_default_MSG_CHG_XBLOCKS_handler(KB * msg, khandle * from, void * me);
void kurome_agent_default_MSG_CHG_YBLOCKS_handler(KB * msg, khandle * from, void * me);
void kurome_agent_default_MSG_GET_GRID_handler(KB * msg, khandle * from, void * me);
void kurome_agent_default_MSG_GET_FULLGRID_handler(KB * msg, khandle * from, void * me);
void kurome_agent_default_MSG_CHGSELF_handler(KB * msg, khandle * from, void * me);
void kurome_agent_default_MSG_CHGGOAL_handler(KB * msg, khandle * from, void * me);
void kurome_agent_default_MSG_CHGFLAGS_handler(KB * msg, khandle * from, void * me);
void kurome_agent_default_MSG_FADD_ENTITY_handler(KB * msg, khandle * from, void * me);
void kurome_agent_default_MSG_FCLEAR_handler(KB * msg, khandle * from, void * me);
void kurome_agent_default_MSG_FCLENSE_handler(KB * msg, khandle * from, void * me);
void kurome_agent_default_MSG_FCHGENTITY_handler(KB * msg, khandle * from, void * me);
void kurome_agent_default_MSG_FREMENTITY_handler(KB * msg, khandle * from, void * me);
void kurome_agent_default_MSG_CLENSE_handler(KB * msg, khandle * from, void * me);
void kurome_agent_default_MSG_GETSELF_handler(KB * msg, khandle * from, void * me);
void kurome_agent_default_MSG_GETGOAL_handler(KB * msg, khandle * from, void * me);
void kurome_agent_default_MSG_GETMAPPER_handler(KB * msg, khandle * from, void * me);
void kurome_agent_default_MSG_GETWAITERS_handler(KB * msg, khandle * from, void * me);
void kurome_agent_default_MSG_ALLSAMPLES_handler(KB * msg, khandle * from, void * me);
void kurome_agent_default_MSG_ALLENTITIES_handler(KB * msg, khandle * from, void * me);
void kurome_agent_default_MSG_FALLSAMPLES_handler(KB * msg, khandle * from, void * me);
void kurome_agent_default_MSG_FALLENTITIES_handler(KB * msg, khandle * from, void * me);

/* default reporter handlers */

void kurome_reporter_default_MSG_ADD_ENTITY_handler(KB * msg, void * me);
void kurome_reporter_default_MSG_FADD_ENTITY_handler(KB * msg, void * me);
void kurome_reporter_default_MSG_MAPCALLBACK_handler(KB * msg, void * me);
void kurome_reporter_default_MSG_SET_IDX_handler(KB * msg, void * me);
void kurome_reporter_default_MSG_CLEAR_handler(KB * msg, void * me);
void kurome_reporter_default_MSG_CHGUNITSIZE_handler(KB * msg, void * me);
void kurome_reporter_default_MSG_CHG_XBLOCKS_handler(KB * msg, void * me);
void kurome_reporter_default_MSG_CHG_YBLOCKS_handler(KB * msg, void * me);
void kurome_reporter_default_MSG_GET_GRID_handler(KB * msg, void * me);
void kurome_reporter_default_MSG_GET_FULLGRID_handler(KB * msg, void * me);
void kurome_reporter_default_MSG_CHGSELF_handler(KB * msg, void * me);
void kurome_reporter_default_MSG_CHGGOAL_handler(KB * msg, void * me);
/*
void kurome_reporter_default_MSG_CHGFLAGS_handler(KB * msg, void * me);
void kurome_reporter_default_MSG_ALLSAMPLES_handler(KB * msg, void * me);
void kurome_reporter_default_MSG_ALLENTITIES_handler(KB * msg, void * me);
void kurome_reporter_default_MSG_ENTITY_handler(KB * msg, void * me);
void kurome_reporter_default_MSG_SELF_handler(KB * msg, void * me);
void kurome_reporter_default_MSG_GOAL_handler(KB * msg, void * me);
*/
void kurome_reporter_default_MSG_SAMPLE_handler(KB * msg, void * me);
void kurome_reporter_default_MSG_FSAMPLE_handler(KB * msg, void * me);
void kurome_reporter_default_MSG_WAITERINFO_handler(KB * msg, void * me);
void kurome_reporter_default_MSG_MAPPERINFO_handler(KB * msg, void * me);
void kurome_reporter_default_MSG_GRID_handler(KB * msg, void * me);
void kurome_reporter_default_MSG_FULLGRID_handler(KB * msg, void * me);
void kurome_reporter_default_MSG_CLENSE_handler(KB * msg, void * me);

#endif
