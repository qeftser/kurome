
#ifndef KUROME_MESSAGES

#define KUROME_MESSAGES
#include "gsocket.h"
#include "Queues.hpp"

#define KUROME_BROADCAST_PORT 60127
#define KUROME_SERV_MAXCONNS  32

#define KB struct kurome_basemsg
#define khandle ll_queue<KB *>

/* data */
#define KUROME_MSG_ENTITY        1
#define KUROME_MSG_SELF         19
#define KUROME_MSG_GOAL         20
#define KUROME_MSG_SAMPLE        9
#define KUROME_MSG_WAITERINFO   10
#define KUROME_MSG_MAPPERINFO   11
#define KUROME_MSG_GRID         15
#define KUROME_MSG_FULLGRID     22

/* requests */
#define KUROME_MSG_ADD_ENTITY    2
#define KUROME_MSG_MAPCALLBACK  12
#define KUROME_MSG_SET_IDX       3
#define KUROME_MSG_CLEAR         4
#define KUROME_MSG_CLENSE       27
#define KUROME_MSG_CHG_UNITSIZE  5
#define KUROME_MSG_CHG_XBLOCKS   6
#define KUROME_MSG_CHG_YBLOCKS   7
#define KUROME_MSG_GET_GRID      8
#define KUROME_MSG_GET_FULLGRID 18
#define KUROME_MSG_CHGSELF      16
#define KUROME_MSG_CHGGOAL      17
#define KUROME_MSG_CHGFLAGS     21
#define KUROME_MSG_ALLSAMPLES   13
#define KUROME_MSG_ALLENTITIES  14
#define KUROME_MSG_FADD_ENTITY  23
#define KUROME_MSG_FCLEAR       24
#define KUROME_MSG_FCLENSE      28
#define KUROME_MSG_FCHGENTITY   25
#define KUROME_MSG_FREMENTITY   26
#define KUROME_MSG_GETSELF      28
#define KUROME_MSG_GETGOAL      29
#define KUROME_MSG_GETMAPPER    30
#define KUROME_MSG_GETWAITERS   31
#define KUROME_MSG_START        32
#define KUROME_MSG_STOP         33
#define KUROME_MSG_PAUSE        34

/*
 * Struct to hold all releavant data for the Grid
 * class. This can be used to construct a new grid
 * that should match the old one, provided the old
 * one hasn't changed since it had a struct of it 
 * generated :)
 */
struct grid_struct {
   int32_t blocksXmax;
   int32_t blocksYmax;
   int32_t blocksXmin;
   int32_t blocksYmin;
   double sizeXmax;
   double sizeYmax;
   double sizeXmin;
   double sizeYmin;
   double unitSize;
   int matrix[];
};

/*
 * grid struct without the grid
 */
struct partial_grid_struct {
   int32_t blocksXmax;
   int32_t blocksYmax;
   int32_t blocksXmin;
   int32_t blocksYmin;
   double sizeXmax;
   double sizeYmax;
   double sizeXmin;
   double sizeYmin;
   double unitSize;
};

/*
 * Struct to hold all releavant data to
 * construct an Entity class
 */
struct entity_struct {
   double posx;
   double posy;
   double xwid;
   double ywid;
   double rot;
   int type;
   int val;
   int id;
};

/*
 * Struct to hold all releavant data to
 * construct a Waiter class
 */
struct waiter_info {
   char            name[40];
   int             id;
   struct timeval  rate;
   struct timespec last;
};

/*
 * Struct to hold all releavant data to
 * construct a Sample class
 */
struct sample_struct {
   double unitSize;
   struct entity_struct orgin;
   int32_t blocksX;
   int32_t blocksY;
   int matrix[];
};

/*
 * Struct to hold all releavant data to
 * construct a Frame class
 */
struct frame_struct {
   double posx;
   double posy;
   double rot;
   uint64_t num;
   uint64_t weight;
};

/*
 * Provide information about which mapping algorithm
 * is being used, and what it is currently doing. Handling
 * this as a client is dependent on knowing the mapping
 * algorithm and how it functions.
 */
struct mapper_info {
   char            name[40];
   int             state;
   struct timespec last;
};

/*
 * Hold information about potential
 * agents to connect to. Used by the 
 * reporter client.
 */
struct agent_values {
   char addr[52];
   char name[12];
   int flags;
   int port;
   long naddr;
   struct agent_values * nextptr;
};

/*
 * This data is broadcast by the Agent
 * server and used by reporters to determine
 * which Agents are avaliable to connect to.
 */
struct agent_discover {
   int flags;
   int start_port;
   char name[12];
};

/*
 * All other messages 'derive'
 * from this message.
 */
struct kurome_basemsg {
   int type;
   int size;
   char more[];
};

/* 
 * Message for passing an
 * entity between Agent and
 * Reporter.
 */
struct kurome_entitymsg {
   int type;
   int size;
   struct entity_struct e;
};

/* 
 * Message for passing a
 * grid index between Agent and
 * Reporter.
 */
struct kurome_setidxmsg {
   int type;
   int size;
   double posx;
   double posy;
   int    weight;
};

/* 
 * Message for passing a
 * double value from Agent
 * to Reporter.
 */
struct kurome_doublemsg {
   int type;
   int size;
   double val;
};

/* 
 * Message for passing a
 * Grid between Agent and 
 * Reporter
 */
struct kurome_gridmsg {
   int type;
   int size;
   struct grid_struct gs;
};

/* 
 * Provide info regarding the
 * Waiters that the Agent is
 * using
 */
struct kurome_waiterinfomsg {
   int type;
   int size;
   struct waiter_info wi;
};

/* 
 * Provide info regarding the
 * Mapper that the Agent is
 * using
 */
struct kurome_mapperinfomsg {
   int type;
   int size;
   struct mapper_info mi;
};

/*
 * Message for passing a
 * Sample from Agent to
 * Reporter.
 */
struct kurome_samplemsg {
   int type;
   int size;
   struct sample_struct s;
};

/* 
 * Message for passing an
 * integer from Agent to
 * Reporter.
 */
struct kurome_intmsg {
   int type;
   int size;
   int val;
};

/*
 * This is a helper struct to 
 * keep track of nonblocking reads/writes
 * of the kurome_*msg struct tyoes
 */
typedef struct KUROME_MSG_READ_STATE {
   int    known;
   gnbstate state;
} kurome_msg_transport;


#endif
