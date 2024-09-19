
#ifndef KUROME_MESSAGES

#define KUROME_MESSAGES
#include "gsocket.h"
#include "Queues.hpp"

#define KUROME_BROADCAST_PORT 60127
#define KUROME_SERV_MAXCONNS  32

#define KB struct kurome_basemsg

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

struct grid_struct {
   int32_t blocksX;
   int32_t blocksY;
   double sizeX;
   double sizeY;
   double unitSize;
   int matrix[];
};

struct entity_struct {
   double posx;
   double posy;
   double xwid;
   double ywid;
   int type;
   int val;
};

struct waiter_info {
   char            name[40];
   int             id;
   struct timeval  rate;
   struct timespec last;
};

struct sample_struct {
   double unitSize;
   struct entity_struct orgin;
   int32_t blocksX;
   int32_t blocksY;
   int matrix[];
};

struct frame_struct {
   double posx;
   double posy;
   double rot;
   uint64_t num;
   uint64_t weight;
};

struct mapper_info {
   char            name[40];
   int             state;
   struct timespec last;
};

struct agent_values {
   char addr[16];
   int flags;
   int port;
   long naddr;
   struct agent_values * nextptr;
};


struct agent_discover {
   int flags;
   int start_port;
};

struct kurome_basemsg {
   int type;
   int size;
   char more[];
};

struct kurome_entitymsg {
   int type;
   int size;
   struct entity_struct e;
};

struct kurome_setidxmsg {
   int type;
   int size;
   double posx;
   double posy;
   int    weight;
};

struct kurome_chgsizemsg {
   int type;
   int size;
   double unitSize;
};

struct kurome_gridmsg {
   int type;
   int size;
   struct grid_struct gs;
};

struct kurome_waiterinfomsg {
   int type;
   int size;
   struct waiter_info wi;
};

struct kurome_mapperinfomsg {
   int type;
   int size;
   struct mapper_info mi;
};

struct kurome_samplemsg {
   int type;
   int size;
   struct sample_struct s;
};

struct kurome_flagmsg {
   int type;
   int size;
   int flag;
};

typedef struct KUROME_MSG_READ_STATE {
   int    known;
   gnbstate state;
} kurome_msg_transport;


#endif
