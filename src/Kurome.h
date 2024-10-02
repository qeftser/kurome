
#ifndef KUROME_INCLUDE_FILE

#define KUROME_INCLUDE_FILE

#include <iostream>
#include <memory>
#include <cmath>
#include <random>
#include <set>
#include <iterator>
#include <queue>
#include <list>
#include <unordered_map>
#include <mutex>
#include <thread>
#include <Eigen/Dense>
#include <stdatomic.h>
#include <atomic>

extern int errno_kurome;
#define errnok errno_kurome

/* pi/2 */
#define PI_1l2 1.57079632679
/* 3pi/2 */
#define PI_3l2 4.71238898038
/* yes there is a 
 * predefined pi but
 * I am too lazy to 
 * figure out how to use it...
 */
#define PI 3.14159265358

/* convert radians to degrees */
#define RAD_DEG (180.0/PI)
/* convert degrees to radians */
#define DEG_RAD (PI/180.0)

/* All good */
#define KUROME_EOK      0
/* Indexing outside of grid bounds */
#define KUROME_ERANGE   1
/* Percent value not used */
#define KUROME_EPERCENT 2
/* You know what you did */
#define KUROME_EBADNUM  3
/* Incorrect shape */
#define KUROME_ETYPE    4
/* Data should have been provided */
#define KUROME_EMISS    5

/* none */
#define KUROME_TYPE_NULL 0
/* rectangle */
#define KUROME_TYPE_RECT 1
/* ellipse */
#define KUROME_TYPE_ELPS 2
/* point */
#define KUROME_TYPE_PONT 3

/* we don't know what is here */
#define KUROME_UNDETERMINED 0

/* timeout */
#define KUROME_MFLAG_TIME   1
/* new data */
#define KUROME_MFLAG_DATA   2
/* movement exceeds threshold */
#define KUROME_MFLAG_MOVE   4
/* cumulative movement from after 
 * last trigger exceeds threshold */
#define KUROME_MGLAG_CMOVE 32
/* some exceptional condition occured */
#define KUROME_MFLAG_EXEC   8
/* the goal for the mapper has changed */
#define KUROME_MFLAG_GOAL  16

/* entities can be received */
#define KUROME_AFLAG_WENTITY      1
/* entities can be sent */
#define KUROME_AFLAG_RENTITY      2
/* samples can be sent */
#define KUROME_AFLAG_WSAMPLE      4
/* samples can be received */
#define KUROME_AFLAG_RSAMPLE      8
/* self entity can be moved */
#define KUROME_AFLAG_MOVESELF    16
/* goal entity can be moved */
#define KUROME_AFLAG_MOVEGOAL    32
/* mapper can be modified */
#define KUROME_AFLAG_MODMAPPER   64
/* fine grained grid modification */
#define KUROME_AFLAG_GMOD       128
/* full environment avaliable (environment is simulated) */
#define KUROME_AFLAG_FULLENV    256
/* waiters have info to send */
#define KUROME_AFLAG_WAITERINFO 512

/* how long do we sleep between checks? */
#define KUROME_POLL_DELAY_MS 100

#include "headers/Queues.hpp"

#include "headers/kmsgs.h"

#include "headers/gsocket.h"

#include "headers/Agent.h"

#include "headers/Frame.h"

#include "headers/Grid.h"

#include "headers/Iterators.h"

#include "headers/Mapper.h"

#include "headers/Reporter.h"

#include "headers/handlers.h"

#include "headers/kcmd.h"

#endif

