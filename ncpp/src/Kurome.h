#pragma once

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
#include <eigen-3.4.0/Eigen/Dense>
#include <stdatomic.h>
#include <atomic>
#include "Queues.hpp"
#include "kmsgs.h"

class Entity;
class Agent;
class Sample;

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

#define RAD_DEG (180.0/PI)
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

class Grid {
private:
   Eigen::MatrixXi            blocks;
   int32_t                    blocksX;
   int32_t                    blocksY;
   double                     sizeX;
   double                     sizeY;
   double                     unitSize;
   std::default_random_engine generator;
   std::set<Entity *>         entities;
   std::set<Sample *>         samples;

public:

   Grid(double,double,double);
   Grid(struct grid_struct *);

   /* normal operations */

   bool inBounds(double,double);
   bool inBounds(int,int);

   int  getIdx(double,double);
   int  setIdx(double,double,int);
   int  setIdx(double,double,int,double);

   int  addEntity(Entity *);
   int  remEntity(Entity *);
   int  apply(Sample *);

   int  avgWeights(int,int);
   int  avgWeights(int,int,double);

   void clear();
   void smooth();
   int  changeUnitSize(double);
   int  changeSizeX(double);
   int  changeSizeY(double);

   void  info();
   void  print();

   /* simulation/alt/helper operations */

   int             getIdx(double,double,double);
   Eigen::MatrixXi slice(int,int,int,int);
   Eigen::MatrixXi slice(double,double,double,double);
   
   double          getUnitSize();
   int *           getIdxPtr(int,int);
   int *           getIdxPtr(double,double);
   int             getXBlocks();

   int root(double);
   int roob(double);
   int roor(double);

   int toStruct(struct grid_struct **);
};

struct ShapeIteratorInfo { double posx; double posy; int * val; };

class RectIterator : public std::iterator<std::input_iterator_tag,int> {
private:
   Grid * g;
   int srtx, srty;
   int posx, endx;
   int posy, endy;
   struct ShapeIteratorInfo info;
public:
   bool done;
   RectIterator() {};
   RectIterator(double, double, double, double, Grid *);
   RectIterator(Entity *, Grid *);
   RectIterator(double, double, Entity *, Grid *);
   RectIterator(const RectIterator &);
   RectIterator& operator++();
   RectIterator& operator++(int);
   bool operator==(const RectIterator &);
   bool operator!=(const RectIterator &);
   int & operator*();
   struct ShapeIteratorInfo & locinfo();
};

class EllipseIterator : public std::iterator<std::input_iterator_tag,int> {
private:
   Grid * g;
   double granularity;
   double k, wrad, hrad, x, y;
   int xP, xC, yG, yL;
   struct ShapeIteratorInfo info;
public:
   bool done;
   EllipseIterator() {};
   EllipseIterator(double, double, double, double, Grid *);
   EllipseIterator(Entity *, Grid *);
   EllipseIterator(double, double, Entity *, Grid *);
   EllipseIterator(const EllipseIterator &);
   EllipseIterator& operator++();
   EllipseIterator& operator++(int);
   bool operator==(const EllipseIterator &);
   bool operator!=(const EllipseIterator &);
   int & operator*();
   struct ShapeIteratorInfo & locinfo();
};

class Entity {
public:
   double posx;
   double posy;
   double xwid;
   double ywid;
   int    type;
   int    val;
   
   Entity(double, double, double, double, int, int);
   Entity(double, double, int);
   Entity(struct entity_struct *);

   void   toStruct(struct entity_struct *);
   double dist2(const Entity &);
   void   operator=(const Entity &);
};

class Sample {
public:
   Entity          orgin;
   Eigen::MatrixXi values;
   double unitSize;
   
   Sample(Entity e, Eigen::MatrixXi m, double d) 
      : orgin(e), values(m), unitSize(d) { values.setZero(); };
   Sample() 
      : orgin(Entity(0,0,0,0,0,0)), values(Eigen::MatrixXi(0,0)), unitSize(0) {}
   Sample(struct sample_struct * s) 
   : orgin(&s->orgin), values(Eigen::MatrixXi(s->blocksX,s->blocksY)), unitSize(s->unitSize) {
      for (int i = 0; i < s->blocksX*s->blocksY; ++i)
         values(i/s->blocksX,i%s->blocksX) = s->matrix[i];
   }

   int & localVal(double x, double y) {
      static int null = 0;
      null = -1;
      x -= (orgin.posx-(orgin.xwid/2));
      y -= (orgin.posy-(orgin.ywid/2));
      int xp = (int)std::round(x/unitSize);
      int yp = (int)std::round(y/unitSize);
      if (xp >= values.rows() || xp < 0 ||
          yp >= values.cols() || yp < 0)
         return null;
      return values(xp,yp);
   }

   int toStruct(struct sample_struct ** s) {
      (*s)->unitSize = unitSize;
      orgin.toStruct(&(*s)->orgin);
      (*s)->blocksX = values.rows();
      (*s)->blocksY = values.cols();
      int size = sizeof(sample_struct)+(sizeof(int)*(*s)->blocksX*(*s)->blocksY);
      *s = (struct sample_struct *)realloc(*s,size);
      for (int i = 0; i < (*s)->blocksX*(*s)->blocksY; ++i)
         (*s)->matrix[i] = values(i/(*s)->blocksX,i%(*s)->blocksX);
      return size;
   }
};

class Waiter {
private:
   std::mutex m;
   Sample * s;
public:
   Waiter() : m(std::mutex()) {};

   void dish(Sample * news) { m.lock(); s = news; m.unlock(); };
   Sample * serve(void) { m.lock(); Sample * ret = s; s = NULL; m.unlock(); return ret; };
   int poll(void) { return s != NULL; };

   virtual void prepare() = 0;
   virtual int  wstat(struct waiter_info *) { return 0; };
};

typedef uint64_t FrameId;
class Frame {
public:
   double posx;
   double posy;
   double rot;
   uint64_t num;
   uint64_t weight;
   Frame * nextptr;

   Frame() {};
   Frame(double posx, double posy, double rot, uint64_t num, uint64_t weight, Frame * nextptr) 
      : posx(posx), posy(posy), rot(rot), num(num), weight(weight), nextptr(nextptr) {}
   Frame(Frame *);
   Frame(struct frame_struct * s)
      : posx(s->posx), posy(s->posy), rot(s->rot), num(s->num), weight(s->weight), nextptr(NULL) {}

   FrameId id() const;
   bool operator=(const Frame &);
   Frame * operator+(const Frame &);
   void operator+=(Frame &);
   int cost(Entity &, Grid &);
   double dist2(const Frame &);
   void toStruct(struct frame_struct *);
};

struct FrameCmp {
   bool operator()(const Frame * f1, const Frame * f2) const {
      if (f1->weight > f2->weight)
         return true;
      return false;
   }
};

class Mapper {
public:
   Agent * self;

   Mapper(Agent * me)
      : self(me) {}
   Mapper() {}

   virtual Frame nextPoint() = 0;
   virtual void callback(int) = 0;
   virtual int  mstat(struct mapper_info *) { return 0; };
};

/* will be turned into an action server later on */
class Agent {
public:
   int flags;
   Entity & self;
   Entity & goal;
   std::vector<Waiter *> waiters;
   Mapper & mapper;
   Grid   & environment;
   Grid   * full_env;
   std::thread serv;
   ll_queue<std::tuple<KB *,ll_queue<KB *> *>> reqs;
   std::set<ll_queue<KB *> *>conns;
   std::unordered_map<int,void (*)(struct kurome_basemsg *, ll_queue<KB *> *, Agent *)> handlers;

   Agent(Entity & self, Entity & goal, Mapper & mapper, Grid & env)
      : self(self), goal(goal), mapper(mapper), environment(env), full_env(NULL) { 
         mapper.self = this; 
         setDefaultHandlers();
      }

   double goalDist();

   void   launchServer(int,int);
   void   updateFromServer(int updates=13);
   void   registerHandler(int, void (*)(struct kurome_basemsg *, ll_queue<KB *> *, Agent *));

   void setDefaultHandlers();
   void sendAll(Entity *);
   void sendAll(Entity *, int);
   void sendAll(Sample *);
   void sendAll(Grid *);
   void sendAll(Grid *, int);
};

/* class for sending kurome_req structs */
class kcmd {
public:
   static void addEntity(Entity *, ll_queue<KB *> *);
   static void chgSelf(Entity *, ll_queue<KB *> *);
   static void chgGoal(Entity *, ll_queue<KB *> *);
   static void chgFlags(int, ll_queue<KB *> *);
   static void mapCallback(int, ll_queue<KB *> *);
   static void setIdx(double, double, int, ll_queue<KB *> *);
   static void clear(ll_queue<KB *> *);
   static void allSamples(ll_queue<KB *> *);
   static void allEntities(ll_queue<KB *> *);
   static void chgUnits(double, ll_queue<KB *> *);
   static void chgX(double, ll_queue<KB *> *);
   static void chgY(double, ll_queue<KB *> *);
   static void getGrid(ll_queue<KB *> *);
   static void getFullGrid(ll_queue<KB *> *);
   static void entity(Entity *, ll_queue<KB *> *);
   static void self(Entity *, ll_queue<KB *> *);
   static void goal(Entity *, ll_queue<KB *> *);
   static void sample(Sample *, ll_queue<KB *> *);
   static void waiterInfo(Waiter *, ll_queue<KB *> *);
   static void mapperInfo(Mapper *, ll_queue<KB *> *);
   static void grid(Grid *, ll_queue<KB *> *);
   static void fullGrid(Grid *, ll_queue<KB *> *);
};

/* agent class client */
class Reporter {
public:
   std::shared_ptr<Grid> environment;
   std::shared_ptr<Grid> full_env;
   std::shared_ptr<Entity> self;
   std::shared_ptr<Entity> goal;
   std::unordered_map<int,void (*)(struct kurome_basemsg *, Reporter *)> handlers;
   std::thread cli;
   ll_queue<KB *> reqs;
   std::vector<struct waiter_info> waiters;
   struct mapper_info mapper;
   struct agent_values * avaliable;
   struct agent_values * conn;

   Reporter() 
      : environment(NULL), full_env(NULL), self(NULL), 
        goal(NULL), avaliable(NULL), conn(NULL)
      { setDefaultHandlers(); };

   void launchClient();
   void clientSend(KB *);
   void registerHandler(int, void (*)(struct kurome_basemsg *, Reporter *));
   void setDefaultHandlers();

   void kcmdConnect(long naddr);

};
