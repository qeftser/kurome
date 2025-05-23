
#include "../../Kurome.h"
#include "../../headers/EllipseShape.hpp"
#include "../../mappers/NILRotAStarMapper.hpp"
#include "../../mappers/SimpleAnyRotStaticPotentialFieldMapper.hpp"
#include <SFML/Graphics.hpp>
#include <chrono>
#include <errno.h>
#include <fcntl.h>
#include <setjmp.h>
#include <signal.h>

#define KUROME_VIEWER_FRAMERATE 30

#define KUROME_VIEWER_APAUSE  1
#define KUROME_VIEWER_ASTART  2
#define KUROME_VIEWER_DALL    0x01
#define KUROME_VIEWER_DFULL   0x02
#define KUROME_VIEWER_DKNOWN  0x04
#define KUROME_VIEWER_DGRID   0x08
#define KUROME_VIEWER_DSELF   0x10
#define KUROME_VIEWER_DMAPP   0x20
#define KUROME_VIEWER_DGOAL   0x40
#define KUROME_VIEWER_DBORDER 0x80
#define KUROME_VIEWER_MNONE   1
#define KUROME_VIEWER_MRECT   2
#define KUROME_VIEWER_MELPS   3
#define KUROME_VIEWER_MDEL    4
#define KUROME_VIEWER_MMOVE   5
#define KUROME_VIEWER_MSMOVE  6
#define KUROME_VIEWER_MGMOVE  7

struct viewer_map_save_header {
   int magic;
   int total;
};

/*
jmp_buf cleanup_jmp;

void kurome_viewer_interrupt_handler(int signo) {
   longjmp(cleanup_jmp,69);
}
*/

void kurome_viewer_draw_entity(Entity & en, sf::RenderWindow & w, double winScale) {
   sf::RectangleShape r;
   EllipseShape e;
   switch (en.type) {
      case KUROME_TYPE_RECT:
         r.setSize(sf::Vector2f(en.xwid*winScale,en.ywid*winScale));
         r.setRotation(en.rot);
         r.setFillColor(sf::Color(160,32,240,255));
         r.setPosition((en.posx*winScale)-(r.getSize().x/2.0),(en.posy*winScale)-(r.getSize().y/2.0));
         w.draw(r);
         break;
      case KUROME_TYPE_ELPS:
         e.setRadius(sf::Vector2f((en.xwid/2)*winScale,(en.ywid/2)*winScale));
         e.setRotation(en.rot);
         e.setFillColor(sf::Color(160,32,240,255));
         e.setPosition((en.posx*winScale)-(e.getRadius().x),(en.posy*winScale)-(e.getRadius().y));
         w.draw(e);
         break;
      case KUROME_TYPE_PONT:
         r.setSize(sf::Vector2f(winScale,winScale));
         r.setFillColor(sf::Color(160,32,240,255));
         r.setPosition((en.posx*winScale)-(r.getSize().x/2.0),(en.posy*winScale)-(r.getSize().y/2.0));
         w.draw(r);
         break;
   }
}

void kurome_viewer_place_NILRotAStarMapper(Reporter * me, sf::RenderWindow * w, double invU) {
   NILRotAStarMapper nrm = NILRotAStarMapper(me);
   nrm.callback(0xffffffff);
   bool done = false;
   Frame curr;
   while (!done) {
      Frame curr = nrm.nextPoint(done);
      sf::RectangleShape r;
      r.setSize(sf::Vector2f(invU,invU));
      r.setPosition((curr.posx)*invU,(curr.posy)*invU);
      r.setFillColor(sf::Color(0,255,255));
      w->draw(r);
   }
   for (Frame * f : nrm.allocated) 
      delete f;
}

void kurome_viewer_place_SimpleAnyRotStaticPotentialFieldMapper(Reporter * me, int c2, sf::RenderWindow * w, double wS, double uS) {
   static SimpleAnyRotStaticPotentialFieldMapper sarspfm = SimpleAnyRotStaticPotentialFieldMapper(c2,me);
   sarspfm.callback(0); 
   double scale = wS*(sarspfm.binfo.unitSize/uS);
   double hcale = scale/2;
   for (int i = sarspfm.binfo.blocksXmin; i < sarspfm.binfo.blocksXmax; ++i) {
      for (int j = sarspfm.binfo.blocksYmin; j < sarspfm.binfo.blocksYmax; ++j) {
         int v = (sarspfm.base->blocks(i,j)*2 > 255 ? 255 : sarspfm.base->blocks(i,j)*2);
         sf::RectangleShape r;
         r.setSize(sf::Vector2f(scale,scale));
         r.setPosition(i*scale,j*scale);
         r.setFillColor(sf::Color(v,255-v,255-v,100));
         w->draw(r);
      }
   }
   for (int i = sarspfm.binfo.blocksXmin+1; i < sarspfm.binfo.blocksXmax-1; ++i) {
      for (int j = sarspfm.binfo.blocksYmin+1; j < sarspfm.binfo.blocksYmax-1; ++j) {
         if (sarspfm.base->blocks(i,j) != SHRT_MAX) {
            int min = sarspfm.base->blocks(i+1,j);
            int v = 1;
            if (sarspfm.base->blocks(i-1,j) == min) {
               v |= 2;
            }
            else if (sarspfm.base->blocks(i-1,j) < min) {
               min = sarspfm.base->blocks(i-1,j);
               v = 2;
            }
            if (sarspfm.base->blocks(i,j+1) == min) {
               v |= 4;
            }
            else if (sarspfm.base->blocks(i,j+1) < min) {
               min = sarspfm.base->blocks(i,j+1);
               v = 4;
            }
            if (sarspfm.base->blocks(i,j-1) == min) {
               v |= 8;
            }
            else if (sarspfm.base->blocks(i,j-1) < min) {
               min = sarspfm.base->blocks(i,j-1);
               v = 8;
            }
            sf::Vertex r[2];
            if (v&2) {
               r[0].position = sf::Vector2f(i*scale,j*scale+hcale);
               r[1].position = sf::Vector2f(i*scale,j*scale);
               r[0].color = sf::Color::White;
               r[1].color = sf::Color::Yellow;
               w->draw(r,2,sf::Lines);
            }
            if (v&1) {
               r[0].position = sf::Vector2f(i*scale+hcale,j*scale+hcale);
               r[1].position = sf::Vector2f(i*scale+hcale,j*scale);
               r[0].color = sf::Color::Yellow;
               r[1].color = sf::Color::White;
               w->draw(r,2,sf::Lines);
            }
            if (v&8) {
               r[0].position = sf::Vector2f(i*scale+hcale,j*scale);
               r[1].position = sf::Vector2f(i*scale,j*scale);
               r[0].color = sf::Color::Yellow;
               r[1].color = sf::Color::White;
               w->draw(r,2,sf::Lines);
            }
            if (v&4) {
               r[0].position = sf::Vector2f(i*scale+hcale,j*scale+hcale);
               r[1].position = sf::Vector2f(i*scale,j*scale+hcale);
               r[0].color = sf::Color::White;
               r[1].color = sf::Color::Yellow;
               w->draw(r,2,sf::Lines);
            }
         }
      }
   }
}

static int kurome_viewer_prioritize_new_weight(int w1, int w2) {
   return w1;
}

void kurome_viewer_MSG_SAMPLE_handler(KB * msg, void * me) {
   if (((Reporter *)me)->environment) {
      struct kurome_samplemsg * sm = (struct kurome_samplemsg *)msg;
      Sample nev = Sample(&sm->s);
      ((Reporter *)me)->environment->apply(&nev,kurome_viewer_prioritize_new_weight);
   }
}

void kurome_viewer_MSG_FSAMPLE_handler(KB * msg, void * me) {
   if (((Reporter *)me)->environment) {
      struct kurome_samplemsg * sm = (struct kurome_samplemsg *)msg;
      Sample nev = Sample(&sm->s);
      ((Reporter *)me)->environment->apply(&nev,kurome_viewer_prioritize_new_weight);
   }
}

void kurome_viewer_MSG_PAUSE_handler(KB * msg, void * me) {
   int * agentState = (int *)me;
   *agentState = KUROME_VIEWER_APAUSE;
}

void kurome_viewer_MSG_START_handler(KB * msg, void * me) {
   int * agentState = (int *)me;
   *agentState = KUROME_VIEWER_ASTART;
}

int main(int argc, char ** argv) {

   srand(time(NULL)*clock());

   sf::RenderWindow window;
   sf::View view = sf::View();
   sf::Event event;

   Reporter reporter;
   reporter.registerHandler(KUROME_MSG_SAMPLE,kurome_viewer_MSG_SAMPLE_handler);
   reporter.registerHandler(KUROME_MSG_FSAMPLE,kurome_viewer_MSG_FSAMPLE_handler);

   int agentState = KUROME_VIEWER_APAUSE;
   reporter.registerHandler(KUROME_MSG_PAUSE,kurome_viewer_MSG_PAUSE_handler);
   reporter.registerHandler(KUROME_MSG_START,kurome_viewer_MSG_START_handler);
   reporter.registerHandlerData(KUROME_MSG_PAUSE,&agentState);
   reporter.registerHandlerData(KUROME_MSG_START,&agentState);

   reporter.launchClient();


   struct timeval tv = {0,500000};
   fd_set stdin_set, useset;
   FD_ZERO(&stdin_set);
   FD_SET(STDIN_FILENO,&stdin_set);
   long i, j, n;
   char buf[1024];

   int mdivisor = KUROME_NOGO_COST/255;
   int fdivisor = KUROME_NOGO_COST/150;

   KUROME_ENTITY_ID_NUM = (INT_MAX/2); // ensure no overlap between our entities
                                       // and the agent provided ones.
   bool   mouseDown;
   int    lastKey;
   int    xpos, ypos;
   double scale;
   Entity * held = NULL;

   double winScale;
   double invU;

   double startX, startY, posX, posY;

   int mouseState = KUROME_VIEWER_MNONE;
   int drawState = KUROME_VIEWER_DKNOWN|KUROME_VIEWER_DBORDER|KUROME_VIEWER_DGOAL;

   /*
   struct sigaction sa;
   bzero(&sa,sizeof(sa));
   sa.sa_handler = kurome_viewer_interrupt_handler;
   sigaction(SIGINT,&sa,NULL);

   if (setjmp(cleanup_jmp))
      goto cleanup;
   */

connection:
   while (1) {
      struct agent_values * curr = reporter.avaliable;
      printf("\nconns\n");
      while (curr) {
         printf("%s:%d:%ld\t%s\n",curr->addr,curr->port,curr->naddr,curr->name);
         curr = curr->nextptr;
      }
      fprintf(stderr,"> ");
      useset = stdin_set;
      tv.tv_sec = 5;
      n = select(STDIN_FILENO+1,&useset,NULL,NULL,&tv);
      if (n) {
         bzero(buf,1024);
         buf[strlen(fgets(buf,1023,stdin))-1]=0;
         errno = 0;
         n = strtol(buf,NULL,10);
         if ((!errno && reporter.connect(n)) || reporter.connect(buf))
            break;
      }
   }

   printf("connecting\n");
   window.create(sf::VideoMode(1280,720),"viewer");
   window.setFramerateLimit(KUROME_VIEWER_FRAMERATE);
   xpos = window.getSize().x/2;
   ypos = window.getSize().y/2;
   scale = 1.0;


   /* get all the data we want and need */

   //printf("getting env\n");
   //uint64_t before = reporter.recved.load();
   //reporter.wait(before);

   //printf("getting fullEnv\n");
   //before = reporter.recved.load();

   {
      kcmd::getGrid(&reporter.reqs);

      while (!reporter.environment) usleep(10000);
      kcmd::allSamples(&reporter.reqs);
      kcmd::allEntities(&reporter.reqs);
   }

   if (reporter.conn->flags&KUROME_AFLAG_FULLENV) {
      kcmd::getFullGrid(&reporter.reqs);

      while (!reporter.full_env) usleep(10000);
      kcmd::fAllEntities(&reporter.reqs);
      kcmd::fAllSamples(&reporter.reqs);
   }

   //reporter.wait(before);

   //printf("getting self\n");
   //before = reporter.recved.load();
   kcmd::getSelf(&reporter.reqs);
   //reporter.wait(before);

   //printf("getting goal\n");
   //before = reporter.recved.load();
   kcmd::getGoal(&reporter.reqs);
   //reporter.wait(before);

   //printf("getting mapper\n");
   //before = reporter.recved.load();
   kcmd::getMapperInfo(&reporter.reqs);
   //reporter.wait(before);

   kcmd::state(&reporter.reqs);

   /* run gui */

   printf("running\n");

   if (argc == 3 && strcmp(argv[1],"-m") == 0) {
      strncpy(buf,argv[2],sizeof(buf)-1);
      goto open_file;
   }

   printf("enter map name to load\n");
   printf("or enter s followed by a name\nto save a config\n");

   fprintf(stderr,"> ");
   while (window.isOpen()) {

      useset = stdin_set;
      tv.tv_sec  = 0;
      tv.tv_usec = 0;
      n = select(STDIN_FILENO+1,&useset,NULL,NULL,&tv);
      if (n) {
         bzero(buf,1024);
         buf[strlen(fgets(buf,1023,stdin))-1]=0;
         if (*buf) {
open_file:
            if (*buf == 's') {
               char * cpy = buf;
               do { ++cpy; } while(isspace(*cpy));
               printf("saving as %s...\n",cpy);
               int fd = open(cpy,O_WRONLY|O_CREAT,S_IRWXU);
               struct viewer_map_save_header 
                  msave = { .magic = 0xc234fda, .total = 0 }; 
               struct entity_struct use;
               lseek(fd,sizeof(msave),SEEK_SET);
               for (Entity * e : reporter.full_env->entities) {
                  ++msave.total;
                  e->toStruct(&use);
                  write(fd,&use,sizeof(use));
               }
               lseek(fd,0,SEEK_SET);
               write(fd,&msave,sizeof(msave));
               close(fd);
            }
            else if (access(buf,F_OK) == 0) {
               printf("opening %s...\n",buf);
               int fd = open(buf,O_RDONLY);
               struct viewer_map_save_header msave;
               read(fd,&msave,sizeof(msave));
               if (msave.magic != 0xc234fda) {
                  printf("not a viewer map file\n");
                  close(fd);
                  goto draw_step;
               }
               kcmd::fClense(&reporter.reqs);
               reporter.full_env->clense();
               struct entity_struct use;
               for (int i = 0; i < msave.total; ++i) {
                  read(fd,&use,sizeof(use));
                  Entity * nev = new Entity(&use);
                  kcmd::fAddEntity(*nev,&reporter.reqs);
                  reporter.full_env->addEntity(nev);
               }
               close(fd);
            }
         }
draw_step:
         fprintf(stderr,"> ");
      }

      int sizeX = reporter.ginfo.blocksXmax-reporter.ginfo.blocksXmin;
      int sizeY = reporter.ginfo.blocksYmax-reporter.ginfo.blocksYmin;
      winScale = (((window.getSize().x>window.getSize().y)?window.getSize().y:window.getSize().x)
                  /(double)((sizeX>sizeY?sizeX:sizeY)));
      invU = winScale*(1/reporter.ginfo.unitSize);

      while (window.pollEvent(event)) {
         if (event.type == sf::Event::Closed) {
            window.close();
         }
         else if (event.type == sf::Event::MouseButtonPressed) {
            sf::Vector2f pos = window.mapPixelToCoords(sf::Mouse::getPosition(window));
            sf::Vector2u ws = window.getSize();
            startX = (((pos.x))/(winScale))*reporter.ginfo.unitSize;
            startY = (((pos.y))/(winScale))*reporter.ginfo.unitSize;
            mouseDown = true;
            if (mouseState == KUROME_VIEWER_MNONE) {
               if (reporter.goal &&
                   startX > reporter.goal->posx-(reporter.goal->xwid) &&
                   startX < reporter.goal->posx+(reporter.goal->xwid) &&
                   startY > reporter.goal->posy-(reporter.goal->ywid) &&
                   startY < reporter.goal->posy+(reporter.goal->ywid))
                  mouseState = KUROME_VIEWER_MGMOVE;
               else if (reporter.self &&
                        startX > reporter.self->posx-(reporter.self->xwid) &&
                        startX < reporter.self->posx+(reporter.self->xwid) &&
                        startY > reporter.self->posy-(reporter.self->ywid) &&
                        startY < reporter.self->posy+(reporter.self->ywid))
                  mouseState = KUROME_VIEWER_MSMOVE;
               else if (reporter.full_env) {
                  for (Entity * e : reporter.full_env->entities) {
                     if (startX > e->posx-(e->xwid/2.0) &&
                         startX < e->posx+(e->xwid/2.0) &&
                         startY > e->posy-(e->ywid/2.0) &&
                         startY < e->posy+(e->ywid/2.0)) {
                        mouseState = KUROME_VIEWER_MMOVE;
                        reporter.full_env->remEntity(e);
                        held = e;
                        break;
                     }
                  }
               }
            }
            else if (reporter.full_env && mouseState == KUROME_VIEWER_MDEL) {
               for (Entity * e : reporter.full_env->entities) {
                  if (startX > e->posx-(e->xwid/2.0) &&
                      startX < e->posx+(e->xwid/2.0) &&
                      startY > e->posy-(e->ywid/2.0) &&
                      startY < e->posy+(e->ywid/2.0)) {
                     reporter.full_env->remEntity(e);
                     kcmd::fRemEntity(*e,&reporter.reqs);
                     delete e;
                     mouseState = KUROME_VIEWER_MNONE;
                     break;
                  }
               }
            }
         }
         else if (event.type == sf::Event::MouseButtonReleased) {
            mouseDown = false;
            Entity * nev;
            switch (mouseState) {
               case KUROME_VIEWER_MRECT:
                  if (reporter.full_env) {
                     if (startX < posX && startY < posY) {
                        for (Entity * e : reporter.full_env->entities) {
                           if (e->id > KUROME_ENTITY_ID_NUM)
                              KUROME_ENTITY_ID_NUM = e->id;
                        }
                        nev = new Entity(startX,startY,(posX-startX),(posY-startY),KUROME_TYPE_RECT,SHRT_MAX);
                        nev->posx += (nev->xwid/2.0);
                        nev->posy += (nev->ywid/2.0);
                        reporter.full_env->addEntity(nev);
                        reporter.full_env->redraw();
                        kcmd::fAddEntity(*nev,&reporter.reqs);
                     }
                  }
                  break;
               case KUROME_VIEWER_MELPS:
                  if (reporter.full_env) {
                     if (startX < posX && startY < posY) {
                        for (Entity * e : reporter.full_env->entities) {
                           if (e->id > KUROME_ENTITY_ID_NUM)
                              KUROME_ENTITY_ID_NUM = e->id;
                        }
                        nev = new Entity(startX,startY,(posX-startX),(posY-startY),KUROME_TYPE_ELPS,SHRT_MAX);
                        nev->posx += (nev->xwid/2.0);
                        nev->posy += (nev->ywid/2.0);
                        reporter.full_env->addEntity(nev);
                        reporter.full_env->redraw();
                        kcmd::fAddEntity(*nev,&reporter.reqs);
                     }
                  }
                  break;
               case KUROME_VIEWER_MSMOVE:
                  if (reporter.self)
                     kcmd::chgSelf(*reporter.self,&reporter.reqs);
                  break;
               case KUROME_VIEWER_MGMOVE:
                  if (reporter.goal) 
                     kcmd::chgGoal(*reporter.goal,&reporter.reqs);
                  break;
               case KUROME_VIEWER_MMOVE:
                  if (reporter.full_env && held) {
                     held->posx = posX;
                     held->posy = posY;
                     reporter.full_env->addEntity(held);
                     reporter.full_env->redraw();
                     kcmd::fChgEntity(*held,&reporter.reqs);
                  }
                  held = NULL;
                  break;
            }
            mouseState = KUROME_VIEWER_MNONE;
         }
         else if (event.type == sf::Event::KeyPressed) {
            lastKey = event.key.code;
            if (lastKey == sf::Keyboard::Q) {
               window.close();
            }
            switch (lastKey) {
               case sf::Keyboard::L:
               case sf::Keyboard::Left:
                  xpos += 10;
                  break;
               case sf::Keyboard::H:
               case sf::Keyboard::Right:
                  xpos -= 10;
                  break;
               case sf::Keyboard::J:
               case sf::Keyboard::Down:
                  ypos += 10;
                  break;
               case sf::Keyboard::K:
               case sf::Keyboard::Up:
                  ypos -= 10;
                  break;
               case sf::Keyboard::I:
                  scale *= 0.909091;
                  break;
               case sf::Keyboard::O:
                  scale *= 1.1;
                  break;
               case sf::Keyboard::Space:
                  if (agentState == KUROME_VIEWER_APAUSE) {
                     agentState = KUROME_VIEWER_ASTART;
                     kcmd::start(&reporter.reqs);
                  } else {
                     agentState = KUROME_VIEWER_APAUSE;
                     kcmd::pause(&reporter.reqs);
                  }
                  break;
               case sf::Keyboard::R:
                  mouseState = KUROME_VIEWER_MRECT;
                  break;
               case sf::Keyboard::E:
                  mouseState = KUROME_VIEWER_MELPS;
                  break;
               case sf::Keyboard::D:
                  mouseState = KUROME_VIEWER_MDEL;
                  break;
               case sf::Keyboard::N:
                  mouseState = KUROME_VIEWER_MNONE;
                  break;
               case sf::Keyboard::F:
                  drawState ^= KUROME_VIEWER_DFULL;
                  break;
               case sf::Keyboard::G:
                  drawState ^= KUROME_VIEWER_DGOAL;
                  break;
               case sf::Keyboard::S:
                  drawState ^= KUROME_VIEWER_DSELF;
                  break;
               case sf::Keyboard::M:
                  drawState ^= KUROME_VIEWER_DMAPP;
                  break;
               case sf::Keyboard::X:
                  drawState ^= KUROME_VIEWER_DGRID;
                  break;
               case sf::Keyboard::A:
                  drawState ^= KUROME_VIEWER_DALL;
                  break;
               case sf::Keyboard::V:
                  drawState ^= KUROME_VIEWER_DKNOWN;
                  break;
               case sf::Keyboard::B:
                  drawState ^= KUROME_VIEWER_DBORDER;
                  break;
            }
         }
      }

      window.clear(sf::Color(0,0,0,0));
      window.setView(window.getDefaultView());
      view.setCenter(xpos,ypos);
      view.setSize(window.getSize().x,window.getSize().y);
      view.zoom(scale);
      window.setView(view);

      sf::RectangleShape r;
      EllipseShape e;
      if (reporter.full_env) {
         if (drawState&(KUROME_VIEWER_DALL|KUROME_VIEWER_DFULL)) {
            double altScale = winScale*(reporter.fginfo.unitSize/reporter.ginfo.unitSize);
            r.setSize(sf::Vector2f(altScale,altScale));
            for (i = reporter.fginfo.blocksXmin; i < reporter.fginfo.blocksXmax; ++i) {
               for (j = reporter.fginfo.blocksYmin; j < reporter.fginfo.blocksYmax; ++j) {
                  int cco = reporter.full_env->blocks(i,j)/fdivisor;
                  if (cco) {
                     if (cco > 255)
                        cco = 255;
                     r.setFillColor(sf::Color(cco,cco,cco,255));
                     r.setPosition(i*altScale,j*altScale);
                     window.draw(r);
                  }
               }
            }
         }
      }
      r.setSize(sf::Vector2f(winScale,winScale));
      r.setFillColor(sf::Color(255,0,0,255));
      if (reporter.environment) {
         if (drawState&(KUROME_VIEWER_DALL|KUROME_VIEWER_DKNOWN)) {
            for (i = reporter.ginfo.blocksXmin; i < reporter.ginfo.blocksXmax; ++i) {
               for (j = reporter.ginfo.blocksYmin; j < reporter.ginfo.blocksYmax; ++j) {
                   int vvo = reporter.environment->blocks(i,j)/mdivisor;
                   if (vvo) {
                     if (vvo > 255)
                        vvo = 255;
                      r.setFillColor(sf::Color(vvo,255-vvo,0,255));
                      r.setPosition(i*winScale,j*winScale);
                      window.draw(r);
                   }
               }
            }
         }
         if (drawState&(KUROME_VIEWER_DALL|KUROME_VIEWER_DGRID)) {
            r.setFillColor(sf::Color(0,0,0,0));
            r.setOutlineColor(sf::Color::White);
            r.setOutlineThickness(0.35);
            for (i = reporter.ginfo.blocksXmin; i < reporter.ginfo.blocksXmax; ++i) {
               for (j = reporter.ginfo.blocksYmin; j < reporter.ginfo.blocksYmax; ++j) {
                   r.setPosition(i*winScale,j*winScale);
                   window.draw(r);
               }
            }
         }
      }

      if (reporter.environment && reporter.self && reporter.goal) {
         if (drawState&(KUROME_VIEWER_DALL|KUROME_VIEWER_DMAPP) && mouseState == KUROME_VIEWER_MNONE) {
            if (strcmp(reporter.mapper.name,"NILRotAStarMapper") == 0) {
               kurome_viewer_place_NILRotAStarMapper(&reporter,&window,invU);
            }
            else if (strcmp(reporter.mapper.name,"SARSPFM Mapper") == 0) {
               kurome_viewer_place_SimpleAnyRotStaticPotentialFieldMapper(&reporter,reporter.mapper.state,&window,winScale,reporter.ginfo.unitSize);
            }
         }
      }

      if (reporter.self) {
         if (drawState&(KUROME_VIEWER_DALL|KUROME_VIEWER_DSELF)) {
            kurome_viewer_draw_entity(*reporter.self,window,invU);
         }
         else {
            sf::CircleShape me(8,3);
            me.setRotation(reporter.self->rot);
            me.setPosition((reporter.self->posx*invU)-6,(reporter.self->posy*invU)-6);
            me.setFillColor(sf::Color(160,32,240,255));
            window.draw(me);
         }
      }

      if (reporter.goal) {
         if (drawState&(KUROME_VIEWER_DALL|KUROME_VIEWER_DGOAL)) {
            kurome_viewer_draw_entity(*reporter.goal,window,invU);
         }
      }

      if (reporter.environment) {
         if (drawState&(KUROME_VIEWER_DALL|KUROME_VIEWER_DBORDER)) {
            r.setFillColor(sf::Color(0,0,0,0));
            r.setOutlineColor(sf::Color::White);
            r.setOutlineThickness(1);
            r.setPosition(reporter.ginfo.blocksXmin*invU,reporter.ginfo.blocksYmin*invU);
            r.setSize(sf::Vector2f((reporter.ginfo.blocksXmax-reporter.ginfo.blocksXmin)*invU*0.5,
                                   (reporter.ginfo.blocksYmax-reporter.ginfo.blocksYmin)*invU*0.5));
            window.draw(r);
         }
      }

      /*
      if (drawState&(KUROME_VIEWER_DALL|KUROME_VIEWER_DFULL) && held) {
         kurome_viewer_draw_entity(*held,window,winScale);
      }
      */

      if (mouseDown) {
         sf::Vector2f pos = window.mapPixelToCoords(sf::Mouse::getPosition(window));
         sf::Vector2u currS = window.getSize();
         posX = (((pos.x))/(winScale))*reporter.ginfo.unitSize;
         posY = (((pos.y))/(winScale))*reporter.ginfo.unitSize;
         r.setOutlineColor(sf::Color(0,0,0,0));
         e.setOutlineColor(sf::Color(0,0,0,0));
         r.setFillColor(sf::Color(0,0,255,255));
         e.setFillColor(sf::Color(0,0,255,255));
         switch (mouseState) {
            case KUROME_VIEWER_MNONE:
               break;
            case KUROME_VIEWER_MMOVE:
               if (held->type == KUROME_TYPE_RECT) {
                  r.setPosition((posX-(held->xwid/2.0))*invU,(posY-(held->ywid/2.0))*invU);
                  r.setSize(sf::Vector2f((held->xwid)*invU,(held->ywid)*invU));
                  r.setRotation(held->rot);
                  window.draw(r);
               }
               else if (held->type == KUROME_TYPE_ELPS) {
                  e.setPosition((posX-(held->xwid/2.0))*invU,(posY-(held->ywid/2.0))*invU);
                  e.setRadius(sf::Vector2f(((held->xwid)*invU)/2.0,((held->ywid)*invU)/2.0));
                  e.setRotation(held->rot);
                  window.draw(e);
               }
               break;
            case KUROME_VIEWER_MSMOVE:
               if (reporter.self) {
                  reporter.self->posx = posX;
                  reporter.self->posy = posY;
               }
               break;
            case KUROME_VIEWER_MGMOVE:
               if (reporter.goal) {
                  reporter.goal->posx = posX;
                  reporter.goal->posy = posY;
               }
               break;
            case KUROME_VIEWER_MRECT:
               if (startX < posX && startY < posY) {
                  r.setPosition(startX*invU,startY*invU);
                  r.setSize(sf::Vector2f((posX-startX)*invU,(posY-startY)*invU));
                  window.draw(r);
               }
               break;
            case KUROME_VIEWER_MELPS:
               if (startX < posX && startY < posY) {
                  e.setPosition(startX*invU,startY*invU);
                  e.setRadius(sf::Vector2f(((posX-startX)*invU)/2.0,((posY-startY)*invU)/2.0));
                  window.draw(e);
               }
               break;
         }
      }

      window.display();

   }


cleanup:

   /*
   if (setjmp(cleanup_jmp))
      goto cleanup;
   */

   reporter.disconnectClient();
   printf("\nConnection closed. Connect again? [y/n]: ");
   char res;
   res = getc(stdin);
   if (res == 'y')
      goto connection;

   exit(0);
}

