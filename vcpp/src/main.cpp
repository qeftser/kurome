
#include <SFML/Graphics.hpp>
#include <SFML/Window/Keyboard.hpp>
#include <SFML/Window/Mouse.hpp>
#include "Grid.h"
#include <cerrno>
#include <cstdlib>
#include <ctime>
#include <chrono>
#include <thread>

#define STATE_NORMAL  0x000
#define STATE_ADD     0x001
#define STATE_RECT    0x002
#define STATE_CIRCLE  0x004
#define STATE_POINT   0x008
#define STATE_DIST1   0x010
#define STATE_DIST2   0x020
#define STATE_POSX    0x040
#define STATE_POSY    0x080
#define STATE_DEMO    0x100

int rec_key(int code, char * buf) {
   switch (code) {
      case sf::Keyboard::Num1:
         *buf = '1';
         return 1;
         break;
      case sf::Keyboard::Num2:
         *buf = '2';
         return 1;
         break;
      case sf::Keyboard::Num3:
         *buf = '3';
         return 1;
         break;
      case sf::Keyboard::Num4:
         *buf = '4';
         return 1;
         break;
      case sf::Keyboard::Num5:
         *buf = '5';
         return 1;
         break;
      case sf::Keyboard::Num6:
         *buf = '6';
         return 1;
         break;
      case sf::Keyboard::Num7:
         *buf = '7';
         return 1;
         break;
      case sf::Keyboard::Num8:
         *buf = '8';
         return 1;
         break;
      case sf::Keyboard::Num9:
         *buf = '9';
         return 1;
         break;
      case sf::Keyboard::Num0:
         *buf = '0';
         return 1;
         break;
      case sf::Keyboard::Period:

         return 1;
         break;
   }
   return 0;
}

#define GRID_LEN 30
#define GRID_WID 20

int main(void) {

   srand(time(NULL)*clock());

   sf::RenderWindow window = sf::RenderWindow(sf::VideoMode(1280,720),"main");
   sf::View view = sf::View();
   window.setFramerateLimit(15);

   int xpos = window.getSize().x/2;
   int ypos = window.getSize().y/2;
   int state = STATE_NORMAL;

   int itercount;

   int numbufpos = 0;
   char numbuf[100];
   double tDist1, tDist2, tPosX, tPosY;

   Grid g = Grid(0.1,GRID_LEN,GRID_WID);
   MovingEntity m = MovingEntity(0x2f,ENTITY_TYPE_RECT,1,1,1,GRID_WID-1,0.5);
   struct MappingState ms;

   Entity::e_rand odds = {
      0.7,
      3,
      3,
      GRID_LEN,
      GRID_WID,
      3,
      0
   };

   while (window.isOpen()) {
start:
      sf::Event event;
      if (STATE_DEMO == state) {
         itercount++;
         if (g.mapEntity(&window,&ms,5*itercount,&m,GRID_LEN-1,1)) {
            window.clear();
            g.display(window);
            window.display();
            std::this_thread::sleep_for(std::chrono::seconds(1));
            g.clear();
            int num = 18+rand()%20;
            for (int i = 0; i < num; ++i)
               g.addEntity(Entity::random(&odds));
            itercount = 0;
            init_MappingState(&ms);
         }
      }
      while (window.pollEvent(event)) {
         if (event.type == sf::Event::Closed) {
            window.close();
         }
         else if (event.type == sf::Event::KeyPressed) {
            int code = event.key.code;
            if (code == sf::Keyboard::Q) {
               if (state == STATE_DEMO)
                  state = STATE_NORMAL;
               else
                  window.close();
            }
            else if (STATE_DEMO == state) {
               switch (code) {
                  case sf::Keyboard::L:
                  case sf::Keyboard::Left:
                     xpos -= 10;
                     break;
                  case sf::Keyboard::H:
                  case sf::Keyboard::Right:
                     xpos += 10;
                     break;
                  case sf::Keyboard::J:
                  case sf::Keyboard::Down:
                     ypos -= 10;
                     break;
                  case sf::Keyboard::K:
                  case sf::Keyboard::Up:
                     ypos += 10;
                     break;
               }
            }
            else if (STATE_NORMAL == state) {
               switch (code) {
                  case sf::Keyboard::L:
                  case sf::Keyboard::Left:
                     xpos -= 10;
                     break;
                  case sf::Keyboard::H:
                  case sf::Keyboard::Right:
                     xpos += 10;
                     break;
                  case sf::Keyboard::J:
                  case sf::Keyboard::Down:
                     ypos -= 10;
                     break;
                  case sf::Keyboard::K:
                  case sf::Keyboard::Up:
                     ypos += 10;
                     break;
                  case sf::Keyboard::A:
                     printf("add ");
                     state = STATE_ADD;
                     break;
                  case sf::Keyboard::Space:
                     goto mouse;
                     break;
                  case sf::Keyboard::M:
                     g.remap();
                     init_MappingState(&ms);
                     g.mapEntity(NULL,&ms,-1,&m,GRID_LEN-1,1);
                     break;
                  case sf::Keyboard::X:
                     itercount = 0;
                     g.remap();
                     init_MappingState(&ms);
                     state = STATE_DEMO;
                     break;
                  case sf::Keyboard::R:
                     g.clear();
                     int num = 18+rand()%20;
                     for (int i = 0; i < num; ++i)
                        g.addEntity(Entity::random(&odds));
                     break;
               }
            }
            else if (STATE_ADD&state) {
               if (STATE_DIST1&state) {
                  if (code == sf::Keyboard::B) {
                     errno = 0;
                     numbuf[numbufpos] = 0;
                     tDist1 = std::strtod(numbuf,NULL);
                     printf("%lf by ",tDist1);
                     if (errno)
                        goto state_reset;
                     state&=(~STATE_DIST1);
                     state|=STATE_DIST2;
                     numbufpos = 0;
                  }
                  else {
                     int ret;
                     if ((ret = rec_key(code,numbuf+numbufpos)) == 0)
                        goto state_reset;
                     numbufpos += ret;
                  }
               }
               else if (STATE_DIST2&state) {
                  if (code == sf::Keyboard::A) {
                     errno = 0;
                     numbuf[numbufpos] = 0;
                     tDist2 = std::strtod(numbuf,NULL);
                     printf("%lf at ",tDist2);
                     if (errno)
                        goto state_reset;
                     state&=(~STATE_DIST2);
                     state|=STATE_POSX;
                     numbufpos = 0;
                  }
                  else {
                     int ret;
                     if ((ret = rec_key(code,numbuf+numbufpos)) == 0)
                        goto state_reset;
                     numbufpos += ret;
                  }
               }
               else if (STATE_POSX&state) {
                  if (code == sf::Keyboard::X) {
                     errno = 0;
                     numbuf[numbufpos] = 0;
                     tPosX = std::strtod(numbuf,NULL);
                     printf("(%lf,",tPosX);
                     if (errno)
                        goto state_reset;
                     state&=(~STATE_POSX);
                     state|=STATE_POSY;
                     numbufpos = 0;
                  }
                  else {
                     int ret;
                     if ((ret = rec_key(code,numbuf+numbufpos)) == 0)
                        goto state_reset;
                     numbufpos += ret;
                  }
               }
               else if (STATE_POSY&state) {
                  if (code == sf::Keyboard::E) {
                     errno = 0;
                     numbuf[numbufpos] = 0;
                     tPosY = std::strtod(numbuf,NULL);
                     printf("%lf)\n",tPosY);
                     if (errno)
                        goto state_reset;
                     Entity * e;
                     if (state&STATE_RECT)
                        e = new Entity(0x2f,ENTITY_TYPE_RECT,tDist1,tDist2,tPosX,tPosY);
                     else if (state&STATE_CIRCLE)
                        e = new Entity(0x2f,ENTITY_TYPE_CIRCLE,tDist2,-1,tPosX,tPosY);
                     else if (state&STATE_POINT)
                        e = new Entity(0x2f,ENTITY_TYPE_POINT,-1,-1,tPosX,tPosY);
                     g.addEntity(e);
                     goto state_reset;
                  }
                  else {
                     int ret;
                     if ((ret = rec_key(code,numbuf+numbufpos)) == 0)
                        goto state_reset;
                     numbufpos += ret;
                  }
               }
               else {
                  switch (code) {
                     case sf::Keyboard::R:
                        printf("rect ");
                        state|=STATE_RECT;
                        state|=STATE_DIST1;
                        break;
                     case sf::Keyboard::C:
                        printf("circle ");
                        state|=STATE_CIRCLE;
                        state|=STATE_DIST2;
                        break;
                     case sf::Keyboard::P:
                        state|=STATE_POINT;
                        state|=STATE_POSX;
                        printf("point ");
                        break;
                     default:
                        goto state_reset;
                        break;
                  }
               }
            }
            else if (sf::Mouse::isButtonPressed(sf::Mouse::Button::Left)) {
mouse:
               printf("mouse!\n");
               sf::Vector2i pos = sf::Mouse::getPosition(window);
               sf::Vector2u currS = window.getSize();
               double lposX = ((pos.x+(xpos-(currS.x/2.0)))/g.getLenW())*g.getUSize(),
                      lposY = ((pos.y+(ypos-(currS.y/2.0)))/g.getLenW())*g.getUSize();
               Entity * held = g.match(lposX,lposY);
               if (held) {
                  held->posX = lposX;
                  held->posY = lposY;
                  g.remap();
               }
               else {
                  printf("Nheld...\n");
               }
            }
         }
      }



      window.clear();
      window.setView(window.getDefaultView());

      view.setCenter(xpos,ypos);
      view.setSize(window.getSize().x,window.getSize().y);
      window.setView(view);

      g.display(window);

      window.display();

   }

   return 0;

state_reset:
   {
      printf("reset\n");
      numbufpos = 0;
      *numbuf = 0;
      tDist1 = 0;
      tDist2 = 0;
      tPosX = 0;
      tPosY = 0;
      state = STATE_NORMAL;
      goto start;
   }
}
