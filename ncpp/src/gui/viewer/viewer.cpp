
#include "../../Kurome.h"
#include <SFML/Graphics.hpp>
#include <errno.h>

#define KUROME_VIEWER_FRAMERATE 30

int main(void) {

   srand(time(NULL)*clock());

   sf::RenderWindow window;
   sf::View view = sf::View();
   sf::Event event;

   Reporter reporter;
   reporter.launchClient();

   struct timeval tv = {0,500000};
   fd_set stdin_set, useset;
   FD_ZERO(&stdin_set);
   FD_SET(STDIN_FILENO,&stdin_set);
   long i, n;
   char buf[1024];

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
   int xpos = window.getSize().x/2;
   int ypos = window.getSize().y/2;

   while (window.isOpen()) {

      while (window.pollEvent(event)) {
         if (event.type == sf::Event::Closed) {
            window.close();
         }
         else if (event.type == sf::Event::MouseButtonPressed) {
            printf("(%d, %d)\n",sf::Mouse::getPosition().x,sf::Mouse::getPosition().y);
         }
         else if (event.type == sf::Event::MouseButtonReleased) {
            printf("(-, -)\n");
         }
      }

      window.clear();
      window.setView(window.getDefaultView());
      view.setCenter(xpos,ypos);
      view.setSize(window.getSize().x,window.getSize().y);
      window.setView(view);

      // drawing functions

      window.display();

   }


   reporter.disconnectClient();
   printf("Connection closed. Connect again? [y/n]: ");
   char res = getc(stdin);
   if (res == 'y')
      goto connection;

   exit(0);
}

