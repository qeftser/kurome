
#include "../../Kurome.h"
#include <SFML/Graphics.hpp>

#define KUROME_VIEWER_FRAMERATE 30

int main(void) {

   srand(time(NULL)*clock());

   sf::RenderWindow window = sf::RenderWindow(sf::VideoMode(1280,720),"viewer");
   sf::View view = sf::View();
   sf::Event event;
   window.setFramerateLimit(KUROME_VIEWER_FRAMERATE);
   int xpos = window.getSize().x/2;
   int ypos = window.getSize().y/2;

   Reporter reporter;
   reporter.launchClient();


connection:
   while (1) {
      sf::sleep(sf::seconds(1));
      struct agent_values * curr = reporter.avaliable;
      printf("conns\n");
      while (curr) {
         printf("%s:%d\n",curr->addr,curr->port);
         curr = curr->nextptr;
      }
   }

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

   printf("Connection closed. Connect again? [y/n]: ");
   char res = getc(stdin);
   if (res == 'y')
      goto connection;

   return 0;
}
