
#include <functional>

#include <rclcpp/rclcpp.hpp>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "kurome.h"
#include "smoother.hpp"
#include "elastic_band.hpp"

/* The smoothing and velocity output node for Kurome. It will 
 * run the elastic band algorithm and forward the computed
 * velocity inputs to the controller.                       */

using std::placeholders::_1;

class Misao : public rclcpp::Node {
public:

   Misao() : Node("misao") {

      /* The topic to recieve the raw path on */
      this->declare_parameter("path_in","path");

      /* The topic to recieve odometry on */
      this->declare_parameter("odom_in","odom");

      /* The topic to recieve the goal position on */
      this->declare_parameter("goal_in","goal");

      /* The topic to send out the velocity on */
      this->declare_parameter("vel_out","cmd_vel");

      /* The topic to receive the environment map on */
      this->declare_parameter("map_in","map");

      /* Time interval in seconds to publish on */
      this->declare_parameter("publish_rate",0.1);

      /* The threshold (0 - 100) at which to consider
       * a position on the occupancy grid an obstacle */
      this->declare_parameter("obstacle_threshold",60);

      /* whether to launch the accompanying gui for the node */
      this->declare_parameter("launch_gui",true);

      /* the algorithm to use on the smoother. Right now
       * the only option avaliable is elastic_band.     */
      this->declare_parameter("algorithm","elastic_band");

      /* parameters only for the elastic band file. See the
       * elastic_band.hpp file for more information on what
       * these variables do in that algorithm.             */
      this->declare_parameter("band_length",-1);
      this->declare_parameter("influence_range", 5.0);
      this->declare_parameter("max_bubble",10.0);
      this->declare_parameter("contraction_gain",1.0);
      this->declare_parameter("repulsion_gain",1.0);
      this->declare_parameter("damping_gain",0.50);


      /* construct the smoother with the provided algorithm */
      if (this->get_parameter("algorithm").as_string() == "elastic_band") {
         smoother = new ElasticBand(this->get_parameter("band_length").as_int(),
                                    this->get_parameter("influence_range").as_double(),
                                    this->get_parameter("max_bubble").as_double(),
                                    this->get_parameter("contraction_gain").as_double(),
                                    this->get_parameter("repulsion_gain").as_double(),
                                    this->get_parameter("damping_gain").as_double());
      }

      /* instantiate the publisher */
      vel_out = this->create_publisher<geometry_msgs::msg::Twist>(
                this->get_parameter("vel_out").as_string(), 10);
      vel_callback = this->create_wall_timer(
            std::chrono::milliseconds((long)(1000 * this->get_parameter("publish_rate").as_double())),
            std::bind(&Misao::publish_vel, this));

      /* instantiate subscribers */
      path_in = this->create_subscription<nav_msgs::msg::Path>(
            this->get_parameter("path_in").as_string(), 10,
            std::bind(&Misao::collect_path, this, _1));

      odom_in = this->create_subscription<nav_msgs::msg::Odometry>(
            this->get_parameter("odom_in").as_string(), 10,
            std::bind(&Misao::collect_odom, this, _1));

      map_in = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            this->get_parameter("map_in").as_string(), 10,
            std::bind(&Misao::collect_map, this, _1));

      goal_in = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            this->get_parameter("goal_in").as_string(), 10,
            std::bind(&Misao::collect_goal, this, _1));

      if (this->get_parameter("launch_gui").as_bool()) {

         /* instantiate the publisher that the gui will use */
         goal_out = this->create_publisher<geometry_msgs::msg::PoseStamped>(
               this->get_parameter("goal_in").as_string(), 10);

         /* launch the gui in a seperate thread */
         gui_handler = std::thread(&Misao::handle_gui, this);
      }

   }


private:

   /* The commands produced by the smoother */
   rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_out;
   rclcpp::TimerBase::SharedPtr vel_callback;

   /* The raw path collected to be smoothed */
   rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_in;
   /* The map the path resides in */
   rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_in;
   /* The odometry output from the robot */
   rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_in;
   /* The current goal the robot is trying to reach */
   rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_in;

   /* a publisher that will be used to publish the
    * goal that is commanded from the gui. Only initialized
    * if the gui is enabled in the parameters.  */
   rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_out;

   /* Handle to the smoothing algorithm being used */
   SmootherBase * smoother;

   /* The last goal received */
   geometry_msgs::msg::PoseStamped goal;

   /* nice to have this from the last grid accepted */
   nav_msgs::msg::MapMetaData grid_metadata;

   /* thread that performs all operations 
    * related to the gui.                */
   std::thread gui_handler;

   void collect_path(const nav_msgs::msg::Path & msg) {

      /* check if the smoother is currently following a path */
      if ( !smoother->is_path_valid() ) {

         /* check that the received path ends at the
          * latest goal, otherwise do not take it   */
         if (fabs(msg.poses.back().pose.position.x - goal.pose.position.x) > grid_metadata.resolution ||
             fabs(msg.poses.back().pose.position.y - goal.pose.position.y) > grid_metadata.resolution)
            return;

         /* path is invalid, propose this path as a
          * new option for the smoother to follow  */
         smoother->propose_path(msg);

      }

   }

   void collect_map(const nav_msgs::msg::OccupancyGrid & msg) {

      /* load the map into the smoother */
      smoother->load_map(msg,this->get_parameter("obstacle_threshold").as_int());

      grid_metadata = msg.info;

   }

   void collect_odom(const nav_msgs::msg::Odometry & msg) {

      /* send the odometry to the smoother, which
       * will potentially increment it's position
       * on some internal path.                  */
      smoother->advance_path(msg);

   }

   void collect_goal(const geometry_msgs::msg::PoseStamped & msg) {

      /* do not use the message if it has the same
       * goal as the previous message that was recieved */
      if (msg.pose.position == goal.pose.position)
         return;

      /* invalidate the current path the smoother
       * is following                            */
      smoother->invalidate_path();

      /* update vars */
      goal = msg;

   }

   void publish_vel() {

      /* collect the current velocity that
       * the smoother desires.            */
      geometry_msgs::msg::Twist msg = smoother->get_vel();

      /* publish the collected velocity */
      vel_out->publish(msg);

   }

   /* method ran in parallel that handles
    * the running of the sfml window with
    * all the nice movement and shortcut 
    * features I am used to.             */
   void handle_gui() {

      double scale = 1.2;
      sf::Vector2f mouse;
      int    xpos = 0, ypos = 0;
      int    lastKey;
      bool leftMouseDown = false, rightMouseDown = false;

      /* SFML vars. For rendering and such */
      sf::RenderWindow window;
      sf::View view = sf::View();
      sf::Event event;

      geometry_msgs::msg::PoseStamped goal_pose;

      /* construct our window */
      window.create(sf::VideoMode(1280,720),"misao");
      window.setFramerateLimit(30);

      while (window.isOpen()) {


         mouse = window.mapPixelToCoords(sf::Mouse::getPosition(window));

         /* handle user inputs */
         while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed) {
               window.close();
            }
            else if (event.type == sf::Event::MouseButtonPressed) {
               if (sf::Mouse::isButtonPressed(sf::Mouse::Button::Left))
                  leftMouseDown = true;
               if (sf::Mouse::isButtonPressed(sf::Mouse::Button::Right))
                  rightMouseDown = true;
            }
            else if (event.type == sf::Event::MouseButtonReleased) {
               if (leftMouseDown)
                  leftMouseDown = false;
               if (rightMouseDown)
                  rightMouseDown = false;
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
                  case sf::Keyboard::G:
                     goal_pose.pose.position.x = 
                        ((grid_metadata.resolution * (mouse.x / 10.0))+grid_metadata.origin.position.x);
                     goal_pose.pose.position.y =
                        ((grid_metadata.resolution * (mouse.y / 10.0))+grid_metadata.origin.position.y);
                     goal_out->publish(goal_pose);
                     break;
              }
            }
         }

         window.clear(sf::Color(0,0,0,0));

         /* update the view */
         window.setView(view);
         view.setCenter(xpos,ypos);
         view.setSize(window.getSize().x,window.getSize().y);
         view.zoom(scale);

         /* draw everything */
         smoother->draw_environment(&window);

         /* display */
         window.display();

      }

   }

};

int main(int argc, char ** argv) {
   rclcpp::init(argc,argv);
   rclcpp::spin(std::make_shared<Misao>());
   rclcpp::shutdown();
   return 0;
}

