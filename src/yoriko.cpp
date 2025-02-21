
#include <functional>

#include <rclcpp/rclcpp.hpp>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/exceptions.h"
#include "tf2/convert.h"

#include "kurome.h"
#include "pathfinder.hpp"
#include "rrt_x_fn.hpp"
#include "mi_rrt_x_fn.hpp"
#include "a_star.hpp"
#include "mi_a_star.hpp"
#include "spatial_bin.hpp"

/* The navigation system for Kurome. Responsible for planning a route 
 * through the map provided by Pino (the SLAM system) and forwarding 
 * it to the smoother.                                               */

using std::placeholders::_1;

class Yoriko : public rclcpp::Node {
public:

   Yoriko() : Node("yoriko") {


      /* == setable parameters. It is hard as some of these
       * will only correspond to certain pathfinding algorithms == */

      /* topic to publish the computed path to */
      this->declare_parameter("path_topic","path");

      /* topic to accept goal poses on */
      this->declare_parameter("goal_topic","goal");

      /* topic to accept the position estimate on */
      this->declare_parameter("position_topic","demo/odom");

      /* topic to accept the occupancy grid on */
      this->declare_parameter("grid_topic","map");

      /* the algorithm to use for the pathfinding. Options
       * are rrt_x_fn and a_star                         */
      this->declare_parameter("algorithm","a_star");

      /* the distance we want to maintain from all surrounding
       * obstacles. used for all pathfinding algorithms.      */
      this->declare_parameter("collision_radius",0.5);

      /* rate to publish the path in seconds */
      this->declare_parameter("publish_rate",0.1);

      /* whether or not to use the gui interface */
      this->declare_parameter("launch_gui",true);

      /* The threshold (0 - 100) at which to consider
       * a position on the occupancy grid an obstacle */
      this->declare_parameter("obstacle_threshold",60);

      /* values for the rrt_x_fn algorithm only. See
       * the comments in it's file for info on what
       * these parameters mean.                     */
      this->declare_parameter("dominance_region",2.0);
      this->declare_parameter("cull_range",1.5);
      this->declare_parameter("expansion_length",1.25);
      this->declare_parameter("node_limit",30000);
      this->declare_parameter("generation_tick_speed",10);
      this->declare_parameter("bin_size",2.0);

      /* values for the a_star algorithm only. See
       * the comments in the a_star file for info
       * on these parameters.                     */
      this->declare_parameter("queue_limit",20000);
      this->declare_parameter("backtrack_count",10);

      /* Additional parameter for the motion informed
       * varients of the pathfinders. The turning radius
       * that is allowed by the robot being planned for */
      this->declare_parameter("turning_radius",2.5);

      /* values for the mi_a_star algorithm only. Find
       * more details in the associated implimentation
       * file. Note: angle_slice values are provided
       * in degrees for ease of use but are used internally
       * as radians, hence the conversion.            */
      this->declare_parameter("angle_slice",15.0);
      this->declare_parameter("position_slice",0.25);
      this->declare_parameter("forward_step",1.0);

      /* set our pathfinding algorithm given the 
       * parameters we have been given.         */
      if (this->get_parameter("algorithm").as_string() == "rrt_x_fn") {
         pathfinder = new RRTX_FN(this->get_parameter("collision_radius").as_double(),
                                  this->get_parameter("dominance_region").as_double(),
                                  this->get_parameter("cull_range").as_double(),
                                  this->get_parameter("bin_size").as_double(),
                                  this->get_parameter("expansion_length").as_double(),
                                  this->get_parameter("node_limit").as_int(),
                                  this->get_parameter("generation_tick_speed").as_int());
      }
      else if (this->get_parameter("algorithm").as_string() == "a_star") {
         pathfinder = new AStar(this->get_parameter("collision_radius").as_double(),
                                this->get_parameter("backtrack_count").as_int(),
                                this->get_parameter("queue_limit").as_int());

      }
      else if (this->get_parameter("algorithm").as_string() == "mi_rrt_x_fn") {
         pathfinder = new MI_RRTX_FN(this->get_parameter("collision_radius").as_double(),
                                     this->get_parameter("dominance_region").as_double(),
                                     this->get_parameter("cull_range").as_double(),
                                     this->get_parameter("bin_size").as_double(),
                                     this->get_parameter("expansion_length").as_double(),
                                     this->get_parameter("node_limit").as_int(),
                                     this->get_parameter("generation_tick_speed").as_int(),
                                     this->get_parameter("turning_radius").as_double());
      }
      else if (this->get_parameter("algorithm").as_string() == "mi_a_star") {
         pathfinder = new MI_AStar(this->get_parameter("collision_radius").as_double(),
                                   this->get_parameter("backtrack_count").as_int(),
                                   this->get_parameter("node_limit").as_int(),
                                   this->get_parameter("turning_radius").as_double(),
                                   (M_PI/180.0) * this->get_parameter("angle_slice").as_double(),
                                   this->get_parameter("position_slice").as_double(),
                                   this->get_parameter("forward_step").as_double());
      }
      else {
         RCLCPP_ERROR(this->get_logger(),"Invalid algorithm selected. Please choose" 
                                         "one of: [ rrt_x_fn, a_star, mi_rrt_x_fn, mi_a_star ]");
      }

      /* instantiate our publisher */
      path_out = this->create_publisher<nav_msgs::msg::Path>(
            this->get_parameter("path_topic").as_string(), 10);
      path_callback = this->create_wall_timer(
            std::chrono::milliseconds((long)(1000.0 * this->get_parameter("publish_rate").as_double())),
            std::bind(&Yoriko::publish_path, this));

      /* instantiate the subsriptions */
      goal_in = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            this->get_parameter("goal_topic").as_string(), 10,
            std::bind(&Yoriko::collect_goal, this, _1));

      position_in = this->create_subscription<nav_msgs::msg::Odometry>(
            this->get_parameter("position_topic").as_string(), 10,
            std::bind(&Yoriko::collect_position, this, _1));

      grid_in = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            this->get_parameter("grid_topic").as_string(), 10,
            std::bind(&Yoriko::collect_grid, this, _1));

      /* launch the gui if directed to */
      if (this->get_parameter("launch_gui").as_bool()) {
         /* launch the gui handler in a seperate thread */
         gui_handler = std::thread(&Yoriko::handle_gui, this);
      }

      /* instantiate our transform listener */
      tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
      tf_listener = std::make_unique<tf2_ros::TransformListener>(*tf_buffer);

   }

private:

   /* =================== publishers =================== */
   /* The path we produce with our pathfinding algorith for use
    * by a controller or for our own use in creating a series of 
    * Twist messages to send directly to something like a 
    * ros2_control controller.                                   */
   rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr       path_out;
   rclcpp::TimerBase::SharedPtr                            path_callback;

   /* =================== subscribers =================== */
   /* Subscription to the goal of the navigation system, presumably
    * published to this node by the system brain. It doesn't have
    * to be though. The system will attempt to navigate towards this
    * goal.                                                      */
   rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_in;
   /* The current besst estimate of the robot's position. Provided by
    * Pino, our resident GraphSLAM algorithm. Yoriko will use this to aid
    * in following our path, and in constructing it.             */
   rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr         position_in;
   /* Subscription to the occupancy grid, which is published
    * periodically by Pino. This system will path through this map,
    * attempting to reach the goal provided.                     */
   rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr    grid_in;

   /* values needed for transforming the odometry into the map 
    * reference frame, as that is the frame we are pathfinding in. */
   std::unique_ptr<tf2_ros::TransformListener> tf_listener;
   std::unique_ptr<tf2_ros::Buffer> tf_buffer;

   /* Pathing algorithm. This is an generic class that we will 
    * use multiple different pathfinding algorithms with.     */
   PathfinderBase * pathfinder;

   /* Last best input of transformation */
   geometry_msgs::msg::PoseStamped transformed;
   /* Last transformation collected */
   geometry_msgs::msg::TransformStamped transformation;
   
   /* Last input of goal */
   geometry_msgs::msg::PoseStamped goal;

   /* information about the grid */
   nav_msgs::msg::MapMetaData grid_metadata;

   /* thread that performs all operations 
    * related to the gui.                */
   std::thread gui_handler;

   void publish_path() {


      nav_msgs::msg::Path msg;
      
      /* get the path from our pathfinder */
      msg = pathfinder->get_path();

      /* if the pathfinder could not generate a
       * path we will compute a placeholder one */
      if (msg.poses.size() == 0) {
         /* add one node that corresponds to our
          * current position estimate.          */
         geometry_msgs::msg::PoseStamped pose;
         pose.pose = transformed.pose;
         msg.poses.push_back(pose);
      }

      /* set generic values for the path */
      msg.header.stamp = this->get_clock()->now();
      /* we are in the map frame of reference */
      msg.header.frame_id = "map";

      /* send out the message */
      path_out->publish(msg);

   }

   void collect_goal(const geometry_msgs::msg::PoseStamped & msg) {


      /* do not use the message if it has the same
       * goal as the previous message that was recieved */
      if (msg.pose.position == goal.pose.position)
         return;

      /* notify the pathfinder */
      pathfinder->set_goal(msg.pose);

      /* update vars */
      goal = msg;


   }

   void collect_position(const nav_msgs::msg::Odometry & msg) {
      static int fail_count = 0;

      /* transform to map frame */
      geometry_msgs::msg::PoseStamped pose_out;
      geometry_msgs::msg::PoseStamped pose_in;
      pose_in.pose = msg.pose.pose;
      pose_in.header = msg.header;

      try {

         /* get the last avaliable transform */
         transformation = tf_buffer->lookupTransform("map","odom",
                                                     tf2::TimePointZero,
                                                     tf2::Duration(std::chrono::milliseconds(100)));

      } 
      catch (const tf2::TransformException & ex) {
         RCLCPP_WARN(this->get_logger(),"transformation of odometry failed for the %dth time",fail_count++);
      }

      /* update our current odometry */
      tf2::doTransform<geometry_msgs::msg::PoseStamped>(pose_in,pose_out,transformation);
      transformed = pose_out;

      /* set the origin of the path */
      pathfinder->set_origin(pose_out.pose);
   }

   void collect_grid(const nav_msgs::msg::OccupancyGrid & msg) {


      /* pass the message to the pathfinder */
      pathfinder->load_map(msg, this->get_parameter("obstacle_threshold").as_int());
      
      /* grab the metadata for later use */
      grid_metadata = msg.info;

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

      geometry_msgs::msg::Pose goal_pose;

      /* construct our window */
      window.create(sf::VideoMode(1280,720),"yoriko");
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
                     goal_pose.position.x = ((grid_metadata.resolution * mouse.x / 10)+grid_metadata.origin.position.x);
                     goal_pose.position.y = ((grid_metadata.resolution * mouse.y / 10)+grid_metadata.origin.position.y);
                     pathfinder->set_goal(goal_pose);
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
         pathfinder->draw_environment(&window);

         /* display */
         window.display();

      }

   }

};

int main(int argc, char ** argv) {
   rclcpp::init(argc,argv);
   rclcpp::spin(std::make_shared<Yoriko>());
   rclcpp::shutdown();
   return 0;
}
