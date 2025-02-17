
/* Path followin algorithm. Given a nav_msgs/Path value and
 * consistant odometry input, follow the path, taking into
 * account each point along it, and attempting to reduce
 * angular speed.                                       */

#include <functional>
#include <chrono>
#include <cmath>
#include <cstdio>

#include <rclcpp/rclcpp.hpp>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "kurome.h"

using std::placeholders::_1;

class TheIronRod : public rclcpp::Node {
public:

   TheIronRod() : Node("the_iron_rod") {

      /* === declare our setable parameters === */
      /* The interval in seconds we send out our speed message.
       * Also corresponds to the time interval at which we check
       * and recompute our speed command.                     */
      this->declare_parameter("publish_rate",0.1);
      /* The topic to send our speed message to, the topic
       * that the drive controller is listening on.          */
      this->declare_parameter("speed_topic","demo/cmd_vel");
      /* The topic we recieve our odometry data from. 
       * This is needed to accurately compute the node
       * on the path that we are trying to get to.    */
      this->declare_parameter("odometry_topic","demo/odom");
      /* The topic we expect to recieve our path message
       * on. This is where we get our input form.       */
      this->declare_parameter("path_topic","path");
      /* This is the fixed value for our linear speed.
       * Value is in meters per second.                 */
      this->declare_parameter("speed",0.1);
      /* Determine at what time we stop sending out speed
       * when we have not seen path values in a while. This
       * is to prevent running into walls or something
       * when another part of the system fails. Defined in
       * seconds.                                           */
      this->declare_parameter("path_update_require",10000.0);
      /* distance to a node where we consider it to be 
       * reached and proceed to the next node in our
       * path.                                        */
      this->declare_parameter("node_reached_radius",0.1);

      /* instantiate our subscribers */
      odom_in = this->create_subscription<nav_msgs::msg::Odometry>(
            this->get_parameter("odometry_topic").as_string(), 10,
            std::bind(&TheIronRod::collect_odometry, this, _1));
      path_in = this->create_subscription<nav_msgs::msg::Path>(
            this->get_parameter("path_topic").as_string(), 10,
            std::bind(&TheIronRod::collect_path, this, _1));

      /* instantiate our publishers */
      twist_out = this->create_publisher<geometry_msgs::msg::Twist>(
            this->get_parameter("speed_topic").as_string(), 10);

      /* construct our twist time trigger */
      twist_callback = this->create_wall_timer(
            std::chrono::milliseconds((long)(1000 * this->get_parameter("publish_rate").as_double())),
            std::bind(&TheIronRod::publish_twist, this));

      /* set current node to 1 to start. This has us assume
       * that we have reached our final position.          */
      current_node = 1;

      /* get our constant speed */
      speed = this->get_parameter("speed").as_double();

      /* get our update requirement */
      update_requirement = this->get_parameter("path_update_require").as_double();

      /* set the node increment count based on the selected algorithm */
      increment_count = 1;

      /* set our node reached radius */
      reached_radius = this->get_parameter("node_reached_radius").as_double();
   }

private:

   /* The position of the robot in the 
    * 2d dof case.                    */
   struct pose_2d {
      double x;
      double y;
      double theta;
   };

   /* The nessesary speed variables
    * in the 2d case.                 */
   struct vel_2d {
      double v; /* linear  */
      double w; /* angular */
   };

   /* =================== publishers =================== */
   /* The speed we are sending to our differential drive
    * controller. This will be output at a consistant rate. */
   rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_out;

   /* =================== subscribers =================== */
   /* The subscription to the odometry we will use to
    * estimate our position on the path.             */
   rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_in;
   /* The subscription we will get our path to follow
    * from.                                           */
   rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_in;

   /* timer that triggers the speed publisher */
   rclcpp::TimerBase::SharedPtr twist_callback;

   /* The pose we are currently at in our path. */
   int current_node;
   /* our collected constant speed */
   double speed;
   /* our collected requirement on the
    * path update rate.               */
   double update_requirement;
   /* how many nodes on the path do we increment
    * at a time? */
   int increment_count;
   /* radius to a node to consider it as reached */
   double reached_radius;

   /* the path currently under consideration */
   nav_msgs::msg::Path path;
   std::vector<pose_2d> path_2d;

   /* the most up to date odometry of our robot */
   nav_msgs::msg::Odometry odom;
   pose_2d pose = pose_2d{0,0,0};
   vel_2d  vel;

   /* the imaginary final node when we are going 
    * to the last node on our path.             */
   pose_2d p_final = { NAN, NAN, NAN };

   /* Process the collected odometry. This involves checking
    * if we have passed the current pose on our path and
    * need to step that value.                              */
   void collect_odometry(const nav_msgs::msg::Odometry & msg) {
      /* get the new pose */
      pose_2d new_pose = pose3d_msg_to_pose2d(msg.pose.pose);

      /* increment our current pose accordingly */
      if (path_2d.size())
         increment_if_past_node(pose);

      /* set our new pose as the old pose */
      pose = new_pose;

      /* get the new speed */
      vel = vel_2d{msg.twist.twist.linear.x,msg.twist.twist.angular.z};

      /* set odom as the message */
      odom = msg;
   }

   /* Process the path provided. This involves setting
    * the new path and moving the current pose back
    * to zero.                                        */
   void collect_path(const nav_msgs::msg::Path & msg) {

      /* set the path as path */
      path = msg;

      /* clear out our old values and reset 
       * position on the path.             */
      path_2d.clear();
      current_node = 0;
      p_final = { NAN, NAN, NAN };

      /* convert all path values to their 2d counterparts */
      for (const geometry_msgs::msg::PoseStamped & pose : path.poses) {
         path_2d.push_back(pose3d_msg_to_pose2d(pose.pose));
      }

      /* we assume that the path is starting at the
       * current position of the robot.            */
   }

   /* Compute the speed needed to interpolate between
    * two points and send out that message.            */
   void publish_twist_two_point() {

      printf("pos: %d\n",current_node);

      geometry_msgs::msg::Twist out_msg;

      /* check if too much time has elapsed since the
       * last path update or if the end of the path
       * has been reached. If so, publish zero for 
       * speed.                                   */
      if (time_dist(path.header.stamp,this->get_clock()->now()) > update_requirement ||
         (int)(current_node - path_2d.size()) > -increment_count) {
         out_msg.linear.x = 0;
         out_msg.angular.z = 0;
      }
      else {
         /* compute the radius we are turning on */
         double v[2] =     { -sin(pose.theta), cos(pose.theta) };
         double v_not[2] = {  cos(pose.theta), sin(pose.theta) };
         double dist_2 = absolute_distance_2(pose,path_2d[current_node]);
         double dot_v_not_diff = ((path_2d[current_node].x - pose.x) * v_not[0]) +
                                 ((path_2d[current_node].y - pose.y) * v_not[1]);
         double r = dist_2 / (2.0 * dot_v_not_diff);


         /* compute the angular speed given this radius */
         double w = speed / r;
         w = (std::isnan(w) || std::isinf(w) || w > 1000 ? 0 : w);

         /* set appropriate values in the message */
         out_msg.linear.x = speed;
         out_msg.angular.z = w;

         printf("v*speed:     [ %f %f ]\n",v[0]*speed,v[1]*speed);
         printf("v_not*speed: [ %f %f ]\n",v_not[0]*speed,v_not[1]*speed);
         printf("dist_2: %f\tdist: %f\n",dist_2,sqrt(dist_2));
         printf("dot_v_not_diff: %f\n",dot_v_not_diff);
         printf("r: %f\n",r);
         printf("w: %f\n",w);
      }

      twist_out->publish(out_msg);
   }

   /* Compute the speed needed to interpolate between
    * three points and send out that message.          */
   void publish_twist() {

      geometry_msgs::msg::Twist out_msg;

      /* check if too much time has elapsed since the
       * last path update or if the end of the path
       * has been reached. If so, publish zero for 
       * speed.                                   */
      if (time_dist(path.header.stamp,this->get_clock()->now()) > update_requirement ||
         (int)(current_node - path_2d.size()) > -increment_count) {
         out_msg.linear.x = 0;
         out_msg.angular.z = 0;
      }
      /* If there is only one node left in our listing, we
       * simply need to steer towards it. Generate a node behind
       * it and use that to reach it.                           */
      else {

         pose_2d p2 = path_2d[current_node];
         pose_2d p3;

         /* There is only one node left in our path. Project an imaginary
          * node behind it using the current heading of the robot.       */
         if ((int)(current_node - path_2d.size()) == -increment_count){

            /* compute p_final if it is not already knwown */
            if (std::isnan(p_final.x)) {
               p_final.x = cos(pose.theta) + p2.x;
               p_final.y = sin(pose.theta) + p2.y;
            }

            p3 = p_final;

         }
         /* We have two or more points left in our algorithm. 
          * Compute the linear and angular velocities for our
          * robot through these points.                      */
         else {
            p3 = path_2d[current_node + 1];
         }

         printf("p2_raw: [ %f %f ]\n",p2.x,p2.y);
         printf("p3_raw: [ %f %f ]\n",p3.x,p3.y);

         /* move path nodes into the local
          * reference frame               */
         p2 = to_local_reference(pose,p2);
         p3 = to_local_reference(pose,p3);

         printf("p2    : [ %f %f ]\n",p2.x,p2.y);
         printf("p3    : [ %f %f ]\n",p3.x,p3.y);

         /* Find vector representing our current pose and 
          * goal pose based on rotation.                 */
         pose_2d H1 = {-sin(pose.theta), cos(pose.theta) , 0};
         pose_2d H2 = { p3.x - p2.x,  p3.y - p2.y   , 0};

         printf("H1:   : [ %f %f ]\n",H1.x,H1.y);
         printf("H2:   : [ %f %f ]\n",H2.x,H2.y);

         /* Compute the distance between our current pose
          * and the goal position.                       */
         double dist_2 = absolute_distance_2(pose,path_2d[current_node]);
         double L = sqrt(dist_2);

         printf("L     : %f\n",L);

         /* Compute the rotation need to get from
          * our current position vector to the 
          * goal vector.                          */
         double dot = (H2.x * H1.x) + (H2.y * H1.y);
         double mag = sqrt((H2.x * H2.x) + (H2.y * H2.y));
         double phi = dot / mag;
         double theta = acos(phi);

         printf("dot   : %f\n",dot);
         printf("mag   : %f\n",mag);
         printf("phi   : %f\n",phi);
         printf("theta : %f\n",theta);

         /* determine the radius of the circle needed
          * to meet that rotation.                    */
         double rad = L / (2.0 * cos( theta / 2.0));

         printf("rad   : %f\n",rad);

         double arc_length = theta * rad;

         printf("arc_l : %f\n",arc_length);

         double time = arc_length / speed;

         /* compute the speed given that value */
         double w = arc_length / speed;
         printf("w_raw : %f\n",w);
         w = (std::isnan(w) || std::isinf(w) || w > 1000 ? 0 : w);
         printf("w     : %f\n",w);

         /* fill values in message */ 
         out_msg.linear.x  = speed;
         out_msg.angular.z = w;

      }

      twist_out->publish(out_msg);
   }

   /* Produce a z rotation in radians given a
    * quaternion ros2 message. Note that this
    * assumes the 3-2-1 rotation application order */
   double quaternion_to_z_rotation(const geometry_msgs::msg::Quaternion & q) {
      double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
      double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
      return atan2(siny_cosp,cosy_cosp);
   }

   /* Produce a pose_2d value given a pose
    * ros2 message. Note that this assumes the
    * 3-2-1 rotation application order.       */
   pose_2d pose3d_msg_to_pose2d(const geometry_msgs::msg::Pose & p) {
      return {p.position.x,p.position.y,quaternion_to_z_rotation(p.orientation)};
   }

   /* Check if the robot has moved past it's given next
    * path node, and if so increment our position on the
    * path.                                              */
   void increment_if_past_node(pose_2d & current) {
      printf("curr: [ %f %f ]\n",current.x,current.y);
      printf("node: [ %f %f ]\n",path_2d[current_node].x,path_2d[current_node].y);
      /* If I am within reached_radius, I assume that I have
       * reached the given node, and proceed to the next one */
      if (sqrt(absolute_distance_2(current,path_2d[current_node])) < reached_radius)
         current_node += increment_count;
   }

   /* Compute the distance squared between two pose_2d structures. */
   double absolute_distance_2(pose_2d & p1, pose_2d & p2) {
      return (p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y);
   }

   /* Move the given pose_2d to the provided local reference frame,
    * creating a new pose.                                         */
   pose_2d to_local_reference(const pose_2d & frame, const pose_2d & target) {
      double move_x = target.x - frame.x;
      double move_y = target.y - frame.y;
      return pose_2d{ move_x*cos(frame.theta) - move_y*sin(frame.theta),
                      move_x*sin(frame.theta) + move_y*cos(frame.theta),
                      target.theta - frame.theta }; /* don't bother normalizing */
   }


};

int main(int argc, char ** argv) {
   rclcpp::init(argc,argv);
   rclcpp::spin(std::make_shared<TheIronRod>());
   rclcpp::shutdown();
   return 0;
}
