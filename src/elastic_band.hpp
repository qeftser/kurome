
#ifndef __ELASTIC_BAND

#define __ELASTIC_BAND
#include "kurome.h"
#include "smoother.hpp"
#include <cmath>
#include <cstdlib>
#include <cassert>
#include <thread>
#include <chrono>

class ElasticBand : public SmootherBase {
private:

   /* a node (bubble) on our elastic band */
   struct node {
      /* position of the node */
      point pos;
      /* closest obstacle */
      point obs;
      /* radius of the safety bubble */
      double bubble_rad;
      node * next;
      node * prev;
      bool new_node = false;
   };

   /* needed for keeping memory seperate */
   std::mutex mut;

   /* how many nodes of the path should 
    * be included in the optimization step
    * at one time?                        */
   int band_length;

   /* what is the range that obstacles have 
    * influence on the elastic band?       */
   double influence_range;

   /* maximum radius a bubble can have on the
    * elastic band. This influences the maximum
    * distance between two nodes on the band.  */
   double max_bubble;

   /* The gain on the contractive force in the
    * elastic band update step. Higher values of 
    * this variable will result in the band wanting
    * to be as straight as possible.               */
   double contraction_gain;

   /* The gain on the repulsive force of the band. The
    * result will be the band pulling away more from obstacles */
   double repulsion_gain;

   /* The inverse force on the band. Higher values of this variable
    * will result in the band moving slower and potentially ocillating
    * less. It is important to keep this value below 1.0, as otherwise
    * the band will start moving the oppisite direction from the way
    * it wants to go.                                                */
   double damping_gain; 

   /* the number of times to run the elastic band steps when 
    * recomputing or updating the path                      */
   int cycle_count;

   /* speed desired for the robot when following the path. In
    * meters per second.                                     */
   double desired_speed;

   /* current position of the robot on the path. */
   int path_position = 0;
   geometry_msgs::msg::Twist current_vel;

   /* distance from a node the robot needs to be
    * in order for it's position on the path to advance */
   double advance_distance;

   /* the front of our path */
   node * path = NULL;

   /* use the given map and the bresenham collision algorithm 
    * to determine if this edge is colliding with the map. */
   inline bool is_collision(const point & p1, const point & p2) {

      int x0 = p1.x;
      int x1 = p2.x;
      int y0 = p1.y;
      int y1 = p2.y;

      /* compute our intermediate values */
      int dx = abs(x1 - x0);
      int sx = x0 < x1 ? 1 : -1;
      int dy = -abs(y1 - y0);
      int sy = y0 < y1 ? 1 : -1;
      int error = dx + dy;

      int pos;

      while (true) {

         /* check if the current section on the line
          * has an obstacle in it.                  */

         pos = x0 + (y0 * grid_metadata.width);

         if (x0 <= 0 || x0 > (int)grid_metadata.width ||
             pos < 0 || pos >= grid_length || grid[pos]) {
            return true;
         }

         if (x0 == x1 && y0 == y1)
            break;

         int e2 = 2 * error;

         if (e2 >= dy) {
            error = error + dy;
            x0 = x0 + sx;
         }
         if (e2 <= dx) {
            error = error + dx;
            y0 = y0 + sy;
         }

      }

      return false;
   }

   inline int block_pos(int x_pos, int y_pos) {
      return x_pos + (y_pos * grid_metadata.width);
   }

   /* return the closest obstacle to a given point */
   inline point closest_obstacle(const point & p) {
      /* the range around us to search in */
      int range_max = ceil(max_bubble / grid_metadata.resolution);
      int range = 0;
      point closest = point{INFINITY,INFINITY};
      int seen = 0;

      while (range <= range_max && !seen) {

         int y_low  = p.y - range;
         int y_high = p.y + range;
         int x_low  = p.x - range;
         int x_high = p.x + range;

         /* check the x edges */
         for (int x_pos = x_low; x_pos <= x_high; ++x_pos) {

            /* bottom */
            if (y_low >= 0 && x_pos >= 0 && x_pos < (int)grid_metadata.width && 
                grid[block_pos(x_pos,y_low)]) {

               point grid_point = point{ x_pos + 0.5, y_low + 0.5 };
               seen = 1;

               if (point_dist2(grid_point,p) < point_dist2(closest,p)) {
                  closest = grid_point;
               }

            }
            /* top */
            if (y_high < (int)grid_metadata.height && x_pos >= 0 && x_pos < (int)grid_metadata.width && 
                grid[block_pos(x_pos,y_high)]) {

               point grid_point = point{ x_pos + 0.5, y_high + 0.5 };
               seen = 1;

               if (point_dist2(grid_point,p) < point_dist2(closest,p)) {
                  closest = grid_point;
               }
            }
         }

         /* check the y edges. We don't need
          * to check the corners because we 
          * already did with the x edges    */
         for (int y_pos = y_low + 1; y_pos <= y_high - 1; ++y_pos) {

            /* left */
            if (x_low >= 0 && y_pos >= 0 && y_pos < (int)grid_metadata.height && 
                grid[block_pos(x_low,y_pos)]) {

               point grid_point = point{ x_low + 0.5, y_pos + 0.5 };
               seen = 1;

               if (point_dist2(grid_point,p) < point_dist2(closest,p)) {
                  closest = grid_point;
               }

            }
            /* right */
            if (x_high < (int)grid_metadata.width && y_pos >= 0 && y_pos < (int)grid_metadata.height && 
                grid[block_pos(x_high,y_pos)]) {

               point grid_point = point{ x_high + 0.5, y_pos + 0.5 };
               seen = 1;

               if (point_dist2(grid_point,p) < point_dist2(closest,p)) {
                  closest = grid_point;
               }
            }
         }

         ++range;
      }
      return closest;
   }

   double compute_bubble(node & n) {
      point closest = closest_obstacle(n.pos);
      double dist = sqrt(point_dist2(closest,n.pos));

      n.obs = closest;

      if (dist > max_bubble)
         return max_bubble;
      return dist;
   }

   /* compute the motion or optimization update
    * for the elastic band at it's current configuration */
   void movement_step() {

      node * curr = path->next;
      int step_remainder = band_length;

      if (curr == NULL)
         return;

      while (curr->next && step_remainder--) {

         /* compute the contraction force 
            this is the distance between where the point is and the midpoint of the line
          * between the points in front and behind it. See that this is an attempt to move
          * the point to a 'straight' position.                                           */
         point contraction_force = 
            { -contraction_gain * ( curr->pos.x - ((curr->prev->pos.x + curr->next->pos.x) / 2.0) ),
              -contraction_gain * ( curr->pos.y - ((curr->prev->pos.y + curr->next->pos.y) / 2.0) ) };

         /* compute the repulsive force 
          * As far as I can tell, this is the movement taken between
          * this point and the closest obstacle, divided by a large value
          * related to the distance. This pushes the node away from the obstacle
          * by a decreasing amount as the node gets farther away.               */
         point repulsion_force = { 0.0, 0.0 };
         if (influence_range >= curr->bubble_rad) {
            double repulsive_multiplier = repulsion_gain * (influence_range - curr->bubble_rad) * 
               (1.0 / curr->bubble_rad);
            repulsion_force = point{ (curr->pos.x - curr->obs.x) * repulsive_multiplier,
                                     (curr->pos.y - curr->obs.y) * repulsive_multiplier };
         }

         /* compute the total force and apply the damping gain, which results
          * in just reducing the force by a fractional amount */
         point force = { (1.0 - damping_gain) * ( contraction_force.x + repulsion_force.x ),
                         (1.0 - damping_gain) * ( contraction_force.y + repulsion_force.y ) };

         /* check if force is greater than the size of the bubble and reduce it if
          * so. This is to avoid massive fluxuations in the path.                */
         if (sqrt(force.x*force.x + force.y*force.y) > curr->bubble_rad) {
            /* compute the unit vector going in the same direction and
             * scale it to a length of bubble_rad for the new force.  */
            double theta = atan2(force.y,force.x);

            force = point{ cos(theta)*curr->bubble_rad, sin(theta)*curr->bubble_rad };
         }

         curr->pos = point{ curr->pos.x + force.x, curr->pos.y + force.y };

         curr->bubble_rad = compute_bubble(*curr);

         curr = curr->next;
      }

   }

   /* add nodes to the elastic band if it has started to stretch too far apart
    * to the point that it is difficult to maintain the structure of the band */
   void addition_step() {

      node * curr = path;
      int step_remainder = band_length;

      while (curr->next && step_remainder--) {

         /* if our nodes are too far apart, but there is still 
          * collision free area between them, add a node.     */
         if (sqrt(point_dist2(curr->pos,curr->next->pos)) > 
             (curr->bubble_rad + curr->next->bubble_rad) / 2.0) {

            node * new_node = new node{ {(curr->pos.x + curr->next->pos.x) / 2.0,
                                         (curr->pos.y + curr->next->pos.y) / 2.0 },
                                        {0.0,0.0}, 0.0, curr->next, curr };
            new_node->bubble_rad = compute_bubble(*new_node);

            new_node->new_node = true;

            curr->next->prev = new_node;
            curr->next = new_node;
         }

         curr = curr->next;
      }

   }

   /* remove nodes from the elastic band that have basically become redundant
    * in describing the system. We have to sort of bounce back and forth 
    * between removal and addition to ensure that the system is stable.      */
   void removal_step() {

      mut.lock();

      node * curr = path;
      int step_remainder = band_length;

      while (curr->next && step_remainder--) {
         
         /* if the next node is inside of our bubble radius and our bubble intersects with
          * the node two up, the current node can be seen as redundant and removed.       */
         if (curr->next->next && sqrt(point_dist2(curr->pos,curr->next->pos)) < curr->bubble_rad &&
             sqrt(point_dist2(curr->next->next->pos,curr->pos)) < 
            (curr->bubble_rad + curr->next->next->bubble_rad) / 2.0) {

            curr->next->next->prev = curr;
            node * to_delete = curr->next;
            curr->next = to_delete->next;

            delete to_delete;
         }
         /* do not advance if we can remove a node, this ensures that
          * we cull all intermediate nodes we can before exiting.    */
         else
            curr = curr->next;
      }

      mut.unlock();
   }

   /* find the velocity update values based on the current
    * position of the robot relative to the next node on the path */
   velocity_2d compute_velocity_output(pose_2d curr_pose, node * curr_node) {

      point p1 = { curr_pose.pos.x * grid_metadata.resolution,
                   curr_pose.pos.y * grid_metadata.resolution };
      point p2 = { curr_node->pos.x * grid_metadata.resolution,
                   curr_node->pos.y * grid_metadata.resolution };
      point p3;

      if ( !curr_node->next ) {
         p3.x = (cos(curr_pose.theta) + curr_node->pos.x) * grid_metadata.resolution;
         p3.y = (sin(curr_pose.theta) + curr_node->pos.y) * grid_metadata.resolution;
      }
      else
         p3 = { curr_node->next->pos.x * grid_metadata.resolution,
                curr_node->next->pos.y * grid_metadata.resolution };

      printf("p1: %f %f %f\n",p1.x,p1.y,curr_pose.theta);
      printf("p2: %f %f\n",p2.x,p2.y);
      printf("p3: %f %f\n",p3.x,p3.y);

      point curr_vector = { -sin(curr_pose.theta), cos(curr_pose.theta) };
      point goal_vector = { p3.x - p2.x, p3.y - p2.y };

      printf("H1: %f %f\n",curr_vector.x,curr_vector.y);
      printf("H2: %f %f\n",goal_vector.x,goal_vector.y);

      double between = sqrt(point_dist2(p1,p2));

      printf("L: %f\n",between);
      
      double theta = acos(((curr_vector.x * goal_vector.x) + (curr_vector.y * goal_vector.y)) /
                          sqrt(goal_vector.x * goal_vector.x + goal_vector.y * goal_vector.y));

      printf("theta: %f\n",theta);

      double rad = between / (2.0 * cos( theta / 2.0));

      printf("rad: %f\n",rad);

      double arc_length = theta * rad;

      double time = arc_length / desired_speed;

      printf("arc_length: %f\n",arc_length);
      printf("time: %f\n",time);

      double w = theta / time;
      w = (std::isnan(w) || std::isinf(w) || w > 1000 ? 0 : w);

      printf("w: %f\n",w);

      return velocity_2d{desired_speed,w};

   }


public:

   ElasticBand(int band_length, double influence_range, double max_bubble,
         double contraction_gain, double repulsion_gain, double damping_gain,
         int cycle_count, double desired_speed, double advance_distance) 
      : band_length(band_length), influence_range(influence_range),
        max_bubble(max_bubble), contraction_gain(contraction_gain), 
        repulsion_gain(repulsion_gain), damping_gain(damping_gain),
        cycle_count(cycle_count), desired_speed(desired_speed), 
        advance_distance(advance_distance) {

      assert(max_bubble >= influence_range &&
            "Elastic Band algorithm requires that the max bubble length be greater than the influence range.");
      assert(damping_gain < 1.0 && "Damping Gain must be less than 1.0");

      assert(damping_gain >= 0.0 && "Damping Gain must be greater than 0.0");
      assert(contraction_gain >= 0.0 && "Contraction Gain must be greater than 0.0");
      assert(repulsion_gain >= 0.0 && "Repulsion Gain must be greater than 0.0");
   }


   bool propose_path(const nav_msgs::msg::Path & msg) override {

      if (valid_path)
         return false; /* reject path */

      mut.lock();

      /* get rid of the old path */
      node * to_delete;
      while (path) {
         to_delete = path;
         path = path->next;
         delete to_delete;
      }

      mut.unlock();

      /* please be gentle... */
      point converted_coords = 
         { (msg.poses[0].pose.position.x - grid_metadata.origin.position.x) / grid_metadata.resolution,
           (msg.poses[0].pose.position.y - grid_metadata.origin.position.y) / grid_metadata.resolution };
      node * prev_node = path = new node{converted_coords,{INFINITY,INFINITY}, 0.0, NULL, NULL };
      prev_node->bubble_rad = compute_bubble(*prev_node);
                                         
      /* construct the remainder of the path */
      for (size_t i = 1; i < msg.poses.size(); ++i) {

         /* convert the poses to our coordinate system */
         converted_coords = 
            { (msg.poses[i].pose.position.x - grid_metadata.origin.position.x) / grid_metadata.resolution,
              (msg.poses[i].pose.position.y - grid_metadata.origin.position.y) / grid_metadata.resolution };

         node * next_node = new node{converted_coords,{INFINITY,INFINITY},0.0,NULL,prev_node};
         next_node->bubble_rad = compute_bubble(*next_node);
         prev_node->next = next_node;
         prev_node = next_node;

      }

      path_position = 0;
      valid_path = true;

      for (int i = 0; i < cycle_count; ++i) {

         movement_step();
         addition_step();
         removal_step();

      }

      valid_path = true;

      /* accept the path */
      return true;
   }

   void advance_path(const geometry_msgs::msg::PoseStamped & msg) override {

      if ( !valid_path || !path || !path->next)
         return;

      //printf("current_node: %d\n",path_position);

      pose_2d curr_pose = ros2_pose_to_pose_2d(msg.pose);
      curr_pose.pos.x = (curr_pose.pos.x - grid_metadata.origin.position.x) / grid_metadata.resolution;
      curr_pose.pos.y = (curr_pose.pos.y - grid_metadata.origin.position.y) / grid_metadata.resolution;
      //printf("curr_pose: %f %f %f\n",curr_pose.pos.x,curr_pose.pos.y,curr_pose.theta);
      //printf("path_pose: %f %f\n",path->pos.x,path->pos.y);

      /* if we have not started on the path yet, turn to face it */
      if ( path_position == 0) {

         point in_frame = { path->next->pos.x*cos(curr_pose.theta) - path->next->pos.y*sin(curr_pose.theta),
                            path->next->pos.y*sin(curr_pose.theta) + path->next->pos.y*cos(curr_pose.theta) };

         double theta = atan2(in_frame.y - path->next->pos.y, in_frame.x - path->next->pos.x);

         //printf("theta %f\n",theta);

         if (fabs(theta) > (M_PI/2.0)) {
            current_vel.linear.x = 0.0;
            current_vel.angular.z = desired_speed * (theta < 0 ? 1.0 : -1.0);
            return;
         }
      }

      //printf("dist: %f %f\n",
      //      sqrt(point_dist2(curr_pose.pos,path->pos)),advance_distance/grid_metadata.resolution);

      /* check if we are within the distance to a node that we can advance on
       * the path. If so, compute the new linear and angular velocity needed to
       * hit the next node, then wait until it is hit to continue.            */
      if (sqrt(point_dist2(curr_pose.pos,path->pos)) < advance_distance / grid_metadata.resolution) {

         mut.lock();

         node * to_delete = path;
         path = path->next;
         path_position += 1;
         delete to_delete;

         mut.unlock();

         if (!path) {
            invalidate_path();
            return; /* end of the road */
         }

      }

      velocity_2d vel_update = compute_velocity_output(curr_pose,path);

      current_vel.linear.x = vel_update.linear;
      current_vel.angular.z = vel_update.angular;

   }

   geometry_msgs::msg::Twist get_vel() override {

      if (valid_path)
         return current_vel;

      return geometry_msgs::msg::Twist();
   }

   void simulate_path(pose_2d curr_pose, geometry_msgs::msg::PoseArray & ret) override {

      /* convert from global coords to map reference frame */
      curr_pose = {{ (curr_pose.pos.x - grid_metadata.origin.position.x) / grid_metadata.resolution,
                     (curr_pose.pos.y - grid_metadata.origin.position.y) / grid_metadata.resolution},
                     curr_pose.theta                                                                };

      int sim_path_position = 0;
      
      if ( !valid_path || !path || !path->next)
         return;

      mut.lock();

      node * curr = path;
      velocity_2d update;
      double initial_dist = DBL_MAX;

      point in_frame = { curr->next->pos.x*cos(curr_pose.theta) - curr->next->pos.y*sin(curr_pose.theta),
                         curr->next->pos.y*sin(curr_pose.theta) + curr->next->pos.y*cos(curr_pose.theta) };

      double theta = atan2(in_frame.y - curr->next->pos.y, in_frame.x - curr->next->pos.x);

      /*
      curr_pose.theta = curr_pose.theta + theta;
      curr_pose.theta = atan2(sin(curr_pose.theta),cos(curr_pose.theta));
      */

      while (true) {

         printf("sim_path_position: %d\n",sim_path_position);

         if (sqrt(point_dist2(curr_pose.pos,curr->pos)) < advance_distance / grid_metadata.resolution) {

            curr = curr->next;
            sim_path_position += 1;
            initial_dist = sqrt(point_dist2(curr_pose.pos,curr->pos));

            if (!curr)
               break;
         }

         update = compute_velocity_output(curr_pose,curr);

update_step:

         printf("vel update: %f %f\n",update.linear,update.angular);

         printf("before: %f %f %f\n",curr_pose.pos.x,curr_pose.pos.y,curr_pose.theta);
         curr_pose = estimate_movement(curr_pose,update,0.5);
         printf("after: %f %f %f\n",curr_pose.pos.x,curr_pose.pos.y,curr_pose.theta);

         pose_2d transformed = {{(curr_pose.pos.x * grid_metadata.resolution) + grid_metadata.origin.position.x,
                                 (curr_pose.pos.y * grid_metadata.resolution) + grid_metadata.origin.position.y},
                                 curr_pose.theta };
         ret.poses.push_back(pose_2d_to_ros2_pose(transformed));

         /* If we have moved massively away from our initial distance estimate,
          * abort the simulation, as something has gone very wrong.           */
         if (sqrt(point_dist2(curr_pose.pos,curr->pos)) > initial_dist * 2.0 || !curr)
            break;

      }

      mut.unlock();

   }

   void load_map(const nav_msgs::msg::OccupancyGrid & map, int occupant_cutoff) override {
      SmootherBase::load_map(map,occupant_cutoff);

      /* ensure that the path is still valid, and if so perform some 
       * smoothing on it.                                           */
      if (is_path_valid() && path) {

         node * curr = path;

         while (curr->next) {

            if (is_collision(curr->pos,curr->next->pos)) {
               invalidate_path();
               return;
            }

            curr = curr->next;
         }

         for (int i = 0; i < cycle_count; ++i) {

            movement_step();
            addition_step();
            removal_step();

         }
      }
   }

   void draw_environment(sf::RenderWindow * window) override {
      SmootherBase::draw_environment(window);

      sf::Vertex point[2];
      sf::CircleShape circle;
      circle.setOutlineColor(sf::Color::White);
      circle.setOutlineThickness(2);
      circle.setFillColor(sf::Color(0,0,0,0));

      mut.lock();

      if (!valid_path || !path) {
         mut.unlock();
         return;
      }

      node * curr = path;
      while (curr->next) {

         /* draw the bubble around the node */
         circle.setRadius(curr->bubble_rad * 5);
         circle.setPosition((curr->pos.x*10) - circle.getRadius(),
                            (curr->pos.y*10) - circle.getRadius());

         if (curr->new_node) {
            circle.setOutlineColor(sf::Color::Green);
            curr->new_node = false;
         }
         else
            circle.setOutlineColor(sf::Color::White);

         window->draw(circle);

         /* draw the edge between this and the next point */
         point[0].position = sf::Vector2f(curr->pos.x * 10, curr->pos.y * 10);
         point[1].position = sf::Vector2f(curr->next->pos.x * 10, curr->next->pos.y * 10);
         window->draw(point,2,sf::Lines);

         curr = curr->next;
      }

      mut.unlock();


   }

   visualization_msgs::msg::MarkerArray construct_visualization() override {

      visualization_msgs::msg::MarkerArray ret;

      if ( !valid_path || !path )
         return ret;

      node * curr = path;

      while (curr) {

         visualization_msgs::msg::Marker next;
         next.action = 0;
         next.type = visualization_msgs::msg::Marker::SPHERE;
         next.lifetime.sec = 1;
         next.color.r = 1.0;
         next.color.g = 0.0;
         next.color.b = 1.0;
         next.color.a = 0.25;
         next.ns = "elastic_band";
         next.scale.x = advance_distance;
         next.scale.y = advance_distance;
         next.scale.z = advance_distance;

         next.pose.position.x = (curr->pos.x * grid_metadata.resolution) + grid_metadata.origin.position.x;
         next.pose.position.y = (curr->pos.y * grid_metadata.resolution) + grid_metadata.origin.position.y;

         curr = curr->next;

         ret.markers.push_back(next);
      }

      return ret;
   }

};

#endif
