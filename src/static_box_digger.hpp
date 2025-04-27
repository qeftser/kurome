
#ifndef __STATIC_BOX_DIGGER

#define __STATIC_BOX_DIGGER

#include "kurome.h"
#include "commander.hpp"
#include <queue>
#include <cfloat>
#include <cassert>

class StaticBoxDigger : public CommanderBase {
private:

   enum state {
      EXPLORING,
      DIGGING,
      DUMPING,
      FINISHED
   };

   struct dig_area {
      point lower;
      point upper;
      bool active = true;
   };

   enum state current_state = FINISHED;
   std::deque<point>  goal_queue;
   std::vector<dig_area *> dig_zones;
   std::vector<dig_area *> dug_zones;
   pose_2d start_pose;
   pose_2d state_pose;
   int wait_steps;

   double advance_distance;

   point zone_center(dig_area * zone) {
      point mid = {0.0,0.0};

      mid.x = (zone->upper.x + zone->lower.x) / 2.0;
      mid.y = (zone->upper.y + zone->lower.y) / 2.0;

      return mid;
   }

   void initialize() {

      /* get the total dig areas that will fit in the x and y
       * directions of the specified competition area.       */
      int total_x = arena.dig.width  / sample_x;
      int total_y = arena.dig.height / sample_y;
      /* the extra space between dig zones for 'formatting' */
      double spacing_x = std::fmod(arena.dig.width, sample_x) / (double)total_x;
      double spacing_y = std::fmod(arena.dig.height,sample_y) / (double)total_y;

      /* values to be updated as we go through
       * placing our dig zones                */
      double off_x = spacing_x + arena.dig.origin.x;
      double off_y = spacing_y + arena.dig.origin.y;

      for (int j = 0; j < total_y; ++j) {
         for (int i = 0; i < total_x; ++i) {
            dig_zones.push_back(new dig_area{
                     {off_x, off_y}, {off_x + sample_x, off_y + sample_y},
                     true} /* all dig zones are good to start */
                     );
            off_x += (sample_x + spacing_x);
         }
         off_x = spacing_x + arena.dig.origin.x;
         off_y += (sample_y + spacing_y);
      }

      /* now fill the goal queue with the exploration areas
       * we want to visit and go to.                       */
      goal_queue.push_back(start_pose.pos);

      /* bottom right corner of the arena */
      goal_queue.push_back({arena.arena.origin.x + arena.arena.width - 0.5,
                            arena.arena.origin.y + 0.5});

      /* top right corner of the arena */
      goal_queue.push_back({arena.arena.origin.x + arena.arena.width  - 0.5,
                            arena.arena.origin.y + arena.arena.height - 0.5});

      /* top left corner of the dig area */
      goal_queue.push_back({arena.dig.origin.x + 0.5,
                            arena.dig.origin.y + arena.dig.height - 0.5});

      /* bottom left corner of the dump area */
      goal_queue.push_back({arena.dump.origin.x + 0.5,
                            arena.dump.origin.y + 0.5});


      /* set the state to exploring to kick things off */
      current_state = EXPLORING;
   }

public:

   StaticBoxDigger(kurome::msg::CompetitionArea arena, double sample_x, double sample_y, pose_2d start,
                   double advance_distance)
      : CommanderBase(arena,sample_x,sample_y), start_pose(start), state_pose({{0,0}}),
        advance_distance(advance_distance) {

         initialize();
   }

   void reset() override {

      dig_zones.clear();
      dug_zones.clear();
      while (!goal_queue.empty())
         goal_queue.pop_front();

      initialize();
   }

   geometry_msgs::msg::Pose get_goal() override {

      switch (current_state) {

         case EXPLORING:
            {
               if (sqrt(point_dist2(goal_queue.front(),state_pose.pos) < advance_distance))
                  goal_queue.pop_front();
               if (goal_queue.empty()) {
                  current_state = DIGGING;
                  wait_steps = 8;
                  return pose_2d_to_ros2_pose(state_pose);
               }
               pose_2d pose = {goal_queue.front(),0.0};
               return pose_2d_to_ros2_pose(pose);
                  
            }
         case DIGGING:
            {
               if (wait_steps <= 0) {
                  dug_zones.push_back(dig_zones.back());
                  dig_zones.pop_back();
                  wait_steps = 6;
                  current_state = DUMPING;
                  return pose_2d_to_ros2_pose(state_pose);
               }
               else {
                  pose_2d mid = {{0.0,0.0}};
                  while (dig_zones.back()->active == false) {
                     dug_zones.push_back(dig_zones.back());
                     dig_zones.pop_back();
                  }
                  if (dig_zones.empty()) {
                     current_state = FINISHED;
                     return pose_2d_to_ros2_pose(state_pose);
                  }

                  mid.pos = zone_center(dig_zones.back());
                  if (sqrt(point_dist2(state_pose.pos,mid.pos)) < advance_distance) {
                     wait_steps -= 1;
                     return pose_2d_to_ros2_pose(state_pose);
                  }

                  return pose_2d_to_ros2_pose(mid);
               }

            }
         case DUMPING:
            {
               if (wait_steps <= 0) {
                  wait_steps = 8;
                  current_state = DIGGING;
                  return pose_2d_to_ros2_pose(state_pose);
               }
               pose_2d dump_goal = {{(arena.dump.width) /2.0+arena.dump.origin.x,
                                     (arena.dump.height)/2.0+arena.dump.origin.y}};

               if (state_pose.pos.x > arena.dump.origin.x                    && 
                   state_pose.pos.x < arena.dump.origin.x + arena.dump.width &&
                   state_pose.pos.y > arena.dump.origin.y                    &&
                   state_pose.pos.y < arena.dump.origin.y + arena.dump.height) {
                  wait_steps -= 1;
               }
               return pose_2d_to_ros2_pose(dump_goal);

            }
         case FINISHED:
            {
               return pose_2d_to_ros2_pose(start_pose);
            }

      }

   }

   void acknowledge_map() override {

      int x_step = sample_x / last_res;
      int y_step = sample_y / last_res;

      for (dig_area * d : dig_zones) {

         double x_pos = d->lower.x;
         double y_pos = d->lower.y;

         for (int i = 0; i < x_step; ++i) {
            for (int j = 0; j < y_step; ++j) {

               if (obs(x_pos,y_pos)) {
                  d->active = false;
                  goto next_zone;
               }

               y_pos += last_res;
            }
            y_pos = d->lower.y;
            x_pos += last_res;
         }
         next_zone:
         {}
      }


   }

   void visualize_motions(visualization_msgs::msg::MarkerArray & msg) override {

      /* all of the dig areas first */

      visualization_msgs::msg::Marker dig_marker;
      dig_marker.ns = "brain";
      dig_marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
      dig_marker.action = visualization_msgs::msg::Marker::ADD;
      dig_marker.pose.position.z = 0.0;
      dig_marker.pose.orientation.x = 0.0;
      dig_marker.pose.orientation.y = 0.0;
      dig_marker.pose.orientation.z = 0.0;
      dig_marker.pose.orientation.w = 1.0;
      dig_marker.scale.z = 0.1;
      dig_marker.color.a = 0.1;
      dig_marker.lifetime.sec = 1.5;

      dig_marker.scale.x = sample_x;
      dig_marker.scale.y = sample_y;
      dig_marker.id = visualization_id++;

      for (dig_area * d : dig_zones) {
         geometry_msgs::msg::Point p;
         std_msgs::msg::ColorRGBA c;
         p.z = 0.0;
         point mid = zone_center(d);
         p.x = mid.x;
         p.y = mid.y;
         c.a = 0.1;
         if (d->active) {
            c.r = 0.0;
            c.g = 1.0;
            c.b = 0.1;
         }
         else {
            c.r = 1.0;
            c.g = 0.0;
            c.b = 0.1;
         }

         dig_marker.points.push_back(p);
         dig_marker.colors.push_back(c);
      }
      for (dig_area * d : dug_zones) {
         geometry_msgs::msg::Point p;
         std_msgs::msg::ColorRGBA c;
         p.z = 0.0;
         point mid = zone_center(d);
         p.x = mid.x;
         p.y = mid.y;
         c.a = 0.1;
         if (d->active) {
            c.r = 1.0;
            c.g = 1.0;
            c.b = 0.1;
         }
         else {
            c.r = 1.0;
            c.g = 0.0;
            c.b = 0.1;
         }

         dig_marker.points.push_back(p);
         dig_marker.colors.push_back(c);
      }

      msg.markers.push_back(dig_marker);


      /* the current exploration waypoint now */
      if (!goal_queue.empty()) {
         visualization_msgs::msg::Marker marker;

         marker.ns = "brain";
         marker.type = visualization_msgs::msg::Marker::CUBE;
         marker.action = visualization_msgs::msg::Marker::ADD;
         marker.pose.orientation.x = 0.0;
         marker.pose.orientation.y = 0.0;
         marker.pose.orientation.z = 0.0;
         marker.pose.orientation.w = 1.0;
         marker.scale.z = 0.1;
         marker.color.r = 1.0;
         marker.color.g = 1.0;
         marker.color.b = 0.0;
         marker.color.a = 0.1;
         marker.pose.position.x = goal_queue.front().x;
         marker.pose.position.y = goal_queue.front().y;
         marker.pose.position.z = 0.0;
         marker.lifetime.sec = 5.0;
         marker.id = visualization_id++;

         marker.scale.x = advance_distance;
         marker.scale.y = advance_distance;

         msg.markers.push_back(marker);
      }
   }

   void update_position(pose_2d & pose) override {
      state_pose = pose;
   }

};

#endif
