
#ifndef __COMMANDER_BASE

#define __COMMANDER_BASE

#include <rclcpp/rclcpp.hpp>

#include "kurome/msg/competition_area.hpp"
#include "geometry_msgs/msg/pose.hpp"

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/exceptions.h"

#include "kurome.h"

#include <cstring>
#include <cmath>

class CommanderBase {
protected:

   /* x and y coordinates in a cartesian
    * two dimensional space.           */
   struct peg {
      int x;
      int y;
   };

   /* arena dimensitions */
   kurome::msg::CompetitionArea arena;

   /* internal map of just the arena */
   uint8_t * map = NULL;

   /* measures of the internal arena size */
   peg p_low, p_high;

   point off_index;
   double last_res;

   /* requested size of the dig samples to retrieve */
   double sample_x;
   double sample_y;

   /* parameter for correctly filling the visualization markers.
    * Should be used by any child class implimenting visualize_motions
    * to set the id variable as well.                                */
   int visualization_id = 0;

public:
   
   CommanderBase(kurome::msg::CompetitionArea arena, double sample_x, double sample_y)
      : arena(arena), sample_x(sample_x), sample_y(sample_y) {}

   /* return the current goal this commander is attempting to reach */
   virtual geometry_msgs::msg::Pose get_goal() = 0;

   /* reset this commander to it's starting state */
   virtual void reset() = 0;

   /* update commands based on a change in the map */
   virtual void acknowledge_map() = 0;

   /* produce an appropriate visualization of the internal state of the commander.
    * Note that a visualization of the arena is already provided by the abscract base */
   virtual void visualize_motions(visualization_msgs::msg::MarkerArray & msg) = 0;

   /* notify the commander of the robot's current position in the environment */
   virtual void update_position(pose_2d & pose) = 0;

   /* return the position in the map given a value in real space */
   peg operator()(point pos) {
      pos.x -= off_index.x;
      pos.y -= off_index.y;
      return peg{(int)floor(pos.x/last_res),(int)floor(pos.y/last_res)};
   }

   /* cut out the portion of the map that we want and load it into our internal representation
    * of the mapped area, which will only include the obstacles present in the arena space. When
    * this is done call acknowledge_map, which will force the child commander to update it's 
    * commands given the new map data.                                                          */
   void update_map(const nav_msgs::msg::OccupancyGrid & msg, int obstacle_threshold) {

      off_index = { floor((arena.arena.origin.x - msg.info.origin.position.x)),
                    floor((arena.arena.origin.y - msg.info.origin.position.y)) };
      off_index = { minimum(0.0,off_index.x), minimum(0.0,off_index.y) };
      double last_res = msg.info.resolution;

      peg a_l = { (int)floor((arena.arena.origin.x - msg.info.origin.position.x) / msg.info.resolution),
                  (int)floor((arena.arena.origin.y - msg.info.origin.position.y) / msg.info.resolution) };
      peg a_h = { (int)ceil ((arena.arena.origin.x + arena.arena.width
                              - msg.info.origin.position.x              ) / msg.info.resolution),
                  (int)ceil ((arena.arena.origin.y + arena.arena.height
                              - msg.info.origin.position.y              ) / msg.info.resolution) };

      peg p_low_new  = { maximum(a_l.x,0), maximum(a_l.y,0) };
      peg p_high_new = { minimum(a_h.x,(int)msg.info.width), minimum(a_h.y,(int)msg.info.height) };

      if (memcmp(&p_low_new,&p_low,sizeof(peg)) || memcmp(&p_high_new,&p_high,sizeof(peg))) {
         free(map);
         p_low  = p_low_new;
         p_high = p_high_new;
         map = (uint8_t *)calloc(sizeof(uint8_t),(1 + (p_high.x - p_low.x))*(1 + (p_high.y - p_low.y)));
      }

      for (int y = p_low.y*msg.info.width, y_loc = 0; y <= p_high.y*msg.info.width;
           y += msg.info.width, y_loc += (1 + (p_high.x - p_low.x))) {
         for (int x = p_low.x, x_loc = 0; x <= p_high.x; ++x, ++x_loc) {

            map[x_loc + y_loc] = (msg.data[x + y] >= obstacle_threshold ? 1 : 0);

         }
      }

      acknowledge_map();
   }

   /* perform the basic visualization operations and then call the child commander for it's specific
    * visualization additions before returning the filled message to the main process.              */
   void construct_visualization(visualization_msgs::msg::MarkerArray & msg) {

      visualization_msgs::msg::Marker marker;

      /* setup universal marker parameters */
      marker.ns = "brain";
      marker.type = visualization_msgs::msg::Marker::CUBE;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.position.z = -0.05;
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;
      marker.scale.z = 0.1;
      marker.color.a = 0.1;
      marker.lifetime.sec = 5.0;

      /* add arena area */
      marker.id = visualization_id++;
      marker.color.r = 1.0;
      marker.color.g = 0.0;
      marker.color.b = 0.0;
      marker.scale.x = arena.arena.width;
      marker.scale.y = arena.arena.height;
      marker.pose.position.x = (arena.arena.origin.x - (arena.arena.width  / 2.0));
      marker.pose.position.y = (arena.arena.origin.y - (arena.arena.height / 2.0));
      msg.markers.push_back(marker);

      /* add dig area */
      marker.id = visualization_id++;
      marker.color.r = 0.0;
      marker.color.g = 1.0;
      marker.color.b = 0.0;
      marker.scale.x = arena.dig.width;
      marker.scale.y = arena.dig.height;
      marker.pose.position.x = (arena.dig.origin.x - (arena.dig.width  / 2.0));
      marker.pose.position.y = (arena.dig.origin.y - (arena.dig.height / 2.0));
      msg.markers.push_back(marker);

      /* add dump area */
      marker.id = visualization_id++;
      marker.color.r = 0.0;
      marker.color.g = 0.0;
      marker.color.b = 1.0;
      marker.scale.x = arena.dump.width;
      marker.scale.y = arena.dump.height;
      marker.pose.position.x = (arena.dump.origin.x - (arena.dump.width  / 2.0));
      marker.pose.position.y = (arena.dump.origin.y - (arena.dump.height / 2.0));
      msg.markers.push_back(marker);

      /* pass to child commander */
      visualize_motions(msg);
   }

};

#endif
