
#ifndef __STATIC_BOX_DIGGER

#define __STATIC_BOX_DIGGER

#include "kurome.h"
#include "commander.hpp"
#include <queue>

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
   };

   std::queue<point,std::vector<point>>  goal_queue;
   std::vector<std::pair<dig_area,bool>> dig_zones;
   geometry_msgs::msg::Pose start_pose;

   void initialize() {
   }

   void update_state() {
   }

public:

   StaticBoxDigger(kurome::msg::CompetitionArea arena, double sample_x, double sample_y)
      : CommanderBase(arena,sample_x,sample_y) {}

   void reset() override {
   }

   geometry_msgs::msg::Pose get_goal() override {
   }

   void acknowledge_map() override {
   }

   void visualize_motions(visualization_msgs::msg::MarkerArray & msg) override {
   }

   void update_position(pose_2d & pose) override {
   }

};

#endif
