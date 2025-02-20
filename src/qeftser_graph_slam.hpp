
#ifndef __QEFTSER_GRAPH_SLAM

#define __QEFTSER_GRAPH_SLAM
#include <mutex>
#include <thread>
#include "kurome.h"
#include "slam_system.hpp"
#include "spatial_bin.hpp"
#include "locked_queue.hpp"

extern "C" {
#include <graph_slam_backend/graph_slam.h>
}

/* A port of my graph slam backend into this system. It does
 * a good amount of stuff besides that, but this is the best
 * name I could think of to keep trach of the system.       */

class QeftserGraphSlam : public SlamSystem {
private:

   struct node {
      /* handle to this node in the pose_graph */
      pgn_handle handle;
      /* the current pose of the node. Some redundancy
       * here as this value is also held in the node
       * pointed to by the pgn_handle, but this is 
       * nessesary for some of the auxillary functions */
      pose_2d best_pose;
      /* the associated observation tied to this node.
       * Kept for map regeneration on loop closure.  */
      Observation * observation;
   };

   struct dist2 {

      double operator()(const point & p, const node & n) {
         return (p.x - n.best_pose.pos.x)*(p.x - n.best_pose.pos.x) +
                (p.y - n.best_pose.pos.y)*(p.y - n.best_pose.pos.y);
      }

      double operator()(const node & n1, const node & n2) {
         return (n1.best_pose.pos.x - n2.best_pose.pos.x)*(n1.best_pose.pos.x - n2.best_pose.pos.x) +
                (n1.best_pose.pos.y - n2.best_pose.pos.y)*(n1.best_pose.pos.y - n2.best_pose.pos.y);
      }

   };

   struct get_xy {

      void operator()(const point & p, double * x, double * y) {
         *x = p.x; *y = p.y;
      }

      void operator()(const node & n, double * x, double * y) {
         *x = n.best_pose.pos.x;
         *y = n.best_pose.pos.y;
      }

   };

   /* The pose graph constructed by our system */
   pose_graph * graph;
   std::thread graph_lock; /* use with care! */

   /* A spatially indexed collection of nodes in the
    * pose graph. Used for loop closure and edge detection */
   struct dist2 dist2; struct get_xy get_xy;
   SpatialBin<point,node,decltype(dist2),decltype(get_xy)> nodes;

   /* queue of observations that need to be processed */
   LockedQueue<std::pair<Observation *,bool>> incoming_observations;

   /* current best representation of the map the robot is navigating
    * through/in. We need a lock here because this value will be accessed
    * by multiple threads asyncronously.                                */
   OccupancyGrid * best_map = NULL;
   std::mutex map_lock;

   /* thread that the actual slam system is ran in. We do all the work
    * on the pose graph (scan matching, loop closure, etc. ) in this thread */
   std::thread graph_manager;

   /* perform all updates and maintenance on the pose graph */
   void manage_graph() {

      while (true) {



      }
   }

public:

   QeftserGraphSlam(rclcpp::Clock::SharedPtr clock,
                    LidarMatcher * lidar_matcher, PointCloudMatcher * point_cloud_matcher,
                    double bin_size)
      : SlamSystem(clock,lidar_matcher,point_cloud_matcher), nodes(bin_size) {

         optimize(graph,NULL);

      graph_manager = std::thread(&QeftserGraphSlam::manage_graph, this);

   }

   void insert_observation(Observation * observation, bool fixed = NOFIX_NODE) override {
      incoming_observations.enqueue(std::make_pair(observation,fixed));
   }

   void construct_visualization(visualization_msgs::msg::MarkerArray & msg) override {
   }

   void get_map(nav_msgs::msg::OccupancyGrid & ret_msg) override {
      map_lock.lock();

      if (best_map)
         best_map->to_msg(ret_msg);

      map_lock.unlock();
   }

   std::pair<pose_2d,pose_2d> get_last_pose() override {
      return std::pair(pose_2d{{0,0},0},
                       pose_2d{{0,0},0});
   }

};

#endif
