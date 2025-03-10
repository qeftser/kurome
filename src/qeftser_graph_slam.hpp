
#ifndef __QEFTSER_GRAPH_SLAM

#define __QEFTSER_GRAPH_SLAM
#include <mutex>
#include <thread>
#include <atomic>
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
   std::mutex graph_lock; /* use with care! */

   /* A spatially indexed collection of nodes in the
    * pose graph. Used for loop closure and edge detection */
   struct dist2 dist2; struct get_xy get_xy;
   SpatialBin<point,node,decltype(dist2),decltype(get_xy)> nodes;

   /* All of the nodes in the system */
   std::vector<node *> node_list;

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

   /* The last node added to the pose graph. Used for linking odometry
    * and returning a value in the get_last_pose call. Atomic because it
    * will get swapped out a decent amount.                             */
   std::atomic<node *> last_node = NULL;

   /* The linear and angular movement needed for
    * a node to be added to the pose graph.     */
   double linear_update_dist;
   double angular_update_dist;

   /* certainty of the lidar matcher needed for the
    * computed observation to be added to the pose graph */
   double lidar_acceptance_threshold;

   /* certainty of the point cloud matcher needed for the
    * computed observation to be added to the pose graph */
   double point_cloud_acceptance_threshold;

   /* range to look for node association */
   double node_association_dist;

   pose_2d safe_a_from_b(pose_2d a, pose_2d b) {
      pv A = pv{a.pos.x,a.pos.y,a.theta};
      pv B = pv{b.pos.x,b.pos.y,b.theta};

      ht hta = as_homogeneous_transformation(A);
      ht htb = as_homogeneous_transformation(B);
      ht htb_inv = invert_homogeneous_transformation(&htb);
      ht a_f_b = merge_homogeneous_transformation(&htb_inv,&hta);
      pv res = destruct_homogeneous_transformation(&a_f_b);
      double angle_from = a.theta - b.theta;
      angle_from += (angle_from > M_PI) ? -(2.0*M_PI) : (angle_from < -M_PI) ? (2.0*M_PI) : 0.0;
      return pose_2d{{res.x,res.y},angle_from};
   }


   /* perform all updates and maintenance on the pose graph */
   void manage_graph() {

      while (true) {
repeat:

         /* perform a small wait if there are no elements in the queue */
         if (incoming_observations.empty()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
         }

         /* collect the element for processing */
         std::pair<Observation *,bool> latest_observation;
         incoming_observations.dequeue(&latest_observation);

         /* unpack for convenience */
         bool do_fix_node = std::get<1>(latest_observation);
         Observation * observation = std::get<0>(latest_observation);

         node * previous_node = last_node.load();

         double angle_dist = 0.0;

         if (previous_node) {
            angle_dist = 
               observation->current_odometry.theta - previous_node->observation->current_odometry.theta;
            angle_dist += (angle_dist > M_PI) ? -(2.0*M_PI) : (angle_dist < -M_PI) ? (2.0*M_PI) : 0.0;
         }

         /* ensure enough movement has been acheived to add an observation */
         if (!previous_node ||
             sqrt(point_dist2(observation->current_odometry.pos,
                              previous_node->observation->current_odometry.pos)) >
               linear_update_dist ||
             fabs(angle_dist) > angular_update_dist) {

            /* needed for use with graph_slam_backend */
            position_vector converted_pose;
            Information3 information;
            symmetric_3x3_matrix converted_information;

            /* add this node to the graph */
            node * new_node = new node;
            position_vector pos = { observation->global_pose_estimate.pos.x,
                                    observation->global_pose_estimate.pos.y,
                                    observation->global_pose_estimate.theta };
            new_node->handle = add_node(pos,graph);
            new_node->observation = observation;
            new_node->best_pose = observation->global_pose_estimate;

            /* fix the node if instructed to */
            if (do_fix_node)
               fix_node(new_node->handle,graph);

            /* add the nessesary odometry edge to keep
             * the graph well constrained.            */
            if (previous_node) {
               pose_2d prev_from_curr = a_from_b(previous_node->observation->current_odometry,
                                                 observation->current_odometry);
               pose_2d prev_from_curr_safe = safe_a_from_b(previous_node->observation->current_odometry,
                                                           observation->current_odometry);
               prev_from_curr = prev_from_curr_safe;

               converted_pose = { prev_from_curr.pos.x,
                                  prev_from_curr.pos.y,
                                  prev_from_curr.theta };
               information = observation->global_pose_covariance.to_information();
               converted_information = { .a00 = information.xx,
                                         .a10 = information.xy,
                                         .a11 = information.yy,
                                         .a20 = information.xz,
                                         .a21 = information.yz,
                                         .a22 = information.zz };
               add_edge(new_node->handle,previous_node->handle,&converted_pose,&converted_information,graph);
            }

            /* get nearby nodes to add constraints to */
            std::vector<node *> nearby;
            nodes.in_range(observation->global_pose_estimate.pos,node_association_dist,&nearby);

            /* collect lidar scan matching results */
            double lidar_certainty;
            for (node * n : nearby) {

               if ((int)n->handle - (int)new_node->handle == -1)
                  continue;

               Covariance3 measurement_covariance;
               pose_2d measurement = safe_a_from_b(n->observation->global_pose_estimate,
                                                   observation->global_pose_estimate);

               lidar_certainty = lidar_matcher->match_scan(observation->laser_scan,n->observation->laser_scan,
                                                           measurement, measurement_covariance);

               if (lidar_certainty > lidar_acceptance_threshold) {

                  /* add an edge to the graph */
                  converted_pose = { measurement.pos.x,
                                     measurement.pos.y,
                                     measurement.theta };
                  information = measurement_covariance.to_information();
                  converted_information = { .a00 = information.xx,
                                            .a10 = information.xy,
                                            .a11 = information.yy,
                                            .a20 = information.xz,
                                            .a21 = information.yz,
                                            .a22 = information.zz };
                  add_edge(new_node->handle,n->handle,&converted_pose,&converted_information,graph);
               }
            }

            double cloud_certainty;
            for (node * n : nearby) {

               Covariance3 measurement_covariance;
               pose_2d measurement = a_from_b(n->best_pose,observation->global_pose_estimate);

               cloud_certainty = point_cloud_matcher->match_point_cloud(observation->point_cloud,
                                                                        n->observation->point_cloud,
                                                                        measurement,measurement_covariance);

               if (cloud_certainty > point_cloud_acceptance_threshold) {

                  /* add an edge to the graph */
                  converted_pose = { measurement.pos.x,
                                     measurement.pos.y,
                                     measurement.theta };
                  information = measurement_covariance.to_information();
                  converted_information = { .a00 = information.xx,
                                            .a10 = information.xy,
                                            .a11 = information.yy,
                                            .a20 = information.xz,
                                            .a21 = information.yz,
                                            .a22 = information.zz };
                  add_edge(new_node->handle,n->handle,&converted_pose,&converted_information,graph);
               }
            }

            last_node.store(new_node);
            node_list.push_back(new_node);

            /* see if a loop closure has occured */
            for (node * n : nearby) {
               if (abs((int)new_node->handle - (int)n->handle) > 20) {
                  perform_loop_closure();
                  goto repeat; /* other steps will be done implicitly if this is the case */
               }
            }

            /* add this node to the graph and the map */
            nodes.add(new_node);
            best_map->add_scan(observation->laser_scan,new_node->best_pose);
            best_map->add_point_cloud(observation->point_cloud,new_node->best_pose);


         }
         else /* simply destroy the observation */
            delete observation;
      }
   }

   void perform_loop_closure() {

      graph_slam_optimization_data info;
      bzero(&info,sizeof(graph_slam_optimization_data));

      graph_lock.lock();

      optimize(graph,&info);
      print_graph_slam_optimization_data(&info);
      OccupancyGrid * new_map = new OccupancyGrid(0.1,20,-20);
      nodes.clear();

      for (size_t i = 0; i < node_list.size(); ++i) {

        position_vector updated_pose = graph->node[node_list[i]->handle].pos;
        node_list[i]->best_pose = { {updated_pose.x, updated_pose.y}, updated_pose.t };

        new_map->add_scan(node_list[i]->observation->laser_scan,node_list[i]->best_pose);
        new_map->add_point_cloud(node_list[i]->observation->point_cloud,node_list[i]->best_pose);
        nodes.add(node_list[i]);
      }

      graph_lock.unlock();

      map_lock.lock();

      delete best_map;
      best_map = new_map;

      map_lock.unlock();
   }

public:

   QeftserGraphSlam(rclcpp::Clock::SharedPtr clock,
                    LidarMatcher * lidar_matcher, PointCloudMatcher * point_cloud_matcher,
                    double bin_size, double linear_update_dist, double angular_update_dist,
                    double lidar_acceptance_threshold, double point_cloud_acceptance_threshold,
                    double node_association_dist)
      : SlamSystem(clock,lidar_matcher,point_cloud_matcher), nodes(bin_size),
        linear_update_dist(linear_update_dist), angular_update_dist(angular_update_dist),
        lidar_acceptance_threshold(lidar_acceptance_threshold), 
        point_cloud_acceptance_threshold(point_cloud_acceptance_threshold),
        node_association_dist(node_association_dist) {

      graph = construct_pose_graph();

      best_map = new OccupancyGrid(0.1,20,-20);

      graph_manager = std::thread(&QeftserGraphSlam::manage_graph, this);

   }

   void insert_observation(Observation * observation, bool fixed = NOFIX_NODE) override {
      incoming_observations.enqueue(std::make_pair(observation,fixed));
   }

   void construct_visualization(visualization_msgs::msg::MarkerArray & msg) override {

      visualization_msgs::msg::Marker nodes;
      nodes.action = 0;
      nodes.type = visualization_msgs::msg::Marker::CUBE_LIST;
      nodes.lifetime.sec = 2;
      nodes.color.r = 1.0;
      nodes.color.g = 1.0;
      nodes.color.b = 0.0;
      nodes.color.a = 1.0;
      nodes.ns = "qeftser_graph_slam";
      nodes.scale.x = 0.05;
      nodes.scale.y = 0.05;
      nodes.scale.z = 0.05;

      visualization_msgs::msg::Marker edges;
      edges.action = 0;
      edges.type = visualization_msgs::msg::Marker::LINE_STRIP;
      edges.lifetime.sec = 2;
      edges.ns = "qeftser_graph_slam";
      edges.scale.x = 0.03;

      std_msgs::msg::ColorRGBA red;
      red.r = red.a = 1.0; red.b = red.g = 0.0;

      std_msgs::msg::ColorRGBA cyan;
      cyan.b = cyan.g = cyan.a = 1.0; cyan.r = 0.0;

      geometry_msgs::msg::Point pos;
      pos.z = 0.0;
      graph_lock.lock();

      int node_count = graph->node_count;

      for (int i = 0; i < node_count; ++i) {

         pos.x = graph->node[i].pos.x;
         pos.y = graph->node[i].pos.y;
         nodes.points.push_back(pos);

      }

      int edge_count = graph->edge_count;

      for (int i = 0; i < edge_count; ++i) {

         if (abs((int)graph->edge[i].xi - (int)graph->edge[i].xj) == 1) {
            edges.colors.push_back(cyan);
            edges.colors.push_back(cyan);
         }
         else {
            edges.colors.push_back(red);
            edges.colors.push_back(red);
         }

         pos.x = graph->node[graph->edge[i].xi].pos.x;
         pos.y = graph->node[graph->edge[i].xi].pos.y;
         edges.points.push_back(pos);

         pos.x = graph->node[graph->edge[i].xj].pos.x;
         pos.y = graph->node[graph->edge[i].xj].pos.y;
         edges.points.push_back(pos);

      }

      graph_lock.unlock();

      msg.markers.push_back(nodes);
      msg.markers.push_back(edges);
   }

   void get_map(nav_msgs::msg::OccupancyGrid & ret_msg) override {
      map_lock.lock();

      if (best_map)
         best_map->to_msg(ret_msg);

      map_lock.unlock();
   }

   std::pair<pose_2d,pose_2d> get_last_pose() override {
      node * last = last_node.load();

      if (!last)
         return std::make_pair(pose_2d{{0,0},0},pose_2d{{0,0},0});

      return std::make_pair(last->best_pose,last->observation->current_odometry);
   }

};

#endif
