
#ifndef __BUILTIN_GRAPH_SLAM

#define __BUILTIN_GRAPH_SLAM
#include <mutex>
#include <thread>
#include <atomic>
#include <list>
#include <iostream>
#include "kurome.h"
#include "slam_system.hpp"
#include "spatial_bin.hpp"
#include "locked_queue.hpp"

#include <ceres/ceres.h>
#include <ceres/autodiff_cost_function.h>
#include <ceres/types.h>

#include <Eigen/Dense>

/* A graph slam algorithm using ceres and eigen and all
 * the bells and whistles that runs while I sit in the
 * corner crying and contemplating by incompetance.   */

class AngleManifold {
public:

   template <typename T>
   bool Plus(const T * x_radians,
             const T * delta_radians,
             T * result_radians) const {
      T new_angle = *x_radians + *delta_radians;
      *result_radians = atan2(sin(new_angle),cos(new_angle));
      return true;
   }

   template <typename T>
   bool Minus(const T * y_radians,
              const T * x_radians,
              T * result_radians) const {
      T x_norm = atan2(sin(*x_radians),cos(*x_radians));
      T y_norm = atan2(sin(*y_radians),cos(*y_radians));
      *result_radians = y_norm - x_norm;
      return true;
   }

   static ceres::Manifold * create() {
      return new ceres::AutoDiffManifold<AngleManifold, 1, 1>;
   }
};

class ErrorTerm {
private:

   /* the obervation of the first pose from the second */
   const pose_2d a_from_b;
   /* the information matrix associated with the observation */
   const Information3 information;

public:

   ErrorTerm(pose_2d & a_from_b, Covariance3 & covariance) 
      : a_from_b(a_from_b), information(covariance.to_information()) {}

   ErrorTerm(pose_2d & a_from_b, Information3 & information) 
      : a_from_b(a_from_b), information(information) {}

   /* compute the error between the poses and the observataion of
    * the edge, including the information matrix in the result.  */
   template <typename T>
   bool operator()(const T * const x_i,
                   const T * const y_i,
                   const T * const t_i,
                   const T * const x_j,
                   const T * const y_j,
                   const T * const t_j,
                   T * residuals) const {

      T sin_i = sin(*t_i); T cos_i = cos(*t_i);
      /*
      T sin_ij = (T)sin(a_from_b.theta); T cos_ij = (T)cos(a_from_b.theta);
      T sin_iij = sin(*t_i + a_from_b.theta);
      T cos_iij = cos(*t_i + a_from_b.theta);
      */
      T dist_x = *x_j - *x_i;
      T dist_y = *y_j - *y_i;

      T interm[3];
      /*
      interm[0] = dist_x*cos_iij + dist_y*sin_iij - (T)(a_from_b.pos.y*sin_ij + a_from_b.pos.x*cos_ij);
      interm[1] = dist_x*sin_iij + dist_y*cos_iij + (T)a_from_b.pos.x*sin_ij - (T)a_from_b.pos.y*cos_ij;
      */
      interm[0] = (dist_x * cos_i + dist_y * sin_i) - (T)(a_from_b.pos.x);
      interm[1] = (dist_y * cos_i - dist_x * sin_i) - (T)(a_from_b.pos.y);
      interm[2] = ((*t_j - *t_i) - (T)(a_from_b.theta));
      interm[2] = atan2(sin(interm[2]),cos(interm[2]));

      /*
      residuals[0] = interm[0] * information.xx +
                     interm[1] * information.xy +
                     interm[2] * information.xz;
      residuals[1] = interm[0] * information.xy +
                     interm[1] * information.yy +
                     interm[2] * information.yz;
      residuals[2] = interm[0] * information.xz +
                     interm[1] * information.yz +
                     interm[2] * information.zz;
                     */
      residuals[0] = interm[0];
      residuals[1] = interm[1];
      residuals[2] = interm[2];

      return true;
   }

   static ceres::CostFunction * create(pose_2d & a_from_b,
                                       Covariance3 & covariance) {
      return new ceres::AutoDiffCostFunction<ErrorTerm, 3, 1, 1, 1, 1, 1, 1>(
            new ErrorTerm(a_from_b, covariance));
   }

   static ceres::CostFunction * create(pose_2d & a_from_b,
                                       Information3 & information) {
      return new ceres::AutoDiffCostFunction<ErrorTerm, 3, 1, 1, 1, 1, 1, 1>(
            new ErrorTerm(a_from_b, information));
   }

};

class BuiltinGraphSlam : public SlamSystem {
private:

   /* a node on the graph */
   struct node {
      /* number of the node */
      uint id = 0;
      /* global position of the node */
      pose_2d pose;
      /* pose used in optimization problems */
      pose_2d o_pose;
      /* observation associated with the node */
      Observation * observation;
   };

   /* an edge on the graph */
   struct edge {
      /* residual block associated with this edge */
      ceres::ResidualBlockId id;
      /* ingoing node */
      uint node_i;
      /* outgoing node */
      uint node_j;
   };

   struct dist2 {

      double operator()(const point & p, const node & n) {
         return (p.x - n.pose.pos.x)*(p.x - n.pose.pos.x) +
                (p.y - n.pose.pos.y)*(p.y - n.pose.pos.y);
      }

      double operator()(const node & n1, const node & n2) {
         return (n1.pose.pos.x - n2.pose.pos.x)*(n1.pose.pos.x - n2.pose.pos.x) +
                (n1.pose.pos.y - n2.pose.pos.y)*(n1.pose.pos.y - n2.pose.pos.y);
      }

  };

   struct get_xy {

      void operator()(const point & p, double * x, double * y) {
         *x = p.x; *y = p.y;
      }

      void operator()(const node & n, double * x, double * y) {
         *x = n.pose.pos.x;
         *y = n.pose.pos.y;
      }

   };

   /* Essentially the pose graph. A problem for ceres to solve */
   ceres::Problem graph;
   /* settings for the solver to use */
   ceres::Solver::Options options;
   /* constraints for the pose graph */
   ceres::Manifold * manifold = AngleManifold::create();
   ceres::LossFunction * loss_function = nullptr;

   /* use with care. Locks nodes on the graph */
   std::mutex graph_lock;

   /* A spatially indexed collection of nodes in the
    * pose graph. Used for loop closure and edge detection */
   struct dist2 dist2; struct get_xy get_xy;
   SpatialBin<point,node,decltype(dist2),decltype(get_xy)> nodes;

   /* All of the nodes in the system */
   std::vector<node *> node_list;
   /* All of the fixed nodes in the system */
   std::unordered_set<long> fixed_nodes;
   /* Used for construction of the visualization */
   std::vector<edge> edge_list;

   /* Used for constructing a map to match against. 
    * Will kind of function like a circular queue */
   std::list<Observation *> recent_observations;
   /* number of recent observations to keep in the buffer */
   int recent_length;

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
   
   /* the number of nodes apart an association must be
    * to trigger an optimization of the pose graph */
   int loop_closure_dist;

   /* compute the pose a as seen from the pose b */
   pose_2d a_from_b(pose_2d a, pose_2d b) {
      double angle_from = a.theta - b.theta;
      angle_from += (angle_from > M_PI) ? -(2.0*M_PI) : (angle_from < -M_PI) ? (2.0*M_PI) : 0.0;

      point dist = { a.pos.x - b.pos.x, a.pos.y - b.pos.y };
      point res = { dist.x * cos(b.theta) + dist.y * sin(b.theta),
                    dist.y * cos(b.theta) - dist.x * sin(b.theta) };

      return pose_2d{{res.x,res.y},angle_from};
   }

   /* perform all updataes and maintenance on the pose graph */
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

            /* add this node to the graph */
            node * new_node = new node;
            new_node->id = node_list.size();
            new_node->observation = observation;
            new_node->pose = observation->global_pose_estimate;
            new_node->o_pose = new_node->pose;

            node_list.push_back(new_node);

            /* fix the node if instructed to */
            if (do_fix_node)
               fixed_nodes.insert(ptr_to_long(new_node));
               

            /* add the nessesary odometry edge to keep
             * the graph well constrained.            */
            if (previous_node) {
               pose_2d prev_from_curr = a_from_b(previous_node->observation->current_odometry,
                                                 observation->current_odometry);

               /* add edge */
               ceres::CostFunction * cost_function = 
                  ErrorTerm::create(prev_from_curr,observation->global_pose_covariance);
               ceres::ResidualBlockId id = 
                  graph.AddResidualBlock(cost_function, loss_function, 
                                         &new_node->o_pose.pos.x, &new_node->o_pose.pos.y,
                                         &new_node->o_pose.theta,
                                         &previous_node->o_pose.pos.x, &previous_node->o_pose.pos.y,
                                         &previous_node->o_pose.theta);
               graph.SetManifold(&new_node->o_pose.theta,manifold);
               graph.SetManifold(&previous_node->o_pose.theta,manifold);
               edge_list.push_back({id,previous_node->id,new_node->id});
            }

            /* get nearby nodes to add constraints to */
            std::vector<node *> nearby;
            nodes.in_range(observation->global_pose_estimate.pos,node_association_dist,&nearby);

            /* collect lidar scan matching results */
            {
               double lidar_certainty;
               std::vector<std::pair<const LidarData *,pose_2d>> input;
               for (Observation * o : recent_observations) {
                  input.push_back(std::make_pair(&o->laser_scan,
                           a_from_b(o->current_odometry,observation->current_odometry)));
               }
               for (node * n : nearby) {
                  input.push_back(std::make_pair(&n->observation->laser_scan,
                           a_from_b(n->pose,observation->current_odometry)));
               }

               pose_2d measurement = {{0,0},0};
               Covariance3 measurement_covariance;
               /*
               lidar_certainty = lidar_matcher->match_scan(observation->laser_scan,input,
                                                           measurement,measurement_covariance);
                                                           */
               if (lidar_certainty > lidar_acceptance_threshold) {

                  for (node * n : nearby) {

                     pose_2d edge = transform(a_from_b(n->pose,observation->current_odometry),measurement);

                     ceres::CostFunction * cost_function = ErrorTerm::create(edge,measurement_covariance);
                     ceres::ResidualBlockId id = 
                        graph.AddResidualBlock(cost_function, loss_function,
                                               &new_node->o_pose.pos.x, &new_node->o_pose.pos.y,
                                               &new_node->o_pose.theta,
                                               &n->o_pose.pos.x, &n->o_pose.pos.y,&n->o_pose.theta);
                     graph.SetManifold(&new_node->o_pose.theta,manifold);
                     graph.SetManifold(&n->o_pose.theta,manifold);
                     edge_list.push_back({id,n->id,new_node->id});
                  }
               }
            }


            /* collect icp results */
            double cloud_certainty;
            for (node * n : nearby) {

               Covariance3 measurement_covariance;
               pose_2d measurement = a_from_b(n->observation->global_pose_estimate,
                                              observation->global_pose_estimate);

               cloud_certainty = point_cloud_matcher->match_point_cloud(observation->point_cloud,
                                                                        n->observation->point_cloud,
                                                                        measurement,measurement_covariance);

               if (cloud_certainty > point_cloud_acceptance_threshold) {

                  /* add an edge to the graph */
                  ceres::CostFunction * cost_function = ErrorTerm::create(measurement,measurement_covariance);
                  ceres::ResidualBlockId id = 
                     graph.AddResidualBlock(cost_function, loss_function, 
                                            &n->o_pose.pos.x, &n->o_pose.pos.y,&n->o_pose.theta, 
                                            &new_node->o_pose.pos.x, &new_node->o_pose.pos.y,
                                            &new_node->o_pose.theta);
                  graph.SetManifold(&new_node->o_pose.theta,manifold);
                  graph.SetManifold(&n->o_pose.theta,manifold);
                  edge_list.push_back({id,n->id,new_node->id});
               }
            }

            last_node.store(new_node);

            /* insert into recents */
            recent_observations.push_back(new_node->observation);
            if (recent_observations.size() > recent_length)
               recent_observations.pop_front();

            /* see if a loop closure has occured */
            for (node * n : nearby) {
               if (abs((int)new_node->id - (int)n->id) >= loop_closure_dist) {
                  if (perform_loop_closure())
                     goto repeat; /* other steps will be done implicitly if this is the case */
               }
            }

            /* add this node to the graph and the map */
            nodes.add(new_node);
            best_map->add_scan(observation->laser_scan,new_node->pose);
            best_map->add_point_cloud(observation->point_cloud,new_node->pose);

         }
         else /* simply destroy the observation */
            delete observation;
      }
   }

   bool perform_loop_closure() {
      bool ret = false;

      graph_lock.lock();

      ceres::Solver::Summary summary;

      graph.SetParameterBlockConstant(&node_list.front()->o_pose.pos.x);
      graph.SetParameterBlockConstant(&node_list.front()->o_pose.pos.y);
      graph.SetParameterBlockConstant(&node_list.front()->o_pose.theta);

      ceres::Solve(options,&graph,&summary);

      //std::cout << summary.FullReport() << std::endl;

      if (summary.IsSolutionUsable()) {
         ret = true;

         nodes.clear();

         OccupancyGrid * new_map = new OccupancyGrid(0.1,20,-3);
         for (size_t i = 0; i < node_list.size(); ++i) {

           node_list[i]->pose = node_list[i]->o_pose;

           new_map->add_scan(node_list[i]->observation->laser_scan,node_list[i]->pose);
           new_map->add_point_cloud(node_list[i]->observation->point_cloud,node_list[i]->pose);
           nodes.add(node_list[i]);
         }

         map_lock.lock();

         delete best_map;
         best_map = new_map;

         map_lock.unlock();
      }

      graph_lock.unlock();

      return ret;
   }

public:

   BuiltinGraphSlam(rclcpp::Clock::SharedPtr clock,
                    LidarMatcher * lidar_matcher, PointCloudMatcher * point_cloud_matcher,
                    double bin_size, double linear_update_dist, double angular_update_dist,
                    double lidar_acceptance_threshold, double point_cloud_acceptance_threshold,
                    double node_association_dist, int recent_length, int loop_closure_dist)
      : SlamSystem(clock,lidar_matcher,point_cloud_matcher), nodes(bin_size),
        linear_update_dist(linear_update_dist), angular_update_dist(angular_update_dist),
        lidar_acceptance_threshold(lidar_acceptance_threshold), 
        point_cloud_acceptance_threshold(point_cloud_acceptance_threshold),
        node_association_dist(node_association_dist), recent_length(recent_length),
        loop_closure_dist(loop_closure_dist) {

      /* initialize the graph */

      options.max_num_iterations = 1000;
      options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
      options.preconditioner_type = ceres::SCHUR_JACOBI; /* a la slam_toolbox */

      best_map = new OccupancyGrid(0.1,20,-20);

      graph_manager = std::thread(&BuiltinGraphSlam::manage_graph, this);

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

      for (node * n : node_list) {

         pos.x = n->pose.pos.x;
         pos.y = n->pose.pos.y;
         nodes.points.push_back(pos);

      }

      for (edge & e : edge_list) {

         if (abs((int)e.node_i - (int)e.node_j) == 1) {
            edges.colors.push_back(cyan);
            edges.colors.push_back(cyan);
         }
         else {
            edges.colors.push_back(red);
            edges.colors.push_back(red);
         }

         pos.x = node_list[e.node_i]->pose.pos.x;
         pos.y = node_list[e.node_i]->pose.pos.y;
         edges.points.push_back(pos);

         pos.x = node_list[e.node_j]->pose.pos.x;
         pos.y = node_list[e.node_j]->pose.pos.y;
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

      return std::make_pair(last->pose,last->observation->current_odometry);
   }

};

#endif
