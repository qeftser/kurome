
#ifndef __RRT_X_FN

#define __RRT_X_FN

#include "kurome.h"
#include "spatial_bin.hpp"
#include "pathfinder.hpp"
#include <queue>
#include <unordered_set>
#include <forward_list>
#include <vector>
#include <mutex>
#include <thread>
#include <chrono>

/**
 * RRT algorithm
 */

class RRTX_FN : public PathfinderBase {
public:

   /* represent a node on the tree */
   struct node {
      /* position of the node */
      point pos;
      /* the cost to reach the node */
      double cost;
      /* edges leading out of the node */
      std::vector<node *> children;
      /* edge leading into the node */
      node * parent;
      /* is this node orphaned? */
      bool orphan;
   };

private:

   /* Each node in the tree has a dominance region. Any 
    * inferior node cannot enter it and any superior node 
    * assumes it when it enters. This parameter gives the 
    * size of that region. Smaller values will result in
    * a denser tree.                                     */
   double dominance_region; 
   /* What range from any new obstacle that is added
    * should we search to remove nodes from our tree?    */
   double cull_range;
   /* length of the edges we attempt to add to our
    * system.                                            */
   double expansion_length;
   /* Total number of nodes allowed in the system.       */
   int    node_limit;
   /* current number of nodes in the system              */
   int    node_count;
   /* how many milliseconds to wait in-between adding a 
    * generation to the tree (100 nodes)                 */
   int generation_tick_speed;

   /* mutex for running commands on the tree. Ensures
    * we do not try and modify the tree as it is growing */
   std::mutex mut;

   /* seperate thread that handles creating the nodes
    * on the tree. Allows the algorithm to respond
    * quickly to requests to change variable or get
    * the current path.                              */
   std::thread tree_handler;

   /* minimum values sampled so far. */
   point min_len;
   /* maximum values sampled so far */
   point max_len;

   /* size of the area we are sampling from */
   point sample_area;
   /* half of the sample area               */
   point sample_offset;

   /* values that determine the sample radius. These
    * are set dyamically given the size of the tree
    * and the size of the environment.              */
   double x_len, y_len, x_off, y_off;

   /* compute the euclidian distance squared between
    * a node and a point or two nodes               */
   struct Dist2 {

      double operator()(const point & k, const node & v) {
         return (k.x-v.pos.x)*(k.x-v.pos.x)+(k.y-v.pos.y)*(k.y-v.pos.y);
      }

      double operator()(const node & v1, const node & v2) {
         return (v1.pos.x-v2.pos.x)*(v1.pos.x-v2.pos.x)+(v1.pos.y-v2.pos.y)*(v1.pos.y-v2.pos.y);
      }

   };

   /* copy the x and y values of a node or point into
    * the provided x and y values.                   */
   struct CoordinateCollector {

      void operator()(const point & k, double * x, double * y) {
         *x = k.x; *y = k.y;
      }

      void operator()(const node & v, double * x, double * y) {
         *x = v.pos.x; *y = v.pos.y;
      }

   };

   struct Dist2 dist2;
   struct CoordinateCollector coord_collector;
   SpatialBin<point,node,decltype(dist2),decltype(coord_collector)> nodes;
   SpatialBin<point,node,decltype(dist2),decltype(coord_collector)> orphanage;

   /* A lot of these functions need a queue that
    * they hold values in for operating on. This
    * queue is used for that purpose to avoid 
    * having to construct a new one whenever we 
    * make a call to a function that needs it.  */
   std::queue<node *> node_queue;

   /* Nodes which have no children. These are the
    * ones that can be culled from the tree in the
    * fixed_node variant of rrt - which we are using */
   std::unordered_set<long> infertile;

   /* Given a list of nodes surrounding a target, return
    * the node that gives the shortest distance to 
    * the target node.                                  */
   inline node * get_node_parent(const point & pos, std::vector<node *> * near_nodes) {
      double low_cost = DBL_MAX;
      node * low_node = NULL;

      for (node * n : *near_nodes) {
         double dist = n->cost + Dist2{}(pos,*n);


         if (!n->orphan                &&
             !is_collision(n->pos,pos) &&
             dist < low_cost) {

            low_cost = dist;
            low_node = n;
         }
      }
      return low_node;
   }

   /* Given a list of surrounding nodes, if the anchor node
    * is closer than the parent of a given node in the list,
    * change that node's parent to the anchor.              */
   inline void rewire_neighbors(node * anchor, std::vector<node *> * near) {

      /* Because we are removing the parent that we invalidate
       * from the list of near elements, a forward_list is
       * user to avoid iterator invalidation.                 */
      std::forward_list<node *> near_list = std::forward_list<node *>(near->begin(),near->end());
      node * current_node;
      double dist;

      while (!near_list.empty()) {

         /* Get the current node and compute the
          * cost of having the anchor as it's parent */
         current_node = near_list.front(); near_list.pop_front();
         dist = anchor->cost + Dist2{}(*anchor,*current_node);

         /* If the anchor is a better parent, swap
          * the old parent out.                   */
         if (!is_collision(anchor->pos,current_node->pos) &&
             dist < current_node->cost) {

            /* This is nessasary becasue the root and any
             * orphaned node will not have a parent.     */
            if (current_node->parent) {

               /* remove the current_node from it's
                * parent's list of children.       */
               erase_from_vector(current_node->parent->children,current_node);

               /* If the current node's former parent has no children
                * we are going to remove it from the tree. This is 
                * inline with the rrt-fn algorithm design.           */
               if (current_node->parent->children.empty()) {

                  /* remove the parent from it's parent's children */
                  erase_from_vector(current_node->parent->parent->children,current_node->parent);

                  /* Clear the node from the data structures. We do
                   * not remove it from the orpahn list because we 
                   * know it had a child.                          */
                  nodes.remove(current_node->parent);
                  near_list.remove(current_node->parent);
                  delete current_node->parent;
                  --node_count;
               }
            }

            /* If our anchor was infertile it
             * is now fertile, remove it from 
             * the infertile list.           */
            if (anchor->children.empty())
               infertile.erase((long)anchor);

            /* Update releavant node values */
            current_node->parent = anchor;
            anchor->children.push_back(current_node);
            current_node->cost = dist;
            make_relative_to_parent(current_node);

            /* If we are adopting an orphan,
             * trigger the appropriate routine. */
            if (current_node->orphan)
               adopt_node(current_node);
         }
      }
   }

   /* Takes care of the adoption process for orphaned nodes.
    * Adoption is a recursive procedure that will adopt
    * any node in range of the adoption process, with the 
    * goal of recovering sections of our tree which were lost
    * when new obstacles were added.                         */
   inline void adopt_node(node * orphan) {

      /* Collection of nearby nodes slated for adoption. */
      std::vector<node *> near_nodes;
      /* Nodes we have seen before. These cannot be adopted */
      std::unordered_set<long> visited;

      double dist;

      /* Seed our adoption process with the given orphan */
      node_queue.push(orphan);
      visited.insert((long)orphan);

      /* Continue until no canidates for adoption remain */
      while (!node_queue.empty()) {
         orphan = node_queue.front(); node_queue.pop();
         near_nodes.clear();

         /* Consider the orphan to be adopted */
         orphan->orphan = false;
         orphanage.remove(orphan);
         nodes.add(orphan);

         /* Collect all nearby points */
         nodes.in_range(orphan->pos,dominance_region,&near_nodes);
         orphanage.in_range(orphan->pos,dominance_region,&near_nodes);

         for (node * current_node : near_nodes) {

            /* Check if the cost of making the current
             * orphan the parent of the current node is less
             * than it's previous parent. Note that this is
             * similiar to the get_node_parent function.    */
            dist = orphan->cost + Dist2{}(*orphan,*current_node);
            if (!is_collision(orphan->pos,current_node->pos) &&
                dist < current_node->cost) {

               /* Remove the swap parents and remove the old parent
                * if it now has no children.                       */
               if (current_node->parent) {
                  erase_from_vector(current_node->parent->children,current_node);
                  if (current_node->parent->children.empty())
                     infertile.insert((long)current_node->parent);
               }

               /* Node is no longer infertile */
               if (orphan->children.empty())
                  infertile.erase((long)orphan);

               current_node->parent = orphan;
               orphan->children.push_back(current_node);
               current_node->cost = dist;
               make_relative_to_parent(current_node);

               /* If the current node is an orphan and we have not
                * already seen it, add it to the list of nodes to
                * perform the same operations on.                 */
               if (current_node->orphan && !visited.count((long)current_node)) {
                  node_queue.push(current_node); 
                  visited.insert((long)current_node);
               }
            }
         }
      }
   }

   /* Collect a node that is infertile to sacrifice
    * to make room for more nodes. This is inefficiant,
    * but every time I go over it this is the best I 
    * can think of.                                    */
   inline node * get_sacrifice() {
      int idx = rand()%infertile.size();
      auto it = infertile.begin();
      for (int i = 0; i < idx; ++i, ++it);
      return (node *)*it;
   }

   /* The inverse of the adopt_node call basically. If
    * we orphan a node, all of it's children are blocked
    * and will now be orphaned.                         */
   inline void propagate_orphanhood(node * originator) {
      /* seed with our originator */
      node_queue.push(originator);

      while (!node_queue.empty()) {
         originator = node_queue.front(); node_queue.pop();
         if (originator->orphan)
            continue;

         /* Set our node as an orphan and
          * fill out the appropriate values */
         originator->orphan = true;
         nodes.remove(originator);
         orphanage.add(originator);
         originator->parent = NULL;
         originator->cost = DBL_MAX;

         /* propigate to children */
         for (node * child : originator->children)
            node_queue.push(child);
         originator->children.clear();

         /* this node is now infertile */
         infertile.insert((long)originator);
      }
   }

   /* empty all values from our tree
    * and restart.                  */
   void reload() {

      nodes.clear();
      orphanage.clear();
      infertile.clear();

      node * start = new node{goal,0.0,std::vector<node *>(),NULL,false};
      nodes.add(start);
      infertile.insert((long)start);
      node_count = 0;

      min_len.x = max_len.x = start->pos.x;
      min_len.y = max_len.y = start->pos.y;
      sample_offset.x = start->pos.x;
      sample_offset.y = start->pos.y;
   }

   /* Construct 100 more nodes. */
   void generate_next() {

      /* set up our values for the sample space
       * on this iteration.                    */
      x_len = (max_len.x-min_len.x) + 20;
      x_off = x_len / 2.0; 
      y_len = (max_len.y-min_len.y) + 20;
      y_off = y_len / 2.0;

      for (int i = 0; i < 100; ++i) {

         /* Do all the math for getting a new node position */
         point random_position = point{(((double)std::rand()/RAND_MAX)*x_len) + sample_offset.x - x_off,
                                       (((double)std::rand()/RAND_MAX)*y_len) + sample_offset.y - y_off};
         node * nearest = nodes.closest(random_position);

         point new_position = point{random_position.x - nearest->pos.x,
                                    random_position.y - nearest->pos.y};
         double length = sqrt(new_position.x*new_position.x + new_position.y*new_position.y);
         new_position = point{new_position.x / length, new_position.y / length };
         new_position.x *= expansion_length; new_position.y *= expansion_length;
         new_position = point{new_position.x + nearest->pos.x, new_position.y + nearest->pos.y};

         /* collect nearby nodes */
         std::vector<node *> near_nodes;
         nodes.in_range(new_position,dominance_region,&near_nodes);

         /* get the parent from these nodes */
         node * proposed_parent = get_node_parent(new_position,&near_nodes);

         if (proposed_parent) {

            /* update the bounding values as nessesary */
            if (new_position.x < min_len.x)
               min_len.x = new_position.x;
            else if (new_position.x > max_len.x)
               max_len.x = new_position.x;
            if (new_position.y < min_len.y)
               min_len.y = new_position.y;
            else if (new_position.y > max_len.y)
               max_len.y = new_position.y;

            /* construct the new node */
            node * new_node = new node{new_position,
                                       proposed_parent->cost + Dist2{}(new_position,*proposed_parent),
                                       std::vector<node *>(),
                                       proposed_parent,
                                       false};
            make_relative_to_parent(new_node);

            /* parent is no longer infertile */
            if (proposed_parent->children.empty())
               infertile.erase((long)proposed_parent);

            /* update appropriate values */
            proposed_parent->children.push_back(new_node);
            nodes.add(new_node);
            infertile.insert((long)new_node);
            ++node_count;

            /* queue up orphans for collection and 
             * rewire our neighbors in accordance with RRTX */
            orphanage.in_range(new_position,dominance_region,&near_nodes);
            rewire_neighbors(new_node,&near_nodes);
         }

         /* Sacrifice a node if we have hit the limit */
         if (node_count > node_limit) {
            node * sacrifice = get_sacrifice();
            infertile.erase((long)sacrifice);

            if (sacrifice->parent) {
               erase_from_vector(sacrifice->parent->children,sacrifice);
               nodes.remove(sacrifice);

               if (sacrifice->parent->children.empty())
                  infertile.insert((long)sacrifice->parent);
            }
            else
               orphanage.remove(sacrifice);

            delete sacrifice;
            --node_count;
         }
      }
   }

   /* function for updating nodes based on the 
    * presense of a new obstacle.             */
   void notify_obstacle(const point & position) {
      /* collect nodes in the cull range */
      std::vector<node *> near_nodes;
      nodes.in_range(position,cull_range,&near_nodes);

      /* remove nodes that collide, and orphan 
       * the children they have.              */  
      for (node * current_node : near_nodes) {
         if (current_node->parent &&
             is_collision(current_node->parent->pos,current_node->pos)) {

            erase_from_vector(current_node->parent->children,current_node);

            if (current_node->parent->children.empty())
               infertile.insert((long)current_node->parent);

            current_node->parent = NULL;

            propagate_orphanhood(current_node);
         }
      }

      /* Collect orphans in range. We are using a forward_list
       * to avoid running into integer invalidation.          */
      orphanage.in_range(position,cull_range,&near_nodes);
      std::forward_list<node *> near_list = std::forward_list<node *>(near_nodes.begin(),near_nodes.end());

      /* Remove all culled nodes and orphans in
       * range from the collected list.        */
      node * current_node;
      while (!near_list.empty()) {
         current_node = near_list.front(); near_list.pop_front();
         if (current_node->orphan) {
            orphanage.remove(current_node);
            near_list.remove(current_node);
            infertile.erase((long)current_node);
            delete current_node;
         }
      }
   }

   /* Return the closest node to a point */
   node * closest(const point & position) {
      return nodes.closest(position);
   }

   /* generate a new random point in the environment */
   inline const point generate_random() const {
      return point{(((double)std::rand()/RAND_MAX)*x_len) + sample_offset.x - x_off,
                   (((double)std::rand()/RAND_MAX)*y_len) + sample_offset.y - y_off};
   }

protected:

   /* Does nothing here, but can be overriden to influence
    * a node's relationship with it's parent.             */
   virtual inline void make_relative_to_parent(node * n) {
      (void)n;
   }

   /* use the given map and the bresenham collision algorithm 
    * to determine if this edge is colliding with the map. */
   virtual inline bool is_collision(const point & p1, const point & p2) {

      /* get the endpoints of our line */
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

public:

   RRTX_FN(double collision_radius, double dominance_region, double cull_range,
           double point_set_divisors, double expansion_length, int node_limit,
           int generation_tick_speed) 
      : PathfinderBase(collision_radius), dominance_region(dominance_region), cull_range(cull_range),
        expansion_length(expansion_length), node_limit(node_limit), 
        node_count(0), generation_tick_speed(generation_tick_speed),
        nodes(SpatialBin<point,node,decltype(dist2),decltype(coord_collector)>(point_set_divisors)),
        orphanage(SpatialBin<point,node,decltype(dist2),decltype(coord_collector)>(point_set_divisors)) {

      tree_handler = std::thread(&RRTX_FN::grow_tree,this);
   }

   RRTX_FN(double collision_radius)
      : PathfinderBase(collision_radius),
        nodes(SpatialBin<point,node,decltype(dist2),decltype(coord_collector)>(0)),
        orphanage(SpatialBin<point,node,decltype(dist2),decltype(coord_collector)>(0)) {};

   ~RRTX_FN() {
      nodes.clear(SpatialBin<point,node,decltype(dist2),decltype(coord_collector)>::SpatialBinClear_delete);
      orphanage.clear(SpatialBin<point,node,decltype(dist2),decltype(coord_collector)>::SpatialBinClear_delete);
   }

   nav_msgs::msg::Path get_path() override {
      nav_msgs::msg::Path ret;

      /* cannot generate a path with no nodes */
      if (nodes.bin_list.empty())
         return ret;

      /* take hold of the tree */
      mut.lock();

      /* get the closest point to our current position */
      node * nearest = closest(origin);

      /* if we cannot reach that point, then the RRT
       * path generation fails and we return NULL   */
      if (RRTX_FN::is_collision(nearest->pos,origin)) {
         mut.unlock();
         return ret;
      }

      /* collect poses back the the tree
       * root, which is the goal position */
      while (nearest) {

         geometry_msgs::msg::PoseStamped pose;
         pose.pose.position.x = (nearest->pos.x * grid_metadata.resolution) + grid_metadata.origin.position.x;
         pose.pose.position.y = (nearest->pos.y * grid_metadata.resolution) + grid_metadata.origin.position.y;

         ret.poses.push_back(pose);

         nearest = nearest->parent;
      }

      /* release the tree */
      mut.unlock();

      return ret;
   }

   void set_goal(const geometry_msgs::msg::Pose & pose) override {
      PathfinderBase::set_goal(pose);

      /* take hold of the tree */
      mut.lock();

      /* reset the entire tree. This is nessesary as
       * we are changing the root node.             */
      reload();

      /* release the tree */
      mut.unlock();
   }

   void grow_tree() {

      while (true) {

         if (!no_goal) {

            /* take hold of the tree */
            mut.lock();

            /* generate some more nodes */
            generate_next();

            /* release the tree */
            mut.unlock();

         }

         std::this_thread::sleep_for(std::chrono::milliseconds(generation_tick_speed));
      }
   }

   void obstacle_callback(int x, int y) {
      /* compute the position of the center of this square in the map */
      point relative_pos = { ((double)x + 0.5),
                             ((double)y + 0.5) };
      mut.lock();
      notify_obstacle(relative_pos);
      mut.unlock();
   }

   void load_map(const nav_msgs::msg::OccupancyGrid & map, int occupant_cutoff) override {
      sample_area.x = map.info.width;
      sample_area.y = map.info.height;
      sample_offset.x = sample_area.x / 2.0;
      sample_offset.y = sample_area.y / 2.0;

      auto func = [this] (int x, int y) -> void { this->obstacle_callback(x,y); };

      PathfinderBase::load_map(map,occupant_cutoff,&func);
   }

   void draw_environment(sf::RenderWindow * window) override {
      /* draw the map */
      PathfinderBase::draw_environment(window);

      /* take hold of the tree */
      mut.lock();

      /* draw all of the edges in the tree */
      sf::Vertex points[2];
      for (SpatialBin<point,node,Dist2,CoordinateCollector>::bin * b : nodes.bin_list) {
         for (node * n : b->values) {
            if (n->parent) {
               points[0].position = sf::Vector2f(n->parent->pos.x*10,n->parent->pos.y*10);
               points[1].position = sf::Vector2f(n->pos.x*10,n->pos.y*10);
               window->draw(points,2,sf::Lines);
            }
         }
      }

      /* draw the closest path to the robot's position */
      if (!nodes.bin_list.empty()) {
         node * start = nodes.closest(origin);
         points[0].color = (sf::Color(0,255,255,255));
         points[1].color = (sf::Color(0,255,255,255));
         while (start) {
            if (start->parent) {
               points[0].position = sf::Vector2f(start->pos.x*10,start->pos.y*10);
               points[1].position = sf::Vector2f(start->parent->pos.x*10,start->parent->pos.y*10);
               window->draw(points,2,sf::Lines);
            }
            start = start->parent;
         }
      }

      /* release the tree */
      mut.unlock();
   }
   
};

#endif
