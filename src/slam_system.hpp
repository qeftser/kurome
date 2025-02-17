
#ifndef __SLAM_SYSTEM

#define __SLAM_SYSTEM
#include "kurome.h"

/* as it sounds. A wrapper over whatever graph slam
 * backend is used in the final system. Similar to the
 * pathfinder and smoother base classes in that it defines
 * a loose outline of how the system is expected to work
 * and leaves the rest to the child classes. Also include some
 * small classes here that will be useful in the other 
 * functions related to the slam system.                     */

class Information3 {
public:

   double xx;
   double xy;
   double xz;
   double yy;
   double yz;
   double zz;

};

class Covariance3 {
public:

   double xx;
   double xy;
   double xz;
   double yy;
   double yz;
   double zz;

   /* 3x3 matrix inverse */
   Information3 to_information() {

      double A =  (yy*zz - yz*yz);
      double B = -(xy*zz - yz*xz);
      double C =  (xy*yz - yy*xz);
      double E =  (xx*zz - xz*xz);
      double F = -(xx*yz - xy*xz);
      double I =  (xx*yy - xy*xy);

      double det = xx*A + xy*B + xy*C;

      return Information3{A/det,B/det,C/det,E/det,F/det,I/det};
   }

};

class SlamSystem {

public:

   static const bool FIX_NODE = true;
   static const bool NOFIX_NODE = false;


   virtual void insert_observation(geometry_msgs::msg::Pose & pose, bool fixed = NOFIX_NODE) = 0;
   /* ... other insert_observation calls with the appropriate data go here ... */

   virtual void construct_visualization(visualization_msgs::msg::MarkerArray & msg) = 0;


};

#endif
