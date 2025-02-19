
#ifndef __OBSERVATION

#define __OBSERVATION
#include "kurome.h"
#include "lidar_data.hpp"

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

/* An observation obtained from sensors on the robot, coupled
 * with an associated covariance and preferably some odometry
 * information. Used to contruct an edge or update of some 
 * kind in the slam system being used.                       */
class Observation {
public:

   /* usually collected from something like odometry */
   pose_2d global_pose_estimate;
   Covariance3 global_pose_covariance;

   /* usually collected from something like a lidar */
   LidarData laser_scan;
   pose_2d laser_scan_pose_estimate;
   Covariance3 laser_scan_covariance;

   /* usually collected from some kind of depth camera or lidar */
   sensor_msgs::msg::PointCloud2 point_cloud;
   pose_2d point_cloud_pose_estimate;
   Covariance3 point_cloud_covariance;

   Observation() {};

};

#endif
