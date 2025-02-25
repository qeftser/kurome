
#ifndef __OBSERVATION

#define __OBSERVATION
#include "kurome.h"
#include <Eigen/Dense>
#include "lidar_data.hpp"
#include "point_cloud_data.hpp"

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

      A /= det; B /= det; C /= det; 
      E /= det; F /= det; I /= det;

      Eigen::Matrix3d inverse;
      inverse << A, B, C,
                 B, E, F,
                 C, F, I;

      Eigen::Matrix3d sqrt_inverse = inverse.llt().matrixU();

      return Information3{sqrt_inverse(0,0),sqrt_inverse(1,0),sqrt_inverse(2,0),
                          sqrt_inverse(1,1),sqrt_inverse(2,1),sqrt_inverse(2,2)};
   }

};

/* An observation obtained from sensors on the robot, coupled
 * with an associated covariance and preferably some odometry
 * information. Used to contruct an edge or update of some 
 * kind in the slam system being used.                       */
class Observation {
public:

   /* used for computing the map -> odom transform
    * later on. Usually not touched much.         */
   pose_2d current_odometry = {{0,0},0};

   /* best estimate of the pose of the robot */
   pose_2d global_pose_estimate = {{0,0},0};
   Covariance3 global_pose_covariance;

   /* usually collected from something like a lidar */
   LidarData laser_scan;

   /* usually collected from some kind of depth camera or lidar */
   PointCloudData point_cloud;

   Observation() {};

   /* absorb another observation into this one */
   void aggregate(const Observation & other) {

      /* compute the offset of the next observation
       * to this one.                              */
      pose_2d offset = {{other.global_pose_estimate.pos.x - global_pose_estimate.pos.x,
                         other.global_pose_estimate.pos.y - global_pose_estimate.pos.y},
                         other.global_pose_estimate.theta - global_pose_estimate.theta };
      offset.theta = atan2(sin(offset.theta),cos(offset.theta));

      /* merge the LiDAR and point cloud data from other with
       * this Observation. Will work even if these fields are empty */
      laser_scan.insert_at_offset(other.laser_scan,offset);
      point_cloud.insert_at_offset(other.point_cloud,offset);

   }

};

#endif
