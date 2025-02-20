
#ifndef __POINT_CLOUD_DATA

#define __POINT_CLOUD_DATA
#include "kurome.h"
#include <cmath>
#include <cstring>

/* An abstraction over the PointCloud2 message ros2
 * gives us. This data is given in 3d coordinates
 * and has some nice functions that make it eaiser
 * to manipulate.                                  */
class PointCloudData {
public:

   /* Origin of the points in the cloud */
   pose_2d center = {{0,0},0};

   /* The points in the cloud */
   std::vector<point3> points;

   PointCloudData() {};

   /* Construct an instance of PointCloudData from an
    * associated ros2 PointCloud2 message type. Assume
    * center of 0,0.                                  */
   PointCloudData(const sensor_msgs::msg::PointCloud2 & msg) {

      int entries = (msg.width * msg.height) / msg.point_step;

      int x_off = -1, x_size;
      int y_off = -1, y_size;
      int z_off = -1, z_size;

      for (const sensor_msgs::msg::PointField & field : msg.fields) {
         if (field.name == "x") {
            x_off = field.offset;
            x_size = (field.datatype < 3 ? 1 : (field.datatype < 5 ? 2 : (field.datatype < 8 ? 4 : 8)));
         }
         else if (field.name == "y") {
            y_off = field.offset;
            y_size = (field.datatype < 3 ? 1 : (field.datatype < 5 ? 2 : (field.datatype < 8 ? 4 : 8)));
         }
         else if (field.name == "z") {
            z_off = field.offset;
            z_size = (field.datatype < 3 ? 1 : (field.datatype < 5 ? 2 : (field.datatype < 8 ? 4 : 8)));
         }
      }

      points = std::vector<point3>(entries);

      /* we have all the data fields we expect, proceed normally */
      if (x_off >= 0 && y_off >= 0 && z_off >= 0) {
         for (size_t i = 0; i < msg.data.size(); i += msg.point_step) {

            point3 new_point;

            memcpy(&new_point.x,&msg.data[i+x_off],x_size); /* x value */
            memcpy(&new_point.y,&msg.data[i+y_off],y_size); /* y value */
            memcpy(&new_point.z,&msg.data[i+z_off],z_size); /* z value */

            points.push_back(new_point);

         }
      }
      /* we need to check before adding each field */
      else {
         for (size_t i = 0; i < msg.data.size(); i += msg.point_step) {

            point3 new_point;

            if (x_off != -1)
               memcpy(&new_point.x,&msg.data[i+x_off],x_size);
            else
               new_point.x = 0.0;

            if (y_off != -1)
               memcpy(&new_point.y,&msg.data[i+y_off],y_size);
            else
               new_point.y = 0.0;

            if (z_off != -1)
               memcpy(&new_point.z,&msg.data[i+z_off],z_size);
            else
               new_point.z = 0.0;

         }
      }

   }

   /* add another PointCloudData object's points into this one
    * at the specified offset from the center.                */
   void insert_at_offset(const PointCloudData & other, pose_2d offset) {

      for (size_t i = 0; i < other.points.size(); ++i) {

         point3 updated = other.points[i];
         updated.pos_2d = transform(other.points[i].pos_2d,offset);

         points.push_back(updated);
      }
      
   }
   void insert_at_offset(const PointCloudData & other, geometry_msgs::msg::Pose & offset) {
      pose_2d as_pose_2d = ros2_pose_to_pose_2d(offset);
      insert_at_offset(other,as_pose_2d);
   }

};

#endif
