
#ifndef __SPATIAL_BIN

/* A very nice data structure for locating the closest
 * and in-range points to a given location. This is a
 * weird data structure, in that it needs coordinates of
 * x,y positions to do the binning. Because of this, the
 * required template functions will be one for returning
 * distance and another for returning the x and y values
 * of the object being stored.                           */

#define __SPATIAL_BIN
#include <vector>
#include <cfloat>
#include <unordered_map>
#include <cstdlib>
#include <cmath>
#include "kurome.h"

/* The type K is the types we will use when attempting to
 * get points from our system. The type V is the type we
 * are storing in the bins. Dist is a class that impliments
 * the operator() that takes a value of type K and one of
 * type v in a call Dist(K,V) and returns the distance 
 * between them. getXY should have two implimentations of
 * operator(), one which takes a value as a first parameter
 * and sets double values x and y, which are the second and
 * third parameters. The form is: GetXY{}(V,&x,&y). We will
 * need an identical function to this that works on 
 * the key: GetXY{}(K,&x,&y)                               */
template <typename K, typename V, class Dist, class GetXY>
class SpatialBin {
public:

   struct bin {
      std::vector<V *> values;
   };

private:

   /* Mapping of x and y coordinates for bins to
    * a given bin.                              */
   std::unordered_map<long,bin *> bins;
   /* What value should we divide our x and y
    * by to place then in a bin?             */
   double divisor;

public:

   /* List of all the bins we have created.
    * Used when freeing the memory in the
    * structure.                          */
   std::vector<bin *> bin_list;

   /* Constructor. Refer to divisor private
    * variable.                             */
   SpatialBin(double divisor) : divisor(divisor) {}

   V * closest(const K & k) {
      return closest_in_range(k,0);
   }

   V * closest_in_range(const K & k, int range_limit) {

      V * closest_value = NULL;
      double min_distance = DBL_MAX;

      /* Collect the x and y positions and 
       * convert them to the bin position. */
      double x_d, y_d;
      int x, y;
      GetXY{}(k,&x_d,&y_d);
      x = (int)(x_d / divisor);
      y = (int)(y_d / divisor);

      /* If there is no bin at the given position,
       * we need to search outward for the bin.   */
      if (!bins.count(long_from_ints(x,y))) {
         /* The current range to search */
         int range = 1;

         /* Contine until we find a value */
         while (range - range_limit) {

            /* Compute the upper and lower
             * edges we are searching. This works
             * by searching an expanding radius 
             * around the bin we land in. These are
             * the current corners of that radius  */
            int y_low  = y - range;
            int y_high = y + range;
            int x_low  = x - range;
            int x_high = x + range;

            /* check the x edges */
            for (int x_pos = x_low; x_pos <= x_high; ++x_pos) {
               /* check the lower x edge */
               if (bins.count(long_from_ints(x_pos,y_low))) {
                  bin * b = bins.at(long_from_ints(x_pos,y_low));
                  /* get the lowest value in the bin */
                  for (V * v : b->values) {
                     if (Dist{}(k,*v) < min_distance) {
                        min_distance = Dist{}(k,*v);
                        closest_value = v;
                     }
                  }
               }
               /* check the higher x edge */
               if (bins.count(long_from_ints(x_pos,y_high))) {
                  bin * b = bins.at(long_from_ints(x_pos,y_high));
                  /* get the lowest value in the bin */
                  for (V * v : b->values) {
                     if (Dist{}(k,*v) < min_distance) {
                        min_distance = Dist{}(k,*v);
                        closest_value = v;
                     }
                  }
               }
            }
            /* check the y edges. We don't need
             * to check the corners because we 
             * already did with the x edges    */
            for (int y_pos = y_low + 1; y_pos <= y_high - 1; ++y_pos) {
               /* check the lower y edge */
               if (bins.count(long_from_ints(x_low,y_pos))) {
                  bin * b = bins.at(long_from_ints(x_low,y_pos));
                  /* get the lowest value in the bin */
                  for (V * v : b->values) {
                     if (Dist{}(k,*v) < min_distance) {
                        min_distance = Dist{}(k,*v);
                        closest_value = v;
                     }
                  }
               }
               /* check the higher y edge */
               if (bins.count(long_from_ints(x_high,y_pos))) {
                  bin * b = bins.at(long_from_ints(x_high,y_pos));
                  /* get the lowest value in the bin */
                  for (V * v : b->values) {
                     if (Dist{}(k,*v) < min_distance) {
                        min_distance = Dist{}(k,*v);
                        closest_value = v;
                     }
                  }
               }
            }
            /* if we have found an edge in this
             * stage of expansion, it is the
             * closest edge we will find. Go
             * with it and exit.               */
            if (closest_value != NULL) {
               return closest_value;
            }
            /* increase the range of 
             * the search if we didn't
             * find anything.         */
            ++range;
         }

         /* we capped out the range limit and
          * no values were found, return NULL */
         return NULL;
      }
      /* Otherwise, we only need to check 
       * the bin we map to and some of the
       * bins surrounding it.             */
      else {

         /* determine which corner we are in 
          * inside the bin we have been dropped
          * in. This will narrow down the surrouding
          * bins we need to consider.                */
         int selector = (x_d - (x*divisor)+(divisor/2.0) < 0.5 ? 0 : 1) |
                        (y_d - (y*divisor)+(divisor/2.0) < 0.5 ? 0 : 2);

         /* Search the appropriate bins based on our
          * position in our bin.                    */
         switch (selector) {
            /* we are in the bottom
             * left corner, check bins
             * below and to the left of us */
            case 0:
            {
               for (int x_pos = x-1; x_pos <= x; ++x_pos) {
                  for (int y_pos = y-1; y_pos <= y; ++y_pos) {
                     if (bins.count(long_from_ints(x_pos,y_pos))) {
                        for (V * v : bins.at(long_from_ints(x_pos,y_pos))->values) {
                           if (Dist{}(k,*v) < min_distance) {
                              min_distance = Dist{}(k,*v);
                              closest_value = v;
                           }
                        }
                     }
                  }
               }
            }
            break;
            /* we are in the bottom
             * right corner, check bins
             * below and to the right of us */
            case 1:
            {
               for (int x_pos = x; x_pos <= x+1; ++x_pos) {
                  for (int y_pos = y-1; y_pos <= y; ++y_pos) {
                     if (bins.count(long_from_ints(x_pos,y_pos))) {
                        for (V * v : bins.at(long_from_ints(x_pos,y_pos))->values) {
                           if (Dist{}(k,*v) < min_distance) {
                              min_distance = Dist{}(k,*v);
                              closest_value = v;
                           }
                        }
                     }
                  }
               }
            }
            break;
            /* we are in the top
             * left corner, check bins
             * above and to the right of us */
            case 2:
            {
               for (int x_pos = x-1; x_pos <= x; ++x_pos) {
                  for (int y_pos = y; y_pos <= y+1; ++y_pos) {
                     if (bins.count(long_from_ints(x_pos,y_pos))) {
                        for (V * v : bins.at(long_from_ints(x_pos,y_pos))->values) {
                           if (Dist{}(k,*v) < min_distance) {
                              min_distance = Dist{}(k,*v);
                              closest_value = v;
                           }
                        }
                     }
                  }
               }
            }
            break;
            /* we are in the top
             * right corner, check bins
             * above and to the right of us */
            case 3:
            {
               for (int x_pos = x; x_pos <= x+1; ++x_pos) {
                  for (int y_pos = y; y_pos <= y+1; ++y_pos) {
                     if (bins.count(long_from_ints(x_pos,y_pos))) {
                        for (V * v : bins.at(long_from_ints(x_pos,y_pos))->values) {
                           if (Dist{}(k,*v) < min_distance) {
                              min_distance = Dist{}(k,*v);
                              closest_value = v;
                           }
                        }
                     }
                  }
               }
            }
            break;
         }
         /* Because at least one of these
          * bins was occupied, we know we
          * will return a value. This one 
          * will be the closest, provided
          * this bin segmentation strategy
          * is correct.                   */
         return closest_value;
      }
   }

   void in_range(const K & k, double range, std::vector<V *> * results) {

      /* get the central bin */
      double x_d, y_d;
      int x, y;
      GetXY{}(k,&x_d,&y_d);
      x = x_d / divisor;
      y = y_d / divisor;

      /* find the farthest bin we need to search
       * given the requested range.             */
      int bin_range = (int)ceil(range / divisor);

      /* search all bins in the range for values in the range */
      for (int x_pos = x - bin_range; x_pos <= x + bin_range; ++x_pos) {
         for (int y_pos = y - bin_range; y_pos <= y + bin_range; ++y_pos) {

            if (bins.count(long_from_ints(x_pos,y_pos))) {
               for (V * v : bins.at(long_from_ints(x_pos,y_pos))->values) {

                  if (Dist{}(k,*v) <= range) {
                     results->push_back(v);
                  }

               }
            }

         }
      }
   }

   void add(V * value) {

      /* get the bin to add to */
      double x_d, y_d;
      int x, y;
      GetXY{}(*value,&x_d,&y_d);
      x = x_d / divisor;
      y = y_d / divisor;

      /* If there is no bin where we are,
       * create the bin.                 */
      if (!bins.count(long_from_ints(x,y))) {
         bin * new_bin = new bin{std::vector<V *>()};
         bin_list.push_back(new_bin);
         bins.insert({long_from_ints(x,y),new_bin});
      }

      /* add the element to the bin */
      bins.at(long_from_ints(x,y))->values.push_back(value);
   }

   void remove(V * value) {

      /* get the bin to add to */
      double x_d, y_d;
      int x, y;
      GetXY{}(*value,&x_d,&y_d);
      x = x_d / divisor;
      y = y_d / divisor;

      /* get the associated bin */
      bin * b = bins.at(long_from_ints(x,y));

      /* remove the value from the bin */
      erase_from_vector(b->values,value);

      /* if the bin is empty, remove it as well */
      if (b->values.empty()) {
         erase_from_vector(bin_list,b);
         bins.erase(long_from_ints(x,y));
         delete b;
      }
   }

   /* Clear the spatial bin. If the first boolean is set
    * we also delete all the values that were added. If
    * the second boolean is set, we additionally use 
    * free to instead of delete to free the pointer memory. */
   enum SpatialBinClearParam { SpatialBinClear_default, SpatialBinClear_delete, SpatialBinClear_free };

   void clear(SpatialBinClearParam param = SpatialBinClear_default) {

      switch(param) {
         /* delete all bins but not  
          * values as requested    */
         case SpatialBinClear_default:
         {
            for (bin * b : bin_list)
               delete b;
         }
         break;
         /* delete all bins and   
          * values as requested    */
         case SpatialBinClear_delete:
         {
            for (bin * b : bin_list) {
               for (V * v : b->values)
                  delete v;
               delete b;
            }

         }
         break;
         /* delete all bins, free
          * all values as requested */
         case SpatialBinClear_free:
         {
            for (bin * b : bin_list) {
               for (V * v : b->values)
                  free(v);
               delete b;
            }
         }
         break;
      }

      /* clear out the associated 
       * data structures */
      bins.clear();
      bin_list.clear();
   }

};

#endif
