
#ifndef CONVERT_HELPER

#define CONVERT_HELPER

#define FEET_PER_METER 3.28084 
#define METER_PER_FEET 0.30480

class Convert {
public:
   /* takes a double in feet and returns the value in meters */
   static double feetToMeters(double);
   /* takes a double in meters and returns the value in feet */
   static double metersToFeet(double);
};

#endif
