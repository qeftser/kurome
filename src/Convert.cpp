
#include "Convert.h"

double Convert::feetToMeters(double feet) {
   return feet * METER_PER_FEET;
}

double Convert::metersToFeet(double meters) {
   return meters * FEET_PER_METER;
}
