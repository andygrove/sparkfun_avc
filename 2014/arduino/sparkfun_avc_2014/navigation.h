#ifndef __NAVIGATION_H_
#define __NAVIGATION_H_

class Location {
public:
  float longitude, latitude;
  
public:
  /** regular constructor */
  Location(float _longitude, float _latitude);
  
  /** constructor for Google Maps KVL export */
  Location(float _longitude, float _latitude, float _ignore);
  
  /** update */
  void set(float _longitude, float _latitude);
};


class Navigation {
public:  
  float convert_to_decimal_degrees(float f);
  double calculate_difference(double l1, double l2);
  float calculate_compass_bearing(double x, double y);
  double calc_bearing_diff(double current_bearing, double required_bearing);
};

#endif __NAVIGATION_H_
