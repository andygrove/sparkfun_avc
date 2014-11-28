#include <arduino.h>
#include "navigation.h"

// regular constructor
Location::Location(float _longitude, float _latitude) {
  longitude = _longitude;
  latitude = _latitude;
}

// constructor for Google Maps KVL export
Location::Location(float _longitude, float _latitude, float _ignore) {
  longitude = _longitude;
  latitude = _latitude;
}
  
void Location::set(float _longitude, float _latitude) {
  longitude = _longitude;
  latitude = _latitude;
}

float  Navigation::convert_to_decimal_degrees(float f) {
  int d = (int) f/100;
  f -= d*100;
  return d + (f/60.0);
}



double Navigation::calc_bearing_diff(double current_bearing, double required_bearing) {
  double ret = required_bearing - current_bearing;
  if (ret < -180) {
    ret += 360;
  }
  else if (ret > 180) {
    ret -= 360;
  }

  /*
  Serial.print("required_bearing: ");
  Serial.print(required_bearing);
  Serial.print("; current_bearing: ");
  Serial.print(current_bearing);
  Serial.print("; diff=");
  Serial.println(ret);
  */

  return ret;
}

float Navigation::calculate_compass_bearing(double x, double y) {
  double ax = x>0 ? x : -x;
  double ay = y>0 ? y : -y;
  double radian = 180 / 3.141592;
  if (x>0) {
    if (y>0) {
      // 0 through 90
      if (ax>ay) {
        return 90 - radian * atan(ay/ax);
      }
      else {
        return radian * atan(ax/ay);
      }
    }
    else {
      if (ax>ay) {
        return 90 + radian * atan(ay/ax);
      }
      else {
        return 180 - radian * atan(ax/ay);
      }
    }
  }
  else {
    if (y>0) {
      if (ax>ay) {
        return 270 + radian * atan(ay/ax);
      }
      else {
        return 360 - radian * atan(ax/ay);
      }
    }
    else {
      if (ax>ay) {
        return 270 - radian * atan(ay/ax);
      }
      else {
        return 180 + radian * atan(ax/ay);
      }
    }
  }
}

/** calc difference between two DMS (degrees, minutes, seconds) numbers ... not needed if we use decimal degrees everywhere */
double Navigation::calculate_difference(double l1, double l2) {

  int d1 = l1 / 100;
  int d2 = l2 / 100;
  double m1 = l1 - (d1 * 100);
  double m2 = l2 - (d2 * 100);
  double ret = ((d1*60.0f)+m1) - ((d2*60.0f)+m2);

  return ret;
   
}

/*
void verify_bearing_diff_calc(double current_bearing, double required_bearing, double expected) {
  double actual = calc_bearing_diff(current_bearing, required_bearing);
  Serial.print("Calculated bearing diff from ");
  Serial.print(current_bearing);
  Serial.print(" to ");
  Serial.print(required_bearing);
  Serial.print(". Expected: ");
  Serial.print(expected);
  Serial.print(". Actual: ");
  Serial.print(actual);
  if (fabs(actual-expected) < 0.1) {
    Serial.print(". Correct.");
  }
  else {
    Serial.print(". INCORRECT.");
  }
}

void verify_bearing_diff_calcs() {
  verify_bearing_diff_calc(0, 10, 10);
  verify_bearing_diff_calc(350, 360, 10);
  verify_bearing_diff_calc(355, 5, 10);
  verify_bearing_diff_calc(5, 355, -10);
  verify_bearing_diff_calc(85, 90, 10);
  verify_bearing_diff_calc(265, 275, 10);
  verify_bearing_diff_calc(90, 265, 175);
  verify_bearing_diff_calc(90, 275, -175);
  verify_bearing_diff_calc(265, 90, -175);
  verify_bearing_diff_calc(275, 90, 175);
}


void confirm_compass_calc(double x, double y, double expected) {
  float actual = calculate_compass_bearing(x, y);
  Serial.print("Checking compass calculations for offset ");
  Serial.print(x);
  Serial.print(", ");
  Serial.print(y);
  Serial.print(". Expected: ");
  Serial.print(expected);
  Serial.print(". Actual: ");
  Serial.print(actual);
  if (abs(expected-actual) < 1) {
    Serial.println(". Correct.");
  }
  else {
    Serial.println(". INCORRECT.");
  }
}

void confirm_compass_calcs() {
  // 0 through 90 degrees
  confirm_compass_calc(0, 2, 0);
  confirm_compass_calc(1, 2, 27);
  confirm_compass_calc(2, 2, 45);
  confirm_compass_calc(2, 1, 63);
  // 90 through 180 degrees
  confirm_compass_calc(2, 0, 90);
  confirm_compass_calc(2, -1, 117);
  confirm_compass_calc(2, -2, 135);
  confirm_compass_calc(1, -2, 153);
  // 180 through 270 degrees
  confirm_compass_calc(0, -2, 180);
  confirm_compass_calc(-1, -2, 207);
  confirm_compass_calc(-2, -2, 225);
  confirm_compass_calc(-2, -1, 243);
  // 270 through 360 degrees
  confirm_compass_calc(-2, 0, 270);
  confirm_compass_calc(-2, 1, 297);
  confirm_compass_calc(-2, 2, 315);
  confirm_compass_calc(-1, 2, 333);
}



*/

