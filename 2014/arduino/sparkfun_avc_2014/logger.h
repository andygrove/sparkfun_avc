#ifndef __LOGGER_H_
#define __LOGGER_H_

#include <SD.h>
#include "RTClib.h"

class Logger {
private:
  boolean ok;
  File dataFile;
  int counter;
  void count();
  String format(int);
public:
  Logger();
  
  void init(String filename, DateTime now);
  void write_destination(int waypoint_number, int max_number, float latitude, float longitude);
  void write_gps(float latitude, float longitude);
  void write_compass_bearing(float bearing, float angle_diff);
  void write_navigation_data(float offset_lat, float offset_long, float required_bearing);
  void write_ultrasonic(int left, int front, int right);
  void write_motor_speed(int left, int right);
  void write_event(String event);
  void flush();
};

#endif __LOGGER_H_
