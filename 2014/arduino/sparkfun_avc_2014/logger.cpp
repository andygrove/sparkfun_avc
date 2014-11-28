#include <arduino.h>
#include "logger.h"
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include "RTClib.h"
 
Logger::Logger() {
}

String Logger::format(int n) {
  String s = String(n);
  if (n < 10) {
    s = "0" + s;
  }
  return s;
} 

void Logger::init(String filename, DateTime now)
{
  counter = 0;
  
  Serial.print("Initializing SD card...");
  // make sure that the default chip select pin is set to
  // output, even if you don't use it:
  pinMode(SS, OUTPUT);
  
  // see if the card is present and can be initialized:
  if (!SD.begin(10,11,12,13)) {
    Serial.println("Card failed, or not present");
    return;
  }
  
  Serial.print("Card initialized. Writing to file '");
  Serial.print(filename);
  Serial.println("'");
  
  if (filename.length() != 12) {
    Serial.println("Invalid filename length");
    return;
  }
  
  
  // Open up the file we're going to log to!
  char filenameChars[12];
  filename.toCharArray(filenameChars, 12);
  dataFile = SD.open(filenameChars, FILE_WRITE);
  if (!dataFile) {
    Serial.print("Error opening ");
    Serial.println(filename);
    return;
  }
  
  ok = true;
  
  dataFile.print(millis());
  dataFile.print(",");
  dataFile.print("CURRENT_DATE,");
  dataFile.print(now.year());
  dataFile.print("-");
  dataFile.print(format(now.month()));
  dataFile.print("-");
  dataFile.print(format(now.day()));
  dataFile.print(" ");
  dataFile.print(format(now.hour()));
  dataFile.print(":");
  dataFile.print(format(now.minute()));
  dataFile.print(":");
  dataFile.println(format(now.second()));
  dataFile.flush();
}

void Logger::write_destination(int waypoint_number, int max_number, float latitude, float longitude) {
  if (!ok) return;
  dataFile.print(millis());
  dataFile.print(",");
  dataFile.print("NEXT_WAYPOINT,");
  dataFile.print(waypoint_number);
  dataFile.print(",");
  dataFile.print(max_number);
  dataFile.print(",");
  dataFile.print(latitude, 6);
  dataFile.print(",");
  dataFile.print(longitude, 6);
  dataFile.println();

  // always flush on new destination
  flush();
}

void Logger::write_gps(float latitude, float longitude) {
  if (!ok) return;
  dataFile.print(millis());
  dataFile.print(",");
  dataFile.print("GPS,");
  dataFile.print(latitude, 6);
  dataFile.print(",");
  dataFile.print(longitude, 6);
  dataFile.println();
  count();
}

void Logger::write_compass_bearing(float bearing, float angle_diff) {
  if (!ok) return;
  dataFile.print(millis());
  dataFile.print(",");
  dataFile.print("COMPASS,");
  dataFile.print(bearing);
  dataFile.print(",");
  dataFile.print(angle_diff);
  dataFile.println();
  count();
}

void Logger::write_navigation_data(float offset_lat, float offset_long, float required_bearing) {
  if (!ok) return;
  dataFile.print(millis());
  dataFile.print(",");
  dataFile.print("NAVIGATION,");
  dataFile.print(offset_lat, 6);
  dataFile.print(",");
  dataFile.print(offset_long, 6);
  dataFile.print(",");
  dataFile.print(required_bearing);
  dataFile.println();
  count();
}

void Logger::write_ultrasonic(int left, int front, int right) {
  if (!ok) return;
  dataFile.print(millis());
  dataFile.print(",");
  dataFile.print("ULTRASONIC,");
  dataFile.print(left);
  dataFile.print(",");
  dataFile.print(front);
  dataFile.print(",");
  dataFile.print(right);
  dataFile.println();
  count();
}

void Logger::write_motor_speed(int left, int right) {
  if (!ok) return;
  dataFile.print(millis());
  dataFile.print(",");
  dataFile.print("MOTOR_SPEED,");
  dataFile.print(left);
  dataFile.print(",");
  dataFile.print(right);
  dataFile.println();
  count();
}

void Logger::write_event(String event) {
  if (!ok) return;
  // debug events
  //Serial.print("EVENT,");
  //Serial.println(event);
  
  dataFile.print(millis());
  dataFile.print(",");
  dataFile.print("EVENT,");
  dataFile.println(event);
  
  // always flush on events
  flush();
}

void Logger::count() {
  counter++;
  if (counter > 1000) {
    flush();
  }
}

void Logger::flush() {
  if (!ok) return;
  //Serial.println("Flushing");
  dataFile.flush();
  counter = 0;
}
