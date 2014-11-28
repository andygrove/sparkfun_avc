/*
 * Source code for Sparkun AVC 2014.
 *
 * Author: Andy Grove
 *
 * License: None. Feel free to use in your projects.
 */

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
#include <hmc6352.h>
#include <SPI.h>
#include <SD.h>
#include <PololuQik.h>

#include "sonic.h"
#include "navigation.h"
#include "logger.h"
#include "RTClib.h"

/* Motor controller */
PololuQik2s12v10 qik(50, 51, 52);
 
/* Use hardware interupts on Mega */
Adafruit_GPS GPS(&Serial1);

/* Real-time clock on the data logger shield */
RTC_DS1307 RTC;

/* Compass */
HMC6352 compass(0x42);

const int front_distance = 200;
const int side_distance = 50;
const int brake_amount_on_turn = 80; // 0 to 127
const int brake_delay_on_turn = 100; // how long to brake in ms
const int MIN_SPEED = 50;
const int MAX_SPEED = 255;
float sharp_turn_degrees = 20.0f; // degrees


/**
 * 0.00005 used in testing and works well.
 */
const float ACCURACY = 0.00004;


UltrasonicSensor uLeft (31);
UltrasonicSensor uFront(28);
UltrasonicSensor uRight(32);

Navigation nav;

Location currentLocation(0,0);

Logger logger;

// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences. 
#define GPSECHO  false

int gpsNoFixCounter = 0;

unsigned long last_gps_reading = 0;

double required_bearing = 0;

boolean gps_fix = false;

uint32_t timer = millis();

// this keeps track of whether we're using the interrupt
// off by default!
boolean usingInterrupt = false;
void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy


/* SparkFun AVC 2014 Course v1 */
/*
Location WAYPOINT[] = {
  Location(-105.23010313510895,40.07124397569352,0.0),  // corner 1
  Location(-105.22996366024017,40.070960723768074,0.0),
  Location(-105.22973835468292,40.070692864885324,0.0), // corner 2
  Location(-105.22939637303352,40.070768809656386,0.0),
  Location(-105.2291040122509,40.07090325221914,0.0),   // corner 3
  Location(-105.22929847240448,40.07112800675052,0.0),
  Location(-105.22947818040848,40.07140304781628,0.0),   // corner 4
  Location(-105.23010313510895,40.07124397569352,0.0),  // corner 1 again
};
*/

/* SparkFun AVC 2014 Course v2 */
/*
Location WAYPOINT[] = {
  // corner 1
  Location(-105.23010313510895,40.07124397569352,0.0),
  // mid point
  Location(-105.22996366024017,40.070960723768074,0.0),
  // corner 2
  Location(-105.22972762584686,40.07070107513485,0.0),
  // mid point
  Location(-105.22944331169128,40.070812939686974,0.0),
  // corner 3
  Location(-105.2291764318943,40.07091762011078,0.0),
  // mid point
  Location(-105.22931724786758,40.07111979655231,0.0),
  // corner 4
//  Location(-105.22946745157242,40.071429730845566,0.0), too tight
  Location(-105.22947818040848,40.07140304781628,0.0), // original settings
  // corner 1 again
  Location(-105.23010313510895,40.07124397569352,0.0),
  // would have been cool to turn and come back to the start...
};
*/
// v3

Location WAYPOINT[] = {
// Point 1
Location(-105.2300763130188,40.07125218587662,0.0),
// Point 2
Location(-105.22992745041847,40.07096585515385,0.0),
// Point 3
Location(-105.22973299026489,40.070736994965046,0.0),
// Point 4
Location(-105.22944331169128,40.070841675505505,0.0),
// Point 5
Location(-105.22910937666893,40.0709176201107,0.0),
// Point 6
Location(-105.2293749153614,40.07126757996722,0.0),
// Point 7
Location(-105.22946745157242,40.071429730845566,0.0),
// Point 1
Location(-105.2300763130188,40.07125218587662,0.0)
};

/* Basketball court */
/*
Location WAYPOINT[] = {
  Location(-105.08160397410393,39.94177796143009,0.0),
  Location(-105.08158653974533,39.94190648894768,0.0),
  Location(-105.08174613118172,39.94186741660787,0.0)
};
*/

/* Short test near house */
/*
Location WAYPOINT[] = {
  Location(-105.08186347782612,39.94053277434033,0.0),
  Location(-105.08191041648388,39.9407610431992,0.0),
  Location(-105.08186347782612,39.94053277434033,0.0)
};
*/

/*
Location WAYPOINT[] = {
  Location(-105.08185610175133,39.94051837898138,0.0),
  Location(-105.08189901709557,39.94059138398486,0.0),
  Location(-105.08191108703613,39.940708603123326,0.0),
  Location(-105.0819043815136,39.940860782056106,0.0),
  Location(-105.08187219500542,39.94094818196584,0.0),
  Location(-105.08179306983948,39.94102941237011,0.0),
  Location(-105.0816710293293,39.941086993357786,0.0),
  Location(-105.08141621947289,39.941167195366965,0.0),
  Location(-105.08132100105286,39.94123197384423,0.0),
  Location(-105.08127272129059,39.94131011922724,0.0),
  Location(-105.08125260472298,39.9414088290572,0.0),
  Location(-105.08125260472298,39.941682336800355,0.0),
  Location(-105.08125528693199,39.94180469517362,0.0),
  Location(-105.08126869797707,39.94188386812209,0.0),
  Location(-105.08131295442581,39.941964069197255,0.0),
  Location(-105.08121840655804,39.941987718214286,0.0),
  Location(-105.0811392813921,39.94202730459407,0.0),
  Location(-105.08106082677841,39.94206997560118,0.0),
  Location(-105.08089050650597,39.94217074076537,0.0),
  Location(-105.08083887398243,39.94219438971098,0.0),
  Location(-105.08077919483185,39.94220261542928,0.0),
  Location(-105.08054248988628,39.94218873452906,0.0),
  Location(-105.08043251931667,39.942188220421606,0.0),
  Location(-105.08010864257812,39.94220672828807,0.0),
  Location(-105.07997788488865,39.94220929882466,0.0),
  Location(-105.07985785603523,39.942204157751334,0.0),
  Location(-105.07968083024025,39.942189248636545,0.0),
  Location(-105.07953397929668,39.94218050880911,0.0),
  Location(-105.07947027683258,39.94217742416388,0.0)
};
*/


int wpIndex = 0;
float wpLat, wpLon;
int numLocations = sizeof(WAYPOINT)/sizeof(Location);

void get_next_waypoint() {

  if (wpIndex == numLocations) {
    qik.setSpeeds(0,0); //TODO: consider braking instead
    logger.write_event("COMPLETED COURSE!");
    while (1) {
      delay(1000);
    }
  }
  
  Location waypoint = WAYPOINT[wpIndex];
  wpIndex++;

  wpLat = waypoint.latitude;
  wpLon = waypoint.longitude;

  logger.write_destination(wpIndex, numLocations, waypoint.latitude, waypoint.longitude);
}  

String format(int n) {
  String s = String(n);
  if (n < 10) {
    s = "0" + s;
  }
  return s;
}


void setup() {
  
  Serial.begin(9600);
  Serial.println("setup()");
  
  // start communicating with the real time clock
  RTC.begin();
  
 if (! RTC.isrunning()) {
    Serial.println("RTC is NOT running!");
    // following line sets the RTC to the date & time this sketch was compiled
    // uncomment it & upload to set the time, date and start run the RTC!
    RTC.adjust(DateTime(__DATE__, __TIME__));
  }
  
  // get the current time
  DateTime now = RTC.now();
  
  // make file name out of date and time in format MMDDHHMM.LOG
  String filename = format(now.month()) + format(now.day()) + format(now.hour()) + format(now.minute()) + ".LOG";
  
  Serial.print("Logging to ");
  Serial.println(filename);
  
  logger.init(filename, now);
  logger.write_event("LOGGING_STARTED");
  
  // validate waypoints versus accuracy
  for (int i=0; i<numLocations-1; i++) {
    if (fabs(WAYPOINT[i].latitude - WAYPOINT[i+1].latitude) < ACCURACY && fabs(WAYPOINT[i].longitude - WAYPOINT[i+1].longitude) < ACCURACY) {
      logger.write_event("WAYPOINTS_TOO_CLOSE");
    }
  }
  
  // change PWM frequency
  TCCR4B &= (int) 7;
  TCCR4B |= (int) 2; // 7800 Hz
  
  // start switch
  pinMode(40, INPUT);
  
   // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600);
  
  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // uncomment this line to turn on only the "minimum recommended" data
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
  // the parser doesn't care about other sentences at this time
  
  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_5HZ);   // 5 Hz update rate
  // For the parsing code to work nicely and have time to sort thru the data, and
  // print it out we don't suggest using anything higher than 1 Hz

  // Request updates on antenna status, comment out to keep quiet
  GPS.sendCommand(PGCMD_ANTENNA);

 // the nice thing about this code is you can have a timer0 interrupt go off
  // every 1 millisecond, and read data from the GPS for you. that makes the
  // loop code a heck of a lot easier!
  useInterrupt(true);
  
  // Ask for firmware version
//  delay(1000);
//  Serial1.println(PMTK_Q_RELEASE);
  
  get_next_waypoint();

  // init motors
  qik.init();
  /*
  setConfigShowResult(QIK_CONFIG_MOTOR_M0_ACCELERATION, 10);
  setConfigShowResult(QIK_CONFIG_MOTOR_M1_ACCELERATION, 10);
  setConfigShowResult(QIK_CONFIG_MOTOR_M0_BRAKE_DURATION, 200);
  setConfigShowResult(QIK_CONFIG_MOTOR_M1_BRAKE_DURATION, 200);
  */
  
  logger.write_event("SETUP_COMPLETED");
}


// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
#ifdef UDR0
  if (GPSECHO)
    if (c) UDR0 = c;  
    // writing direct to UDR0 is much much faster than Serial.print 
    // but only one character can be written at a time. 
#endif
}

void useInterrupt(boolean v) {
  if (v) {
    // Timer0 is already used for millis() - we'll just interrupt somewhere
    // in the middle and call the "Compare A" function above
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  } else {
    // do not call the interrupt function COMPA anymore
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
}

float get_compass_reading() {
  return compass.get_heading()/10;
}

void test_compass() {
  double current_bearing = get_compass_reading();
  Serial.print("COMPASS: ");
  Serial.println(current_bearing);
  delay(1000);
}

void test_gps() {
  
  // check for update from GPS
  if (GPS.newNMEAreceived()) {
    if (GPS.parse(GPS.lastNMEA())) {
      //logger.write_event("PARSED_GPS_DATA");
      if (GPS.fix) {
        Serial.print(GPS.latitude);
        Serial.print(",");
        Serial.print(GPS.longitude);
        Serial.println();
      }
    }
  }
  
  
}

void test_obstacle_avoidance() {
  
  int f = uFront.get_distance();
  int l = uLeft.get_distance();
  int r = uRight.get_distance();
  
  f = min(f, 200);
  l = min(l, 200);
  r = min(r, 200);
  
  /*
  Serial.print("LEFT: ");
  Serial.print(l);
  Serial.print(", FRONT: ");
  Serial.print(f);
  Serial.print(", RIGHT: ");
  Serial.print(r);
  Serial.println();
  */

  int closest = min(f, min(l,r));
  
  float speed_multiplier =  closest / 200.0f;
  if (speed_multiplier < 0.2) {
    speed_multiplier = 0.2;
  }
  
  int left  = MAX_SPEED;
  int right = MAX_SPEED;
  
  
  int n1 = 100;
  int n2 = 50;

  if (f < n1) {
    // approaching something, start to turn
    if (l < r) {
      // turn right
      right = 0;
    }
    else {
      // turn left
      left = 0;
    }    
  }  
  else {
    if (l < n2) {
      // turn right
      right = 0;
    }
    else if (r < n2) {
      // turn left
      left = 0;
    }
  }  
  
  qik.setSpeeds(left*speed_multiplier, right*speed_multiplier);
  delay(10);
}

void test_motion() {
  
  
  // forward one second
  Serial.println("FORWARD");
  qik.setSpeeds(50, 50);
  delay(1000);
  
//  // turn left
//  Serial.println("LEFT");
//  qik.setSpeeds(0, 255);
//  delay(1000);

  // turn right
//  Serial.println("RIGHT");
//  qik.setSpeeds(255, 0);
//  delay(1000);

  Serial.println("STOP");
  qik.setSpeeds(0,0);
  delay(1000);  
  
}

/** test ultrasonic sensors */
void test_ultrasonic() {
  int f = uFront.get_distance();
  int l = uLeft.get_distance();
  int r = uRight.get_distance();
  Serial.print("LEFT: ");
  Serial.print(l);
  Serial.print(", FRONT: ");
  Serial.print(f);
  Serial.print(", RIGHT: ");
  Serial.print(r);
  Serial.println();
  delay(1000);
}

void stop_moving() {
  qik.setSpeeds(0,0); //TODO: consider braking instead
}  

void set_motor_speeds(int left_speed, int right_speed) {
  logger.write_motor_speed(left_speed, right_speed);
  qik.setSpeeds(left_speed, right_speed);
}

/**
 * If obstacle avoidance is stuck in a loop then it calls this routine to backup and try again 
 */
void stop_and_backup() {
  logger.write_event("STOP_AND_BACKUP");
  set_motor_speeds(0,0);
  delay(500);
  set_motor_speeds(-random(MIN_SPEED,MAX_SPEED),-random(MIN_SPEED,MAX_SPEED));
  delay(1000);
  set_motor_speeds(0,0);
  delay(500);
}

void test_start_switch() {
  int n = analogRead(15);
  Serial.println(n);
  delay(1000);
}

boolean started = false;

void real_loop()                   
{
  unsigned long now = millis();

  // check for update from GPS
  if (GPS.newNMEAreceived()) {
    if (GPS.parse(GPS.lastNMEA())) {
      //logger.write_event("PARSED_GPS_DATA");
      if (GPS.fix) {
        
        last_gps_reading = now;
        gps_fix = true;
        
        // update current location
        currentLocation.set(
          0 - nav.convert_to_decimal_degrees(GPS.longitude), // convert to negative number for WEST
          nav.convert_to_decimal_degrees(GPS.latitude)
        );

        if (started) {        
          // record current gps location
          logger.write_gps(currentLocation.latitude, currentLocation.longitude);
  
          // now that we have a gps location, calculate how to get to the destination
          double diffLon = nav.calculate_difference(wpLon, currentLocation.longitude); 
          double diffLat = nav.calculate_difference(wpLat, currentLocation.latitude); 
          required_bearing = nav.calculate_compass_bearing(diffLon, diffLat);
          logger.write_navigation_data(diffLat, diffLon, required_bearing);
  
          // have we reached the waypoint yet?
          if (fabs(diffLon) <= ACCURACY && fabs(diffLat) <= ACCURACY) {
            logger.write_event("REACHED_WAYPOINT");
            get_next_waypoint();
          }
        }
        
      }
      else {
        //logger.write_event("GPS_NO_FIX");
      }
    }
    else {
      //logger.write_event("FAILED_TO_PARSE_GPS_DATA");
    }
  }
  
  if (!started) {
    int n = analogRead(15);
    if (n > 950) {
      logger.write_event("START_BUTTON_ACTIVATED");
      started = true;
    }
  }
  
  if (!started) {
    delay(50);
    return;
  }
  

  // return immediately if we have never had a gps signal  
  if (fabs(currentLocation.longitude) < 1) {
    return;
  }

  // if no GPS fix in N seconds then stop and wait
  if (!GPS.fix && now - last_gps_reading > 2000) {
    if (gps_fix) {
      gps_fix = false;
      logger.write_event("NO_GPS_SIGNAL");
      stop_moving();
    }
    return;
  }

  // get compass bearing
  float current_bearing = get_compass_reading();
  
  // calculate how much we need to turn by (negative = turn left, positive = turn right)
  float angle_diff = nav.calc_bearing_diff(current_bearing, required_bearing);
  float angle_diff_abs = fabs(angle_diff);
  logger.write_compass_bearing(current_bearing, angle_diff);

  // determine new motor speeds
  int left_speed = MAX_SPEED;
  int right_speed = MAX_SPEED;

  // if turning by more than 10 degrees then do sharp turn otherwise gradual adjustment
  if (angle_diff < 0) {
    if (angle_diff_abs > sharp_turn_degrees) {
      left_speed = 0;
    }
    else {
      left_speed = (180 - angle_diff_abs) / 180 * MAX_SPEED;
    }
  }
  else if (angle_diff > 0) {
    if (angle_diff_abs > sharp_turn_degrees) {
      right_speed = 0;
    }
    else {
      right_speed = (180 - angle_diff_abs) / 180 * MAX_SPEED;
    }
  }

  // check for obstacles  
  int f = uFront.get_distance();
  int l = uLeft.get_distance();
  int r = uRight.get_distance();
  logger.write_ultrasonic(l, f, r);
  
  if (f < front_distance || l < side_distance || r < side_distance) {

    // approaching something, turn left or right until clear
    if (l < r) {
      // turn right
      logger.write_event("TURNING_RIGHT_TO_AVOID_OBSTACLE");
      
      // put right brakes on briefly
      qik.setM1Brake(brake_amount_on_turn);
      delay(brake_delay_on_turn);
      qik.setM1Brake(0);

      set_motor_speeds(MAX_SPEED, 0);
      int count = 0;
      while (f < front_distance || l < side_distance) {
        if (++count == 40) {
          stop_and_backup();
          break;
        }
        delay(50);
        f = uFront.get_distance();  
        l = uLeft.get_distance();
        r = uRight.get_distance();
        //logger.write_ultrasonic(l, f, r);
      }
    }
    else {
      // turn left
      logger.write_event("TURNING_LEFT_TO_AVOID_OBSTACLE");

      // put left brakes on briefly
      qik.setM0Brake(brake_amount_on_turn);
      delay(brake_delay_on_turn);
      qik.setM0Brake(0);

      set_motor_speeds(0, MAX_SPEED);
      int count = 0;
      while (f < front_distance || r < side_distance) {
        if (++count == 40) {
          stop_and_backup();
          break;
        }
        delay(50);
        f = uFront.get_distance();  
        l = uLeft.get_distance();
        r = uRight.get_distance();
        //logger.write_ultrasonic(l, f, r);
      }
    }    
    
    logger.write_event("FINISHED_AVOIDING_OBSTACLE");
  }  

  // set speed according to navigation
  set_motor_speeds(left_speed, right_speed);
  
}

void loop() {
  
  //unsigned long t1 = millis();

  //test_gps();
  //test_compass();
  //test_motion();
  //test_obstacle_avoidance();
  //test_ultrasonic();
  //test_start_switch();
  
  
  real_loop();

  //unsigned long t2 = millis();
  //unsigned long duration = t1-t1;
  
  //logger.write_event("LOOP_TIME," + String(duration));
}

