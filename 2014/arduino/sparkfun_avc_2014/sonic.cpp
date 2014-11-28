#include <arduino.h>
#include "sonic.h"

UltrasonicSensor::UltrasonicSensor(int _pingPin) {
  pingPin = _pingPin;
}

void UltrasonicSensor::init() {
}

int UltrasonicSensor::get_distance() {

  // send signal
  pinMode(pingPin, OUTPUT);
  digitalWrite(pingPin, LOW);
  delayMicroseconds(2);
  digitalWrite(pingPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(pingPin, LOW);

  // The same pin is used to read the signal from the PING))): a HIGH
  // pulse whose duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  pinMode(pingPin, INPUT);
  long duration = pulseIn(pingPin, HIGH);
  
  long cm = duration / 29 / 2;
  
  return cm;
}

