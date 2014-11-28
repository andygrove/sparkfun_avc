#ifndef __SONIC_H
#define __SONIC_H

class UltrasonicSensor {
private:
  int pingPin;
public:  
  UltrasonicSensor(int);
  void init();
  int get_distance();
};

#endif __SONIC_H

