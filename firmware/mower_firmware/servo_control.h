#pragma once
#include <Servo.h>

class ServoControl {
public:
  ServoControl(uint8_t pin, int center_us = 1500, int range_us = 500);
  void begin();
  void set_angle(float degrees);  // -45 to +45
private:
  uint8_t _pin;
  int _center_us, _range_us;
  Servo _servo;
};
