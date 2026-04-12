#include "servo_control.h"

ServoControl::ServoControl(uint8_t pin, int center_us, int range_us)
  : _pin(pin), _center_us(center_us), _range_us(range_us) {}

void ServoControl::begin() {
  _servo.attach(_pin);
  set_angle(0);
}

void ServoControl::set_angle(float degrees) {
  degrees = constrain(degrees, -45.0f, 45.0f);
  int us = _center_us + (int)(degrees / 45.0f * _range_us);
  _servo.writeMicroseconds(us);
}
