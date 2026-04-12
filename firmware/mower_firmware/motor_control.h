#pragma once
#include <Arduino.h>

// Cytron MDD10A: DIR pin + PWM pin per channel
class MotorControl {
public:
  MotorControl(uint8_t dir_pin, uint8_t pwm_pin);
  void begin();
  void set_speed(float speed);  // -1.0 (reverse) to 1.0 (forward)
  void stop();
private:
  uint8_t _dir_pin, _pwm_pin;
};
