#include "motor_control.h"

MotorControl::MotorControl(uint8_t dir_pin, uint8_t pwm_pin)
  : _dir_pin(dir_pin), _pwm_pin(pwm_pin) {}

void MotorControl::begin() {
  pinMode(_dir_pin, OUTPUT);
  pinMode(_pwm_pin, OUTPUT);
  stop();
}

void MotorControl::set_speed(float speed) {
  speed = constrain(speed, -1.0f, 1.0f);
  digitalWrite(_dir_pin, speed >= 0 ? HIGH : LOW);
  analogWrite(_pwm_pin, (uint8_t)(abs(speed) * 255));
}

void MotorControl::stop() {
  digitalWrite(_dir_pin, LOW);
  analogWrite(_pwm_pin, 0);
}
