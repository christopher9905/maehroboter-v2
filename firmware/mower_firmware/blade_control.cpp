#include "blade_control.h"

BladeControl::BladeControl(uint8_t esc_pin) : _esc_pin(esc_pin), _running(false) {}

void BladeControl::begin() {
  pinMode(_esc_pin, OUTPUT);
  // Teensy 4.1 default PWM resolution is 12-bit (0–4095).
  // At 50 Hz: 1000µs idle = 1000/20000 * 4095 ≈ 205
  analogWriteResolution(12);
  analogWriteFrequency(_esc_pin, 50);
  analogWrite(_esc_pin, 205);  // 1000µs idle — arms ESC
  _running = false;
}

void BladeControl::set_on(bool state) {
  if (state) {
    analogWrite(_esc_pin, 307);  // 1500µs — half throttle for mowing (12-bit @ 50Hz)
  } else {
    analogWrite(_esc_pin, 205);  // 1000µs idle
  }
  _running = state;
}

bool BladeControl::is_running() { return _running; }
