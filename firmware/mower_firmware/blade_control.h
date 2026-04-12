#pragma once
#include <Arduino.h>

// Brushless ESC controlled via PWM signal (1000–2000µs)
class BladeControl {
public:
  BladeControl(uint8_t esc_pin);
  void begin();       // Arms ESC with idle signal
  void set_on(bool state);
  bool is_running();
private:
  uint8_t _esc_pin;
  bool _running;
};
