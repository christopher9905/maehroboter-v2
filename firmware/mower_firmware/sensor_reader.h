#pragma once
#include <Arduino.h>

class SensorReader {
public:
  SensorReader(uint8_t rain_pin, uint8_t lift_pin, uint8_t encoder_pin_a, uint8_t charge_pin);
  void begin();
  uint16_t read_rain_adc();
  bool read_lift();
  int32_t read_encoder_ticks();
  bool read_charging();
  void encoder_isr();  // Call from interrupt
private:
  uint8_t _rain_pin, _lift_pin, _encoder_pin_a, _charge_pin;
  volatile int32_t _encoder_ticks;
};

extern SensorReader* g_sensor_reader;  // For ISR linkage
