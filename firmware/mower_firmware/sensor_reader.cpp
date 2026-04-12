#include "sensor_reader.h"

SensorReader* g_sensor_reader = nullptr;

SensorReader::SensorReader(uint8_t rain_pin, uint8_t lift_pin, uint8_t encoder_pin_a)
  : _rain_pin(rain_pin), _lift_pin(lift_pin), _encoder_pin_a(encoder_pin_a), _encoder_ticks(0) {}

void SensorReader::begin() {
  pinMode(_rain_pin, INPUT);
  pinMode(_lift_pin, INPUT_PULLUP);  // Active LOW
  pinMode(_encoder_pin_a, INPUT_PULLUP);
  g_sensor_reader = this;
  attachInterrupt(digitalPinToInterrupt(_encoder_pin_a), [](){
    if (g_sensor_reader) g_sensor_reader->encoder_isr();
  }, RISING);
}

uint16_t SensorReader::read_rain_adc() {
  return analogRead(_rain_pin);
}

bool SensorReader::read_lift() {
  return digitalRead(_lift_pin) == LOW;  // Active LOW
}

int32_t SensorReader::read_encoder_ticks() {
  return _encoder_ticks;
}

void SensorReader::encoder_isr() {
  // Phase 1: single-channel encoder, always increments.
  // TODO Phase 2: use motor direction state for signed ticks (bidirectional odometry).
  _encoder_ticks++;
}
