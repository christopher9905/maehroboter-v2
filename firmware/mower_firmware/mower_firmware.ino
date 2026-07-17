#include "protocol.h"
#include "motor_control.h"
#include "servo_control.h"
#include "blade_control.h"
#include "sensor_reader.h"
#include "watchdog.h"
#include <Wire.h>

// Pin-Belegung (anpassen nach Verdrahtung)
#define MOTOR_DIR_PIN   2
#define MOTOR_PWM_PIN   3
#define SERVO_PIN       4
#define BLADE_ESC_PIN   5
#define RAIN_ADC_PIN    A0
#define LIFT_PIN        6
#define ENCODER_PIN_A   7
// Charge-contact detect: placeholder pin, no dock circuit built yet
// (Phase 6 bring-up) — adjust once the charging-station contacts exist.
#define CHARGE_DETECT_PIN 8
#define FRONT_DECK_LIFT_PIN 9
#define REAR_DECK_LIFT_PIN 10

MotorControl motor(MOTOR_DIR_PIN, MOTOR_PWM_PIN);
ServoControl steering(SERVO_PIN);
BladeControl blade(BLADE_ESC_PIN);
// End positions may need inversion/calibration for the installed actuators.
ServoControl front_deck_lift(FRONT_DECK_LIFT_PIN);
ServoControl rear_deck_lift(REAR_DECK_LIFT_PIN);
SensorReader sensors(RAIN_ADC_PIN, LIFT_PIN, ENCODER_PIN_A, CHARGE_DETECT_PIN);
Watchdog watchdog(500);

uint32_t last_sensors_tx = 0;
uint32_t last_soc_tx = 0;
uint32_t last_status_tx = 0;
uint8_t recv_buf[64];
uint8_t recv_idx = 0;

void handle_lift_interrupt() {
  if (sensors.read_lift()) {
    blade.set_on(false);
    motor.stop();
  }
}

void process_command(CmdType cmd, const uint8_t* payload, uint8_t len) {
  watchdog.feed();
  switch (cmd) {
    case CMD_DRIVE: {
      float speed, steer;
      memcpy(&speed, payload, 4);
      memcpy(&steer, payload + 4, 4);
      motor.set_speed(speed);
      steering.set_angle(steer);
      break;
    }
    case CMD_BLADE:
      blade.set_on(payload[0] != 0);
      break;
    case CMD_ESTOP:
      motor.stop();
      blade.set_on(false);
      break;
    case CMD_PING:
      // Watchdog already fed above
      break;
    case CMD_DECK_LIFT:
      if (len >= 2) {
        // Never move a mowing deck while the blade is powered.
        if (payload[0] || payload[1]) blade.set_on(false);
        front_deck_lift.set_angle(payload[0] ? 45.0f : -45.0f);
        rear_deck_lift.set_angle(payload[1] ? 45.0f : -45.0f);
      }
      break;
    default:
      break;
  }
}

void setup() {
  Serial.begin(921600);  // USB-Serial to RPi 5
  motor.begin();
  steering.begin();
  blade.begin();
  front_deck_lift.begin();
  rear_deck_lift.begin();
  front_deck_lift.set_angle(-45.0f);
  rear_deck_lift.set_angle(-45.0f);
  sensors.begin();
  watchdog.reset();
  // Lift interrupt (hardware safety — independent of main loop)
  attachInterrupt(digitalPinToInterrupt(LIFT_PIN), handle_lift_interrupt, CHANGE);
}

void loop() {
  // Read incoming frames
  while (Serial.available()) {
    uint8_t b = Serial.read();
    if (recv_idx == 0 && b != FRAME_START) continue;
    recv_buf[recv_idx++] = b;
    if (recv_idx >= 3) {
      uint8_t payload_len = recv_buf[2];
      uint8_t expected = 3 + payload_len + 1;
      if (recv_idx >= expected) {
        // Validate CRC
        uint8_t calc_crc = crc8_maxim(recv_buf, 3 + payload_len);
        if (calc_crc == recv_buf[3 + payload_len]) {
          process_command((CmdType)recv_buf[1], recv_buf + 3, payload_len);
        }
        recv_idx = 0;
      }
    }
    if (recv_idx >= sizeof(recv_buf)) recv_idx = 0;
  }

  // Watchdog check
  if (watchdog.is_triggered()) {
    motor.stop();
    blade.set_on(false);
  }

  uint32_t now = millis();

  // Send SENSORS at 50 Hz
  if (now - last_sensors_tx >= 20) {
    last_sensors_tx = now;
    uint16_t rain = sensors.read_rain_adc();
    uint8_t lift = sensors.read_lift() ? 1 : 0;
    int32_t enc = sensors.read_encoder_ticks();
    uint8_t payload[7];
    memcpy(payload, &rain, 2);
    payload[2] = lift;
    memcpy(payload + 3, &enc, 4);  // deliberate unaligned write — safe with memcpy
    send_frame(Serial, CMD_SENSORS, payload, 7);
    if (lift) { blade.set_on(false); }
  }

  // Send SOC at 1 Hz
  // TODO Phase 1: Voltage divider fallback (A1). Phase 4 replaces with BMS I2C (BQ76940).
  // Linear approximation: 12000mV=0%, 16800mV=100% → 48mV per percent
  if (now - last_soc_tx >= 1000) {
    last_soc_tx = now;
    uint16_t raw = analogRead(A1);
    // Teensy 4.1 ADC is 12-bit (0–4095), Vref=3.3V, voltage divider: R1=100k, R2=10k → factor=11
    uint16_t voltage_mv = (uint16_t)(raw * (3300.0f / 4095.0f) * 11.0f);
    uint8_t soc = (uint8_t)constrain((voltage_mv - 12000) / 48, 0, 100);
    uint8_t payload[3];
    payload[0] = soc;
    memcpy(payload + 1, &voltage_mv, 2);
    send_frame(Serial, CMD_SOC, payload, 3);
  }

  // Send STATUS at 10 Hz
  if (now - last_status_tx >= 100) {
    last_status_tx = now;
    uint8_t payload[4] = {
      (uint8_t)(!watchdog.is_triggered()),
      (uint8_t)blade.is_running(),
      0x00,
      (uint8_t)sensors.read_charging(),
    };
    send_frame(Serial, CMD_STATUS, payload, 4);
  }
}
