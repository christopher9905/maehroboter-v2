#pragma once
#include <stdint.h>
#include <string.h>  // memcpy

// Frame structure: [0xAA][CMD_TYPE][PAYLOAD_LEN][PAYLOAD...][CRC8]
static const uint8_t FRAME_START = 0xAA;

enum CmdType : uint8_t {
  CMD_DRIVE   = 0x01,
  CMD_BLADE   = 0x02,
  CMD_ESTOP   = 0x03,
  CMD_PING    = 0x04,
  // Telemetry (Teensy → RPi)
  CMD_SENSORS = 0x10,
  CMD_SOC     = 0x11,
  CMD_STATUS  = 0x12,
};

// CRC-8/MAXIM (Dallas 1-Wire, poly=0x31, reflect in/out)
// Must produce identical results to Python crcmod 'crc-8-maxim'
inline uint8_t crc8_maxim(const uint8_t* data, uint8_t len) {
  uint8_t crc = 0x00;
  for (uint8_t i = 0; i < len; i++) {
    uint8_t byte = data[i];
    for (uint8_t j = 0; j < 8; j++) {
      uint8_t mix = (crc ^ byte) & 0x01;
      crc >>= 1;
      if (mix) crc ^= 0x8C;  // reflected poly 0x31
      byte >>= 1;
    }
  }
  return crc;
}

// Send a frame over the given serial port.
// Uses static buffer (max payload 255 bytes) — not re-entrant, call from loop() only.
inline void send_frame(HardwareSerial& port, CmdType cmd, const uint8_t* payload, uint8_t len) {
  uint8_t header[3] = { FRAME_START, (uint8_t)cmd, len };
  // CRC over header + payload. Static buffer avoids VLA (not valid C++).
  static uint8_t buf[3 + 255];  // max payload 255 bytes
  memcpy(buf, header, 3);
  memcpy(buf + 3, payload, len);
  uint8_t crc = crc8_maxim(buf, 3 + len);
  port.write(header, 3);
  port.write(payload, len);
  port.write(&crc, 1);
}
