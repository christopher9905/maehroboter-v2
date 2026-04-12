#pragma once
#include <Arduino.h>

class Watchdog {
public:
  Watchdog(uint32_t timeout_ms = 500);
  void feed();            // Call when PING received
  bool is_triggered();    // Returns true if timeout elapsed
  void reset();
private:
  uint32_t _timeout_ms;
  uint32_t _last_feed_ms;
  bool _triggered;
};
