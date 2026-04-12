#include "watchdog.h"

Watchdog::Watchdog(uint32_t timeout_ms)
  : _timeout_ms(timeout_ms), _last_feed_ms(0), _triggered(false) {}

void Watchdog::feed() {
  _last_feed_ms = millis();
  _triggered = false;
}

bool Watchdog::is_triggered() {
  if (!_triggered && (millis() - _last_feed_ms > _timeout_ms)) {
    _triggered = true;
  }
  return _triggered;
}

void Watchdog::reset() {
  _last_feed_ms = millis();
  _triggered = false;
}
