import pytest
from mower.safety.rain_guard import RainGuard, RAIN_ADC_THRESHOLD, RAIN_DEBOUNCE_READINGS, RAIN_DEBOUNCE_INTERVAL_S


class TestRainGuard:
    def test_single_wet_reading_no_trigger(self):
        guard = RainGuard()
        called = []
        guard.on_rain_detected = lambda: called.append(1)
        guard.check(700, 0.0)
        assert called == []

    def test_two_wet_readings_no_trigger(self):
        guard = RainGuard()
        called = []
        guard.on_rain_detected = lambda: called.append(1)
        guard.check(700, 0.0)
        guard.check(700, 0.2)
        assert called == []

    def test_three_wet_readings_triggers(self):
        guard = RainGuard()
        called = []
        guard.on_rain_detected = lambda: called.append(1)
        guard.check(700, 0.0)
        guard.check(700, 0.2)
        guard.check(700, 0.4)
        assert called == [1]

    def test_readings_too_close_not_counted(self):
        """Readings within debounce interval don't each count toward trigger."""
        guard = RainGuard()
        called = []
        guard.on_rain_detected = lambda: called.append(1)
        guard.check(700, 0.0)
        guard.check(700, 0.05)   # < 200 ms → not counted
        guard.check(700, 0.10)   # < 200 ms → not counted
        assert called == []

    def test_dry_reading_resets_debounce(self):
        guard = RainGuard()
        called = []
        guard.on_rain_detected = lambda: called.append(1)
        guard.check(700, 0.0)
        guard.check(700, 0.2)
        guard.check(100, 0.4)   # dry → reset
        guard.check(700, 0.6)   # 1st counted reading since reset
        guard.check(700, 0.8)   # 2nd — not enough
        assert called == []

    def test_threshold_exact_value_not_wet(self):
        """ADC == threshold is NOT wet (must be strictly greater)."""
        guard = RainGuard()
        called = []
        guard.on_rain_detected = lambda: called.append(1)
        guard.check(RAIN_ADC_THRESHOLD, 0.0)
        guard.check(RAIN_ADC_THRESHOLD, 0.2)
        guard.check(RAIN_ADC_THRESHOLD, 0.4)
        assert called == []

    def test_no_double_trigger_while_raining(self):
        guard = RainGuard()
        called = []
        guard.on_rain_detected = lambda: called.append(1)
        guard.check(700, 0.0)
        guard.check(700, 0.2)
        guard.check(700, 0.4)   # triggers
        guard.check(700, 0.6)   # should NOT re-trigger
        guard.check(700, 0.8)
        assert called == [1]

    def test_rain_cleared_after_resume_time(self):
        guard = RainGuard(resume_after_s=30.0)
        detected, cleared = [], []
        guard.on_rain_detected = lambda: detected.append(1)
        guard.on_rain_cleared = lambda: cleared.append(1)
        guard.check(700, 0.0)
        guard.check(700, 0.2)
        guard.check(700, 0.4)   # rain detected
        assert detected == [1]
        guard.check(100, 1.0)   # dry_since = 1.0
        guard.check(100, 30.9)  # 29.9 s dry — not yet
        assert cleared == []
        guard.check(100, 31.1)  # 30.1 s dry → cleared
        assert cleared == [1]

    def test_rain_not_cleared_before_resume_time(self):
        guard = RainGuard(resume_after_s=30.0)
        cleared = []
        guard.on_rain_cleared = lambda: cleared.append(1)
        guard.check(700, 0.0); guard.check(700, 0.2); guard.check(700, 0.4)
        guard.check(100, 1.0)
        guard.check(100, 20.0)  # only 19 s dry
        assert cleared == []

    def test_wet_reading_resets_dry_timer(self):
        """A wet reading during drying restarts the dry-since clock."""
        guard = RainGuard(resume_after_s=30.0)
        cleared = []
        guard.on_rain_cleared = lambda: cleared.append(1)
        guard.check(700, 0.0); guard.check(700, 0.2); guard.check(700, 0.4)
        guard.check(100, 1.0)   # dry_since = 1.0
        guard.check(700, 25.0)  # wet again → dry_since reset
        guard.check(100, 26.0)  # dry again; dry_since = 26.0
        guard.check(100, 55.9)  # 29.9 s from 26.0 — not yet
        assert cleared == []
        guard.check(100, 56.1)  # 30.1 s → cleared
        assert cleared == [1]
