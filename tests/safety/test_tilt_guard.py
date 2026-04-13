import math
import pytest
from mower.safety.tilt_guard import TiltGuard
from mower.nav.imu_reader import ImuReading


def _make_imu(pitch: float = 0.0, roll: float = 0.0) -> ImuReading:
    return ImuReading(
        heading_deg=0.0,
        pitch_deg=pitch,
        roll_deg=roll,
        yaw_rate_dps=0.0,
        timestamp=0.0,
    )


class TestTiltGuard:
    def test_flat_no_trigger(self):
        triggered: list[ImuReading] = []
        tg = TiltGuard(threshold_deg=30.0)
        tg.on_tilt = triggered.append
        tg.check(_make_imu(pitch=0.0, roll=0.0))
        assert len(triggered) == 0

    def test_pitch_over_threshold_triggers(self):
        triggered: list[ImuReading] = []
        tg = TiltGuard(threshold_deg=30.0)
        tg.on_tilt = triggered.append
        tg.check(_make_imu(pitch=35.0))
        assert len(triggered) == 1

    def test_roll_over_threshold_triggers(self):
        triggered: list[ImuReading] = []
        tg = TiltGuard(threshold_deg=30.0)
        tg.on_tilt = triggered.append
        tg.check(_make_imu(roll=-31.0))
        assert len(triggered) == 1

    def test_combined_tilt_uses_vector_magnitude(self):
        """pitch=25°, roll=25° → combined ≈ 35.4° > 30° → trigger."""
        triggered: list[ImuReading] = []
        tg = TiltGuard(threshold_deg=30.0)
        tg.on_tilt = triggered.append
        tg.check(_make_imu(pitch=25.0, roll=25.0))
        assert len(triggered) == 1

    def test_just_below_threshold_no_trigger(self):
        triggered: list[ImuReading] = []
        tg = TiltGuard(threshold_deg=30.0)
        tg.on_tilt = triggered.append
        tg.check(_make_imu(pitch=29.9))
        assert len(triggered) == 0

    def test_triggered_reading_passed_to_callback(self):
        triggered: list[ImuReading] = []
        tg = TiltGuard(threshold_deg=30.0)
        tg.on_tilt = triggered.append
        reading = _make_imu(pitch=45.0)
        tg.check(reading)
        assert triggered[0] is reading

    def test_custom_threshold(self):
        triggered: list[ImuReading] = []
        tg = TiltGuard(threshold_deg=15.0)
        tg.on_tilt = triggered.append
        tg.check(_make_imu(pitch=16.0))
        assert len(triggered) == 1
