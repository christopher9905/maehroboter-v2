import pytest
from mower.nav.odometry import Odometry, OdometryUpdate, METERS_PER_TICK


class TestOdometry:
    def test_first_update_returns_none(self):
        odo = Odometry()
        result = odo.update(ticks=100, timestamp=1.0)
        assert result is None

    def test_speed_calculated_from_ticks(self):
        odo = Odometry(meters_per_tick=0.01)
        odo.update(ticks=0, timestamp=0.0)
        result = odo.update(ticks=10, timestamp=1.0)  # 10 ticks in 1 s
        assert result is not None
        assert abs(result.speed_mps - 0.10) < 1e-6

    def test_delta_distance_calculated(self):
        odo = Odometry(meters_per_tick=0.01)
        odo.update(ticks=0, timestamp=0.0)
        result = odo.update(ticks=5, timestamp=0.5)
        assert abs(result.delta_distance_m - 0.05) < 1e-6

    def test_total_distance_accumulates(self):
        odo = Odometry(meters_per_tick=0.01)
        odo.update(ticks=0, timestamp=0.0)
        odo.update(ticks=10, timestamp=1.0)
        result = odo.update(ticks=20, timestamp=2.0)
        assert abs(result.total_distance_m - 0.20) < 1e-6

    def test_32bit_wraparound_handled(self):
        odo = Odometry(meters_per_tick=0.01)
        odo.update(ticks=0xFFFFFF00, timestamp=0.0)
        # Wraps: new_ticks=0x00000010 → delta = 0x110 = 272 ticks
        result = odo.update(ticks=0x10, timestamp=1.0)
        assert result is not None
        assert result.delta_distance_m > 0  # wrap handled, not negative

    def test_zero_dt_returns_none(self):
        odo = Odometry()
        odo.update(ticks=0, timestamp=1.0)
        result = odo.update(ticks=100, timestamp=1.0)  # same timestamp
        assert result is None

    def test_reset_clears_state(self):
        odo = Odometry(meters_per_tick=0.01)
        odo.update(ticks=100, timestamp=1.0)
        odo.reset()
        result = odo.update(ticks=200, timestamp=2.0)
        assert result is None  # first update after reset
