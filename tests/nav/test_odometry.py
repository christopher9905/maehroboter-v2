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
        assert abs(result.delta_distance_m - 0x110 * 0.01) < 1e-6  # 272 ticks * 0.01

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

    def test_quantized_ticks_produce_continuous_speed_estimate(self):
        meters_per_tick = 0.0094
        target_speed = 0.1944
        odo = Odometry(meters_per_tick=meters_per_tick, speed_window_s=0.4)
        estimates = []
        for index in range(41):
            timestamp = index * 0.05
            ticks = int(target_speed * timestamp / meters_per_tick)
            result = odo.update(ticks=ticks, timestamp=timestamp)
            if result is not None and timestamp >= 0.5:
                estimates.append(result.speed_mps)

        assert estimates
        assert max(estimates) - min(estimates) < 0.03
        assert sum(estimates) / len(estimates) == pytest.approx(target_speed, abs=0.015)

    def test_speed_decays_to_zero_only_after_measurement_window(self):
        odo = Odometry(meters_per_tick=0.01, speed_window_s=0.4)
        for index in range(5):
            result = odo.update(ticks=index, timestamp=index * 0.1)
        assert result is not None
        assert result.speed_mps == pytest.approx(0.1)

        shortly_stopped = odo.update(ticks=4, timestamp=0.5)
        fully_stopped = odo.update(ticks=4, timestamp=0.9)

        assert shortly_stopped is not None and shortly_stopped.speed_mps > 0.0
        assert fully_stopped is not None and fully_stopped.speed_mps == 0.0
