# tests/executive/test_mission_executive.py
import threading
import pytest
from unittest.mock import MagicMock
from mower.executive.mission_executive import (
    MissionExecutive, MowerState,
    LOW_BATTERY_SOC, MAX_AVOIDANCE_ATTEMPTS,
)


def _executive(hw=None):
    return MissionExecutive(hardware_interface=hw)


class TestMowerStateTransitions:
    def test_initial_state_is_idle(self):
        ex = _executive()
        assert ex.state == MowerState.IDLE

    def test_start_teach_in_from_idle(self):
        ex = _executive()
        ex.start_teach_in()
        assert ex.state == MowerState.TEACH_IN

    def test_stop_teach_in_returns_to_idle(self):
        ex = _executive()
        ex.start_teach_in()
        ex.stop_teach_in()
        assert ex.state == MowerState.IDLE

    def test_start_mission_from_idle(self):
        ex = _executive()
        ex.start_mission()
        assert ex.state == MowerState.MOWING

    def test_stop_mission_from_mowing_goes_returning(self):
        ex = _executive()
        ex.start_mission()
        ex.stop_mission()
        assert ex.state == MowerState.RETURNING

    def test_on_obstacle_detected_from_mowing(self):
        ex = _executive()
        ex.start_mission()
        ex.on_obstacle_detected([])
        assert ex.state == MowerState.OBSTACLE_AVOIDANCE

    def test_on_obstacle_cleared_returns_to_mowing(self):
        ex = _executive()
        ex.start_mission()
        ex.on_obstacle_detected([])
        ex.on_obstacle_cleared()
        assert ex.state == MowerState.MOWING

    def test_obstacle_timeout_goes_error(self):
        ex = _executive()
        ex.start_mission()
        ex.on_obstacle_detected([])
        assert ex.state == MowerState.OBSTACLE_AVOIDANCE
        ex._obstacle_timeout()  # simulate 60s expiry
        assert ex.state == MowerState.ERROR

    def test_max_avoidance_attempts_goes_error(self):
        ex = _executive()
        ex.start_mission()
        for _ in range(MAX_AVOIDANCE_ATTEMPTS):
            ex.on_obstacle_detected([])
            ex.on_obstacle_cleared()
        # Next detection after max attempts → ERROR
        ex.on_obstacle_detected([])
        assert ex.state == MowerState.ERROR

    def test_on_battery_low_from_mowing(self):
        ex = _executive()
        ex.start_mission()
        ex.on_battery_low(15)
        assert ex.state == MowerState.RETURNING

    def test_dock_charge_cycle(self):
        ex = _executive()
        ex.start_mission()
        ex.stop_mission()               # → RETURNING
        ex.on_dock_success()            # → DOCKING
        ex.on_charge_started()          # → CHARGING
        ex.on_charge_complete()         # → IDLE
        assert ex.state == MowerState.IDLE

    def test_on_lift_from_mowing_goes_error(self):
        hw = MagicMock()
        ex = _executive(hw)
        ex.start_mission()
        ex.on_lift()
        assert ex.state == MowerState.ERROR
        hw.estop.assert_called_once()
        hw.set_blade.assert_called_once_with(False)

    def test_on_tilt_from_mowing_goes_error(self):
        ex = _executive()
        ex.start_mission()
        reading = MagicMock()
        reading.pitch_deg = 35.0
        reading.roll_deg = 5.0
        ex.on_tilt(reading)
        assert ex.state == MowerState.ERROR

    def test_on_geofence_violation_goes_error(self):
        ex = _executive()
        ex.start_mission()
        pose = MagicMock()
        pose.utm_x, pose.utm_y = 1000.0, 2000.0
        ex.on_geofence_violation(pose)
        assert ex.state == MowerState.ERROR

    def test_reset_error_from_error_goes_idle(self):
        ex = _executive()
        ex.start_mission()
        ex.on_lift()
        assert ex.state == MowerState.ERROR
        ex.reset_error()
        assert ex.state == MowerState.IDLE

    def test_reset_error_from_idle_is_noop(self):
        ex = _executive()
        ex.reset_error()  # no-op, should not raise
        assert ex.state == MowerState.IDLE

    def test_start_mission_when_already_mowing_is_noop(self):
        ex = _executive()
        ex.start_mission()
        ex.start_mission()  # should be ignored
        assert ex.state == MowerState.MOWING

    def test_on_state_change_callback_fires(self):
        ex = _executive()
        transitions = []
        ex.on_state_change = lambda old, new: transitions.append((old, new))
        ex.start_mission()
        assert transitions == [(MowerState.IDLE, MowerState.MOWING)]


class TestDockingRobustness:
    """Deferred robustness items from the Phase 6 docking review."""

    def test_blade_cut_when_entering_returning(self):
        hw = MagicMock()
        ex = _executive(hw)
        ex.start_mission()          # MOWING
        hw.set_blade.reset_mock()
        ex.stop_mission()           # → RETURNING
        hw.set_blade.assert_called_once_with(False)

    def test_blade_cut_on_battery_low_returning(self):
        hw = MagicMock()
        ex = _executive(hw)
        ex.start_mission()
        hw.set_blade.reset_mock()
        ex.on_battery_low(10)       # → RETURNING
        hw.set_blade.assert_called_once_with(False)

    def test_blade_cut_when_entering_docking(self):
        hw = MagicMock()
        ex = _executive(hw)
        ex.start_mission()
        ex.stop_mission()           # RETURNING
        hw.set_blade.reset_mock()
        ex.on_dock_success()        # → DOCKING
        hw.set_blade.assert_called_once_with(False)

    def _charging(self, hw=None):
        ex = _executive(hw)
        ex.start_mission()
        ex.stop_mission()
        ex.on_dock_success()
        ex.on_charge_started()
        assert ex.state == MowerState.CHARGING
        return ex

    def test_charge_timeout_goes_error(self):
        hw = MagicMock()
        ex = self._charging(hw)
        ex._charge_timeout()        # simulate the timer firing
        assert ex.state == MowerState.ERROR
        hw.estop.assert_called()

    def test_charge_complete_prevents_timeout(self):
        ex = self._charging()
        ex.on_charge_complete()     # → IDLE (timer cancelled)
        assert ex.state == MowerState.IDLE
        ex._charge_timeout()        # stale fire must be a no-op
        assert ex.state == MowerState.IDLE

    def test_charge_timeout_noop_when_not_charging(self):
        ex = _executive()
        ex._charge_timeout()        # never charging
        assert ex.state == MowerState.IDLE
