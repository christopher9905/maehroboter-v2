# tests/control/test_docking_controller.py
import pytest

from mower.cv.aruco_detector import MarkerPose
from mower.control.docking_controller import (
    DockingController, DockingCommand, DockPhase,
    BEARING_TOL_DEG, CONTACT_DISTANCE_M, STEERING_GAIN,
    MAX_APPROACH_SPEED, TAPER_DISTANCE_M,
)


def _marker(distance, bearing=0.0, offset=0.0):
    return MarkerPose(marker_id=0, distance_m=distance,
                      lateral_offset_m=offset, bearing_deg=bearing, yaw_deg=0.0)


def test_no_marker_searches_in_place():
    cmd = DockingController().compute(None, charge_detected=False)
    assert cmd.phase == DockPhase.SEARCHING
    assert cmd.speed == 0.0
    assert cmd.steering_deg != 0.0        # rotating to search
    assert cmd.docked is False


def test_off_centre_marker_aligns_without_forward_motion():
    cmd = DockingController().compute(_marker(1.0, bearing=20.0), charge_detected=False)
    assert cmd.phase == DockPhase.ALIGNING
    assert cmd.speed == 0.0
    assert cmd.steering_deg < 0           # marker to the right → steer right (negative)


def test_marker_to_left_steers_left():
    cmd = DockingController().compute(_marker(1.0, bearing=-20.0), charge_detected=False)
    assert cmd.steering_deg > 0           # marker to the left → steer left (positive)


def test_steering_magnitude_matches_gain():
    # bearing=20 is unclamped (1.2 * 20 = 24 < 45), so the exact gain is observable.
    cmd = DockingController().compute(_marker(1.0, bearing=20.0), charge_detected=False)
    assert cmd.steering_deg == pytest.approx(-STEERING_GAIN * 20)


def test_bearing_exactly_at_tolerance_approaches():
    # abs(bearing) > tol → ALIGNING, so exactly the tolerance is APPROACHING.
    cmd = DockingController().compute(
        _marker(1.0, bearing=BEARING_TOL_DEG), charge_detected=False)
    assert cmd.phase == DockPhase.APPROACHING
    assert cmd.speed > 0.0


def test_distance_exactly_at_contact_docks():
    # distance <= contact → DOCKED, so exactly the contact distance docks.
    cmd = DockingController().compute(
        _marker(CONTACT_DISTANCE_M, bearing=0.0), charge_detected=False)
    assert cmd.phase == DockPhase.DOCKED
    assert cmd.docked is True
    assert cmd.speed == 0.0


def test_speed_saturates_at_taper_distance():
    ctrl = DockingController()
    at_taper = ctrl.compute(_marker(TAPER_DISTANCE_M, bearing=0.0), charge_detected=False)
    beyond = ctrl.compute(_marker(2.0, bearing=0.0), charge_detected=False)
    assert at_taper.speed == pytest.approx(MAX_APPROACH_SPEED)
    assert beyond.speed == pytest.approx(MAX_APPROACH_SPEED)


def test_centred_marker_approaches_forward():
    cmd = DockingController().compute(_marker(1.0, bearing=1.0), charge_detected=False)
    assert cmd.phase == DockPhase.APPROACHING
    assert cmd.speed > 0.0


def test_speed_tapers_with_distance():
    ctrl = DockingController()
    far = ctrl.compute(_marker(1.1, bearing=0.0), charge_detected=False)
    near = ctrl.compute(_marker(0.3, bearing=0.0), charge_detected=False)
    assert far.speed > near.speed
    assert near.speed > 0.0


def test_contact_distance_docks():
    cmd = DockingController().compute(_marker(0.04, bearing=0.0), charge_detected=False)
    assert cmd.phase == DockPhase.DOCKED
    assert cmd.speed == 0.0
    assert cmd.docked is True


def test_charge_detected_docks_immediately():
    cmd = DockingController().compute(_marker(0.5, bearing=0.0), charge_detected=True)
    assert cmd.phase == DockPhase.DOCKED
    assert cmd.docked is True
    assert cmd.speed == 0.0


def test_steering_is_clamped():
    cmd = DockingController().compute(_marker(1.0, bearing=170.0), charge_detected=False)
    assert -45.0 <= cmd.steering_deg <= 45.0


def test_search_steering_is_injectable():
    cmd = DockingController(search_steering_deg=20.0).compute(None, charge_detected=False)
    assert cmd.phase == DockPhase.SEARCHING
    assert cmd.steering_deg == pytest.approx(20.0)
