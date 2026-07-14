# tests/executive/test_docking_manager.py
import pytest

from mower.cv.aruco_detector import MarkerPose
from mower.control.docking_controller import DockingController
from mower.executive.mission_executive import MissionExecutive, MowerState
from mower.executive.docking_manager import DockingManager


class _FakeHW:
    def __init__(self):
        self.drives = []
        self.blade = True
    def drive(self, speed, steering): self.drives.append((speed, steering))
    def set_blade(self, state): self.blade = state
    def estop(self): pass


class _StubDetector:
    def __init__(self, pose=None): self.pose = pose
    def detect(self, frame): return self.pose


def _marker(distance, bearing=0.0):
    return MarkerPose(marker_id=0, distance_m=distance,
                      lateral_offset_m=0.0, bearing_deg=bearing, yaw_deg=0.0)


def _mgr(detector, executive, hw):
    return DockingManager(
        detector=detector,
        controller=DockingController(),
        executive=executive,
        hardware=hw,
        frame_source=lambda: object(),   # opaque frame; stub detector ignores it
        handoff_distance_m=1.2,
    )


def _returning_executive():
    ex = MissionExecutive(hardware_interface=None)
    ex.start_mission()      # IDLE → MOWING
    ex.stop_mission()       # MOWING → RETURNING
    assert ex.state == MowerState.RETURNING
    return ex


def test_no_marker_while_returning_stays_returning():
    ex = _returning_executive()
    mgr = _mgr(_StubDetector(None), ex, _FakeHW())
    mgr._tick(frame=object(), soc=50, charging=False)
    assert ex.state == MowerState.RETURNING


def test_marker_too_far_does_not_hand_off():
    ex = _returning_executive()
    mgr = _mgr(_StubDetector(_marker(1.8)), ex, _FakeHW())
    mgr._tick(frame=object(), soc=50, charging=False)
    assert ex.state == MowerState.RETURNING


def test_marker_within_handoff_enters_docking():
    ex = _returning_executive()
    mgr = _mgr(_StubDetector(_marker(1.0)), ex, _FakeHW())
    mgr._tick(frame=object(), soc=50, charging=False)
    assert ex.state == MowerState.DOCKING


def test_docking_drives_toward_marker():
    ex = _returning_executive()
    hw = _FakeHW()
    det = _StubDetector(_marker(1.0))
    mgr = _mgr(det, ex, hw)
    mgr._tick(frame=object(), soc=50, charging=False)   # → DOCKING
    hw.drives.clear()
    mgr._tick(frame=object(), soc=50, charging=False)   # servo
    assert ex.state == MowerState.DOCKING
    assert len(hw.drives) == 1                          # a drive command was issued


def test_charge_contact_starts_charging_and_stops_blade():
    ex = _returning_executive()
    hw = _FakeHW()
    det = _StubDetector(_marker(0.8))
    mgr = _mgr(det, ex, hw)
    mgr._tick(frame=object(), soc=50, charging=False)   # → DOCKING
    mgr._tick(frame=object(), soc=50, charging=True)    # contact
    assert ex.state == MowerState.CHARGING
    assert hw.blade is False
    assert hw.drives[-1] == (0.0, 0.0)                  # stopped


def test_full_soc_completes_charge_to_idle():
    ex = _returning_executive()
    hw = _FakeHW()
    mgr = _mgr(_StubDetector(_marker(0.8)), ex, hw)
    mgr._tick(frame=object(), soc=50, charging=False)   # → DOCKING
    mgr._tick(frame=object(), soc=50, charging=True)    # → CHARGING
    mgr._tick(frame=object(), soc=100, charging=True)   # full
    assert ex.state == MowerState.IDLE


def test_tick_is_noop_when_idle():
    ex = MissionExecutive(hardware_interface=None)      # IDLE
    hw = _FakeHW()
    mgr = _mgr(_StubDetector(_marker(0.5)), ex, hw)
    mgr._tick(frame=object(), soc=50, charging=False)
    assert ex.state == MowerState.IDLE
    assert hw.drives == []
