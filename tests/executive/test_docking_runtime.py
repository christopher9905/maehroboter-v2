# tests/executive/test_docking_runtime.py
import numpy as np
import pytest

from mower.executive.mission_executive import MissionExecutive, MowerState
from mower.executive.docking_manager import DockingManager
from mower.executive.docking_runtime import LatestSoc, LatestCharging, build_docking_manager


class _FakeCamera:
    def __init__(self, frame):
        self._frame = frame

    def read(self):
        return self._frame


class _FakeHW:
    def drive(self, speed, steering): pass
    def set_blade(self, state): pass
    def estop(self): pass


CAMERA_MATRIX = np.array([[600.0, 0.0, 320.0],
                          [0.0, 600.0, 240.0],
                          [0.0, 0.0, 1.0]])
DIST_COEFFS = np.zeros((5, 1))


class TestLatestSoc:
    def test_defaults_to_zero(self):
        assert LatestSoc().get() == 0

    def test_update_then_get_returns_latest_value(self):
        soc = LatestSoc()
        soc.update(42)
        assert soc.get() == 42

    def test_update_overwrites_previous_value(self):
        soc = LatestSoc()
        soc.update(10)
        soc.update(77)
        assert soc.get() == 77


class TestLatestCharging:
    def test_defaults_to_false(self):
        assert LatestCharging().get() is False

    def test_update_then_get_returns_latest_value(self):
        charging = LatestCharging()
        charging.update(True)
        assert charging.get() is True

    def test_update_overwrites_previous_value(self):
        charging = LatestCharging()
        charging.update(True)
        charging.update(False)
        assert charging.get() is False


class TestBuildDockingManager:
    def test_returns_a_docking_manager(self):
        ex = MissionExecutive(hardware_interface=None)
        mgr = build_docking_manager(
            hardware=_FakeHW(), executive=ex, camera=_FakeCamera(None),
            camera_matrix=CAMERA_MATRIX, dist_coeffs=DIST_COEFFS,
        )
        assert isinstance(mgr, DockingManager)

    def test_frame_source_is_camera_read(self):
        # bound methods are not `is`-comparable (each access mints a new
        # object) — assert identity via __self__/__func__ instead.
        cam = _FakeCamera(None)
        ex = MissionExecutive(hardware_interface=None)
        mgr = build_docking_manager(
            hardware=_FakeHW(), executive=ex, camera=cam,
            camera_matrix=CAMERA_MATRIX, dist_coeffs=DIST_COEFFS,
        )
        assert mgr._frame_source.__self__ is cam
        assert mgr._frame_source.__func__ is _FakeCamera.read

    def test_tick_on_blank_frame_is_a_noop_when_returning(self):
        # End-to-end smoke test through the REAL ArucoDetector pipeline
        # (no stubbed cv2.aruco backend) — a blank frame has no marker,
        # so RETURNING should stay RETURNING without raising.
        ex = MissionExecutive(hardware_interface=None)
        ex.start_mission()
        ex.stop_mission()
        assert ex.state == MowerState.RETURNING
        blank_frame = np.zeros((480, 640, 3), dtype=np.uint8)
        mgr = build_docking_manager(
            hardware=_FakeHW(), executive=ex, camera=_FakeCamera(blank_frame),
            camera_matrix=CAMERA_MATRIX, dist_coeffs=DIST_COEFFS,
        )
        mgr._tick(blank_frame, soc=50, charging=False)
        assert ex.state == MowerState.RETURNING
