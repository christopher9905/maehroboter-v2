# tests/hal/test_camera.py
import numpy as np
import pytest

from mower.hal.camera import Camera, load_camera_calibration


class _FakeCapture:
    def __init__(self, frames):
        self._frames = list(frames)
        self.released = False

    def read(self):
        if not self._frames:
            return False, None
        return True, self._frames.pop(0)

    def release(self):
        self.released = True

    def isOpened(self):
        return True


def test_read_returns_frame_from_capture():
    frame = np.zeros((4, 4, 3), dtype=np.uint8)
    cam = Camera(capture=_FakeCapture([frame]))
    result = cam.read()
    assert result is frame


def test_read_returns_none_when_capture_reports_failure():
    cam = Camera(capture=_FakeCapture([]))  # empty -> (False, None)
    assert cam.read() is None


def test_release_calls_capture_release():
    fake = _FakeCapture([])
    cam = Camera(capture=fake)
    cam.release()
    assert fake.released is True


def test_is_opened_reflects_capture():
    cam = Camera(capture=_FakeCapture([]))
    assert cam.is_opened is True


def test_load_camera_calibration_roundtrip(tmp_path):
    path = tmp_path / "calib.npz"
    camera_matrix = np.array([[600.0, 0.0, 320.0],
                              [0.0, 600.0, 240.0],
                              [0.0, 0.0, 1.0]])
    dist_coeffs = np.zeros((5, 1))
    np.savez(path, camera_matrix=camera_matrix, dist_coeffs=dist_coeffs)

    loaded_matrix, loaded_dist = load_camera_calibration(path)

    assert np.allclose(loaded_matrix, camera_matrix)
    assert np.allclose(loaded_dist, dist_coeffs)


def test_load_camera_calibration_missing_file_raises(tmp_path):
    missing = tmp_path / "does_not_exist.npz"
    with pytest.raises(FileNotFoundError, match="calibration"):
        load_camera_calibration(missing)
