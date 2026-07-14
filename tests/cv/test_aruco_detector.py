# tests/cv/test_aruco_detector.py
import math

import numpy as np
import cv2
import pytest

from mower.cv.aruco_detector import ArucoDetector

# Simple 640x480 pinhole intrinsics for tests
CAMERA_MATRIX = np.array([[600.0, 0.0, 320.0],
                          [0.0, 600.0, 240.0],
                          [0.0, 0.0, 1.0]], dtype=np.float64)
DIST_COEFFS = np.zeros((5, 1), dtype=np.float64)
MARKER_LEN = 0.15


def _project_marker(tvec, rvec=(0.0, 0.0, 0.0)):
    """Synthesize the 4 image corners of a marker at a known pose."""
    half = MARKER_LEN / 2
    obj = np.array([[-half,  half, 0.0],
                    [ half,  half, 0.0],
                    [ half, -half, 0.0],
                    [-half, -half, 0.0]], dtype=np.float64)
    img_pts, _ = cv2.projectPoints(
        obj, np.array(rvec, dtype=np.float64), np.array(tvec, dtype=np.float64),
        CAMERA_MATRIX, DIST_COEFFS)
    return img_pts.reshape(4, 2).astype(np.float32)


class _StubDetector:
    """Stands in for cv2.aruco.ArucoDetector — returns preset corners/ids."""
    def __init__(self, corners, ids):
        self._corners = corners
        self._ids = ids

    def detectMarkers(self, frame):
        return self._corners, self._ids, None


def _make(corners_list, ids_list):
    corners = [c.reshape(1, 4, 2) for c in corners_list]
    ids = np.array(ids_list, dtype=np.int32).reshape(-1, 1) if ids_list else None
    stub = _StubDetector(corners, ids)
    return ArucoDetector(CAMERA_MATRIX, DIST_COEFFS, MARKER_LEN,
                         target_id=0, detector=stub)


def test_no_markers_returns_none():
    det = _make([], [])
    assert det.detect(np.zeros((480, 640, 3), np.uint8)) is None


def test_recovers_distance_straight_ahead():
    corners = _project_marker(tvec=(0.0, 0.0, 1.0))
    det = _make([corners], [0])
    pose = det.detect(np.zeros((480, 640, 3), np.uint8))
    assert pose is not None
    assert pose.marker_id == 0
    assert pose.distance_m == pytest.approx(1.0, abs=0.02)
    assert pose.lateral_offset_m == pytest.approx(0.0, abs=0.02)
    assert pose.bearing_deg == pytest.approx(0.0, abs=1.0)


def test_marker_to_the_right_has_positive_offset():
    corners = _project_marker(tvec=(0.3, 0.0, 1.2))
    det = _make([corners], [0])
    pose = det.detect(np.zeros((480, 640, 3), np.uint8))
    assert pose.lateral_offset_m == pytest.approx(0.3, abs=0.03)
    assert pose.bearing_deg > 0


def test_marker_to_the_left_has_negative_offset():
    corners = _project_marker(tvec=(-0.3, 0.0, 1.2))
    det = _make([corners], [0])
    pose = det.detect(np.zeros((480, 640, 3), np.uint8))
    assert pose.lateral_offset_m < 0
    assert pose.lateral_offset_m == pytest.approx(-0.3, abs=0.03)
    assert pose.bearing_deg < 0


def test_yaw_recovered_for_rotated_marker():
    # Marker rotated +20 deg about the camera's vertical (y) axis, facing straight ahead.
    corners = _project_marker(tvec=(0.0, 0.0, 1.0),
                              rvec=(0.0, math.radians(20.0), 0.0))
    det = _make([corners], [0])
    pose = det.detect(np.zeros((480, 640, 3), np.uint8))
    assert pose is not None
    # A positive rotation about the vertical axis yields a positive yaw_deg.
    assert pose.yaw_deg == pytest.approx(20.0, abs=1.0)
    assert pose.yaw_deg > 0


def test_ignores_non_target_ids():
    corners = _project_marker(tvec=(0.0, 0.0, 1.0))
    det = _make([corners], [7])   # target_id is 0
    assert det.detect(np.zeros((480, 640, 3), np.uint8)) is None


def test_picks_closest_target_marker():
    far = _project_marker(tvec=(0.0, 0.0, 2.0))
    near = _project_marker(tvec=(0.0, 0.0, 0.8))
    det = _make([far, near], [0, 0])
    pose = det.detect(np.zeros((480, 640, 3), np.uint8))
    assert pose.distance_m == pytest.approx(0.8, abs=0.03)
