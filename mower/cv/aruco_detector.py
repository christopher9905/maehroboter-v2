"""ArUco charging-station marker detector for precision docking.

Detects DICT_4X4_50 markers and recovers each marker's pose with solvePnP.
The cv2.aruco.ArucoDetector is injectable so pose math can be tested with
synthetic corners (no camera, no image files).

Camera frame convention (OpenCV): x right, y down, z forward (into scene).
"""
import logging
import math
from dataclasses import dataclass
from typing import Optional

import cv2
import numpy as np

logger = logging.getLogger(__name__)

MARKER_LENGTH_M: float = 0.15
TARGET_MARKER_ID: int = 0


@dataclass
class MarkerPose:
    marker_id: int
    distance_m: float          # straight-line camera → marker (||tvec||)
    lateral_offset_m: float    # marker x in camera frame; + = to the RIGHT
    bearing_deg: float         # atan2(x, z); + = marker is to the right of boresight
    yaw_deg: float             # marker rotation about vertical vs. camera


def _default_detector():
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    params = cv2.aruco.DetectorParameters()
    return cv2.aruco.ArucoDetector(dictionary, params)


class ArucoDetector:
    def __init__(
        self,
        camera_matrix: np.ndarray,
        dist_coeffs: np.ndarray,
        marker_length_m: float = MARKER_LENGTH_M,
        target_id: int = TARGET_MARKER_ID,
        detector=None,
    ):
        self._camera_matrix = np.asarray(camera_matrix, dtype=np.float64)
        self._dist_coeffs = np.asarray(dist_coeffs, dtype=np.float64)
        self._marker_length = marker_length_m
        self._target_id = target_id
        self._detector = detector if detector is not None else _default_detector()

        half = marker_length_m / 2
        # Object points in the marker's own frame, matching the corner order
        # returned by detectMarkers (top-left, top-right, bottom-right, bottom-left).
        self._obj_points = np.array([
            [-half,  half, 0.0],
            [ half,  half, 0.0],
            [ half, -half, 0.0],
            [-half, -half, 0.0],
        ], dtype=np.float64)

    def detect(self, frame: np.ndarray) -> Optional[MarkerPose]:
        """Return the pose of the CLOSEST target-id marker, or None."""
        corners, ids, _ = self._detector.detectMarkers(frame)
        if ids is None or len(corners) == 0:
            return None

        best: Optional[MarkerPose] = None
        for marker_corners, marker_id in zip(corners, ids.flatten()):
            if int(marker_id) != self._target_id:
                continue
            pose = self._solve(marker_corners.reshape(4, 2), int(marker_id))
            if pose is None:
                continue
            if best is None or pose.distance_m < best.distance_m:
                best = pose
        return best

    def _solve(self, image_points: np.ndarray, marker_id: int) -> Optional[MarkerPose]:
        ok, rvec, tvec = cv2.solvePnP(
            self._obj_points,
            image_points.astype(np.float64),
            self._camera_matrix,
            self._dist_coeffs,
            flags=cv2.SOLVEPNP_IPPE_SQUARE,
        )
        if not ok:
            return None

        tx, _ty, tz = tvec.flatten()   # tvec is (3,1); flatten to avoid numpy scalar warning
        x, z = float(tx), float(tz)
        distance = float(np.linalg.norm(tvec))
        bearing = math.degrees(math.atan2(x, z))

        # Marker yaw about the vertical (camera y) axis from the rotation vector.
        rot, _ = cv2.Rodrigues(rvec)
        yaw = math.degrees(math.atan2(rot[0, 2], rot[2, 2]))

        return MarkerPose(
            marker_id=marker_id,
            distance_m=distance,
            lateral_offset_m=x,
            bearing_deg=bearing,
            yaw_deg=yaw,
        )
