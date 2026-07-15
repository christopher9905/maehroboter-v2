"""Camera frame source + calibration loading for the CV pipeline.

Camera wraps a pull-based capture backend (cv2.VideoCapture on the real
robot) behind read()/release(), injectable exactly like ArucoDetector's
cv2.aruco backend — testable without a physical camera.
"""
import logging
from pathlib import Path
from typing import Optional

import numpy as np

logger = logging.getLogger(__name__)


def _default_capture(device):
    import cv2
    return cv2.VideoCapture(device)


class Camera:
    """Frame source for ObstacleDetector / ArucoDetector: read() -> Optional[np.ndarray]."""

    def __init__(self, device: int = 0, capture=None):
        self._capture = capture if capture is not None else _default_capture(device)

    def read(self) -> Optional[np.ndarray]:
        ok, frame = self._capture.read()
        if not ok:
            return None
        return frame

    def release(self) -> None:
        self._capture.release()

    @property
    def is_opened(self) -> bool:
        return bool(self._capture.isOpened())


def load_camera_calibration(path) -> tuple[np.ndarray, np.ndarray]:
    """Load (camera_matrix, dist_coeffs) from a .npz calibration file.

    Produced by the hardware bring-up calibration step (cv2.calibrateCamera).
    """
    p = Path(path)
    if not p.exists():
        raise FileNotFoundError(
            f"Camera calibration file not found: {p}. Run the camera "
            "calibration bring-up step (cv2.calibrateCamera) first — see "
            "docs/superpowers/plans/2026-07-14-phase6-docking-charging.md."
        )
    data = np.load(p)
    return data["camera_matrix"], data["dist_coeffs"]
