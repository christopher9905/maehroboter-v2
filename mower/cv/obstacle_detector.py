"""MobileNet-SSD v2 COCO obstacle detector for the lawn mower CV pipeline.

Uses OpenCV DNN backend. Runs in a background thread; on_obstacle callback
is fired from that thread on each frame that contains relevant detections.

COCO class IDs (1-indexed, COCO 2017 label map):
  person=1, bench=15, bird=16, cat=17, dog=18, chair=62, potted plant=64
"""
import logging
import threading
from dataclasses import dataclass
from typing import Callable, Optional

import cv2
import numpy as np

logger = logging.getLogger(__name__)

MIN_CONFIDENCE: float = 0.5
TARGET_FPS: float = 10.0

OBSTACLE_CLASS_IDS: frozenset[int] = frozenset({
    1,   # person  (safety-critical)
    15,  # bench
    16,  # bird    (safety-critical)
    17,  # cat     (safety-critical)
    18,  # dog     (safety-critical)
    62,  # chair
    64,  # potted plant
})

COCO_CLASSES: dict[int, str] = {
    1:  "person",
    15: "bench",
    16: "bird",
    17: "cat",
    18: "dog",
    62: "chair",
    64: "potted plant",
}


@dataclass
class Detection:
    class_id: int
    class_name: str
    confidence: float
    bbox: tuple[float, float, float, float]  # (xmin, ymin, w, h), normalised 0-1


class ObstacleDetector:
    """Runs MobileNet-SSD inference and fires on_obstacle for relevant detections.

    Args:
        net: A cv2.dnn.Net (injectable for testing; on real hardware pass
             cv2.dnn.readNetFromTensorflow(model_pb, config_pbtxt)).
        frame_source: Callable[[], Optional[np.ndarray]] — returns the latest
             camera frame, or None if no frame is available.
        class_ids: frozenset of COCO class IDs to report (default OBSTACLE_CLASS_IDS).
        min_confidence: Minimum detection confidence (default 0.5).
    """

    def __init__(
        self,
        net,
        frame_source: Callable[[], Optional[np.ndarray]],
        class_ids: frozenset[int] = OBSTACLE_CLASS_IDS,
        min_confidence: float = MIN_CONFIDENCE,
    ):
        self._net = net
        self._frame_source = frame_source
        self._class_ids = class_ids
        self._min_confidence = min_confidence
        self._stop_event = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._on_obstacle: Optional[Callable[[list[Detection]], None]] = None
        self._callback_lock = threading.Lock()

    @property
    def on_obstacle(self):
        with self._callback_lock:
            return self._on_obstacle

    @on_obstacle.setter
    def on_obstacle(self, callback):
        with self._callback_lock:
            self._on_obstacle = callback

    def start(self):
        """Start the background detection loop."""
        if self._thread is not None and self._thread.is_alive():
            return  # already running — prevent double-start
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._run_loop, daemon=True)
        self._thread.start()

    def stop(self):
        """Stop the background detection loop."""
        self._stop_event.set()
        if self._thread:
            self._thread.join(timeout=2.0)

    def _detect_once(self, frame: np.ndarray) -> list[Detection]:
        """Run one inference on frame. Returns filtered Detection list.

        Called directly from tests — no threading required.
        """
        blob = cv2.dnn.blobFromImage(
            frame, 1 / 127.5, (300, 300), (127.5, 127.5, 127.5),
            swapRB=True, crop=False,
        )
        self._net.setInput(blob)
        output = self._net.forward()  # shape (1, 1, N, 7)

        detections: list[Detection] = []
        for i in range(output.shape[2]):
            confidence = float(output[0, 0, i, 2])
            class_id = int(output[0, 0, i, 1])
            if confidence < self._min_confidence:
                continue
            if class_id not in self._class_ids:
                continue
            xmin = float(output[0, 0, i, 3])
            ymin = float(output[0, 0, i, 4])
            xmax = float(output[0, 0, i, 5])
            ymax = float(output[0, 0, i, 6])
            detections.append(Detection(
                class_id=class_id,
                class_name=COCO_CLASSES.get(class_id, f"class_{class_id}"),
                confidence=confidence,
                bbox=(xmin, ymin, xmax - xmin, ymax - ymin),
            ))
        return detections

    def _run_frame(self, frame: np.ndarray):
        """Detect obstacles in frame and fire callback if any found.

        Called directly from tests — no threading required.
        """
        detections = self._detect_once(frame)
        with self._callback_lock:
            cb = self._on_obstacle
        if detections and cb:
            cb(detections)

    def _run_loop(self):
        interval = 1.0 / TARGET_FPS
        while not self._stop_event.is_set():
            try:
                frame = self._frame_source()
                if frame is not None:
                    self._run_frame(frame)
            except Exception:
                logger.exception("Obstacle detection error")
            self._stop_event.wait(timeout=interval)  # interruptible sleep
