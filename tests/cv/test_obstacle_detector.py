# tests/cv/test_obstacle_detector.py
import numpy as np
import pytest
from unittest.mock import MagicMock

from mower.cv.obstacle_detector import (
    ObstacleDetector,
    Detection,
    OBSTACLE_CLASS_IDS,
    MIN_CONFIDENCE,
)


def _make_net(detections: list[tuple]) -> MagicMock:
    """Build a mock cv2.dnn.Net whose forward() returns the given detections.

    Each detection is (class_id, confidence, xmin, ymin, xmax, ymax).
    """
    net = MagicMock()
    n = len(detections)
    output = np.zeros((1, 1, max(n, 1), 7), dtype=np.float32)
    for i, (cls, conf, x1, y1, x2, y2) in enumerate(detections):
        output[0, 0, i] = [0, cls, conf, x1, y1, x2, y2]
    net.forward.return_value = output
    return net


def _blank_frame() -> np.ndarray:
    return np.zeros((480, 640, 3), dtype=np.uint8)


class TestObstacleDetector:
    def test_detection_above_threshold_returned(self):
        net = _make_net([(1, 0.9, 0.1, 0.1, 0.9, 0.9)])  # person
        det = ObstacleDetector(net=net, frame_source=_blank_frame)
        result = det._detect_once(_blank_frame())
        assert len(result) == 1
        assert result[0].class_id == 1
        assert result[0].class_name == "person"
        assert abs(result[0].confidence - 0.9) < 1e-5

    def test_detection_below_threshold_filtered(self):
        net = _make_net([(1, 0.3, 0.1, 0.1, 0.9, 0.9)])  # confidence 0.3 < 0.5
        det = ObstacleDetector(net=net, frame_source=_blank_frame)
        result = det._detect_once(_blank_frame())
        assert result == []

    def test_non_obstacle_class_filtered(self):
        net = _make_net([(3, 0.9, 0.1, 0.1, 0.9, 0.9)])  # class 3 = car — not relevant
        det = ObstacleDetector(net=net, frame_source=_blank_frame)
        result = det._detect_once(_blank_frame())
        assert result == []

    def test_multiple_detections_filtered_correctly(self):
        net = _make_net([
            (1,  0.9, 0.1, 0.1, 0.9, 0.9),  # person, high conf → KEEP
            (1,  0.3, 0.1, 0.1, 0.9, 0.9),  # person, low conf  → FILTER
            (3,  0.9, 0.1, 0.1, 0.9, 0.9),  # car, high conf    → FILTER (wrong class)
            (18, 0.8, 0.2, 0.2, 0.8, 0.8),  # dog, high conf    → KEEP
        ])
        det = ObstacleDetector(net=net, frame_source=_blank_frame)
        result = det._detect_once(_blank_frame())
        assert len(result) == 2
        class_ids = {d.class_id for d in result}
        assert class_ids == {1, 18}

    def test_bbox_computed_as_xywh_normalised(self):
        net = _make_net([(1, 0.9, 0.1, 0.2, 0.7, 0.8)])
        det = ObstacleDetector(net=net, frame_source=_blank_frame)
        result = det._detect_once(_blank_frame())
        xmin, ymin, w, h = result[0].bbox
        assert abs(xmin - 0.1) < 1e-5
        assert abs(ymin - 0.2) < 1e-5
        assert abs(w - 0.6) < 1e-5    # 0.7 - 0.1
        assert abs(h - 0.6) < 1e-5    # 0.8 - 0.2

    def test_run_frame_fires_callback_on_detection(self):
        net = _make_net([(1, 0.9, 0.1, 0.1, 0.9, 0.9)])
        det = ObstacleDetector(net=net, frame_source=_blank_frame)
        fired = []
        det.on_obstacle = lambda dets: fired.append(dets)
        det._run_frame(_blank_frame())
        assert len(fired) == 1
        assert len(fired[0]) == 1

    def test_run_frame_no_callback_when_empty(self):
        net = _make_net([(1, 0.1, 0.1, 0.1, 0.9, 0.9)])  # below threshold
        det = ObstacleDetector(net=net, frame_source=_blank_frame)
        fired = []
        det.on_obstacle = lambda dets: fired.append(dets)
        det._run_frame(_blank_frame())
        assert fired == []

    def test_default_constants(self):
        assert 1 in OBSTACLE_CLASS_IDS    # person
        assert 16 in OBSTACLE_CLASS_IDS   # bird
        assert 17 in OBSTACLE_CLASS_IDS   # cat
        assert 18 in OBSTACLE_CLASS_IDS   # dog
        assert MIN_CONFIDENCE == 0.5
