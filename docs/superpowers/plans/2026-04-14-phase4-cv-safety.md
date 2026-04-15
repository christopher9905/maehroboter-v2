# Phase 4: Computer Vision & Safety Guards — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Rain sensor debounce guard, lift-detection edge-trigger guard, and MobileNet-SSD obstacle detector — all testable without hardware.

**Architecture:** Two new safety guards in `mower/safety/` following the existing `TiltGuard`/`Geofence` callback pattern, and a new `mower/cv/` package for MobileNet-SSD inference. The rain guard reads `rain_adc` from HAL SENSORS telemetry and implements a 3-reading debounce with a configurable 30-minute dry resume. The lift guard fires once per rising edge on `lift==True`. The CV detector wraps OpenCV DNN, runs in a background thread, and accepts an injectable `net` object for unit testing.

**Tech Stack:** Python 3.11+, opencv-python, numpy, pytest, unittest.mock

---

## File Structure

```
mower/
├── safety/
│   ├── rain_guard.py        Create — ADC>600 debounce, 30-min dry resume
│   └── lift_guard.py        Create — rising-edge lift detection
└── cv/
    ├── __init__.py          Create — empty package marker
    └── obstacle_detector.py Create — MobileNet-SSD via OpenCV DNN, injectable net
tests/
├── safety/
│   ├── test_rain_guard.py   Create — 10 tests
│   └── test_lift_guard.py   Create — 5 tests
└── cv/
    ├── __init__.py          Create — empty package marker
    └── test_obstacle_detector.py  Create — 8 tests
requirements.txt             Modify — add opencv-python>=4.8.0
```

### Key design decisions

- **Rain guard**: accepts `(rain_adc: int, timestamp: float)` — caller passes `time.monotonic()` so the guard is fully time-injectable in tests (no `time.sleep`).
- **Lift guard**: receives the full SENSORS dict so it mirrors the existing `HardwareInterface.on_sensors` callback signature.
- **CV detector**: `_detect_once(frame)` and `_run_frame(frame)` are public-ish (`_` prefix but documented) to enable direct testing without starting the background thread. The injected `net` mock handles `setInput()` + `forward()` calls; real `cv2.dnn.blobFromImage` is called with a real numpy frame (works once cv2 is installed).

---

## Task 1: Rain Guard

**Files:**
- Create: `mower/safety/rain_guard.py`
- Create: `tests/safety/test_rain_guard.py`

### Background

The capacitive rain sensor is read by the Teensy ADC and arrives in the SENSORS telemetry (`rain_adc: uint16`). Threshold is 600. Debounce requires 3 qualified readings (each ≥ 200 ms after the previous one) above the threshold before declaring rain. Any dry reading resets the debounce counter. After rain is declared, the guard waits until the sensor has been continuously dry for `resume_after_s` (default 30 min = 1800 s) before calling `on_rain_cleared`.

State variables:
- `_is_raining: bool` — True after debounce fires
- `_wet_times: list[float]` — timestamps of qualified wet readings during debounce
- `_dry_since: Optional[float]` — timestamp when dryness started (set on first dry reading while raining)

- [ ] **Step 1: Write the failing tests**

```python
# tests/safety/test_rain_guard.py
import pytest
from mower.safety.rain_guard import RainGuard, RAIN_ADC_THRESHOLD, RAIN_DEBOUNCE_READINGS, RAIN_DEBOUNCE_INTERVAL_S


class TestRainGuard:
    def test_single_wet_reading_no_trigger(self):
        guard = RainGuard()
        called = []
        guard.on_rain_detected = lambda: called.append(1)
        guard.check(700, 0.0)
        assert called == []

    def test_two_wet_readings_no_trigger(self):
        guard = RainGuard()
        called = []
        guard.on_rain_detected = lambda: called.append(1)
        guard.check(700, 0.0)
        guard.check(700, 0.2)
        assert called == []

    def test_three_wet_readings_triggers(self):
        guard = RainGuard()
        called = []
        guard.on_rain_detected = lambda: called.append(1)
        guard.check(700, 0.0)
        guard.check(700, 0.2)
        guard.check(700, 0.4)
        assert called == [1]

    def test_readings_too_close_not_counted(self):
        """Readings within debounce interval don't each count toward trigger."""
        guard = RainGuard()
        called = []
        guard.on_rain_detected = lambda: called.append(1)
        guard.check(700, 0.0)
        guard.check(700, 0.05)   # < 200 ms → not counted
        guard.check(700, 0.10)   # < 200 ms → not counted
        assert called == []

    def test_dry_reading_resets_debounce(self):
        guard = RainGuard()
        called = []
        guard.on_rain_detected = lambda: called.append(1)
        guard.check(700, 0.0)
        guard.check(700, 0.2)
        guard.check(100, 0.4)   # dry → reset
        guard.check(700, 0.6)   # 1st counted reading since reset
        guard.check(700, 0.8)   # 2nd — not enough
        assert called == []

    def test_threshold_exact_value_not_wet(self):
        """ADC == threshold is NOT wet (must be strictly greater)."""
        guard = RainGuard()
        called = []
        guard.on_rain_detected = lambda: called.append(1)
        guard.check(RAIN_ADC_THRESHOLD, 0.0)
        guard.check(RAIN_ADC_THRESHOLD, 0.2)
        guard.check(RAIN_ADC_THRESHOLD, 0.4)
        assert called == []

    def test_no_double_trigger_while_raining(self):
        guard = RainGuard()
        called = []
        guard.on_rain_detected = lambda: called.append(1)
        guard.check(700, 0.0)
        guard.check(700, 0.2)
        guard.check(700, 0.4)   # triggers
        guard.check(700, 0.6)   # should NOT re-trigger
        guard.check(700, 0.8)
        assert called == [1]

    def test_rain_cleared_after_resume_time(self):
        guard = RainGuard(resume_after_s=30.0)
        detected, cleared = [], []
        guard.on_rain_detected = lambda: detected.append(1)
        guard.on_rain_cleared = lambda: cleared.append(1)
        guard.check(700, 0.0)
        guard.check(700, 0.2)
        guard.check(700, 0.4)   # rain detected
        assert detected == [1]
        guard.check(100, 1.0)   # dry_since = 1.0
        guard.check(100, 30.9)  # 29.9 s dry — not yet
        assert cleared == []
        guard.check(100, 31.1)  # 30.1 s dry → cleared
        assert cleared == [1]

    def test_rain_not_cleared_before_resume_time(self):
        guard = RainGuard(resume_after_s=30.0)
        cleared = []
        guard.on_rain_cleared = lambda: cleared.append(1)
        guard.check(700, 0.0); guard.check(700, 0.2); guard.check(700, 0.4)
        guard.check(100, 1.0)
        guard.check(100, 20.0)  # only 19 s dry
        assert cleared == []

    def test_wet_reading_resets_dry_timer(self):
        """A wet reading during drying restarts the dry-since clock."""
        guard = RainGuard(resume_after_s=30.0)
        cleared = []
        guard.on_rain_cleared = lambda: cleared.append(1)
        guard.check(700, 0.0); guard.check(700, 0.2); guard.check(700, 0.4)
        guard.check(100, 1.0)   # dry_since = 1.0
        guard.check(700, 25.0)  # wet again → dry_since reset
        guard.check(100, 26.0)  # dry again; dry_since = 26.0
        guard.check(100, 55.9)  # 29.9 s from 26.0 — not yet
        assert cleared == []
        guard.check(100, 56.1)  # 30.1 s → cleared
        assert cleared == [1]
```

- [ ] **Step 2: Run tests to verify they fail**

```bash
cd "/Users/christopher/Desktop/Modellbau Projekte/Mähroboter V2"
python3 -m pytest tests/safety/test_rain_guard.py -v
```

Expected: `ModuleNotFoundError: No module named 'mower.safety.rain_guard'`

- [ ] **Step 3: Implement `mower/safety/rain_guard.py`**

```python
import logging
from typing import Callable, Optional

logger = logging.getLogger(__name__)

RAIN_ADC_THRESHOLD: int = 600
RAIN_DEBOUNCE_READINGS: int = 3
RAIN_DEBOUNCE_INTERVAL_S: float = 0.2
RAIN_RESUME_AFTER_S: float = 1800.0  # 30 minutes


class RainGuard:
    """Fires on_rain_detected after 3 qualified wet readings (≥200 ms apart).

    A wet reading is rain_adc > RAIN_ADC_THRESHOLD (600). Any dry reading resets
    the debounce counter. After rain is declared, fires on_rain_cleared once the
    sensor has been continuously dry for resume_after_s.

    Call check(rain_adc, timestamp) on each SENSORS telemetry frame.
    timestamp should be time.monotonic() — injectable for unit tests.
    """

    def __init__(
        self,
        threshold: int = RAIN_ADC_THRESHOLD,
        debounce_readings: int = RAIN_DEBOUNCE_READINGS,
        debounce_interval_s: float = RAIN_DEBOUNCE_INTERVAL_S,
        resume_after_s: float = RAIN_RESUME_AFTER_S,
    ):
        self._threshold = threshold
        self._debounce_readings = debounce_readings
        self._debounce_interval_s = debounce_interval_s
        self._resume_after_s = resume_after_s
        self._is_raining = False
        self._wet_times: list[float] = []
        self._dry_since: Optional[float] = None
        self.on_rain_detected: Optional[Callable[[], None]] = None
        self.on_rain_cleared: Optional[Callable[[], None]] = None

    def check(self, rain_adc: int, timestamp: float):
        is_wet = rain_adc > self._threshold

        if is_wet:
            self._dry_since = None  # cancel dry timer
            if not self._is_raining:
                if (not self._wet_times or
                        timestamp - self._wet_times[-1] >= self._debounce_interval_s):
                    self._wet_times.append(timestamp)
                if len(self._wet_times) >= self._debounce_readings:
                    self._is_raining = True
                    self._wet_times.clear()
                    logger.warning("Rain detected (ADC=%d)", rain_adc)
                    if self.on_rain_detected:
                        self.on_rain_detected()
        else:
            self._wet_times.clear()  # reset debounce on any dry reading
            if self._is_raining:
                if self._dry_since is None:
                    self._dry_since = timestamp
                elif timestamp - self._dry_since >= self._resume_after_s:
                    self._is_raining = False
                    self._dry_since = None
                    logger.info("Rain cleared after %.0f s dry", self._resume_after_s)
                    if self.on_rain_cleared:
                        self.on_rain_cleared()
```

- [ ] **Step 4: Run tests to verify they pass**

```bash
python3 -m pytest tests/safety/test_rain_guard.py -v
```

Expected: all 10 tests PASS

- [ ] **Step 5: Run full suite**

```bash
python3 -m pytest tests/ --tb=short -q
```

Expected: 120 + 10 = 130 passed, 0 failed

- [ ] **Step 6: Commit**

```bash
git add mower/safety/rain_guard.py tests/safety/test_rain_guard.py
git commit -m "feat(safety): rain guard with 3-reading debounce and 30-min dry resume"
```

---

## Task 2: Lift Guard

**Files:**
- Create: `mower/safety/lift_guard.py`
- Create: `tests/safety/test_lift_guard.py`

### Background

The Teensy reads the Reed/Hall switch on the mower deck. If the deck is raised, `lift: True` arrives in every SENSORS frame at 50 Hz. The HAL (`hardware_interface.py`) already sends an immediate ESTOP + blade-off when lift is True. The `LiftGuard` provides the mission-layer callback so the state machine can transition to ERROR.

Edge detection is essential: only fire `on_lift` on the rising edge (False → True), not on every subsequent True frame.

- [ ] **Step 1: Write the failing tests**

```python
# tests/safety/test_lift_guard.py
import pytest
from mower.safety.lift_guard import LiftGuard


class TestLiftGuard:
    def _sensors(self, lift: bool) -> dict:
        return {'lift': lift, 'rain_adc': 0, 'encoder_ticks': 0}

    def test_no_lift_no_callback(self):
        guard = LiftGuard()
        called = []
        guard.on_lift = lambda: called.append(1)
        guard.check(self._sensors(False))
        assert called == []

    def test_lift_fires_callback(self):
        guard = LiftGuard()
        called = []
        guard.on_lift = lambda: called.append(1)
        guard.check(self._sensors(True))
        assert called == [1]

    def test_lift_only_fires_on_rising_edge(self):
        """Sustained lift (multiple True frames) fires on_lift exactly once."""
        guard = LiftGuard()
        called = []
        guard.on_lift = lambda: called.append(1)
        guard.check(self._sensors(True))
        guard.check(self._sensors(True))
        guard.check(self._sensors(True))
        assert called == [1]

    def test_lift_fires_again_after_reset(self):
        """After deck lowered (False), a new lift fires the callback again."""
        guard = LiftGuard()
        called = []
        guard.on_lift = lambda: called.append(1)
        guard.check(self._sensors(True))    # rising edge → fires
        guard.check(self._sensors(False))   # deck lowered
        guard.check(self._sensors(True))    # rising edge again → fires
        assert called == [1, 1]

    def test_no_callback_if_not_assigned(self):
        guard = LiftGuard()
        guard.check(self._sensors(True))    # should not raise
```

- [ ] **Step 2: Run tests to verify they fail**

```bash
python3 -m pytest tests/safety/test_lift_guard.py -v
```

Expected: `ModuleNotFoundError: No module named 'mower.safety.lift_guard'`

- [ ] **Step 3: Implement `mower/safety/lift_guard.py`**

```python
import logging
from typing import Callable, Optional

logger = logging.getLogger(__name__)


class LiftGuard:
    """Fires on_lift once per rising edge of the deck-lift switch.

    The HAL already sends ESTOP + blade-off when lift is detected.
    This guard propagates the event to the mission executive so it can
    transition to ERROR state.

    Call check(sensors_dict) on each SENSORS telemetry frame.
    sensors_dict must contain the 'lift' key (bool) as decoded by protocol.decode_sensors().
    """

    def __init__(self):
        self._was_lifted = False
        self.on_lift: Optional[Callable[[], None]] = None

    def check(self, sensors: dict):
        lifted = bool(sensors.get('lift', False))
        if lifted and not self._was_lifted:
            logger.warning("Deck lift detected — notifying mission executive")
            if self.on_lift:
                self.on_lift()
        self._was_lifted = lifted
```

- [ ] **Step 4: Run tests to verify they pass**

```bash
python3 -m pytest tests/safety/test_lift_guard.py -v
```

Expected: all 5 tests PASS

- [ ] **Step 5: Run full suite**

```bash
python3 -m pytest tests/ --tb=short -q
```

Expected: 130 + 5 = 135 passed, 0 failed

- [ ] **Step 6: Commit**

```bash
git add mower/safety/lift_guard.py tests/safety/test_lift_guard.py
git commit -m "feat(safety): lift guard with rising-edge detection"
```

---

## Task 3: CV Obstacle Detector

**Files:**
- Create: `mower/cv/__init__.py`
- Create: `mower/cv/obstacle_detector.py`
- Create: `tests/cv/__init__.py`
- Create: `tests/cv/test_obstacle_detector.py`
- Modify: `requirements.txt` — add `opencv-python>=4.8.0`

### Background

MobileNet-SSD v2 (COCO) is run via OpenCV DNN backend. The model outputs a float32 tensor of shape `(1, 1, N, 7)` where each row is `[image_id, class_id, confidence, xmin, ymin, xmax, ymax]` with coordinates normalised to [0, 1].

Relevant COCO class IDs (from the official COCO 2017 label map, 1-indexed):
- 1: person (safety-critical)
- 15: bench
- 16: bird (safety-critical)
- 17: cat (safety-critical)
- 18: dog (safety-critical)
- 62: chair
- 64: potted plant

**Testability strategy:** `_detect_once(frame)` is called directly from tests with a real `numpy` frame (zeros). `cv2.dnn.blobFromImage` runs for real (cv2 is installed). Only `net.setInput()` and `net.forward()` are mocked — the mock returns a synthetic `(1, 1, N, 7)` numpy array so tests control exactly which detections are returned.

- [ ] **Step 1: Add `opencv-python>=4.8.0` to `requirements.txt`**

Append to `requirements.txt`:
```
opencv-python>=4.8.0
```

Install:
```bash
pip install "opencv-python>=4.8.0"
```

Verify:
```bash
python3 -c "import cv2; print(cv2.__version__)"
```

Expected: a version string like `4.x.x`

- [ ] **Step 2: Write the failing tests**

```python
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
```

- [ ] **Step 3: Run tests to verify they fail**

```bash
python3 -m pytest tests/cv/test_obstacle_detector.py -v
```

Expected: `ModuleNotFoundError: No module named 'mower.cv'`

- [ ] **Step 4: Create package markers**

`mower/cv/__init__.py` — empty file
`tests/cv/__init__.py` — empty file

- [ ] **Step 5: Implement `mower/cv/obstacle_detector.py`**

```python
"""MobileNet-SSD v2 COCO obstacle detector for the lawn mower CV pipeline.

Uses OpenCV DNN backend. Runs in a background thread; on_obstacle callback
is fired from that thread on each frame that contains relevant detections.

COCO class IDs (1-indexed, COCO 2017 label map):
  person=1, bench=15, bird=16, cat=17, dog=18, chair=62, potted plant=64
"""
import logging
import threading
import time
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
    bbox: tuple[float, float, float, float]  # (xmin, ymin, w, h), normalised 0–1


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
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self.on_obstacle: Optional[Callable[[list[Detection]], None]] = None

    def start(self):
        """Start the background detection loop."""
        self._running = True
        self._thread = threading.Thread(target=self._run_loop, daemon=True)
        self._thread.start()

    def stop(self):
        """Stop the background detection loop."""
        self._running = False
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
        if detections and self.on_obstacle:
            self.on_obstacle(detections)

    def _run_loop(self):
        interval = 1.0 / TARGET_FPS
        while self._running:
            try:
                frame = self._frame_source()
                if frame is not None:
                    self._run_frame(frame)
                time.sleep(interval)
            except Exception as e:
                logger.error("Obstacle detection error: %s", e)
                time.sleep(interval)
```

- [ ] **Step 6: Run tests to verify they pass**

```bash
python3 -m pytest tests/cv/test_obstacle_detector.py -v
```

Expected: all 8 tests PASS

- [ ] **Step 7: Run full suite**

```bash
python3 -m pytest tests/ --tb=short -q
```

Expected: 135 + 8 = 143 passed, 0 failed

- [ ] **Step 8: Commit**

```bash
git add mower/cv/__init__.py mower/cv/obstacle_detector.py \
        tests/cv/__init__.py tests/cv/test_obstacle_detector.py \
        requirements.txt
git commit -m "feat(cv): MobileNet-SSD obstacle detector with injectable net for testing"
```

---

## Acceptance Criteria

After all three tasks are complete:

- [ ] `python3 -m pytest tests/ -q` → **143 passed, 0 failed**
- [ ] `mower/safety/rain_guard.py` — `RainGuard.check(rain_adc, timestamp)` correctly debounces and clears rain
- [ ] `mower/safety/lift_guard.py` — `LiftGuard.check(sensors)` fires once per rising edge
- [ ] `mower/cv/obstacle_detector.py` — `ObstacleDetector._detect_once(frame)` filters by class and confidence; `_run_frame` fires callback; background thread in `start()`/`stop()`
- [ ] All new modules importable without hardware (`import mower.safety.rain_guard` etc.)
- [ ] `opencv-python>=4.8.0` added to `requirements.txt`
