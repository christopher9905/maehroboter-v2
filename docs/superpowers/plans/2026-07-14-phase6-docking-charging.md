# Phase 6 — Docking & Charging Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Give the mower the ability to precisely dock at its charging station by detecting an ArUco marker with the camera, visually servoing the last ~1.2 m to the charge contacts, and driving the existing `DOCKING → CHARGING → IDLE` executive transitions.

**Architecture:** Three focused modules that mirror the Phase 4/5 patterns. `ArucoDetector` estimates a 6-DoF marker pose from a camera frame using `cv2.aruco` + `cv2.solvePnP` (injectable detector, exactly like `ObstacleDetector` injects its `net`). `DockingController` is a pure visual-servoing state machine that turns a `MarkerPose` into a `DockingCommand` (speed + steering). `DockingManager` wires detector + controller + `MissionExecutive` + `HardwareInterface` together, performs the RTK→visual **handoff** during `RETURNING`, and fires `on_dock_success` / `on_charge_started` / `on_charge_complete`. All logic is exercised through synchronous, hardware-free test units.

**Tech Stack:** Python 3.10, `cv2.aruco` (ArUco `DICT_4X4_50`, 15 cm markers), `cv2.solvePnP` for pose, NumPy, `pytest`. Reuses `MissionExecutive` (states `RETURNING/DOCKING/CHARGING` already exist), `HardwareInterface.drive/set_blade/estop`.

---

## File Structure

- Create: `mower/cv/aruco_detector.py` — marker detection + pose estimation. One responsibility: frame → `Optional[MarkerPose]`.
- Create: `mower/control/docking_controller.py` — visual-servoing state machine. One responsibility: `MarkerPose` (+ contact flag) → `DockingCommand`.
- Create: `mower/executive/docking_manager.py` — orchestration/handoff. One responsibility: tie the above to the executive + hardware, with a background loop mirroring `ObstacleDetector`.
- Create: `tests/cv/test_aruco_detector.py`, `tests/control/test_docking_controller.py`, `tests/executive/test_docking_manager.py`
- Modify: `requirements.txt` — ensure the ArUco module is available (see Task 1, Step 1).

### Interface contracts

```python
# mower/cv/aruco_detector.py
@dataclass
class MarkerPose:
    marker_id: int
    distance_m: float          # straight-line distance camera → marker (tvec norm)
    lateral_offset_m: float    # marker x in camera frame; + = marker is to the RIGHT
    bearing_deg: float         # angle to marker from camera boresight; + = right
    yaw_deg: float             # marker's rotation about vertical vs. camera; + = marker facing left

class ArucoDetector:
    def __init__(self, camera_matrix, dist_coeffs, marker_length_m=0.15,
                 target_id=0, detector=None): ...
    def detect(self, frame) -> Optional[MarkerPose]: ...   # closest target marker, or None

# mower/control/docking_controller.py
class DockPhase(Enum):
    SEARCHING = auto(); ALIGNING = auto(); APPROACHING = auto(); DOCKED = auto()

@dataclass
class DockingCommand:
    speed: float          # -1.0..1.0 (forward +)
    steering_deg: float   # -45..45, + = steer LEFT (matches StanleyOutput/HAL)
    phase: DockPhase
    docked: bool

class DockingController:
    def compute(self, marker: Optional[MarkerPose], charge_detected: bool) -> DockingCommand: ...

# mower/executive/docking_manager.py
class DockingManager:
    def __init__(self, detector, controller, executive, hardware,
                 frame_source, handoff_distance_m=1.2): ...
    def _tick(self, frame, soc: int, charging: bool) -> None: ...  # testable unit
    def start(self) -> None; def stop(self) -> None                 # background loop
```

---

## Task 1: ArUco Marker Detector

### Background

The charging station carries an ArUco marker (`DICT_4X4_50`, 15×15 cm) at its entrance. The RPi Camera Module 3 sees it reliably to ~1.5 m. We detect the marker with `cv2.aruco.ArucoDetector`, then recover its 3D pose with `cv2.solvePnP` against the four known corner positions in the marker's own frame. From the translation vector `tvec = [x, y, z]` (camera frame: x right, y down, z forward) we derive `distance`, `lateral_offset` (x), and `bearing` (atan2(x, z)). Marker `yaw` comes from the rotation vector.

**Why injectable `detector`:** mirrors `ObstacleDetector(net=...)`. Tests build corners with `cv2.projectPoints` from a *known* pose, feed them through a stub detector, and assert `detect()` recovers that pose — no camera, no image files.

**Camera intrinsics:** provided by the caller (`camera_matrix`, `dist_coeffs`). Tests use a simple pinhole matrix. Real calibration values are produced later during hardware bring-up (see Acceptance Criteria).

**Files:**
- Create: `mower/cv/aruco_detector.py`
- Test: `tests/cv/test_aruco_detector.py`

- [ ] **Step 1: Ensure the ArUco module is available in `requirements.txt`**

`cv2.aruco` ships in `opencv-contrib-python`, not the base `opencv-python` wheel. Replace the existing line so the marker API is guaranteed on the RPi target:

```
# requirements.txt — change:
#   opencv-python>=4.8.0
# to:
opencv-contrib-python>=4.8.0
```

`opencv-contrib-python` is a superset of `opencv-python` (same `cv2` import, adds `cv2.aruco`), so the Phase 4 obstacle detector keeps working unchanged. Then:

```bash
pip install -r requirements.txt
python3 -c "import cv2.aruco; print(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50))"
```

Expected: prints a dictionary object, no ImportError.

- [ ] **Step 2: Write the failing tests**

```python
# tests/cv/test_aruco_detector.py
import numpy as np
import cv2
import pytest

from mower.cv.aruco_detector import ArucoDetector, MarkerPose

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
```

- [ ] **Step 3: Run tests to verify they fail**

Run: `python3 -m pytest tests/cv/test_aruco_detector.py -v`
Expected: FAIL / collection error — `ModuleNotFoundError: mower.cv.aruco_detector`.

- [ ] **Step 4: Implement `mower/cv/aruco_detector.py`**

```python
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
```

- [ ] **Step 5: Run tests to verify they pass**

Run: `python3 -m pytest tests/cv/test_aruco_detector.py -v`
Expected: all tests PASS.

- [ ] **Step 6: Run full suite**

Run: `python3 -m pytest tests/ --tb=short -q`
Expected: 171 + 5 = **176 passed, 0 failed**.

- [ ] **Step 7: Commit**

```bash
git add mower/cv/aruco_detector.py tests/cv/test_aruco_detector.py requirements.txt
git commit -m "feat(cv): ArUco marker detector with solvePnP pose estimation"
```

---

## Task 2: Docking Controller (visual servoing)

### Background

Once the marker pose is known, the controller drives the final approach as a small state machine:

- **SEARCHING** — no marker visible: rotate slowly in place (`speed=0`, small steering) so the camera sweeps for the marker. Never drive forward blind.
- **ALIGNING** — marker visible but off-centre (`|bearing| > BEARING_TOL_DEG`): rotate toward it (proportional steering, `speed≈0`) until it is roughly centred.
- **APPROACHING** — marker centred: creep forward with speed tapered by distance (slower as it closes), correcting steering from bearing to stay centred.
- **DOCKED** — `charge_detected` is True, or `distance ≤ CONTACT_DISTANCE_M`: full stop (`speed=0`), `docked=True`.

Steering sign matches `StanleyOutput`/HAL: **+ = steer LEFT**. A marker to the right (`bearing_deg > 0`) needs a right turn → **negative** steering. The controller is pure (no I/O, no threading), so every branch is a one-line assert.

**Files:**
- Create: `mower/control/docking_controller.py`
- Test: `tests/control/test_docking_controller.py`

- [ ] **Step 1: Write the failing tests**

```python
# tests/control/test_docking_controller.py
import pytest

from mower.cv.aruco_detector import MarkerPose
from mower.control.docking_controller import (
    DockingController, DockingCommand, DockPhase,
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
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `python3 -m pytest tests/control/test_docking_controller.py -v`
Expected: FAIL — `ModuleNotFoundError: mower.control.docking_controller`.

- [ ] **Step 3: Implement `mower/control/docking_controller.py`**

```python
"""Visual-servoing controller for the final ArUco docking approach.

Consumes a MarkerPose (from mower.cv.aruco_detector) and produces a
DockingCommand for HardwareInterface.drive(speed, steering).

Steering sign matches StanleyOutput / HAL: + = steer LEFT (CCW).
A marker to the RIGHT (bearing_deg > 0) yields NEGATIVE steering.
"""
import math
from dataclasses import dataclass
from enum import Enum, auto
from typing import Optional

from mower.cv.aruco_detector import MarkerPose

BEARING_TOL_DEG: float = 6.0      # within this, considered "centred"
CONTACT_DISTANCE_M: float = 0.05  # marker this close ⇒ docked
SEARCH_STEERING_DEG: float = 12.0 # in-place rotation while searching
STEERING_GAIN: float = 1.2        # deg steering per deg bearing
MAX_STEERING_DEG: float = 45.0
MAX_APPROACH_SPEED: float = 0.25  # forward speed cap (m/s-ish, -1..1 scale)
MIN_APPROACH_SPEED: float = 0.05
TAPER_DISTANCE_M: float = 1.2     # distance at/above which speed is at max


class DockPhase(Enum):
    SEARCHING = auto()
    ALIGNING = auto()
    APPROACHING = auto()
    DOCKED = auto()


@dataclass
class DockingCommand:
    speed: float
    steering_deg: float
    phase: DockPhase
    docked: bool


def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


class DockingController:
    def __init__(
        self,
        bearing_tol_deg: float = BEARING_TOL_DEG,
        contact_distance_m: float = CONTACT_DISTANCE_M,
        steering_gain: float = STEERING_GAIN,
        max_steering_deg: float = MAX_STEERING_DEG,
        max_speed: float = MAX_APPROACH_SPEED,
        min_speed: float = MIN_APPROACH_SPEED,
        taper_distance_m: float = TAPER_DISTANCE_M,
    ):
        self._bearing_tol = bearing_tol_deg
        self._contact = contact_distance_m
        self._gain = steering_gain
        self._max_steer = max_steering_deg
        self._max_speed = max_speed
        self._min_speed = min_speed
        self._taper = taper_distance_m

    def compute(self, marker: Optional[MarkerPose], charge_detected: bool) -> DockingCommand:
        if charge_detected:
            return DockingCommand(0.0, 0.0, DockPhase.DOCKED, True)

        if marker is None:
            # Rotate in place to sweep for the marker.
            return DockingCommand(0.0, SEARCH_STEERING_DEG, DockPhase.SEARCHING, False)

        if marker.distance_m <= self._contact:
            return DockingCommand(0.0, 0.0, DockPhase.DOCKED, True)

        # Marker to the right (bearing +) → steer right (negative).
        steering = _clamp(-self._gain * marker.bearing_deg,
                          -self._max_steer, self._max_steer)

        if abs(marker.bearing_deg) > self._bearing_tol:
            return DockingCommand(0.0, steering, DockPhase.ALIGNING, False)

        # Centred enough: creep forward, speed tapered by distance.
        frac = _clamp(marker.distance_m / self._taper, 0.0, 1.0)
        speed = self._min_speed + (self._max_speed - self._min_speed) * frac
        return DockingCommand(speed, steering, DockPhase.APPROACHING, False)
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `python3 -m pytest tests/control/test_docking_controller.py -v`
Expected: all tests PASS.

- [ ] **Step 5: Run full suite**

Run: `python3 -m pytest tests/ --tb=short -q`
Expected: 176 + 8 = **184 passed, 0 failed**.

- [ ] **Step 6: Commit**

```bash
git add mower/control/docking_controller.py tests/control/test_docking_controller.py
git commit -m "feat(control): visual-servoing docking controller state machine"
```

---

## Task 3: Docking Manager (handoff + executive integration)

### Background

The manager is the glue that runs during return-to-dock. It mirrors `ObstacleDetector`'s structure: a testable synchronous unit `_tick(frame, soc, charging)` plus a background thread loop.

Behaviour per tick, keyed off `executive.state`:

- **RETURNING** — run the detector. If a target marker is seen **and** `distance ≤ handoff_distance_m` (1.2 m), call `executive.on_dock_success()` (→ DOCKING). This is the RTK→visual **handoff**; RTK waypoint following (Phase 3) drives the robot here, the manager only watches for the marker.
- **DOCKING** — run detector + `DockingController`. Send `hardware.drive(cmd.speed, cmd.steering_deg)` each tick. When `cmd.docked` (charge contact detected via `charging` flag, or marker within contact distance): stop the robot (`drive(0,0)`), `set_blade(False)`, and call `executive.on_charge_started()` (→ CHARGING).
- **CHARGING** — hold still (`drive(0,0)`). When `soc >= FULL_SOC`, call `executive.on_charge_complete()` (→ IDLE).
- **Any other state** — do nothing (idempotent no-op).

`charging` and `soc` come from the HAL status/SOC callbacks in the real system; tests pass them directly. The blade must be off throughout docking — enforced when entering DOCKED.

**Files:**
- Create: `mower/executive/docking_manager.py`
- Test: `tests/executive/test_docking_manager.py`

- [ ] **Step 1: Write the failing tests**

```python
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
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `python3 -m pytest tests/executive/test_docking_manager.py -v`
Expected: FAIL — `ModuleNotFoundError: mower.executive.docking_manager`.

- [ ] **Step 3: Implement `mower/executive/docking_manager.py`**

```python
"""Docking manager: RTK→visual handoff and DOCKING/CHARGING orchestration.

Bridges ArucoDetector + DockingController + MissionExecutive + HardwareInterface.
Mirrors ObstacleDetector: a synchronous _tick() unit (tested directly) plus a
background thread loop for the real system.
"""
import logging
import threading
from typing import Callable, Optional

from mower.executive.mission_executive import MissionExecutive, MowerState

logger = logging.getLogger(__name__)

FULL_SOC: int = 95          # SOC at/above which charging is considered complete
HANDOFF_DISTANCE_M: float = 1.2
LOOP_HZ: float = 10.0


class DockingManager:
    def __init__(
        self,
        detector,
        controller,
        executive: MissionExecutive,
        hardware,
        frame_source: Callable[[], object],
        handoff_distance_m: float = HANDOFF_DISTANCE_M,
        full_soc: int = FULL_SOC,
    ):
        self._detector = detector
        self._controller = controller
        self._executive = executive
        self._hw = hardware
        self._frame_source = frame_source
        self._handoff = handoff_distance_m
        self._full_soc = full_soc
        self._stop_event = threading.Event()
        self._thread: Optional[threading.Thread] = None

    def _tick(self, frame, soc: int, charging: bool) -> None:
        state = self._executive.state

        if state == MowerState.RETURNING:
            marker = self._detector.detect(frame)
            if marker is not None and marker.distance_m <= self._handoff:
                logger.info("Marker acquired at %.2f m — handoff to visual docking",
                            marker.distance_m)
                self._executive.on_dock_success()   # → DOCKING
            return

        if state == MowerState.DOCKING:
            marker = self._detector.detect(frame)
            cmd = self._controller.compute(marker, charge_detected=charging)
            if cmd.docked:
                if self._hw:
                    self._hw.drive(0.0, 0.0)
                    self._hw.set_blade(False)
                self._executive.on_charge_started()  # → CHARGING
            elif self._hw:
                self._hw.drive(cmd.speed, cmd.steering_deg)
            return

        if state == MowerState.CHARGING:
            if self._hw:
                self._hw.drive(0.0, 0.0)
            if soc >= self._full_soc:
                logger.info("Charge complete (SOC %d%%)", soc)
                self._executive.on_charge_complete()  # → IDLE
            return

        # IDLE / MOWING / TEACH_IN / OBSTACLE_AVOIDANCE / ERROR: nothing to do.

    # --- Background loop (real system) ---

    def start(self, soc_source: Callable[[], int], charging_source: Callable[[], bool]):
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop_event.clear()
        self._thread = threading.Thread(
            target=self._run_loop, args=(soc_source, charging_source), daemon=True)
        self._thread.start()

    def stop(self):
        self._stop_event.set()
        if self._thread:
            self._thread.join(timeout=2.0)

    def _run_loop(self, soc_source, charging_source):
        interval = 1.0 / LOOP_HZ
        while not self._stop_event.is_set():
            try:
                frame = self._frame_source()
                if frame is not None:
                    self._tick(frame, soc_source(), charging_source())
            except Exception:
                logger.exception("Docking manager error")
            self._stop_event.wait(timeout=interval)
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `python3 -m pytest tests/executive/test_docking_manager.py -v`
Expected: all tests PASS.

- [ ] **Step 5: Run full suite**

Run: `python3 -m pytest tests/ --tb=short -q`
Expected: 184 + 7 = **191 passed, 0 failed**.

- [ ] **Step 6: Commit**

```bash
git add mower/executive/docking_manager.py tests/executive/test_docking_manager.py
git commit -m "feat(executive): docking manager with RTK->visual handoff"
```

---

## Acceptance Criteria

After all tasks are complete:

- [ ] `python3 -m pytest tests/ -q` → **199 passed, 0 failed** (171 existing + 28 new; the plan's 20 core tests plus 8 added during code review — yaw/left-offset, controller edge/gain cases, and the safety-race guard)
- [ ] `mower/cv/aruco_detector.py` — `ArucoDetector.detect(frame)` returns the closest target-id `MarkerPose` (distance, lateral offset, bearing, yaw) or `None`; pose math validated against synthetic `projectPoints` corners
- [ ] `mower/control/docking_controller.py` — `DockingController.compute(marker, charge_detected)` implements SEARCHING/ALIGNING/APPROACHING/DOCKED with correct steering sign (+ = LEFT), distance-tapered speed, and clamped steering
- [ ] `mower/executive/docking_manager.py` — `_tick()` performs the RTK→visual handoff at ≤1.2 m, drives the controller during DOCKING, cuts the blade and enters CHARGING on contact, and returns to IDLE at full SOC; background loop mirrors `ObstacleDetector`
- [ ] All new modules importable without hardware (`import mower.cv.aruco_detector`, etc.)
- [ ] `requirements.txt` uses `opencv-contrib-python>=4.8.0` (provides `cv2.aruco`)
- [ ] The `DOCKING → CHARGING → IDLE` executive transitions are now driven by real logic, not just unit-tested in isolation

### Manual / hardware bring-up (not automatable here — track separately)

- [ ] Mount the DICT_4X4_50 15×15 cm marker at the charging-station entrance
- [ ] Calibrate the RPi Camera Module 3 (`cv2.calibrateCamera`) → real `camera_matrix` / `dist_coeffs` saved as `mower/config/camera_calibration.npz` (loaded via `mower.hal.camera.load_camera_calibration`)
- [x] **Wire `DockingManager` into the app runtime** — done in software: `mower/main.py` is the new robot entrypoint. It builds `HardwareInterface` (real serial port) and, if a camera + calibration file are present, `DockingManager` via `mower.executive.docking_runtime.build_docking_manager` (real `Camera` frame source, `ArucoDetector`). SOC is tracked via `LatestSoc` fed from `HardwareInterface.on_soc`; `create_app(executive, docking_manager, soc_source, charging_source)` starts/stops the manager's background loop in the FastAPI lifespan. Everything degrades gracefully (logs a warning, runs API-only) when hardware/camera/calibration aren't present yet — safe to run on a partially-assembled robot. `charging_source` is still a `lambda: False` stub: no charge-contact sensor exists in the serial protocol yet, so `DockingManager` relies on its documented distance-only `DOCKED` trigger (`CONTACT_DISTANCE_M`) until a real signal is added to the firmware protocol (separate, firmware-side task). Tests: `tests/hal/test_camera.py`, `tests/executive/test_docking_runtime.py`, `tests/api/test_app.py::TestDockingManagerLifecycle`.
- [ ] Field-tune `CONTACT_DISTANCE_M`, approach speeds, and `handoff_distance_m` against the physical dock
- [ ] Commission the blade for real mowing runs ("Mähwerk in Betrieb nehmen")
- [ ] Add a real charge-contact signal to the serial protocol (firmware + `decode_status`/`decode_soc`) so `charging_source` reflects true electrical contact instead of the distance-only approximation

### Deferred robustness items (surfaced in code review) — ✅ resolved

- [x] **Actuation ordering vs. safety estop.** `SerialDriver.send(frame, priority=True)` now drains any queued normal frames (discarding an in-flight drive) and jumps a dedicated priority queue; `HardwareInterface.estop()` uses it. Combined with `DockingManager`'s pre-drive state re-check, a concurrent tilt/lift/geofence ESTOP always wins serial ordering. Tests: `tests/hal/test_hardware_interface.py::TestEstopPriority`.
- [x] **CHARGING stuck-detection.** `MissionExecutive` starts `CHARGE_TIMEOUT_S` (2 h) on entering CHARGING; if `on_charge_complete` doesn't arrive it falls back to ERROR + ESTOP (`_charge_timeout`), cancelled on completion or any fault. Mirrors the `OBSTACLE_TIMEOUT_S` pattern. Tests: `tests/executive/test_mission_executive.py::TestDockingRobustness`.
- [x] **Cut the blade on DOCKING/RETURNING entry, not only at contact.** `MissionExecutive._hw_blade_off()` fires on the transitions into RETURNING (`stop_mission`, `on_battery_low`) and DOCKING (`on_dock_success`), so the blade is off throughout the return/approach — not just at the DOCKED instant. Tests in `TestDockingRobustness`.

---

## Notes

- **DRY:** `ArucoDetector` reuses the injectable-backend pattern from `ObstacleDetector`; `DockingManager` reuses its background-loop shape. Steering sign and `MAX_STEERING_DEG` follow `StanleyController`.
- **YAGNI:** No new executive states — `RETURNING/DOCKING/CHARGING` already exist. No re-planning of RTK return (Phase 3 owns waypoint following); this phase only handles the last ~1.2 m and charge handshake.
- **Camera intrinsics** are injected, never hard-coded — real calibration is a hardware-bring-up step, kept out of the code.
