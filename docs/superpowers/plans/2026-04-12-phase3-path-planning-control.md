# Phase 3: Path Planning & Lateral Control — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Boustrophedon (lawn-strip) path generation per zone, nearest-neighbour TSP zone ordering, and Stanley lateral controller that produces a clamped steering angle from cross-track error + heading error.

**Architecture:** Two new packages: `mower/path/` (boustrophedon planner, zone ordering) and `mower/control/` (Stanley controller). Path planners accept shapely Polygons and return lists of (x,y) UTM waypoints. The Stanley controller is a pure function — it takes the current `Pose` and a path segment and returns a `StanleyOutput`. No hardware dependencies; everything is unit-testable with synthetic geometry.

**Tech Stack:** Python 3.11+, shapely 2.0.6, numpy 2.x, pytest

---

## File Structure

```
mower/
├── path/
│   ├── __init__.py              Create — empty package marker
│   ├── boustrophedon.py         Create — MBR optimal angle, scan-line intersection, 38 cm lanes
│   └── zone_order.py            Create — nearest-neighbour TSP on zone centroids
└── control/
    ├── __init__.py              Create — empty package marker
    └── stanley.py               Create — Stanley lateral controller, StanleyOutput dataclass
tests/
├── path/
│   ├── __init__.py              Create
│   ├── test_boustrophedon.py    Create
│   └── test_zone_order.py       Create
└── control/
    ├── __init__.py              Create
    └── test_stanley.py          Create
```

### Interface contracts

**`boustrophedon.py`**
```python
LANE_SPACING_M: float = 0.38  # 40 cm blade, 5% side overlap

def compute_boustrophedon(
    zone: Polygon,
    lane_spacing: float = LANE_SPACING_M,
) -> list[tuple[float, float]]:
    """Return ordered UTM waypoints covering zone in boustrophedon pattern.

    Algorithm:
    1. Find optimal sweep angle via minimum_rotated_rectangle longest edge.
    2. Rotate zone by -angle so longest edge is horizontal.
    3. Generate scan lines at lane_spacing intervals from ymin+lane_spacing/2 to ymax.
    4. Intersect each scan line with rotated polygon (may be MultiLineString).
    5. Alternate left→right / right→left for boustrophedon.
    6. Rotate all waypoints back by +angle around original centroid.

    Returns [] for degenerate polygons (< 3 vertices, zero area).
    """
```

**`zone_order.py`**
```python
def nearest_neighbor_order(
    zones: list[Polygon],
    start_idx: int = 0,
) -> list[int]:
    """Return zone indices in nearest-neighbour visit order starting from start_idx.

    Uses centroid-to-centroid distance. O(n²) — fine for ≤ 50 zones.
    Returns [] for empty input. Returns [start_idx] for single zone.
    Raises IndexError if start_idx is out of range.
    """
```

**`stanley.py`**
```python
STANLEY_K: float = 2.0          # proportional gain
MAX_STEERING_DEG: float = 45.0  # hard clamp
SPEED_EPSILON: float = 0.1      # m/s — prevents division by zero at standstill

@dataclass
class StanleyOutput:
    steering_deg: float   # positive = steer LEFT (CCW/math convention)
    heading_error_deg: float
    cross_track_error_m: float  # positive = robot to RIGHT of path

class StanleyController:
    def __init__(self, k: float = STANLEY_K,
                 max_steering_deg: float = MAX_STEERING_DEG,
                 speed_epsilon: float = SPEED_EPSILON): ...

    def compute(
        self,
        pose: Pose,            # from mower.nav.localizer
        path_start: tuple[float, float],
        path_end: tuple[float, float],
    ) -> StanleyOutput:
        """Compute Stanley steering.

        cross_track_error_m:
            Signed distance from robot to directed line (path_start → path_end).
            Positive = robot is to the RIGHT of the path direction.
            Formula: -(dx*(pose_y - ay) - dy*(pose_x - ax)) / segment_length
            where dx=path_end_x-path_start_x, dy=path_end_y-path_start_y.

        heading_error_rad:
            Difference between path heading (atan2(dy,dx)) and robot heading
            (pose.heading_rad, in ENU math frame). Wrapped to (-π, π].

        Stanley formula:
            delta_rad = heading_error_rad + atan2(k * cross_track_m, speed + eps)
            clamped to [-max_steering_rad, +max_steering_rad]
        """
```

---

## Task 1: Boustrophedon Path Planner

**Files:**
- Create: `mower/path/__init__.py`
- Create: `mower/path/boustrophedon.py`
- Create: `tests/path/__init__.py`
- Create: `tests/path/test_boustrophedon.py`

### Background

The boustrophedon ("ox-turning") pattern covers a polygon by sweeping parallel strips. The key challenge is choosing the optimal sweep angle to minimise the number of turns — this equals the orientation of the minimum-rotated bounding rectangle's longest edge. Scan lines are generated at `lane_spacing` intervals, intersected with the polygon, and alternated in direction.

Sign and geometry notes:
- `shapely.minimum_rotated_rectangle` returns a rectangle; extract its exterior ring coordinates (4 corners + closing point = 5 coords). The longest edge determines the sweep angle.
- Rotate the polygon with `shapely.affinity.rotate(zone, -angle_deg, origin='centroid', use_radians=False)`.
- Scan lines span the full x-extent of the rotated polygon's bounding box, stepped at `lane_spacing` in y.
- `intersection()` may return LineString, MultiLineString, GeometryCollection, or empty — handle all cases.
- Rotate waypoints back: apply rotation matrix `R(+angle_rad)` around the original zone centroid.

- [ ] **Step 1: Write the failing tests**

```python
# tests/path/test_boustrophedon.py
import math
import pytest
from shapely.geometry import Polygon
from mower.path.boustrophedon import compute_boustrophedon, LANE_SPACING_M


class TestBoustrophedon:
    def _square(self, side=5.0):
        return Polygon([(0,0),(side,0),(side,side),(0,side)])

    def test_empty_for_degenerate_polygon(self):
        result = compute_boustrophedon(Polygon())
        assert result == []

    def test_returns_list_of_tuples(self):
        pts = compute_boustrophedon(self._square())
        assert isinstance(pts, list)
        assert all(isinstance(p, tuple) and len(p) == 2 for p in pts)

    def test_covers_square_with_expected_lane_count(self):
        side = 5.0
        pts = compute_boustrophedon(self._square(side), lane_spacing=1.0)
        # 5 m / 1.0 m spacing → expect ~5 lanes; each lane has 2 endpoints
        assert len(pts) >= 8  # at least 4 lanes × 2 points

    def test_boustrophedon_alternates_direction(self):
        """Consecutive lanes should start on opposite sides (x alternates)."""
        pts = compute_boustrophedon(self._square(4.0), lane_spacing=1.0)
        assert len(pts) >= 4
        # Even lanes: first point has smaller x; odd lanes: first point has larger x
        # Check that adjacent lane starts differ in x-direction
        if len(pts) >= 4:
            lane0_start_x = pts[0][0]
            lane1_start_x = pts[2][0]
            assert abs(lane0_start_x - lane1_start_x) > 0.5  # on opposite sides

    def test_waypoints_inside_or_on_polygon(self):
        poly = self._square(5.0)
        pts = compute_boustrophedon(poly, lane_spacing=1.0)
        from shapely.geometry import Point
        for p in pts:
            assert poly.buffer(0.01).contains(Point(p)), f"Point {p} outside polygon"

    def test_rotated_rectangle_orientation(self):
        """A tall narrow rectangle should produce horizontal lanes (fewer turns)."""
        rect = Polygon([(0,0),(1,0),(1,10),(0,10)])
        pts = compute_boustrophedon(rect, lane_spacing=0.38)
        assert len(pts) >= 2

    def test_custom_lane_spacing(self):
        pts_narrow = compute_boustrophedon(self._square(5.0), lane_spacing=0.5)
        pts_wide   = compute_boustrophedon(self._square(5.0), lane_spacing=2.0)
        assert len(pts_narrow) > len(pts_wide)

    def test_default_lane_spacing_is_38cm(self):
        assert abs(LANE_SPACING_M - 0.38) < 1e-9
```

- [ ] **Step 2: Run tests to verify they fail**

```bash
cd "/Users/christopher/Desktop/Modellbau Projekte/Mähroboter V2"
python3 -m pytest tests/path/test_boustrophedon.py -v
```

Expected: `ModuleNotFoundError: No module named 'mower.path'`

- [ ] **Step 3: Create package markers**

`mower/path/__init__.py` — empty file  
`tests/path/__init__.py` — empty file

- [ ] **Step 4: Implement `mower/path/boustrophedon.py`**

```python
import math
from shapely.geometry import Polygon, LineString, MultiLineString, GeometryCollection
from shapely.affinity import rotate

LANE_SPACING_M: float = 0.38


def _longest_edge_angle_deg(poly: Polygon) -> float:
    """Return the angle (degrees) of the longest edge of poly's MBR."""
    mrr = poly.minimum_rotated_rectangle
    coords = list(mrr.exterior.coords)  # 5 points (closing = first)
    best_angle = 0.0
    best_len = -1.0
    for i in range(len(coords) - 1):
        x1, y1 = coords[i]
        x2, y2 = coords[i + 1]
        length = math.hypot(x2 - x1, y2 - y1)
        if length > best_len:
            best_len = length
            best_angle = math.degrees(math.atan2(y2 - y1, x2 - x1))
    return best_angle


def _rotate_point(x: float, y: float, angle_rad: float,
                  cx: float, cy: float) -> tuple[float, float]:
    cos_a = math.cos(angle_rad)
    sin_a = math.sin(angle_rad)
    dx, dy = x - cx, y - cy
    return cx + cos_a * dx - sin_a * dy, cy + sin_a * dx + cos_a * dy


def compute_boustrophedon(
    zone: Polygon,
    lane_spacing: float = LANE_SPACING_M,
) -> list[tuple[float, float]]:
    if zone.is_empty or not zone.is_valid or zone.area == 0:
        return []

    centroid = zone.centroid
    cx, cy = centroid.x, centroid.y

    angle_deg = _longest_edge_angle_deg(zone)
    angle_rad = math.radians(angle_deg)

    rotated = rotate(zone, -angle_deg, origin=centroid, use_radians=False)
    minx, miny, maxx, maxy = rotated.bounds

    waypoints: list[tuple[float, float]] = []
    y = miny + lane_spacing / 2.0
    lane_idx = 0
    while y <= maxy:
        scan_line = LineString([(minx - 1.0, y), (maxx + 1.0, y)])
        inter = rotated.intersection(scan_line)

        segments: list[LineString] = []
        if isinstance(inter, LineString) and not inter.is_empty:
            segments = [inter]
        elif isinstance(inter, MultiLineString):
            segments = list(inter.geoms)
        elif isinstance(inter, GeometryCollection):
            segments = [g for g in inter.geoms
                        if isinstance(g, LineString) and not g.is_empty]

        for seg in segments:
            p1 = (seg.coords[0][0], seg.coords[0][1])
            p2 = (seg.coords[-1][0], seg.coords[-1][1])
            if lane_idx % 2 == 1:
                p1, p2 = p2, p1
            waypoints.append(_rotate_point(p1[0], p1[1], angle_rad, cx, cy))
            waypoints.append(_rotate_point(p2[0], p2[1], angle_rad, cx, cy))
        lane_idx += 1
        y += lane_spacing

    return waypoints
```

- [ ] **Step 5: Run tests to verify they pass**

```bash
python3 -m pytest tests/path/test_boustrophedon.py -v
```

Expected: all 8 tests PASS

- [ ] **Step 6: Run full suite**

```bash
python3 -m pytest tests/ --tb=short -q
```

Expected: 93 + 8 = 101 passed, 0 failed

- [ ] **Step 7: Commit**

```bash
git add mower/path/__init__.py mower/path/boustrophedon.py \
        tests/path/__init__.py tests/path/test_boustrophedon.py
git commit -m "feat(path): boustrophedon path planner with MBR optimal angle"
```

---

## Task 2: Zone Ordering (Nearest-Neighbour TSP)

**Files:**
- Create: `mower/path/zone_order.py`
- Create: `tests/path/test_zone_order.py`

### Background

When a lawn has multiple zones, the robot should visit them in an order that minimises travel between zones. Nearest-neighbour heuristic: start at `start_idx`, greedily pick the closest unvisited zone (centroid-to-centroid) until all are visited. This is O(n²) — appropriate for ≤ 50 zones.

- [ ] **Step 1: Write the failing tests**

```python
# tests/path/test_zone_order.py
import pytest
from shapely.geometry import Polygon
from mower.path.zone_order import nearest_neighbor_order


def _square(ox: float, oy: float, side: float = 1.0) -> Polygon:
    return Polygon([
        (ox, oy), (ox+side, oy), (ox+side, oy+side), (ox, oy+side)
    ])


class TestNearestNeighborOrder:
    def test_empty_input_returns_empty(self):
        assert nearest_neighbor_order([]) == []

    def test_single_zone_returns_single(self):
        assert nearest_neighbor_order([_square(0, 0)]) == [0]

    def test_two_zones_closer_first(self):
        # Zone 0 at (0,0), Zone 1 at (2,0), Zone 2 at (10,0)
        zones = [_square(0, 0), _square(2, 0), _square(10, 0)]
        order = nearest_neighbor_order(zones, start_idx=0)
        assert order[0] == 0
        assert order[1] == 1   # zone 1 is closer to zone 0 than zone 2
        assert order[2] == 2

    def test_start_idx_respected(self):
        zones = [_square(0, 0), _square(2, 0), _square(10, 0)]
        order = nearest_neighbor_order(zones, start_idx=2)
        assert order[0] == 2

    def test_all_zones_visited_exactly_once(self):
        zones = [_square(i * 3.0, 0) for i in range(5)]
        order = nearest_neighbor_order(zones)
        assert sorted(order) == list(range(5))

    def test_invalid_start_raises(self):
        with pytest.raises(IndexError):
            nearest_neighbor_order([_square(0, 0)], start_idx=5)

    def test_returns_list_of_ints(self):
        zones = [_square(0, 0), _square(5, 0)]
        result = nearest_neighbor_order(zones)
        assert isinstance(result, list)
        assert all(isinstance(i, int) for i in result)
```

- [ ] **Step 2: Run tests to verify they fail**

```bash
python3 -m pytest tests/path/test_zone_order.py -v
```

Expected: `ModuleNotFoundError: No module named 'mower.path.zone_order'`

- [ ] **Step 3: Implement `mower/path/zone_order.py`**

```python
import math
from shapely.geometry import Polygon


def _centroid(poly: Polygon) -> tuple[float, float]:
    c = poly.centroid
    return c.x, c.y


def _dist(a: tuple[float, float], b: tuple[float, float]) -> float:
    return math.hypot(b[0] - a[0], b[1] - a[1])


def nearest_neighbor_order(
    zones: list[Polygon],
    start_idx: int = 0,
) -> list[int]:
    """Return zone indices in nearest-neighbour visit order."""
    if not zones:
        return []
    if start_idx < 0 or start_idx >= len(zones):
        raise IndexError(f"start_idx {start_idx} out of range for {len(zones)} zones")

    centroids = [_centroid(z) for z in zones]
    unvisited = list(range(len(zones)))
    order: list[int] = []
    current = start_idx
    unvisited.remove(current)
    order.append(current)

    while unvisited:
        cur_c = centroids[current]
        nearest = min(unvisited, key=lambda i: _dist(cur_c, centroids[i]))
        order.append(nearest)
        unvisited.remove(nearest)
        current = nearest

    return order
```

- [ ] **Step 4: Run tests to verify they pass**

```bash
python3 -m pytest tests/path/test_zone_order.py -v
```

Expected: all 7 tests PASS

- [ ] **Step 5: Run full suite**

```bash
python3 -m pytest tests/ --tb=short -q
```

Expected: 101 + 7 = 108 passed, 0 failed

- [ ] **Step 6: Commit**

```bash
git add mower/path/zone_order.py tests/path/test_zone_order.py
git commit -m "feat(path): nearest-neighbour TSP zone ordering"
```

---

## Task 3: Stanley Lateral Controller

**Files:**
- Create: `mower/control/__init__.py`
- Create: `mower/control/stanley.py`
- Create: `tests/control/__init__.py`
- Create: `tests/control/test_stanley.py`

### Background

The Stanley controller (used on Stanford's DARPA Grand Challenge car) computes a steering angle from two error terms:

1. **Heading error**: `heading_error = path_heading - robot_heading` (wrapped to (-π, π])
   - `path_heading = atan2(dy, dx)` where `dx, dy = path_end - path_start`
   - `robot_heading = pose.heading_rad` (ENU math frame, E=0, CCW positive)

2. **Cross-track error** (`e`): signed distance from robot to directed path line, positive = robot is to the **RIGHT** of the path direction.
   ```
   e = -(dx*(pose.utm_y - ay) - dy*(pose.utm_x - ax)) / segment_length
   ```
   where `ax, ay = path_start`, `dx = path_end_x - ax`, `dy = path_end_y - ay`.

3. **Stanley formula**: `delta = heading_error + atan2(k * e, v + eps)`

4. **Clamp**: `delta` clamped to `[-max_steering_rad, +max_steering_rad]`

5. Output: `steering_deg = degrees(delta)`, positive = steer **LEFT** (math CCW convention).

Edge cases:
- Degenerate segment (zero length): return zero steering with zeroed errors.
- Robot at rest (`speed_mps ≤ 0`): `v = eps` prevents division by zero (Stanley still provides heading correction).

- [ ] **Step 1: Write the failing tests**

```python
# tests/control/test_stanley.py
import math
import pytest
from mower.nav.localizer import Pose
from mower.control.stanley import StanleyController, StanleyOutput, STANLEY_K, MAX_STEERING_DEG


def _pose(x: float, y: float, heading_rad: float, speed: float = 1.0) -> Pose:
    return Pose(
        utm_x=x, utm_y=y, heading_rad=heading_rad,
        speed_mps=speed, yaw_rate_rps=0.0,
        timestamp=0.0, fix_quality=4,
    )


class TestStanleyController:
    def test_output_type(self):
        ctrl = StanleyController()
        out = ctrl.compute(_pose(0, 0, 0.0), (0.0, 0.0), (10.0, 0.0))
        assert isinstance(out, StanleyOutput)
        assert hasattr(out, 'steering_deg')
        assert hasattr(out, 'heading_error_deg')
        assert hasattr(out, 'cross_track_error_m')

    def test_perfectly_aligned_on_path_zero_steering(self):
        """Robot at (0,1) heading East (0 rad), path going East, 1m right of path."""
        # Path: (0,0)→(10,0) going East. Robot at (0,1) — 1m to LEFT of path.
        # cross_track_error should be NEGATIVE (robot is left of path).
        ctrl = StanleyController()
        # Place robot ON the path, heading East, path going East
        out = ctrl.compute(_pose(0, 0, 0.0), (0.0, 0.0), (10.0, 0.0))
        assert abs(out.cross_track_error_m) < 1e-9
        assert abs(out.heading_error_deg) < 1e-6
        assert abs(out.steering_deg) < 1e-6

    def test_cross_track_right_of_path_steers_left(self):
        """Robot to RIGHT of eastward path → positive cross-track → positive steering (left)."""
        ctrl = StanleyController()
        # Robot at (5, -1): 1 m to the RIGHT of eastward path (y < 0 = right)
        out = ctrl.compute(_pose(5.0, -1.0, 0.0), (0.0, 0.0), (10.0, 0.0))
        assert out.cross_track_error_m > 0   # robot is right of path
        assert out.steering_deg > 0          # steer left to correct

    def test_cross_track_left_of_path_steers_right(self):
        """Robot to LEFT of eastward path → negative cross-track → negative steering (right)."""
        ctrl = StanleyController()
        out = ctrl.compute(_pose(5.0, 1.0, 0.0), (0.0, 0.0), (10.0, 0.0))
        assert out.cross_track_error_m < 0   # robot is left of path
        assert out.steering_deg < 0          # steer right to correct

    def test_heading_error_contributes(self):
        """Robot facing North on eastward path → -90° heading error → steers right."""
        ctrl = StanleyController()
        # Robot ON path but heading North (π/2 rad), path going East (0 rad)
        # heading_error = 0 - π/2 = -π/2 → steer right (negative)
        out = ctrl.compute(_pose(5.0, 0.0, math.pi / 2), (0.0, 0.0), (10.0, 0.0))
        assert out.steering_deg < 0  # steer right
        assert abs(out.heading_error_deg - (-90.0)) < 0.1

    def test_steering_clamped_to_max(self):
        """Extreme error should be clamped to ±MAX_STEERING_DEG."""
        ctrl = StanleyController()
        # 50m off-path and high gain → without clamp would exceed 45°
        out = ctrl.compute(_pose(5.0, -50.0, 0.0), (0.0, 0.0), (10.0, 0.0))
        assert abs(out.steering_deg) <= MAX_STEERING_DEG + 1e-9

    def test_zero_speed_no_crash(self):
        """At rest, speed_epsilon prevents division by zero."""
        ctrl = StanleyController()
        out = ctrl.compute(_pose(5.0, -1.0, 0.0, speed=0.0), (0.0, 0.0), (10.0, 0.0))
        assert math.isfinite(out.steering_deg)

    def test_degenerate_segment_returns_zero(self):
        """Zero-length path segment → zero steering."""
        ctrl = StanleyController()
        out = ctrl.compute(_pose(0, 0, 0.0), (5.0, 5.0), (5.0, 5.0))
        assert out.steering_deg == 0.0
        assert out.cross_track_error_m == 0.0
        assert out.heading_error_deg == 0.0

    def test_custom_gain(self):
        """Higher k → larger steering for same cross-track error."""
        ctrl_low  = StanleyController(k=1.0)
        ctrl_high = StanleyController(k=5.0)
        pose = _pose(5.0, -1.0, 0.0)
        out_low  = ctrl_low.compute(pose,  (0.0, 0.0), (10.0, 0.0))
        out_high = ctrl_high.compute(pose, (0.0, 0.0), (10.0, 0.0))
        assert abs(out_high.steering_deg) > abs(out_low.steering_deg)

    def test_heading_wrap_near_pi(self):
        """Heading wrap: path heading -π+ε, robot heading +π-ε → small error not ~2π."""
        ctrl = StanleyController()
        # Path going West-ish: path_heading ≈ π (atan2(0,-1))
        # Robot heading ≈ -π + small → difference should be small, not ≈ 2π
        path_heading = math.pi - 0.05
        robot_heading = -(math.pi - 0.05)
        # Place robot on path, only heading error matters
        out = ctrl.compute(_pose(5.0, 0.0, robot_heading), (10.0, 0.0), (0.0, 0.0))
        assert abs(out.heading_error_deg) < 20  # should be ~5.7°, NOT ~360°

    def test_default_constants(self):
        assert STANLEY_K == 2.0
        assert MAX_STEERING_DEG == 45.0
```

- [ ] **Step 2: Run tests to verify they fail**

```bash
python3 -m pytest tests/control/test_stanley.py -v
```

Expected: `ModuleNotFoundError: No module named 'mower.control'`

- [ ] **Step 3: Create package markers**

`mower/control/__init__.py` — empty file  
`tests/control/__init__.py` — empty file

- [ ] **Step 4: Implement `mower/control/stanley.py`**

```python
import math
from dataclasses import dataclass

from mower.nav.localizer import Pose

STANLEY_K: float = 2.0
MAX_STEERING_DEG: float = 45.0
SPEED_EPSILON: float = 0.1


@dataclass
class StanleyOutput:
    steering_deg: float          # positive = steer LEFT (CCW/math convention)
    heading_error_deg: float
    cross_track_error_m: float   # positive = robot to RIGHT of path


class StanleyController:
    def __init__(
        self,
        k: float = STANLEY_K,
        max_steering_deg: float = MAX_STEERING_DEG,
        speed_epsilon: float = SPEED_EPSILON,
    ):
        self._k = k
        self._max_steering_rad = math.radians(max_steering_deg)
        self._eps = speed_epsilon

    def compute(
        self,
        pose: Pose,
        path_start: tuple[float, float],
        path_end: tuple[float, float],
    ) -> StanleyOutput:
        ax, ay = path_start
        bx, by = path_end
        dx, dy = bx - ax, by - ay
        seg_len = math.hypot(dx, dy)

        if seg_len < 1e-9:
            return StanleyOutput(
                steering_deg=0.0,
                heading_error_deg=0.0,
                cross_track_error_m=0.0,
            )

        # Cross-track error: positive = robot is to the RIGHT of path direction
        # Using the signed area formula for point-to-line distance
        cross_track = -(dx * (pose.utm_y - ay) - dy * (pose.utm_x - ax)) / seg_len

        # Heading error: path heading − robot heading, wrapped to (−π, π]
        path_heading = math.atan2(dy, dx)
        heading_error = path_heading - pose.heading_rad
        heading_error = (heading_error + math.pi) % (2 * math.pi) - math.pi

        # Stanley formula
        speed = max(pose.speed_mps, 0.0)
        delta = heading_error + math.atan2(self._k * cross_track, speed + self._eps)
        delta = max(-self._max_steering_rad, min(self._max_steering_rad, delta))

        return StanleyOutput(
            steering_deg=math.degrees(delta),
            heading_error_deg=math.degrees(heading_error),
            cross_track_error_m=cross_track,
        )
```

- [ ] **Step 5: Run tests to verify they pass**

```bash
python3 -m pytest tests/control/test_stanley.py -v
```

Expected: all 11 tests PASS

- [ ] **Step 6: Run full suite**

```bash
python3 -m pytest tests/ --tb=short -q
```

Expected: 108 + 11 = 119 passed, 0 failed

- [ ] **Step 7: Commit**

```bash
git add mower/control/__init__.py mower/control/stanley.py \
        tests/control/__init__.py tests/control/test_stanley.py
git commit -m "feat(control): Stanley lateral controller with cross-track + heading error"
```

---

## Acceptance Criteria

After all three tasks are complete:

- [ ] `python3 -m pytest tests/ -q` → **119 passed, 0 failed**
- [ ] `mower/path/boustrophedon.py` — `compute_boustrophedon(polygon)` returns waypoints covering the polygon, all inside or on the boundary, alternating lane directions
- [ ] `mower/path/zone_order.py` — `nearest_neighbor_order(zones)` returns all zone indices exactly once in greedy nearest-neighbour order
- [ ] `mower/control/stanley.py` — `StanleyController.compute()` returns `StanleyOutput` with correct cross-track sign, heading wrap, and clamped steering
- [ ] No shapely deprecation warnings; no numpy dtype warnings
- [ ] All new modules importable without hardware (`import mower.path.boustrophedon` etc.)
