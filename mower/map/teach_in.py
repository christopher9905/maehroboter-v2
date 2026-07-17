import logging
import math
from enum import Enum, auto
from typing import Optional

from mower.nav.localizer import Pose

logger = logging.getLogger(__name__)

DEFAULT_DISTANCE_THRESHOLD_M: float = 0.5
DEFAULT_TIME_THRESHOLD_S: float = 1.0
DEFAULT_CLOSE_THRESHOLD_M: float = 1.0
DEFAULT_MIN_POINTS: int = 4


class TeachInState(Enum):
    IDLE = auto()
    RECORDING = auto()
    CLOSED = auto()
    CANCELLED = auto()


def _distance(x1: float, y1: float, x2: float, y2: float) -> float:
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)


class TeachIn:
    """Records GPS poses during manual driving to build a GeoJSON polygon.

    Sampling rule: record a new point if distance since last point >= 0.5 m
    OR time since last point >= 1 s (whichever comes first).
    Auto-close: if robot returns within 1 m of start AND min_points reached.
    """

    def __init__(
        self,
        distance_threshold_m: float = DEFAULT_DISTANCE_THRESHOLD_M,
        time_threshold_s: float = DEFAULT_TIME_THRESHOLD_S,
        close_threshold_m: float = DEFAULT_CLOSE_THRESHOLD_M,
        min_points: int = DEFAULT_MIN_POINTS,
    ):
        self._dist_thresh = distance_threshold_m
        self._time_thresh = time_threshold_s
        self._close_thresh = close_threshold_m
        self._min_points = min_points
        self._state = TeachInState.IDLE
        self._feature_type: Optional[str] = None
        self._points: list[tuple[float, float]] = []
        self._point_times: list[float] = []
        self._last_point: Optional[tuple[float, float]] = None
        self._last_time: Optional[float] = None
        self._left_start_area = False

    @property
    def state(self) -> TeachInState:
        return self._state

    @property
    def points(self) -> list[tuple[float, float]]:
        return list(self._points)

    @property
    def path_length_m(self) -> float:
        return sum(
            _distance(*self._points[index - 1], *self._points[index])
            for index in range(1, len(self._points))
        )

    def start(self, feature_type: str):
        if self._state == TeachInState.RECORDING:
            raise RuntimeError("TeachIn already recording")
        self._feature_type = feature_type
        self._points.clear()
        self._point_times.clear()
        self._last_point = None
        self._last_time = None
        self._left_start_area = False
        self._state = TeachInState.RECORDING

    def update(self, pose: Pose):
        if self._state != TeachInState.RECORDING:
            return

        x, y, ts = pose.utm_x, pose.utm_y, pose.timestamp

        if self._last_point is None:
            self._record(x, y, ts)
            return

        dist = _distance(self._last_point[0], self._last_point[1], x, y)
        elapsed = ts - self._last_time
        if dist >= self._dist_thresh or elapsed >= self._time_thresh:
            self._record(x, y, ts)

        if self._points:
            start_x, start_y = self._points[0]
            start_distance = _distance(start_x, start_y, x, y)
            # Hysteresis: merely touching the close radius during the first
            # straight/corner must not count as leaving and returning.
            if start_distance >= self._close_thresh * 1.5:
                self._left_start_area = True
            if (self._left_start_area and len(self._points) >= self._min_points
                    and start_distance < self._close_thresh):
                # Record the closing pose so the final boundary segment is complete
                self._record(x, y, ts)
                logger.info("TeachIn auto-closed after %d points", len(self._points))
                self._state = TeachInState.CLOSED

    def cancel(self):
        self._state = TeachInState.CANCELLED

    def trim_last_distance(self, distance_m: float) -> float:
        """Remove a distance from the recorded tail and reopen the recording.

        The final segment is interpolated when only part of it must be removed,
        so the correction target is not limited to the sampling interval.
        At least the first point is retained as the boundary anchor.
        """
        if distance_m <= 0:
            raise ValueError("distance_m must be positive")
        if self._state not in (TeachInState.RECORDING, TeachInState.CLOSED):
            raise RuntimeError("TeachIn is not recording")
        removed = 0.0
        while len(self._points) > 1 and removed < distance_m:
            end = self._points[-1]
            start = self._points[-2]
            segment = _distance(*start, *end)
            remaining = distance_m - removed
            if segment <= remaining + 1e-9:
                self._points.pop()
                self._point_times.pop()
                removed += segment
                continue
            ratio = remaining / segment
            self._points[-1] = (
                end[0] + (start[0] - end[0]) * ratio,
                end[1] + (start[1] - end[1]) * ratio,
            )
            end_time = self._point_times[-1]
            start_time = self._point_times[-2]
            self._point_times[-1] = end_time + (start_time - end_time) * ratio
            removed += remaining

        self._last_point = self._points[-1] if self._points else None
        self._last_time = self._point_times[-1] if self._point_times else None
        if self._points:
            start_x, start_y = self._points[0]
            self._left_start_area = any(
                _distance(start_x, start_y, x, y) >= self._close_thresh * 1.5
                for x, y in self._points[1:]
            )
        else:
            self._left_start_area = False
        self._state = TeachInState.RECORDING
        return removed

    def to_geojson_feature(self) -> dict:
        if not self._points:
            raise RuntimeError("No points recorded — call start() and update() before to_geojson_feature()")
        is_draft = self._state != TeachInState.CLOSED
        coords = [[x, y] for x, y in self._points]
        if coords and coords[0] != coords[-1]:
            coords.append(coords[0])  # close the ring

        props = {"type": self._feature_type}
        if is_draft:
            props["draft"] = True

        return {
            "type": "Feature",
            "properties": props,
            "geometry": {
                "type": "Polygon",
                "coordinates": [coords],
            },
        }

    def _record(self, x: float, y: float, ts: float):
        self._points.append((x, y))
        self._point_times.append(ts)
        self._last_point = (x, y)
        self._last_time = ts
