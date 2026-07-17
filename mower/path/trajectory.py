"""Arc-length based continuous reference trajectory for vehicle control."""

from __future__ import annotations

import bisect
import math
from dataclasses import dataclass

from mower.path.tractor_route import TractorWaypoint


def _angle_delta(value: float) -> float:
    return (value + math.pi) % (2.0 * math.pi) - math.pi


@dataclass(frozen=True)
class TrajectorySample:
    x: float
    y: float
    heading_rad: float
    curvature: float
    gear: int
    operation: str
    turn_id: int | None
    s: float
    segment_index: int


@dataclass(frozen=True)
class Projection:
    sample: TrajectorySample
    cross_track_m: float
    distance_m: float


class ContinuousTrajectory:
    """Piecewise-continuous path with stable projection and arc-length lookup.

    The planner may sample curves very densely, but the controller no longer
    treats every sample as a target that must be individually reached.
    """

    def __init__(self, waypoints: list[TractorWaypoint]) -> None:
        filtered: list[TractorWaypoint] = []
        for point in waypoints:
            if filtered and math.hypot(point.x - filtered[-1].x, point.y - filtered[-1].y) < 1e-8:
                if point.gear == filtered[-1].gear and point.operation == filtered[-1].operation:
                    continue
            filtered.append(point)
        if len(filtered) < 2:
            raise ValueError("trajectory needs at least two distinct points")
        self.points = tuple(filtered)
        self._s = [0.0]
        self._segment_heading: list[float] = []
        for left, right in zip(filtered, filtered[1:]):
            dx, dy = right.x - left.x, right.y - left.y
            length = math.hypot(dx, dy)
            self._s.append(self._s[-1] + length)
            self._segment_heading.append(math.atan2(dy, dx))
        self._segment_curvature = self._compute_curvatures()

    @property
    def length_m(self) -> float:
        return self._s[-1]

    def arc_length_at_waypoint(self, index: int) -> float:
        return self._s[max(0, min(len(self._s) - 1, int(index)))]

    def _compute_curvatures(self) -> list[float]:
        result: list[float] = []
        for index, heading in enumerate(self._segment_heading):
            left = max(0, index - 3)
            right = min(len(self._segment_heading) - 1, index + 3)
            span = self._s[right + 1] - self._s[left]
            if span < 1e-8:
                result.append(0.0)
                continue
            delta = _angle_delta(self._segment_heading[right] - self._segment_heading[left])
            result.append(delta / span)
        return result

    def sample(self, s: float) -> TrajectorySample:
        s = max(0.0, min(self.length_m, float(s)))
        index = min(len(self._segment_heading) - 1, max(0, bisect.bisect_right(self._s, s) - 1))
        length = self._s[index + 1] - self._s[index]
        fraction = 0.0 if length < 1e-9 else (s - self._s[index]) / length
        left, right = self.points[index], self.points[index + 1]
        return TrajectorySample(
            left.x + (right.x - left.x) * fraction,
            left.y + (right.y - left.y) * fraction,
            self._segment_heading[index],
            self._segment_curvature[index],
            right.gear,
            right.operation,
            right.turn_id,
            s,
            index,
        )

    def project(
        self,
        x: float,
        y: float,
        *,
        previous_s: float = 0.0,
        window_m: float = 2.5,
        heading_rad: float | None = None,
        gear: int = 1,
        max_forward_m: float | None = None,
    ) -> Projection:
        """Project locally without mistaking an adjacent return lane for progress.

        Position alone is ambiguous in a narrow hairpin: the opposite lane can
        be closer than the lane the tractor is actually following.  Its travel
        direction and the physically possible progress since the previous
        control tick disambiguate that case.
        """
        centre = min(len(self._segment_heading) - 1, max(0, bisect.bisect_right(self._s, previous_s) - 1))
        first = centre
        while first > 0 and self._s[centre] - self._s[first] < window_m:
            first -= 1
        last = centre
        while last < len(self._segment_heading) - 1 and self._s[last + 1] - self._s[centre] < window_m:
            last += 1
        best: tuple[float, float, int, float, float] | None = None
        for index in range(first, last + 1):
            a, b = self.points[index], self.points[index + 1]
            dx, dy = b.x - a.x, b.y - a.y
            denominator = dx * dx + dy * dy
            fraction = 0.0 if denominator < 1e-12 else max(0.0, min(1.0, ((x - a.x) * dx + (y - a.y) * dy) / denominator))
            px, py = a.x + dx * fraction, a.y + dy * fraction
            distance = math.hypot(x - px, y - py)
            s = self._s[index] + math.sqrt(denominator) * fraction
            # A small backward penalty avoids oscillating between adjacent
            # samples.  Penalize heading-incompatible candidates much more
            # strongly: the next arm of a hairpin is close in position, but it
            # points in the opposite direction.
            score = distance + max(0.0, previous_s - s - 0.08) * 2.0
            if heading_rad is not None:
                travel_heading = heading_rad + (math.pi if gear < 0 else 0.0)
                heading_error = abs(_angle_delta(
                    self._segment_heading[index] - travel_heading
                ))
                score += 0.40 * heading_error
            if max_forward_m is not None:
                score += max(
                    0.0, s - previous_s - max(0.0, max_forward_m),
                ) * 8.0
            if best is None or score < best[0]:
                cross = math.sin(self._segment_heading[index]) * (x - px) - math.cos(self._segment_heading[index]) * (y - py)
                best = score, s, index, cross, distance
        assert best is not None
        selected_s = best[1]
        if max_forward_m is not None:
            selected_s = min(
                selected_s,
                previous_s + max(0.0, max_forward_m),
            )
        return Projection(self.sample(selected_s), best[3], best[4])

    def next_gear_change_s(self, s: float) -> float:
        """Arc length of the next gear boundary after ``s`` (length if none).

        The returned value is the exact cusp position: the arc length of the
        first point whose outgoing segment drives in the opposite gear.
        """
        s = max(0.0, min(self.length_m, float(s)))
        index = min(len(self._segment_heading) - 1, max(0, bisect.bisect_right(self._s, s) - 1))
        gear = self.points[index + 1].gear
        for i in range(index + 1, len(self._segment_heading)):
            if self.points[i + 1].gear != gear:
                return self._s[i]
        return self.length_m

    def clamp_to_current_motion(self, start_s: float, target_s: float) -> float:
        """Let preview cross operation boundaries, but never a gear switch.

        Seeing the beginning of the next mowing lane while finishing a turn is
        essential for an early, tangent-aligned exit.  A direction change is
        different: it still requires a deliberate stop before the preview may
        continue in the opposite gear.
        """
        start = self.sample(start_s)
        index = start.segment_index + 1
        limit = min(self.length_m, target_s)
        while index < len(self._segment_heading) and self._s[index] < limit:
            point = self.points[index + 1]
            if point.gear != start.gear:
                return max(start_s, self._s[index] - 1e-5)
            index += 1
        return limit
