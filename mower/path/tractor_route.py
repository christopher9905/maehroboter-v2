"""Tractor-aware coverage route expansion and turn manoeuvre planning."""

from __future__ import annotations

import heapq
import itertools
import math
from dataclasses import dataclass
from functools import lru_cache
from typing import Callable, Sequence

import numpy as np
from shapely import contains_xy
from shapely.geometry import LineString, MultiPolygon, Polygon

from mower.path.boustrophedon import compute_boustrophedon


TURN_STRATEGIES = frozenset({"auto", "forward", "reverse", "multi_point"})

RouteValidator = Callable[[Sequence["TractorWaypoint"]], bool]


@dataclass(frozen=True)
class TractorWaypoint:
    x: float
    y: float
    operation: str = "mow"  # mow | headland | turn
    gear: int = 1             # +1 forward | -1 reverse
    turn_id: int | None = None


def measure_minimum_turn_radius(
    waypoints: Sequence[TractorWaypoint],
) -> tuple[float, int | None]:
    """Measure the tightest executable turn without crossing gear seams.

    The planner stores dense position samples rather than a steering command.
    Local heading change per travelled metre therefore gives the curvature
    actually presented to the controller. Operation and gear boundaries are
    deliberately skipped because a stopped gear change has no geometric
    curvature and must not be interpreted as an instantaneous steering arc.
    """
    minimum_radius = math.inf
    minimum_index: int | None = None
    for index in range(2, len(waypoints) - 2):
        left, middle, right = (
            waypoints[index - 1], waypoints[index], waypoints[index + 1],
        )
        local_window = waypoints[index - 2:index + 3]
        if (
            middle.turn_id is None
            or any(point.operation != "turn" for point in local_window)
            or any(point.turn_id != middle.turn_id for point in local_window)
            or any(point.gear != middle.gear for point in local_window)
        ):
            continue
        left_distance = math.hypot(middle.x - left.x, middle.y - left.y)
        right_distance = math.hypot(right.x - middle.x, right.y - middle.y)
        mean_distance = (left_distance + right_distance) / 2.0
        if mean_distance <= 1e-7:
            continue
        left_heading = math.atan2(middle.y - left.y, middle.x - left.x)
        right_heading = math.atan2(right.y - middle.y, right.x - middle.x)
        heading_change = abs(_normalize_angle(right_heading - left_heading))
        if heading_change <= 1e-9:
            continue
        # ``mean_distance`` is a chord, not the arc length. Dividing it by
        # the angle would report a nominal 25 cm sampled arc a fraction too
        # small. The exact chord relation keeps the diagnostic and acceptance
        # threshold in physical metres.
        radius = mean_distance / (2.0 * math.sin(heading_change / 2.0))
        if radius < minimum_radius:
            minimum_radius = radius
            minimum_index = index
    return minimum_radius, minimum_index


def _unit(dx: float, dy: float) -> tuple[float, float]:
    length = math.hypot(dx, dy)
    if length < 1e-9:
        return 1.0, 0.0
    return dx / length, dy / length


def _append_leg(
    route: list[TractorWaypoint],
    target: tuple[float, float],
    *,
    point_spacing: float,
    operation: str,
    gear: int,
    turn_id: int | None,
) -> None:
    start = route[-1]
    dx, dy = target[0] - start.x, target[1] - start.y
    distance = math.hypot(dx, dy)
    if distance < 1e-9:
        return
    steps = max(1, math.ceil(distance / point_spacing))
    for step in range(1, steps + 1):
        fraction = step / steps
        route.append(TractorWaypoint(
            start.x + dx * fraction,
            start.y + dy * fraction,
            operation,
            1 if gear >= 0 else -1,
            turn_id,
        ))


def _largest_polygon(geometry: Polygon | MultiPolygon) -> Polygon | None:
    if isinstance(geometry, Polygon):
        return geometry if not geometry.is_empty else None
    if isinstance(geometry, MultiPolygon) and geometry.geoms:
        return max(geometry.geoms, key=lambda item: item.area)
    return None


def compute_headland_route(
    zone: Polygon,
    *,
    lane_spacing: float,
    mowing_width: float | None = None,
    point_spacing: float = 0.005,
    turn_radius: float = 0.25,
    passes: int = 1,
    start_pose: tuple[float, float, float] | None = None,
    connection_boundary: Polygon | None = None,
    route_validator: RouteValidator | None = None,
    corner_clearance_m: float = 0.0,
) -> list[TractorWaypoint]:
    """Plan one or more outside-first perimeter lanes with rounded corners.

    All passes are derived from one common rounded perimeter.  Its outer
    centreline radius is enlarged by the total pass depth, so every inward
    offset remains concentric and the innermost pass still keeps at least the
    configured tractor radius.  Building each pass with an independent,
    identical corner radius would move the arc centres diagonally and enlarge
    a 90-degree pass gap by ``sqrt(2)``.
    """
    if lane_spacing <= 0 or point_spacing <= 0 or turn_radius <= 0:
        raise ValueError("Headland spacing and turn radius must be positive")
    mowing_width = lane_spacing if mowing_width is None else max(lane_spacing, mowing_width)
    passes = max(1, min(3, int(passes)))
    maximum_mower_offset = mowing_width / 2.0 + (passes - 1) * lane_spacing
    family_opening_radius = (
        turn_radius + maximum_mower_offset + max(0.0, corner_clearance_m)
    )
    core = _largest_polygon(zone.buffer(-family_opening_radius))
    if core is None:
        raise ValueError(
            f"{passes} Außenbahnen passen mit {turn_radius:.2f} m "
            "Mindestwenderadius nicht lückenlos in die Zone"
        )
    route: list[TractorWaypoint] = []
    turn_id = 0

    for pass_index in range(passes):
        mower_offset = mowing_width / 2.0 + pass_index * lane_spacing
        centerline_radius = family_opening_radius - mower_offset
        quad_segments = max(
            16,
            math.ceil(math.pi * centerline_radius / (2.0 * point_spacing)),
        )
        # Dilate the same core directly to each requested centreline radius.
        # Avoiding a second negative buffer keeps the arcs exactly concentric
        # and prevents numerical curvature spikes at polygon segment joints.
        center_area = _largest_polygon(core.buffer(
            centerline_radius, join_style="round", quad_segs=quad_segments,
        ))
        if center_area is None or center_area.length <= 0:
            raise ValueError(f"Außenbahn {pass_index + 1} ist geometrisch leer")

        ring_line = LineString(center_area.exterior.coords)
        sample_count = max(3, math.ceil(ring_line.length / point_spacing))
        ring = [
            ring_line.interpolate(index * ring_line.length / sample_count).coords[0]
            for index in range(sample_count)
        ]
        if len(ring) < 3:
            raise ValueError(f"Außenbahn {pass_index + 1} enthält zu wenige Punkte")
        reference_pose = None
        if route and len(route) >= 2:
            reference_pose = (
                route[-1].x,
                route[-1].y,
                math.atan2(route[-1].y - route[-2].y, route[-1].x - route[-2].x),
            )
        elif not route:
            reference_pose = start_pose

        connection: list[TractorWaypoint] | None = None
        if reference_pose is not None:
            nearest_index = min(
                range(len(ring)),
                key=lambda index: math.hypot(
                    ring[index][0] - reference_pose[0],
                    ring[index][1] - reference_pose[1],
                ),
            )
            # The nearest point on a parallel perimeter can be too close for a
            # legal S-curve. Sample progressively farther entry points so the
            # tractor can change lanes gently instead of taking a full loop.
            entry_distances = (
                0.0, 0.25, 0.50, 0.75, 1.0, 1.5,
                max(2.0, 2.0 * turn_radius),
            )
            candidate_indices = sorted({
                (nearest_index + direction * round(distance / point_spacing)) % len(ring)
                for distance in entry_distances
                for direction in (-1, 1)
            })
            best: tuple[float, list[tuple[float, float]], list[TractorWaypoint] | None] | None = None
            fallback_orientations: list[
                tuple[float, list[tuple[float, float]], float]
            ] = []
            for index in candidate_indices:
                for direction in (1, -1):
                    oriented = [
                        ring[(index + direction * offset) % len(ring)]
                        for offset in range(len(ring))
                    ]
                    target_heading = math.atan2(
                        oriented[1][1] - oriented[0][1],
                        oriented[1][0] - oriented[0][0],
                    )
                    if route:
                        geometric_score = (
                            math.hypot(
                                oriented[0][0] - reference_pose[0],
                                oriented[0][1] - reference_pose[1],
                            )
                            + 2.0
                            * abs(_normalize_angle(target_heading - reference_pose[2]))
                        )
                        fallback_orientations.append(
                            (geometric_score, oriented, target_heading)
                        )
                        try:
                            candidate_connection = compute_curvature_bounded_connection(
                                reference_pose,
                                (oriented[0][0], oriented[0][1], target_heading),
                                turn_radius=turn_radius,
                                point_spacing=point_spacing,
                                boundary=connection_boundary if connection_boundary is not None else zone,
                                allow_reverse=True,
                                allow_multi_point=False,
                                operation="turn",
                                turn_id=turn_id + 1,
                                route_validator=route_validator,
                            )
                        except ValueError:
                            continue
                        score = sum(
                            math.hypot(right.x - left.x, right.y - left.y)
                            for left, right in zip(candidate_connection, candidate_connection[1:])
                        )
                    else:
                        candidate_connection = None
                        score = math.hypot(
                            oriented[0][0] - reference_pose[0],
                            oriented[0][1] - reference_pose[1],
                        ) + 2.0 * abs(_normalize_angle(target_heading - reference_pose[2]))
                    if best is None or score < best[0]:
                        best = (score, oriented, candidate_connection)
            if best is None and route:
                for _, oriented, target_heading in sorted(fallback_orientations)[:3]:
                    try:
                        candidate_connection = compute_curvature_bounded_connection(
                            reference_pose,
                            (oriented[0][0], oriented[0][1], target_heading),
                            turn_radius=turn_radius,
                            point_spacing=point_spacing,
                            boundary=connection_boundary if connection_boundary is not None else zone,
                            allow_reverse=True,
                            operation="turn",
                            turn_id=turn_id + 1,
                            route_validator=route_validator,
                        )
                    except ValueError:
                        continue
                    best = (
                        sum(
                            math.hypot(right.x - left.x, right.y - left.y)
                            for left, right in zip(
                                candidate_connection, candidate_connection[1:]
                            )
                        ),
                        oriented,
                        candidate_connection,
                    )
                    break
            if best is not None:
                ring = best[1]
                connection = best[2]

        if route:
            if connection is None:
                raise ValueError(
                    f"Keine radiusbegrenzte Verbindung zu Außenbahn {pass_index + 1} gefunden"
                )
            turn_id += 1
            route.extend(connection[1:])
        else:
            route.append(TractorWaypoint(ring[0][0], ring[0][1], "headland"))

        for target in ring[1:] + [ring[0]]:
            _append_leg(
                route, target, point_spacing=point_spacing,
                operation="headland", gear=1, turn_id=None,
            )
    return route


def _normalize_angle(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def _advance_pose(
    pose: tuple[float, float, float],
    gear: int,
    curvature: float,
    distance: float,
) -> tuple[float, float, float]:
    """Advance an Ackermann/bicycle pose along one constant-steering arc."""
    x, y, heading = pose
    signed_distance = (1 if gear >= 0 else -1) * distance
    new_heading = heading + curvature * signed_distance
    if abs(curvature) < 1e-12:
        return (
            x + signed_distance * math.cos(heading),
            y + signed_distance * math.sin(heading),
            new_heading,
        )
    return (
        x + (math.sin(new_heading) - math.sin(heading)) / curvature,
        y + (-math.cos(new_heading) + math.cos(heading)) / curvature,
        new_heading,
    )


def _mod2pi(angle: float) -> float:
    return angle % (2.0 * math.pi)


def _dubins_words(alpha: float, beta: float, distance: float):
    """Yield all valid normalized Dubins words for two oriented poses."""
    sa, sb = math.sin(alpha), math.sin(beta)
    ca, cb = math.cos(alpha), math.cos(beta)
    cab = math.cos(alpha - beta)
    distance_sq = distance * distance

    p_sq = 2.0 + distance_sq - 2.0 * cab + 2.0 * distance * (sa - sb)
    if p_sq >= -1e-12:
        p = math.sqrt(max(0.0, p_sq))
        tmp = math.atan2(cb - ca, distance + sa - sb)
        yield "LSL", (_mod2pi(-alpha + tmp), p, _mod2pi(beta - tmp))

    p_sq = 2.0 + distance_sq - 2.0 * cab + 2.0 * distance * (-sa + sb)
    if p_sq >= -1e-12:
        p = math.sqrt(max(0.0, p_sq))
        tmp = math.atan2(ca - cb, distance - sa + sb)
        yield "RSR", (_mod2pi(alpha - tmp), p, _mod2pi(-beta + tmp))

    p_sq = -2.0 + distance_sq + 2.0 * cab + 2.0 * distance * (sa + sb)
    if p_sq >= -1e-12:
        p = math.sqrt(max(0.0, p_sq))
        tmp = math.atan2(-ca - cb, distance + sa + sb) - math.atan2(-2.0, p)
        yield "LSR", (_mod2pi(-alpha + tmp), p, _mod2pi(-beta + tmp))

    p_sq = distance_sq - 2.0 + 2.0 * cab - 2.0 * distance * (sa + sb)
    if p_sq >= -1e-12:
        p = math.sqrt(max(0.0, p_sq))
        tmp = math.atan2(ca + cb, distance - sa - sb) - math.atan2(2.0, p)
        yield "RSL", (_mod2pi(alpha - tmp), p, _mod2pi(beta - tmp))

    tmp = (6.0 - distance_sq + 2.0 * cab + 2.0 * distance * (sa - sb)) / 8.0
    if abs(tmp) <= 1.0 + 1e-12:
        middle = _mod2pi(2.0 * math.pi - math.acos(max(-1.0, min(1.0, tmp))))
        first = _mod2pi(alpha - math.atan2(ca - cb, distance - sa + sb) + middle / 2.0)
        yield "RLR", (first, middle, _mod2pi(alpha - beta - first + middle))

    tmp = (6.0 - distance_sq + 2.0 * cab + 2.0 * distance * (-sa + sb)) / 8.0
    if abs(tmp) <= 1.0 + 1e-12:
        middle = _mod2pi(2.0 * math.pi - math.acos(max(-1.0, min(1.0, tmp))))
        first = _mod2pi(-alpha - math.atan2(ca - cb, distance + sa - sb) + middle / 2.0)
        yield "LRL", (first, middle, _mod2pi(beta - alpha - first + middle))


def _sample_dubins_candidate(
    start: tuple[float, float, float],
    target: tuple[float, float, float],
    word: str,
    parameters: tuple[float, float, float],
    *,
    turn_radius: float,
    point_spacing: float,
    gear: int,
    operation: str,
    turn_id: int | None,
) -> tuple[list[TractorWaypoint], float] | None:
    virtual_pose = start
    points = [TractorWaypoint(start[0], start[1], operation, gear, turn_id)]
    total_length = 0.0
    for mode, normalized_length in zip(word, parameters):
        length = normalized_length * turn_radius
        if length <= 1e-10:
            continue
        steps = max(1, math.ceil(length / point_spacing))
        step_length = length / steps
        curvature = {"L": 1.0 / turn_radius, "R": -1.0 / turn_radius, "S": 0.0}[mode]
        for _ in range(steps):
            virtual_pose = _advance_pose(virtual_pose, 1, curvature, step_length)
            points.append(TractorWaypoint(
                virtual_pose[0], virtual_pose[1], operation, gear, turn_id,
            ))
        total_length += length
    position_error = math.hypot(virtual_pose[0] - target[0], virtual_pose[1] - target[1])
    heading_error = abs(_normalize_angle(virtual_pose[2] - target[2]))
    if position_error > 1e-5 or heading_error > 1e-5:
        return None
    # Resample the complete word uniformly. This removes tiny remainder steps
    # at primitive boundaries and keeps the controller's curvature estimate
    # stable when it crosses from arc to straight or from one arc to another.
    line = LineString([(point.x, point.y) for point in points])
    sample_count = max(1, math.ceil(line.length / point_spacing))
    points = [
        TractorWaypoint(
            *line.interpolate(index * line.length / sample_count).coords[0],
            operation,
            gear,
            turn_id,
        )
        for index in range(sample_count + 1)
    ]
    points[-1] = TractorWaypoint(target[0], target[1], operation, gear, turn_id)
    return points, total_length


def _longest_reverse_run(points: Sequence[TractorWaypoint]) -> float:
    """Return the longest uninterrupted reverse section in metres."""
    longest = 0.0
    current = 0.0
    for left, right in zip(points, points[1:]):
        distance = math.hypot(right.x - left.x, right.y - left.y)
        if right.gear < 0:
            current += distance
            longest = max(longest, current)
        else:
            current = 0.0
    return longest


def _physical_path_rotation(points: Sequence[TractorWaypoint]) -> float:
    """Return accumulated absolute tractor-heading change in radians."""
    headings: list[float] = []
    for left, right in zip(points, points[1:]):
        if math.hypot(right.x - left.x, right.y - left.y) <= 1e-9:
            continue
        heading = math.atan2(right.y - left.y, right.x - left.x)
        if right.gear < 0:
            heading = _normalize_angle(heading + math.pi)
        headings.append(heading)
    return sum(
        abs(_normalize_angle(right - left))
        for left, right in zip(headings, headings[1:])
    )


def _dubins_turn_is_reasonable(
    start_heading: float,
    target_heading: float,
    word: str,
    parameters: tuple[float, float, float],
) -> bool:
    """Reject full-circle Dubins detours for a small heading correction.

    A mathematical Dubins solution always exists, but for nearby poses its
    shortest forward-only word can still contain an almost complete circle.
    A real tractor should use a short multi-point correction in that case.
    """
    direct = abs(_normalize_angle(target_heading - start_heading))
    accumulated_turn = sum(
        value for mode, value in zip(word, parameters) if mode != "S"
    )
    maximum_turn = max(math.pi, direct + math.pi / 2.0)
    return accumulated_turn <= maximum_turn + 1e-6


def _refine_generic_actions(
    actions: list[tuple[int, int, float]],
    target: tuple[float, float, float],
    *,
    turn_radius: float,
) -> tuple[tuple[int, int, float], ...] | None:
    """Refine a coarse hybrid-search result to an exact oriented target."""
    runs: list[list[float | int]] = []
    for gear, steering, length in actions:
        if runs and runs[-1][0] == gear and runs[-1][1] == steering:
            runs[-1][2] = float(runs[-1][2]) + length
        else:
            runs.append([gear, steering, length])
    if len(runs) < 3:
        return None
    lengths = np.array([float(run[2]) for run in runs], dtype=float)
    lower = np.maximum(0.002, lengths - max(0.12, turn_radius * 0.8))
    upper = lengths + max(0.12, turn_radius * 0.8)

    def residual(values: np.ndarray) -> np.ndarray:
        pose = (0.0, 0.0, 0.0)
        for run, length in zip(runs, values):
            pose = _advance_pose(
                pose, int(run[0]), int(run[1]) / turn_radius, float(length),
            )
        return np.array([
            pose[0] - target[0],
            pose[1] - target[1],
            turn_radius * _normalize_angle(pose[2] - target[2]),
        ])

    damping = 1e-4
    for _ in range(80):
        error = residual(lengths)
        error_norm = float(np.linalg.norm(error))
        if error_norm < 1e-7:
            break
        epsilon = 1e-5
        jacobian = np.empty((3, len(lengths)), dtype=float)
        for index in range(len(lengths)):
            perturbed = lengths.copy()
            perturbed[index] += epsilon
            jacobian[:, index] = (residual(perturbed) - error) / epsilon
        try:
            correction = -jacobian.T @ np.linalg.solve(
                jacobian @ jacobian.T + damping * np.eye(3), error,
            )
        except np.linalg.LinAlgError:
            damping *= 10.0
            continue
        improved = False
        for scale in (1.0, 0.5, 0.25, 0.1):
            candidate = np.clip(lengths + scale * correction, lower, upper)
            if float(np.linalg.norm(residual(candidate))) < error_norm:
                lengths = candidate
                damping = max(1e-8, damping * 0.3)
                improved = True
                break
        if not improved:
            damping *= 10.0
    if float(np.linalg.norm(residual(lengths))) >= 1e-5:
        return None
    return tuple(
        (int(run[0]), int(run[1]), float(length))
        for run, length in zip(runs, lengths)
    )


def _compute_hybrid_connection(
    start: tuple[float, float, float],
    target: tuple[float, float, float],
    *,
    turn_radius: float,
    point_spacing: float,
    boundary: Polygon | None,
    operation: str,
    turn_id: int | None,
    maximum_shunts: int = 6,
    route_validator: RouteValidator | None = None,
) -> list[TractorWaypoint] | None:
    """Find a boundary-aware forward/reverse connector with exact curvature."""
    dx, dy = target[0] - start[0], target[1] - start[1]
    cos_h, sin_h = math.cos(start[2]), math.sin(start[2])
    local_target = (
        dx * cos_h + dy * sin_h,
        -dx * sin_h + dy * cos_h,
        _normalize_angle(target[2] - start[2]),
    )
    step_length = min(0.05, max(0.025, turn_radius / 6.0))
    xy_resolution = step_length * 0.65
    heading_resolution = math.radians(7.5)
    search_extent = math.hypot(dx, dy) + 4.0 * turn_radius + 0.5
    allowed = boundary.buffer(0.01) if boundary is not None else None
    start_state = (0.0, 0.0, 0.0, 1, 0, 0)

    def state_key(state: tuple[float, float, float, int, int, int]) -> tuple[int, ...]:
        return (
            round(state[0] / xy_resolution),
            round(state[1] / xy_resolution),
            round(_normalize_angle(state[2]) / heading_resolution),
            state[3], state[4], state[5],
        )

    def heuristic(state: tuple[float, float, float, int, int, int]) -> float:
        distance = math.hypot(state[0] - local_target[0], state[1] - local_target[1])
        heading = abs(_normalize_angle(state[2] - local_target[2]))
        return distance + 0.45 * turn_radius * heading

    def is_allowed(x: float, y: float) -> bool:
        if allowed is None:
            return True
        global_x = start[0] + x * cos_h - y * sin_h
        global_y = start[1] + x * sin_h + y * cos_h
        return bool(contains_xy(allowed, global_x, global_y))

    start_key = state_key(start_state)
    counter = itertools.count()
    queue = [(heuristic(start_state), 0.0, next(counter), start_state)]
    best_cost = {start_key: 0.0}
    parents: dict[tuple[int, ...], tuple[tuple[int, ...], tuple[int, int, float]]] = {}

    for _ in range(250_000):
        if not queue:
            break
        _, cost, _, state = heapq.heappop(queue)
        key = state_key(state)
        if cost > best_cost.get(key, math.inf) + 1e-9:
            continue
        if (
            state[3] == 1
            and math.hypot(state[0] - local_target[0], state[1] - local_target[1])
            < max(0.04, step_length * 1.4)
            and abs(_normalize_angle(state[2] - local_target[2])) < math.radians(11.0)
        ):
            raw_actions: list[tuple[int, int, float]] = []
            cursor = key
            while cursor != start_key:
                parent, action = parents[cursor]
                raw_actions.append(action)
                cursor = parent
            raw_actions.reverse()
            refined = _refine_generic_actions(
                raw_actions, local_target, turn_radius=turn_radius,
            )
            if refined is not None:
                pose = start
                points = [TractorWaypoint(
                    start[0], start[1], operation, refined[0][0], turn_id,
                )]
                for gear, steering, length in refined:
                    steps = max(1, math.ceil(length / point_spacing))
                    substep = length / steps
                    for _ in range(steps):
                        pose = _advance_pose(
                            pose, gear, steering / turn_radius, substep,
                        )
                        points.append(TractorWaypoint(
                            pose[0], pose[1], operation, gear, turn_id,
                        ))
                if (
                    math.hypot(pose[0] - target[0], pose[1] - target[1]) < 1e-5
                    and abs(_normalize_angle(pose[2] - target[2])) < 1e-5
                ):
                    points[-1] = TractorWaypoint(
                        target[0], target[1], operation, points[-1].gear, turn_id,
                    )
                    path = LineString([(point.x, point.y) for point in points])
                    if allowed is not None and not allowed.covers(path):
                        continue
                    if route_validator is not None and not route_validator(points):
                        continue
                    return points

        for gear in (-1, 1):
            switches = state[5] + int(gear != state[3])
            if switches > maximum_shunts:
                continue
            for steering in (-1, 0, 1):
                x, y, heading = _advance_pose(
                    state[:3], gear, steering / turn_radius, step_length,
                )
                if math.hypot(x, y) > search_extent:
                    continue
                if not is_allowed(x, y):
                    continue
                candidate = (
                    x, y, _normalize_angle(heading), gear, steering, switches,
                )
                candidate_key = state_key(candidate)
                candidate_cost = (
                    cost + step_length * (4.0 if gear < 0 else 1.0)
                    + (0.75 if gear != state[3] else 0.0)
                    + (0.008 if steering != state[4] else 0.0)
                )
                if candidate_cost >= best_cost.get(candidate_key, math.inf):
                    continue
                best_cost[candidate_key] = candidate_cost
                parents[candidate_key] = (key, (gear, steering, step_length))
                heapq.heappush(queue, (
                    candidate_cost + 1.35 * heuristic(candidate),
                    candidate_cost,
                    next(counter),
                    candidate,
                ))
    return None


def compute_curvature_bounded_connection(
    start: tuple[float, float, float],
    target: tuple[float, float, float],
    *,
    turn_radius: float = 0.25,
    point_spacing: float = 0.005,
    boundary: Polygon | None = None,
    allow_reverse: bool = True,
    allow_multi_point: bool = True,
    operation: str = "turn",
    turn_id: int | None = None,
    route_validator: RouteValidator | None = None,
    allow_staging: bool = True,
) -> list[TractorWaypoint]:
    """Return the shortest exact Dubins connection accepted by ``boundary``.

    Forward and, when requested, all-reverse candidates are considered. Every
    curved primitive has exactly ``turn_radius``; straight primitives have zero
    curvature. The returned list contains both oriented endpoints.
    """
    if turn_radius <= 0 or point_spacing <= 0:
        raise ValueError("Connection radius and spacing must be positive")
    candidates: list[tuple[int, float, list[TractorWaypoint]]] = []
    # A short recovery shunt may need roughly one metre so the complete
    # front/rear implement clears a field edge.  Longer uninterrupted reverse
    # transits are never accepted as a substitute for a forward connection.
    maximum_reverse_run = max(1.0, min(1.50, 3.0 * turn_radius))
    direct_heading_change = abs(_normalize_angle(target[2] - start[2]))
    maximum_path_rotation = max(
        math.pi, direct_heading_change + math.pi / 2.0,
    )
    drive_modes = [(1, start, target)]
    if allow_reverse:
        drive_modes.append((
            -1,
            (start[0], start[1], _normalize_angle(start[2] + math.pi)),
            (target[0], target[1], _normalize_angle(target[2] + math.pi)),
        ))
    for gear, virtual_start, virtual_target in drive_modes:
        dx = (virtual_target[0] - virtual_start[0]) / turn_radius
        dy = (virtual_target[1] - virtual_start[1]) / turn_radius
        distance = math.hypot(dx, dy)
        theta = math.atan2(dy, dx) if distance > 1e-12 else 0.0
        alpha = _mod2pi(virtual_start[2] - theta)
        beta = _mod2pi(virtual_target[2] - theta)
        for word, parameters in _dubins_words(alpha, beta, distance):
            if not _dubins_turn_is_reasonable(
                virtual_start[2], virtual_target[2], word, parameters,
            ):
                continue
            sampled = _sample_dubins_candidate(
                virtual_start,
                virtual_target,
                word,
                parameters,
                turn_radius=turn_radius,
                point_spacing=point_spacing,
                gear=gear,
                operation=operation,
                turn_id=turn_id,
            )
            if sampled is None:
                continue
            points, length = sampled
            if _physical_path_rotation(points) > maximum_path_rotation + 1e-3:
                continue
            if boundary is not None:
                path = LineString([(point.x, point.y) for point in points])
                if not boundary.buffer(0.003).covers(path):
                    continue
            if route_validator is not None and not route_validator(points):
                continue
            # Prefer normal forward transit. Pure reverse Dubins paths can be
            # a few centimetres shorter mathematically but look surprising,
            # require more deck/gear transitions and are harder to track.
            score = length if gear > 0 else 4.0 * length + max(0.75, 3.0 * turn_radius)
            if gear < 0 and _longest_reverse_run(points) > maximum_reverse_run:
                continue
            candidates.append((gear, score, points))
    forward_candidates = [item for item in candidates if item[0] > 0]
    if forward_candidates:
        return min(forward_candidates, key=lambda item: item[1])[2]
    if not candidates and allow_staging:
        # A pose close to a boundary can be physically valid while an
        # immediate minimum-radius arc would swing a longitudinally offset
        # mower over the edge.  Move straight into available space first and
        # only then start the radius-bounded connector.  Straight motion has
        # zero curvature, so the configured minimum radius remains intact.
        staging_distances = sorted({
            round(max(0.25, turn_radius * factor), 6)
            for factor in (2.0, 4.0)
        })
        for staging_gear in (1, -1):
            for staging_distance in staging_distances:
                staging_pose = _advance_pose(
                    start, staging_gear, 0.0, staging_distance,
                )
                prefix = [TractorWaypoint(
                    start[0], start[1], operation, staging_gear, turn_id,
                )]
                _append_leg(
                    prefix,
                    (staging_pose[0], staging_pose[1]),
                    point_spacing=point_spacing,
                    operation=operation,
                    gear=staging_gear,
                    turn_id=turn_id,
                )
                if boundary is not None:
                    prefix_line = LineString([(point.x, point.y) for point in prefix])
                    if not boundary.buffer(0.003).covers(prefix_line):
                        continue
                staged_validator: RouteValidator | None = None
                if route_validator is not None:
                    def staged_validator(
                        tail_candidate: Sequence[TractorWaypoint],
                        *,
                        prefix_candidate: tuple[TractorWaypoint, ...] = tuple(prefix),
                    ) -> bool:
                        return route_validator([
                            *prefix_candidate,
                            *tail_candidate[1:],
                        ])
                try:
                    tail = compute_curvature_bounded_connection(
                        staging_pose,
                        target,
                        turn_radius=turn_radius,
                        point_spacing=point_spacing,
                        boundary=boundary,
                        allow_reverse=allow_reverse,
                        allow_multi_point=allow_multi_point,
                        operation=operation,
                        turn_id=turn_id,
                        route_validator=staged_validator,
                        allow_staging=False,
                    )
                except ValueError:
                    continue
                combined = [*prefix, *tail[1:]]
                if _physical_path_rotation(combined) > maximum_path_rotation + 1e-3:
                    continue
                if route_validator is not None and not route_validator(combined):
                    continue
                reverse_distance = 0.0
                gear_changes = 0
                previous_gear = combined[0].gear
                total_length = 0.0
                for left, right in zip(combined, combined[1:]):
                    distance = math.hypot(right.x - left.x, right.y - left.y)
                    total_length += distance
                    if right.gear < 0:
                        reverse_distance += distance
                    if right.gear != previous_gear:
                        gear_changes += 1
                    previous_gear = right.gear
                score = total_length + 3.0 * reverse_distance + 0.75 * gear_changes
                if _longest_reverse_run(combined) <= maximum_reverse_run:
                    candidates.append((0, score, combined))
    # When no sensible forward-only connector exists, compare the short
    # forward/reverse manoeuvre against reverse candidates.  Previously the
    # mere existence of a Dubins loop or an all-reverse path prevented this
    # physically more realistic search from running.
    if not forward_candidates and allow_reverse and allow_multi_point:
        hybrid = _compute_hybrid_connection(
            start,
            target,
            turn_radius=turn_radius,
            point_spacing=point_spacing,
            boundary=boundary,
            operation=operation,
            turn_id=turn_id,
            route_validator=route_validator,
        )
        if (
            hybrid is not None
            and _longest_reverse_run(hybrid) <= maximum_reverse_run
            and _physical_path_rotation(hybrid) <= maximum_path_rotation + 1e-3
        ):
            reverse_distance = 0.0
            gear_changes = 0
            previous_gear = hybrid[0].gear
            for left, right in zip(hybrid, hybrid[1:]):
                distance = math.hypot(right.x - left.x, right.y - left.y)
                if right.gear < 0:
                    reverse_distance += distance
                if right.gear != previous_gear:
                    gear_changes += 1
                previous_gear = right.gear
            hybrid_length = sum(
                math.hypot(right.x - left.x, right.y - left.y)
                for left, right in zip(hybrid, hybrid[1:])
            )
            hybrid_score = hybrid_length + 3.0 * reverse_distance + 0.75 * gear_changes
            candidates.append((0, hybrid_score, hybrid))
    if not candidates:
        raise ValueError(
            "Keine Verbindung mit dem eingestellten Mindestwenderadius gefunden"
        )
    return min(candidates, key=lambda item: item[1])[2]


@lru_cache(maxsize=64)
def _search_turn_actions(
    lane_offset: float,
    turn_radius: float,
    maximum_shunts: int,
    boundary_clearance: float,
) -> tuple[tuple[int, int, float], ...]:
    """Search a collision-bounded, minimum-radius multi-point turn.

    The local start pose is ``(0, 0, 0)`` and the target is
    ``(0, lane_offset, pi)``. Actions contain gear, steering sign and length.
    Unlike the former straight-line V connectors, every primitive follows the
    configured Ackermann turning radius exactly.
    """
    lane_offset = round(lane_offset, 6)
    turn_radius = round(turn_radius, 6)
    clearance = max(0.05, round(boundary_clearance, 6))
    shunts = max(2, min(6, int(maximum_shunts)))
    if shunts % 2:
        shunts = min(6, shunts + 1)

    step_length = min(0.10, max(0.06, turn_radius / 12.0))
    xy_resolution = step_length * 0.60
    heading_resolution = math.radians(7.5)
    # The tractor arrives at the lane end in forward gear.  Starting the
    # search synthetically in reverse made the first backwards movement free
    # and produced implausibly long backing manoeuvres.
    start = (0.0, 0.0, 0.0, 1, 0, 0)  # x, y, heading, gear, steer, shunts

    def state_key(state: tuple[float, float, float, int, int, int]) -> tuple[int, ...]:
        return (
            round(state[0] / xy_resolution),
            round(state[1] / xy_resolution),
            round(_normalize_angle(state[2]) / heading_resolution),
            state[3], state[4], state[5],
        )

    def heuristic(state: tuple[float, float, float, int, int, int]) -> float:
        position = math.hypot(state[0], state[1] - lane_offset)
        heading = abs(_normalize_angle(state[2] - math.pi))
        return position + turn_radius * heading * 0.50

    start_key = state_key(start)
    counter = itertools.count()
    queue = [(heuristic(start), 0.0, next(counter), start)]
    best_cost = {start_key: 0.0}
    nodes = {start_key: start}
    parents: dict[tuple[int, ...], tuple[tuple[int, ...], tuple[int, int, float]]] = {}
    goal_key: tuple[int, ...] | None = None
    max_depth = max(1.5, min(5.0, turn_radius * 3.5))
    outward_limit = clearance * 0.95
    lateral_min = -clearance * 0.95
    lateral_max = lane_offset + clearance * 0.95

    for _ in range(300_000):
        if not queue:
            break
        _, cost, _, state = heapq.heappop(queue)
        key = state_key(state)
        if cost > best_cost.get(key, math.inf) + 1e-9:
            continue
        target_dx = -state[0]
        target_dy = lane_offset - state[1]
        along = target_dx * math.cos(state[2]) + target_dy * math.sin(state[2])
        cross = abs(-target_dx * math.sin(state[2]) + target_dy * math.cos(state[2]))
        if (
            state[3] == 1
            and 0.0 <= along < 0.14
            and cross < 0.035
            and abs(_normalize_angle(state[2] - math.pi)) < math.radians(9.5)
        ):
            goal_key = key
            break

        for gear in (-1, 1):
            used_shunts = state[5] + int(gear != state[3])
            if used_shunts > shunts:
                continue
            for steering in (-1, 0, 1):
                x, y, heading = _advance_pose(
                    state[:3], gear, steering / turn_radius, step_length,
                )
                if (
                    x < -max_depth or x > outward_limit
                    or y < lateral_min or y > lateral_max
                ):
                    continue
                candidate = (
                    x, y, _normalize_angle(heading), gear, steering, used_shunts,
                )
                candidate_key = state_key(candidate)
                candidate_cost = (
                    cost
                    + step_length * (2.5 if gear < 0 else 1.0)
                    + (0.75 if gear != state[3] else 0.0)
                    + (0.015 if steering != state[4] else 0.0)
                )
                if candidate_cost >= best_cost.get(candidate_key, math.inf):
                    continue
                best_cost[candidate_key] = candidate_cost
                nodes[candidate_key] = candidate
                parents[candidate_key] = (
                    key, (gear, steering, step_length),
                )
                heapq.heappush(queue, (
                    candidate_cost + 1.4 * heuristic(candidate),
                    candidate_cost,
                    next(counter),
                    candidate,
                ))

    if goal_key is None:
        raise ValueError(
            "Kein fahrbares Wendemanöver für Arbeitsbreite, Wenderadius und "
            "maximale Zugzahl gefunden"
        )
    actions: list[tuple[int, int, float]] = []
    key = goal_key
    while key != start_key:
        parent_key, action = parents[key]
        actions.append(action)
        key = parent_key
    actions.reverse()
    return tuple(actions)


@lru_cache(maxsize=64)
def _symmetric_three_point_actions(
    lane_offset: float,
    turn_radius: float,
    *,
    reverse_first: bool,
) -> tuple[tuple[int, int, float], ...]:
    """Return an exact symmetric U-turn for two parallel lanes."""
    if reverse_first:
        lower = 0.0
        upper = math.pi * turn_radius / 3.0
    else:
        lower = math.pi * turn_radius / 3.0
        upper = math.pi * turn_radius / 2.0
    for _ in range(60):
        outer_length = (lower + upper) / 2.0
        middle_length = math.pi * turn_radius - 2.0 * outer_length
        sequence = (
            ((-1, -1, outer_length), (1, 1, middle_length), (-1, -1, outer_length))
            if reverse_first
            else ((1, 1, outer_length), (-1, -1, middle_length), (1, 1, outer_length))
        )
        pose = (0.0, 0.0, 0.0)
        for gear, steering, length in sequence:
            pose = _advance_pose(
                pose, gear, steering / turn_radius, length,
            )
        if reverse_first:
            if pose[1] > lane_offset:
                lower = outer_length
            else:
                upper = outer_length
        elif pose[1] < lane_offset:
            lower = outer_length
        else:
            upper = outer_length
    outer_length = (lower + upper) / 2.0
    middle_length = max(
        0.0, math.pi * turn_radius - 2.0 * outer_length,
    )
    if reverse_first:
        return (
            (-1, -1, outer_length),
            (1, 1, middle_length),
            (-1, -1, outer_length),
        )
    return (
        (1, 1, outer_length),
        (-1, -1, middle_length),
        (1, 1, outer_length),
    )


def compute_compact_heading_reversal(
    start: tuple[float, float, float],
    *,
    turn_radius: float,
    point_spacing: float,
    lateral_offset: float | None = None,
    side: int = 1,
    reverse_first: bool = True,
    operation: str = "turn",
    turn_id: int | None = None,
) -> list[TractorWaypoint]:
    """Return a deterministic exact three-point 180-degree reversal.

    The result is intentionally independent from the generic hybrid search.
    It is used when a safe in-field start pose faces away from the first
    mission waypoint and needs the same compact manoeuvre a driver would use.
    """
    if turn_radius <= 0.0 or point_spacing <= 0.0:
        raise ValueError("Reversal radius and spacing must be positive")
    offset = (
        min(1.9 * turn_radius, max(0.10, float(lateral_offset)))
        if lateral_offset is not None
        else 1.8 * turn_radius
    )
    direction = 1.0 if side >= 0 else -1.0
    ux, uy = math.cos(start[2]), math.sin(start[2])
    vx, vy = -uy * direction, ux * direction
    target = (start[0] + vx * offset, start[1] + vy * offset)
    actions = _symmetric_three_point_actions(
        offset, turn_radius, reverse_first=reverse_first,
    )
    route = [TractorWaypoint(
        start[0], start[1], operation, actions[0][0], turn_id,
    )]
    _append_kinematic_actions(
        route,
        target,
        lane_heading=(ux, uy),
        lane_offset=(vx * offset, vy * offset),
        actions=actions,
        point_spacing=point_spacing,
        turn_radius=turn_radius,
        turn_id=turn_id or 0,
    )
    return route


@lru_cache(maxsize=64)
def _refined_turn_actions(
    lane_offset: float,
    turn_radius: float,
    maximum_shunts: int,
    boundary_clearance: float,
) -> tuple[tuple[int, int, float], ...]:
    """Refine the discrete search result to the exact next-lane pose."""
    # For regular parallel lanes below the forward U-turn diameter, use the
    # exact symmetric three-point manoeuvre directly: reverse-right,
    # forward-left, reverse-right.  At a field edge the tractor must first
    # move back into the already mown area; a forward-first manoeuvre swings
    # the front implement across the geofence.
    if 0.0 <= lane_offset < 2.0 * turn_radius and maximum_shunts >= 2:
        return _symmetric_three_point_actions(
            lane_offset, turn_radius, reverse_first=True,
        )
    raw_actions = _search_turn_actions(
        lane_offset, turn_radius, maximum_shunts, boundary_clearance,
    )
    runs: list[list[float | int]] = []
    for gear, steering, length in raw_actions:
        if runs and runs[-1][0] == gear and runs[-1][1] == steering:
            runs[-1][2] = float(runs[-1][2]) + length
        else:
            runs.append([gear, steering, length])
    lengths = np.array([float(run[2]) for run in runs], dtype=float)
    lower = np.maximum(0.005, lengths - 0.20)
    upper = lengths + 0.20

    def residual(values: np.ndarray) -> np.ndarray:
        pose = (0.0, 0.0, 0.0)
        for run, length in zip(runs, values):
            pose = _advance_pose(
                pose,
                int(run[0]),
                int(run[1]) / turn_radius,
                float(length),
            )
        return np.array([
            pose[0],
            pose[1] - lane_offset,
            turn_radius * _normalize_angle(pose[2] - math.pi),
        ])

    damping = 1e-4
    for _ in range(60):
        error = residual(lengths)
        error_norm = float(np.linalg.norm(error))
        if error_norm < 1e-8:
            break
        epsilon = 1e-5
        jacobian = np.empty((3, len(lengths)), dtype=float)
        for index in range(len(lengths)):
            perturbed = lengths.copy()
            perturbed[index] += epsilon
            jacobian[:, index] = (residual(perturbed) - error) / epsilon
        try:
            correction = -jacobian.T @ np.linalg.solve(
                jacobian @ jacobian.T + damping * np.eye(3), error,
            )
        except np.linalg.LinAlgError:
            damping *= 10.0
            continue
        improved = False
        for scale in (1.0, 0.5, 0.25, 0.1):
            candidate = np.clip(lengths + scale * correction, lower, upper)
            if float(np.linalg.norm(residual(candidate))) < error_norm:
                lengths = candidate
                damping = max(1e-8, damping * 0.3)
                improved = True
                break
        if not improved:
            damping *= 10.0
    if float(np.linalg.norm(residual(lengths))) >= 1e-6:
        raise ValueError("Wendemanöver konnte nicht exakt an die nächste Bahn angeschlossen werden")
    return tuple(
        (int(run[0]), int(run[1]), float(length))
        for run, length in zip(runs, lengths)
    )


def _append_kinematic_actions(
    route: list[TractorWaypoint],
    target: tuple[float, float],
    *,
    lane_heading: tuple[float, float],
    lane_offset: tuple[float, float],
    actions: tuple[tuple[int, int, float], ...],
    point_spacing: float,
    turn_radius: float,
    turn_id: int,
) -> None:
    start = route[-1]
    ux, uy = lane_heading
    offset_length = math.hypot(*lane_offset)
    vx, vy = lane_offset[0] / offset_length, lane_offset[1] / offset_length
    local_pose = (0.0, 0.0, 0.0)
    for gear, steering, length in actions:
        steps = max(1, math.ceil(length / point_spacing))
        substep = length / steps
        for _ in range(steps):
            local_pose = _advance_pose(
                local_pose, gear, steering / turn_radius, substep,
            )
            local_x, local_y, _ = local_pose
            route.append(TractorWaypoint(
                start.x + local_x * ux + local_y * vx,
                start.y + local_x * uy + local_y * vy,
                "turn", gear, turn_id,
            ))
    endpoint_error = math.hypot(route[-1].x - target[0], route[-1].y - target[1])
    if endpoint_error > 1e-5:
        raise ValueError("Wendemanöver endet nicht exakt an der nächsten Mähbahn")
    # Eliminate the remaining floating-point nanometres. This is not another
    # path segment: the refined radius-bounded arc already ends at this pose.
    last = route[-1]
    route[-1] = TractorWaypoint(target[0], target[1], "turn", last.gear, turn_id)


def _forward_turn_actions(
    lane_offset: float,
    turn_radius: float,
) -> tuple[tuple[int, int, float], ...]:
    straight = max(0.0, lane_offset - 2.0 * turn_radius)
    quarter_arc = math.pi * turn_radius / 2.0
    actions: list[tuple[int, int, float]] = [(1, 1, quarter_arc)]
    if straight > 1e-9:
        actions.append((1, 0, straight))
    actions.append((1, 1, quarter_arc))
    return tuple(actions)


def _compute_longitudinally_offset_turn(
    start: tuple[float, float],
    target: tuple[float, float],
    *,
    lane_heading: tuple[float, float],
    along_offset: float,
    lateral_offset: float,
    point_spacing: float,
    turn_radius: float,
    strategy: str,
    maneuver_passes: int,
    clearance: float,
    boundary: Polygon | None,
    route_validator: RouteValidator | None,
    turn_id: int,
) -> list[TractorWaypoint] | None:
    """Build an exact U-turn between staggered parallel swath ends.

    A pure kinematic U-turn changes only the lateral coordinate.  Staggered
    endpoints additionally need a longitudinal displacement. Split that
    displacement between straight travel before and after the U-turn instead
    of shearing the curved geometry. Several split positions are evaluated so
    a concave field can place the manoeuvre in already mown free space.
    """
    ux, uy = lane_heading
    vx, vy = -uy, ux
    lateral_distance = abs(lateral_offset)
    if lateral_distance <= 1e-7:
        return None
    if strategy == "forward":
        if lateral_distance + 1e-9 < 2.0 * turn_radius:
            return None
        actions = _forward_turn_actions(lateral_distance, turn_radius)
    else:
        maximum_shunts = 6 if strategy == "reverse" else maneuver_passes
        try:
            actions = _refined_turn_actions(
                round(lateral_distance, 6),
                round(turn_radius, 6),
                maximum_shunts,
                round(clearance, 6),
            )
        except ValueError:
            return None

    candidates: list[tuple[float, list[TractorWaypoint]]] = []
    for fraction in (-1.0, -0.5, 0.0, 0.25, 0.5, 0.75, 1.0, 1.5, 2.0):
        before = along_offset * fraction
        after = along_offset - before
        first_gear = 1 if before >= 0.0 else -1
        candidate = [TractorWaypoint(
            start[0], start[1], "turn", first_gear, turn_id,
        )]
        if abs(before) > 1e-8:
            _append_leg(
                candidate,
                (start[0] + before * ux, start[1] + before * uy),
                point_spacing=point_spacing,
                operation="turn",
                gear=first_gear,
                turn_id=turn_id,
            )
        turn_start = candidate[-1]
        turn_target = (
            turn_start.x + lateral_offset * vx,
            turn_start.y + lateral_offset * vy,
        )
        _append_kinematic_actions(
            candidate,
            turn_target,
            lane_heading=(ux, uy),
            lane_offset=(lateral_offset * vx, lateral_offset * vy),
            actions=actions,
            point_spacing=point_spacing,
            turn_radius=turn_radius,
            turn_id=turn_id,
        )
        if abs(after) > 1e-8:
            # The tractor now faces -u. Reverse gear moves it along +u.
            _append_leg(
                candidate,
                target,
                point_spacing=point_spacing,
                operation="turn",
                gear=-1 if after >= 0.0 else 1,
                turn_id=turn_id,
            )
        elif math.hypot(candidate[-1].x - target[0], candidate[-1].y - target[1]) > 1e-5:
            continue
        if boundary is not None:
            path = LineString([(point.x, point.y) for point in candidate])
            if not boundary.buffer(0.003).covers(path):
                continue
        if route_validator is not None and not route_validator(candidate):
            continue
        total_length = sum(
            math.hypot(right.x - left.x, right.y - left.y)
            for left, right in zip(candidate, candidate[1:])
        )
        reverse_length = sum(
            math.hypot(right.x - left.x, right.y - left.y)
            for left, right in zip(candidate, candidate[1:])
            if right.gear < 0
        )
        candidates.append((total_length + 2.0 * reverse_length, candidate))
    return min(candidates, key=lambda item: item[0])[1] if candidates else None


def compute_tractor_route(
    zone: Polygon,
    *,
    lane_spacing: float,
    mowing_width: float | None = None,
    point_spacing: float = 0.005,
    turn_strategy: str = "auto",
    turn_radius: float = 0.25,
    maneuver_passes: int = 4,
    boundary_zone: Polygon | None = None,
    boundary_clearance: float | None = None,
    start_pose: tuple[float, float, float] | None = None,
    lane_endpoint_trim_m: float = 0.0,
    base_waypoints: list[tuple[float, float]] | None = None,
    route_validator: RouteValidator | None = None,
) -> list[TractorWaypoint]:
    """Expand coverage lanes into a dense route with explicit turn segments.

    ``auto`` uses a forward connector only when the lane spacing provides the
    configured turning diameter. Otherwise it generates a multi-point turn
    inside the already covered side of the field. Turn waypoints carry gear and
    operation metadata so execution can raise the decks and reverse safely.
    """
    if lane_spacing <= 0 or point_spacing <= 0 or turn_radius <= 0:
        raise ValueError("Route spacing and turn radius must be positive")
    if turn_strategy not in TURN_STRATEGIES:
        raise ValueError(f"Unknown turn strategy: {turn_strategy}")
    maneuver_passes = max(2, min(6, int(maneuver_passes)))
    clearance = max(
        0.05,
        float(boundary_clearance) if boundary_clearance is not None else lane_spacing / 2.0,
    )
    safe_boundary = boundary_zone.buffer(0.003) if boundary_zone is not None else None

    mowing_width = lane_spacing if mowing_width is None else max(lane_spacing, mowing_width)
    base = list(base_waypoints) if base_waypoints is not None else compute_boustrophedon(
        zone,
        lane_spacing=lane_spacing,
        mowing_width=mowing_width,
        endpoint_trim_m=lane_endpoint_trim_m,
    )
    if not base:
        return []
    if start_pose is not None and len(base) >= 2:
        alternatives = (base, list(reversed(base)))

        def entry_score(points: list[tuple[float, float]]) -> float:
            heading = math.atan2(
                points[1][1] - points[0][1], points[1][0] - points[0][0],
            )
            return (
                math.hypot(points[0][0] - start_pose[0], points[0][1] - start_pose[1])
                # Prefer a longer straight approach over starting the entire
                # field with a 180-degree manoeuvre. The old radius-scaled
                # penalty made opposite-heading entries almost free for the
                # 25 cm tractor and triggered expensive loops/reverse paths.
                + 2.0 * abs(_normalize_angle(heading - start_pose[2]))
            )

        base = min(alternatives, key=entry_score)
    route = [TractorWaypoint(base[0][0], base[0][1])]
    turn_id = 0

    for segment_index, target in enumerate(base[1:]):
        is_turn = segment_index % 2 == 1
        if not is_turn:
            _append_leg(
                route, target, point_spacing=point_spacing,
                operation="mow", gear=1, turn_id=None,
            )
            continue

        turn_id += 1
        start = base[segment_index]
        previous = base[segment_index - 1]
        ux, uy = _unit(start[0] - previous[0], start[1] - previous[1])
        cx, cy = target[0] - start[0], target[1] - start[1]
        # A clipped/fragmented swath can end substantially before its
        # neighbour. Its endpoint delta then contains both a lateral lane
        # spacing and a longitudinal displacement. Treating that diagonal as
        # a pure lane spacing applies an affine shear to the kinematic U-turn
        # and can turn a requested 25 cm arc into only 2.5 cm. Regular turns
        # keep the fast exact template; irregular endpoints use the general
        # oriented Dubins/Reeds-Shepp connector instead.
        along_offset = cx * ux + cy * uy
        lateral_offset = cx * (-uy) + cy * ux
        offset_length = abs(lateral_offset)
        irregular_endpoints = abs(along_offset) > 1e-5
        strategy = turn_strategy
        if strategy == "auto":
            strategy = "forward" if offset_length >= 2.0 * turn_radius else "multi_point"

        turn_start_index = len(route) - 1
        if irregular_endpoints:
            start_heading = math.atan2(uy, ux)
            target_heading = _normalize_angle(start_heading + math.pi)
            connection = _compute_longitudinally_offset_turn(
                start,
                target,
                lane_heading=(ux, uy),
                along_offset=along_offset,
                lateral_offset=lateral_offset,
                point_spacing=point_spacing,
                turn_radius=turn_radius,
                strategy=strategy,
                maneuver_passes=maneuver_passes,
                clearance=clearance,
                boundary=boundary_zone,
                route_validator=route_validator,
                turn_id=turn_id,
            )
            if connection is None:
                connection = compute_curvature_bounded_connection(
                    (start[0], start[1], start_heading),
                    (target[0], target[1], target_heading),
                    turn_radius=turn_radius,
                    point_spacing=point_spacing,
                    boundary=boundary_zone,
                    allow_reverse=True,
                    operation="turn",
                    turn_id=turn_id,
                    route_validator=route_validator,
                )
            route.extend(connection[1:])
        elif strategy == "forward":
            if offset_length + 1e-9 < 2.0 * turn_radius:
                raise ValueError(
                    "Vorwärtswende nicht möglich: Spurabstand ist kleiner als "
                    "der doppelte Wenderadius"
                )
            _append_kinematic_actions(
                route,
                target,
                lane_heading=(ux, uy),
                lane_offset=(cx, cy),
                actions=_forward_turn_actions(offset_length, turn_radius),
                point_spacing=point_spacing,
                turn_radius=turn_radius,
                turn_id=turn_id,
            )
        else:
            maximum_shunts = 6 if strategy == "reverse" else maneuver_passes
            try:
                actions = _refined_turn_actions(
                    round(offset_length, 6),
                    round(turn_radius, 6),
                    maximum_shunts,
                    round(clearance, 6),
                )
                _append_kinematic_actions(
                    route,
                    target,
                    lane_heading=(ux, uy),
                    lane_offset=(cx, cy),
                    actions=actions,
                    point_spacing=point_spacing,
                    turn_radius=turn_radius,
                    turn_id=turn_id,
                )
            except ValueError:
                # Irregular polygons can leave a narrow final spacing that is
                # intentionally smaller than the regular lanes. Fall back to
                # the generic boundary-aware hybrid connector for that pose.
                start_heading = math.atan2(uy, ux)
                target_heading = _normalize_angle(start_heading + math.pi)
                connection = compute_curvature_bounded_connection(
                    (start[0], start[1], start_heading),
                    (target[0], target[1], target_heading),
                    turn_radius=turn_radius,
                    point_spacing=point_spacing,
                    boundary=boundary_zone,
                    allow_reverse=True,
                    operation="turn",
                    turn_id=turn_id,
                    route_validator=route_validator,
                )
                route.extend(connection[1:])
        footprint_invalid = (
            route_validator is not None
            and not route_validator(route[turn_start_index:])
        )
        boundary_invalid = False
        if safe_boundary is not None:
            turn_path = LineString([
                (point.x, point.y) for point in route[turn_start_index:]
            ])
            boundary_invalid = not safe_boundary.covers(turn_path)
        if footprint_invalid or boundary_invalid:
            del route[turn_start_index + 1:]
            staged_turn_found = False
            if (
                strategy != "forward"
                and offset_length < 2.0 * turn_radius
                and maneuver_passes >= 4
            ):
                core_actions = _symmetric_three_point_actions(
                    offset_length, turn_radius, reverse_first=False,
                )
                # Back straight into the already mown lane, execute the
                # compact three-point turn there, then reverse out to the next
                # lane start. This mirrors how a real tractor turns at a wall
                # without swinging its front implement across the boundary.
                for staging_distance in (0.15, 0.25, 0.35, 0.50, 0.70, 0.90):
                    candidate_actions = (
                        (-1, 0, staging_distance),
                        *core_actions,
                        (-1, 0, staging_distance),
                    )
                    _append_kinematic_actions(
                        route,
                        target,
                        lane_heading=(ux, uy),
                        lane_offset=(cx, cy),
                        actions=candidate_actions,
                        point_spacing=point_spacing,
                        turn_radius=turn_radius,
                        turn_id=turn_id,
                    )
                    candidate_invalid = (
                        route_validator is not None
                        and not route_validator(route[turn_start_index:])
                    )
                    if not candidate_invalid and safe_boundary is not None:
                        candidate_path = LineString([
                            (point.x, point.y)
                            for point in route[turn_start_index:]
                        ])
                        candidate_invalid = not safe_boundary.covers(candidate_path)
                    if not candidate_invalid:
                        staged_turn_found = True
                        break
                    del route[turn_start_index + 1:]
            if staged_turn_found:
                continue
            start_heading = math.atan2(uy, ux)
            target_heading = _normalize_angle(start_heading + math.pi)
            connection = compute_curvature_bounded_connection(
                (start[0], start[1], start_heading),
                (target[0], target[1], target_heading),
                turn_radius=turn_radius,
                point_spacing=point_spacing,
                boundary=boundary_zone,
                allow_reverse=True,
                operation="turn",
                turn_id=turn_id,
                route_validator=route_validator,
            )
            route.extend(connection[1:])

    return route
