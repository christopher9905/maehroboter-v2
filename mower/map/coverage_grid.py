"""Raster-based mower-deck footprints and actual coverage accounting."""

from __future__ import annotations

import math
import threading
from dataclasses import dataclass
from typing import Any, Sequence

import numpy as np
from shapely import contains_xy
from shapely.geometry import LineString, Polygon, box, mapping
from shapely.geometry.base import BaseGeometry
from shapely.ops import unary_union


# Physical path tracking can never follow a mathematical boundary with zero
# error.  This reserve keeps the widest machine point clear of walls/no-go
# boundaries while the separate runtime tolerance only absorbs RTK noise.
GEOFENCE_TRACKING_RESERVE_M = 0.05
_MAX_NUMERICAL_OUTSIDE_AREA_M2 = 0.000005  # 0.05 cm²


@dataclass(frozen=True)
class DeckFootprint:
    name: str
    forward_m: float
    left_m: float
    width_m: float
    depth_m: float
    group: str  # front | rear


def mower_decks_from_settings(settings: dict[str, Any]) -> tuple[DeckFootprint, ...]:
    """Return the three physical tool rectangles relative to the rear axle."""
    total_width = max(0.30, float(settings.get("mowing_width_cm", 60)) / 100.0)
    front_ratio = max(0.35, min(1.0, float(settings.get("front_mower_width_percent", 60)) / 100.0))
    gap = max(0.0, min(total_width * 0.5, float(settings.get("rear_mower_gap_cm", 6)) / 100.0))
    rear_width = max(0.05, (total_width - gap) / 2.0)
    depth = max(0.05, min(0.40, float(settings.get("mower_deck_depth_cm", 14)) / 100.0))
    wheelbase = max(0.10, float(settings.get("vehicle_wheelbase_cm", 25)) / 100.0)
    front_clearance = float(settings.get("front_mower_offset_cm", 35)) / 100.0
    front_offset = wheelbase + front_clearance
    rear_offset = -float(settings.get("rear_mower_offset_cm", 22)) / 100.0
    rear_left = (gap + rear_width) / 2.0
    return (
        DeckFootprint("front", front_offset, 0.0, total_width * front_ratio, depth, "front"),
        DeckFootprint("rear_left", rear_offset, rear_left, rear_width, depth, "rear"),
        DeckFootprint("rear_right", rear_offset, -rear_left, rear_width, depth, "rear"),
    )


def deck_polygon(
    x: float,
    y: float,
    heading_rad: float,
    deck: DeckFootprint,
) -> Polygon:
    """Transform one local mower rectangle into metric world coordinates."""
    cos_h, sin_h = math.cos(heading_rad), math.sin(heading_rad)
    half_depth, half_width = deck.depth_m / 2.0, deck.width_m / 2.0
    corners: list[tuple[float, float]] = []
    for forward, left in (
        (deck.forward_m + half_depth, deck.left_m + half_width),
        (deck.forward_m + half_depth, deck.left_m - half_width),
        (deck.forward_m - half_depth, deck.left_m - half_width),
        (deck.forward_m - half_depth, deck.left_m + half_width),
    ):
        corners.append((
            x + forward * cos_h - left * sin_h,
            y + forward * sin_h + left * cos_h,
        ))
    return Polygon(corners)


def vehicle_body_from_settings(settings: dict[str, Any]) -> DeckFootprint:
    """Return a conservative tractor-body rectangle relative to rear axle."""
    wheelbase = max(0.10, float(settings.get("vehicle_wheelbase_cm", 25)) / 100.0)
    width = max(0.15, float(settings.get("vehicle_width_cm", 34)) / 100.0)
    front_overhang = max(
        0.05, float(settings.get("vehicle_front_overhang_cm", 12)) / 100.0,
    )
    rear_overhang = max(
        0.05, float(settings.get("vehicle_rear_overhang_cm", 12)) / 100.0,
    )
    length = rear_overhang + wheelbase + front_overhang
    centre = (wheelbase + front_overhang - rear_overhang) / 2.0
    return DeckFootprint("tractor", centre, 0.0, width, length, "vehicle")


def machine_footprint(
    x: float,
    y: float,
    heading_rad: float,
    settings: dict[str, Any],
    decks: tuple[DeckFootprint, ...] | None = None,
) -> BaseGeometry:
    """Return the complete oriented tractor and mower collision footprint.

    Raised decks are included deliberately: lifting changes the mowing state,
    but not the lateral space occupied by the physical implement.
    """
    physical_decks = decks if decks is not None else mower_decks_from_settings(settings)
    parts = [
        deck_polygon(x, y, heading_rad, vehicle_body_from_settings(settings)),
        *(deck_polygon(x, y, heading_rad, deck) for deck in physical_decks),
    ]
    return unary_union(parts)


def machine_lateral_bounds(
    settings: dict[str, Any],
    decks: tuple[DeckFootprint, ...] | None = None,
) -> tuple[float, float]:
    """Return right/left local extents used by Teach-In reference selection."""
    footprint = machine_footprint(0.0, 0.0, 0.0, settings, decks)
    _min_forward, min_left, _max_forward, max_left = footprint.bounds
    return min_left, max_left


def machine_geofence_width(
    x: float,
    y: float,
    heading_rad: float,
    settings: dict[str, Any],
    decks: tuple[DeckFootprint, ...] | None = None,
) -> LineString:
    """Return the oriented right-to-left segment of the widest machine point.

    Geofencing uses the operator-requested outer width rather than the GPS
    centre. Longitudinal clearance is handled by the route endpoints, while
    this segment makes wall/no-go contact on either side explicit and keeps
    the same reference geometry as left/right Teach-In.
    """
    right, left = machine_lateral_bounds(settings, decks)
    sin_h, cos_h = math.sin(heading_rad), math.cos(heading_rad)
    return LineString([
        (x - right * sin_h, y + right * cos_h),
        (x - left * sin_h, y + left * cos_h),
    ])


def machine_longitudinal_bounds(
    settings: dict[str, Any],
    decks: tuple[DeckFootprint, ...] | None = None,
) -> tuple[float, float]:
    """Return rear/front local extents relative to the rear axle."""
    footprint = machine_footprint(0.0, 0.0, 0.0, settings, decks)
    min_forward, _min_left, max_forward, _max_left = footprint.bounds
    return min_forward, max_forward


def machine_curve_clearance(
    settings: dict[str, Any],
    turn_radius: float,
    decks: tuple[DeckFootprint, ...] | None = None,
) -> float:
    """Return the required centreline clearance during a minimum-radius turn.

    A front/rear implement does not merely occupy its lateral half-width in a
    curve.  Its longitudinal offset swings around the instantaneous turn
    centre.  Both left and right turns are evaluated for every actual polygon
    corner so asymmetric mower layouts remain safe without using an overly
    large bounding circle.
    """
    if turn_radius <= 0.0:
        raise ValueError("turn_radius must be positive")
    physical_decks = decks if decks is not None else mower_decks_from_settings(settings)
    components = (
        vehicle_body_from_settings(settings),
        *physical_decks,
    )
    required = 0.0
    for component in components:
        half_depth = component.depth_m / 2.0
        half_width = component.width_m / 2.0
        for forward in (
            component.forward_m - half_depth,
            component.forward_m + half_depth,
        ):
            for left in (
                component.left_m - half_width,
                component.left_m + half_width,
            ):
                for turn_sign in (-1.0, 1.0):
                    radial = math.hypot(
                        forward,
                        turn_radius + turn_sign * left,
                    )
                    required = max(required, radial - turn_radius)
    return max(required, machine_lateral_bounds(settings, physical_decks)[1],
               abs(machine_lateral_bounds(settings, physical_decks)[0]))


def route_pose_heading(waypoints: Sequence[Any], index: int) -> float:
    """Return the physical vehicle heading at one sampled route point.

    Route coordinates describe motion direction.  During reverse motion the
    tractor itself faces the opposite direction, which matters for offset
    front and rear implements.
    """
    if not waypoints:
        return 0.0
    index = max(0, min(len(waypoints) - 1, int(index)))
    point = waypoints[index]
    vector: tuple[float, float] | None = None
    gear = int(getattr(point, "gear", 1))
    # A waypoint's gear describes the segment by which that pose was reached.
    # Looking forward with the old gear at a forward/reverse switch creates a
    # false 180-degree heading jump (and an apparent visual robot spin).
    for candidate_index in range(index - 1, -1, -1):
        candidate = waypoints[candidate_index]
        dx, dy = point.x - candidate.x, point.y - candidate.y
        if math.hypot(dx, dy) > 1e-9:
            vector = (dx, dy)
            break
    if vector is None:
        for candidate_index in range(index + 1, len(waypoints)):
            candidate = waypoints[candidate_index]
            dx, dy = candidate.x - point.x, candidate.y - point.y
            if math.hypot(dx, dy) > 1e-9:
                vector = (dx, dy)
                gear = int(getattr(candidate, "gear", gear))
                break
    if vector is None:
        return 0.0
    heading = math.atan2(vector[1], vector[0])
    if gear < 0:
        heading += math.pi
    return (heading + math.pi) % (2.0 * math.pi) - math.pi


def validate_swept_machine_route(
    waypoints: Sequence[Any],
    allowed_zone: BaseGeometry,
    settings: dict[str, Any],
    decks: tuple[DeckFootprint, ...] | None = None,
    *,
    reserve_m: float = GEOFENCE_TRACKING_RESERVE_M,
    no_go_zones: Sequence[BaseGeometry] = (),
    maximum_sample_step_m: float = 0.02,
    geometry_tolerance_m: float = 0.001,
) -> dict[str, Any]:
    """Validate the complete swept tractor/implement footprint.

    The check is deliberately independent from coverage estimation.  Every
    accepted route must keep the physical tractor and all raised/lowered mower
    decks inside the geofence, including their longitudinal swing in curves.
    Consecutive footprint samples are joined conservatively to cover the swept
    area between them.
    """
    if len(waypoints) < 2:
        return {
            "valid": False,
            "reason": "route_too_short",
            "sample_count": 0,
            "violation_index": 0,
            "outside_area_m2": 0.0,
        }
    reserve_m = max(0.0, float(reserve_m))
    maximum_sample_step_m = max(0.005, min(0.05, float(maximum_sample_step_m)))
    geometry_tolerance_m = max(0.0, min(0.005, float(geometry_tolerance_m)))
    permitted = allowed_zone.buffer(-reserve_m, join_style="round")
    if permitted.is_empty:
        return {
            "valid": False,
            "reason": "no_safe_area",
            "sample_count": 0,
            "violation_index": 0,
            "outside_area_m2": round(allowed_zone.area, 6),
        }
    for obstacle in no_go_zones:
        if obstacle is not None and not obstacle.is_empty:
            permitted = permitted.difference(obstacle.buffer(reserve_m, join_style="round"))
    if permitted.is_empty:
        return {
            "valid": False,
            "reason": "no_safe_area",
            "sample_count": 0,
            "violation_index": 0,
            "outside_area_m2": round(allowed_zone.area, 6),
        }

    physical_decks = decks if decks is not None else mower_decks_from_settings(settings)
    sample_indices = {0, len(waypoints) - 1}
    accumulated = 0.0
    for index, (left, right) in enumerate(zip(waypoints, waypoints[1:]), start=1):
        accumulated += math.hypot(right.x - left.x, right.y - left.y)
        state_changed = (
            getattr(left, "gear", 1) != getattr(right, "gear", 1)
            or getattr(left, "operation", None) != getattr(right, "operation", None)
        )
        if accumulated >= maximum_sample_step_m or state_changed:
            sample_indices.add(index)
            sample_indices.add(max(0, index - 1))
            accumulated = 0.0

    previous_footprint: BaseGeometry | None = None
    previous_index: int | None = None
    checked = 0
    tolerance = permitted.buffer(geometry_tolerance_m)
    for index in sorted(sample_indices):
        point = waypoints[index]
        heading = route_pose_heading(waypoints, index)
        footprint = machine_footprint(
            point.x, point.y, heading, settings, physical_decks,
        )
        checked += 1
        if not tolerance.covers(footprint):
            outside = footprint.difference(tolerance)
            if outside.area > _MAX_NUMERICAL_OUTSIDE_AREA_M2:
                return {
                    "valid": False,
                    "reason": "footprint_outside_geofence",
                    "sample_count": checked,
                    "violation_index": index,
                    "operation": getattr(point, "operation", None),
                    "outside_area_m2": round(outside.area, 6),
                    "x": point.x,
                    "y": point.y,
                    "heading_rad": heading,
                }
        if previous_footprint is not None and previous_index is not None:
            swept = unary_union((previous_footprint, footprint)).convex_hull
            if not tolerance.covers(swept):
                outside = swept.difference(tolerance)
                if outside.area > _MAX_NUMERICAL_OUTSIDE_AREA_M2:
                    return {
                        "valid": False,
                        "reason": "swept_footprint_outside_geofence",
                        "sample_count": checked,
                        "violation_index": previous_index,
                        "operation": getattr(point, "operation", None),
                        "outside_area_m2": round(outside.area, 6),
                        "x": point.x,
                        "y": point.y,
                        "heading_rad": heading,
                    }
        previous_footprint = footprint
        previous_index = index
    return {
        "valid": True,
        "reason": None,
        "sample_count": checked,
        "violation_index": None,
        "outside_area_m2": 0.0,
        "reserve_m": reserve_m,
    }


class CoverageGrid:
    """Centimetre-scale occupancy grid for each physical mower deck."""

    def __init__(self, zone: Polygon, *, resolution_m: float = 0.02) -> None:
        if zone.is_empty or not zone.is_valid:
            raise ValueError("coverage zone must be a valid polygon")
        self.resolution_m = max(0.01, min(0.10, float(resolution_m)))
        min_x, min_y, max_x, max_y = zone.bounds
        padding = 0.75
        self.min_x = min_x - padding
        self.min_y = min_y - padding
        self.width = max(1, math.ceil((max_x - min_x + 2.0 * padding) / self.resolution_m))
        self.height = max(1, math.ceil((max_y - min_y + 2.0 * padding) / self.resolution_m))
        self._layers = {
            "front": np.zeros((self.height, self.width), dtype=np.bool_),
            "rear_left": np.zeros((self.height, self.width), dtype=np.bool_),
            "rear_right": np.zeros((self.height, self.width), dtype=np.bool_),
        }
        xs = self.min_x + (np.arange(self.width) + 0.5) * self.resolution_m
        ys = self.min_y + (np.arange(self.height) + 0.5) * self.resolution_m
        mesh_x, mesh_y = np.meshgrid(xs, ys)
        self._zone_mask = contains_xy(zone.buffer(self.resolution_m * 0.05), mesh_x, mesh_y)
        self._last_pose: tuple[float, float, float] | None = None
        self._last_active: frozenset[str] = frozenset()
        self._dirty = True
        self._geometry_cache: dict[str, Any] | None = None
        self._generation = 0
        self._lock = threading.RLock()

    def mark_pose(
        self,
        x: float,
        y: float,
        heading_rad: float,
        decks: tuple[DeckFootprint, ...],
        *,
        front_active: bool,
        rear_active: bool,
    ) -> None:
        with self._lock:
            active = frozenset(
                deck.name for deck in decks
                if (deck.group == "front" and front_active) or (deck.group == "rear" and rear_active)
            )
            if not active:
                self._last_pose = (x, y, heading_rad)
                self._last_active = active
                return
            samples = [(x, y, heading_rad)]
            if self._last_pose is not None and self._last_active == active:
                old_x, old_y, old_heading = self._last_pose
                distance = math.hypot(x - old_x, y - old_y)
                angle = (heading_rad - old_heading + math.pi) % (2.0 * math.pi) - math.pi
                count = max(
                    1,
                    math.ceil(distance / (self.resolution_m * 0.75)),
                    math.ceil(abs(angle) / math.radians(2.0)),
                )
                samples = [
                    (
                        old_x + (x - old_x) * index / count,
                        old_y + (y - old_y) * index / count,
                        old_heading + angle * index / count,
                    )
                    for index in range(1, count + 1)
                ]
            deck_by_name = {deck.name: deck for deck in decks}
            for sample_x, sample_y, sample_heading in samples:
                for name in active:
                    self._mark_polygon(
                        self._layers[name],
                        deck_polygon(sample_x, sample_y, sample_heading, deck_by_name[name]),
                    )
            self._last_pose = (x, y, heading_rad)
            self._last_active = active
            self._dirty = True
            self._generation += 1

    def _mark_polygon(self, layer: np.ndarray, polygon: Polygon) -> None:
        min_x, min_y, max_x, max_y = polygon.bounds
        col_start = max(0, math.floor((min_x - self.min_x) / self.resolution_m))
        col_end = min(self.width, math.ceil((max_x - self.min_x) / self.resolution_m))
        row_start = max(0, math.floor((min_y - self.min_y) / self.resolution_m))
        row_end = min(self.height, math.ceil((max_y - self.min_y) / self.resolution_m))
        if col_start >= col_end or row_start >= row_end:
            return
        xs = self.min_x + (np.arange(col_start, col_end) + 0.5) * self.resolution_m
        ys = self.min_y + (np.arange(row_start, row_end) + 0.5) * self.resolution_m
        mesh_x, mesh_y = np.meshgrid(xs, ys)
        inside = contains_xy(polygon.buffer(self.resolution_m * 0.08), mesh_x, mesh_y)
        layer[row_start:row_end, col_start:col_end] |= inside

    def coverage_percent(self) -> float:
        with self._lock:
            combined = self.combined_grid()
            target_cells = int(self._zone_mask.sum())
            if target_cells == 0:
                return 0.0
            return round(float(np.logical_and(combined, self._zone_mask).sum()) / target_cells * 100.0, 2)

    def combined_grid(self) -> np.ndarray:
        with self._lock:
            return np.logical_or.reduce(tuple(self._layers.values()))

    def geojson(self, to_wgs84) -> dict[str, Any]:
        """Return a snapshot without blocking live marking during polygon union."""
        with self._lock:
            if not self._dirty and self._geometry_cache is not None:
                return self._geometry_cache
            layers = {name: grid.copy() for name, grid in self._layers.items()}
            generation = self._generation
        features = []
        colors = {"front": "#55d98b", "rear_left": "#39bf73", "rear_right": "#2fa965"}
        for name, grid in layers.items():
            geometry = self._grid_geometry(grid).simplify(
                min(0.009, self.resolution_m * 0.45), preserve_topology=True,
            )
            if geometry.is_empty:
                continue
            mapped = mapping(geometry)
            mapped["coordinates"] = self._transform_coordinates(mapped["coordinates"], to_wgs84)
            features.append({
                "type": "Feature",
                "properties": {"deck": name, "color": colors[name]},
                "geometry": mapped,
            })
        result = {"type": "FeatureCollection", "features": features}
        with self._lock:
            if generation == self._generation:
                self._geometry_cache = result
                self._dirty = False
        return result

    def _grid_geometry(self, grid: np.ndarray):
        rectangles = []
        for row in range(self.height):
            columns = np.flatnonzero(grid[row])
            if not len(columns):
                continue
            run_start = previous = int(columns[0])
            for column_value in columns[1:]:
                column = int(column_value)
                if column != previous + 1:
                    rectangles.append(self._run_box(row, run_start, previous + 1))
                    run_start = column
                previous = column
            rectangles.append(self._run_box(row, run_start, previous + 1))
        return unary_union(rectangles) if rectangles else Polygon()

    def _run_box(self, row: int, start: int, end: int):
        y0 = self.min_y + row * self.resolution_m
        return box(
            self.min_x + start * self.resolution_m,
            y0,
            self.min_x + end * self.resolution_m,
            y0 + self.resolution_m,
        )

    @classmethod
    def _transform_coordinates(cls, coordinates, transform):
        if not coordinates:
            return coordinates
        if isinstance(coordinates[0], (float, int)):
            lon, lat = transform(float(coordinates[0]), float(coordinates[1]))
            return [lon, lat]
        return [cls._transform_coordinates(item, transform) for item in coordinates]
