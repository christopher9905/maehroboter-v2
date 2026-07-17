"""Shared autonomous mission runtime for real and simulated hardware.

This module deliberately contains no simulation-specific behaviour.  It joins
the existing production Teach-In recorder, coverage planner, localizer output
and predictive controller and sends commands through ``HardwareInterface``.
"""

from __future__ import annotations

import copy
import hashlib
import json
import logging
import math
import os
import threading
import time
from pathlib import Path
from typing import Any, Callable, Optional, Sequence

import utm
from shapely.geometry import (
    GeometryCollection,
    LineString,
    MultiPolygon,
    Point,
    Polygon,
    mapping,
    shape,
)
from shapely.ops import unary_union
from shapely.validation import make_valid

from mower.control.mpc import MPCController
from mower.executive.mission_executive import MissionExecutive, MowerState
from mower.map.teach_in import TeachIn
from mower.map.coverage_grid import (
    CoverageGrid,
    machine_footprint,
    machine_curve_clearance,
    machine_lateral_bounds,
    machine_longitudinal_bounds,
    mower_decks_from_settings,
    route_pose_heading,
    validate_swept_machine_route,
)
from mower.nav.gps_reader import GpsFix
from mower.nav.localizer import Pose
from mower.path.tractor_route import (
    TractorWaypoint,
    compute_compact_heading_reversal,
    compute_curvature_bounded_connection,
    compute_headland_route,
    compute_tractor_route,
    measure_minimum_turn_radius,
)
from mower.path.boustrophedon import compute_best_boustrophedon
from mower.path.trajectory import ContinuousTrajectory
from mower.path.fields2cover_native import Fields2CoverPlan, plan_fields2cover

logger = logging.getLogger(__name__)

_DOCK_POSITION_TOLERANCE_M = 1.5
_DOCK_DEPARTURE_MAX_OUTSIDE_M = 3.0
_DOCK_DEPARTURE_CORRIDOR_HALF_WIDTH_M = 1.25
_STEERING_RATE_LIMIT_DEG_S = 120.0

# Cusp (gear-change) maneuvers are executed as explicit, distance-bounded
# steps instead of generic trajectory tracking: the projection cannot advance
# reliably across a direction reversal, and short reverse shunts (7-25 cm)
# span fewer pose updates than the projection needs to converge.  Observed
# failure without this: the follower sticks at the gear boundary, never
# switches gear, and sails past the cusp in a full-lock loop.
_CUSP_HANDOVER_WINDOW_M = 0.12       # latched progress this close to the cusp
_CUSP_HANDOVER_DISTANCE_M = 0.20     # …and the vehicle this close to the cusp point
_CUSP_MANEUVER_MAX_LENGTH_M = 1.5    # segments longer than this track normally
_CUSP_ABORT_DEVIATION_M = 0.35       # watchdog: stop + pause beyond this deviation
# The approach INTO a cusp must be scripted as well: the MPC preview is
# clamped at the gear boundary, so its lookahead collapses on the final arc
# before every reversal — steering faded out and the tractor sailed straight
# past the cusp (observed as bulb-shaped driven turns instead of the planned
# tight three-point turns).
_CUSP_SCRIPT_APPROACH_M = 1.2
_MIN_COVERAGE_ZONE_AREA_M2 = 0.01
_MAX_REPAIR_DISCARDED_AREA_M2 = 0.01
_MAX_REPAIR_DISCARDED_RATIO = 0.001
_VISUAL_PATH_TOLERANCE_M = 0.002
_GEOFENCE_RTK_TOLERANCE_M = 0.01
_PLAN_CACHE_SCHEMA = 1


def _planner_source_fingerprint() -> str:
    """Invalidate persisted routes automatically after planner code changes."""
    mower_root = Path(__file__).resolve().parents[1]
    digest = hashlib.sha256()
    for relative in (
        "executive/mission_runtime.py",
        "path/tractor_route.py",
        "path/boustrophedon.py",
        "path/fields2cover_native.py",
        "path/trajectory.py",
        "map/coverage_grid.py",
    ):
        path = mower_root / relative
        digest.update(relative.encode("utf-8"))
        try:
            digest.update(path.read_bytes())
        except OSError:
            digest.update(b"missing")
    return digest.hexdigest()


_PLANNER_SOURCE_FINGERPRINT = _planner_source_fingerprint()


def _canonical_cache_value(value: Any) -> Any:
    """Normalize JSON-equivalent numbers so 5 and 5.0 hash identically."""
    if value is None or isinstance(value, (str, bool)):
        return value
    if isinstance(value, (int, float)):
        numeric = float(value)
        if not math.isfinite(numeric):
            raise ValueError("non-finite cache input")
        return round(numeric, 12)
    if isinstance(value, dict):
        return {
            str(key): _canonical_cache_value(item)
            for key, item in value.items()
        }
    if isinstance(value, (list, tuple)):
        return [_canonical_cache_value(item) for item in value]
    return str(value)


def _normalize_angle(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def _polygon_parts(geometry: Any) -> list[Polygon]:
    """Return all polygon components from a repaired Shapely geometry."""
    if isinstance(geometry, Polygon):
        return [geometry]
    if isinstance(geometry, (MultiPolygon, GeometryCollection)):
        parts: list[Polygon] = []
        for item in geometry.geoms:
            parts.extend(_polygon_parts(item))
        return parts
    return []


def _valid_coverage_polygon(polygon: Polygon) -> Polygon:
    """Repair harmless GPS artefacts while rejecting ambiguous zone shapes.

    Teach-In can produce repeated or almost identical points.  Shapely may
    interpret the resulting nanometre-sized loop as a second polygon even
    though the recorded field boundary is unambiguous.  Keep the dominant
    polygon only when every discarded component is negligible; a real
    figure-eight or another materially self-intersecting boundary remains a
    hard error and must be corrected by the operator.
    """
    if polygon.is_empty:
        raise ValueError("Die Zonengrenze umschließt keine ausreichend große Fläche")
    if polygon.is_valid:
        if polygon.area < _MIN_COVERAGE_ZONE_AREA_M2:
            raise ValueError("Die Zonengrenze umschließt keine ausreichend große Fläche")
        return polygon

    parts = [part for part in _polygon_parts(make_valid(polygon)) if not part.is_empty]
    if not parts:
        parts = [part for part in _polygon_parts(polygon.buffer(0)) if not part.is_empty]
    if not parts:
        raise ValueError("Die Zonengrenze konnte nicht als Polygon aufbereitet werden")

    parts.sort(key=lambda part: part.area, reverse=True)
    dominant = parts[0]
    discarded_area = sum(part.area for part in parts[1:])
    allowed_discard = max(
        _MAX_REPAIR_DISCARDED_AREA_M2,
        dominant.area * _MAX_REPAIR_DISCARDED_RATIO,
    )
    if discarded_area > allowed_discard:
        raise ValueError(
            "Die Zonengrenze schneidet sich selbst. Bitte die betroffenen "
            "Teach-In-Meter zurücknehmen oder die Zone neu aufzeichnen"
        )
    if not dominant.is_valid or dominant.area < _MIN_COVERAGE_ZONE_AREA_M2:
        raise ValueError("Die Zonengrenze konnte nicht als gültiges Polygon aufbereitet werden")
    return dominant


def _native_boundary_retry_insets(
    boundary_reserve: float,
    curved_clearance: float,
    lateral_clearance: float,
) -> tuple[float, ...]:
    """Return precise Fields2Cover boundary retries without a diagonal box.

    The old fallback used a circle around the complete longitudinal machine
    envelope. A narrow front implement could therefore create a very large
    field-wide inset even though it sits well inside the rear working width.
    Start at the operator-selected reserve and increase only by the actually
    measured curve-over-side-envelope amount. The exact swept-footprint
    validator still decides whether every candidate is collision-safe.
    """
    reserve = max(0.0, float(boundary_reserve))
    curve_extra = max(
        0.0,
        float(curved_clearance) - max(0.0, float(lateral_clearance)),
    )
    extras = (
        0.0,
        curve_extra,
        curve_extra + 0.05,
        curve_extra + 0.10,
        curve_extra + 0.20,
        curve_extra + 0.35,
        curve_extra + 0.50,
    )
    return tuple(dict.fromkeys(round(reserve + extra, 3) for extra in extras))


def _mow_swath_endpoints(
    waypoints: Sequence[TractorWaypoint],
) -> list[tuple[float, float]]:
    """Extract ordered native swaths without their native turn curves."""
    endpoints: list[tuple[float, float]] = []
    active: list[TractorWaypoint] = []
    for point in waypoints:
        if point.operation == "mow":
            active.append(point)
            continue
        if len(active) >= 2:
            endpoints.extend(((active[0].x, active[0].y), (active[-1].x, active[-1].y)))
        active = []
    if len(active) >= 2:
        endpoints.extend(((active[0].x, active[0].y), (active[-1].x, active[-1].y)))
    return endpoints


class MissionRuntime:
    """Execute Teach-In and coverage missions using the production pipeline."""

    def __init__(
        self,
        executive: MissionExecutive,
        hardware: Any,
        store: Any,
        *,
        control_hz: float = 10.0,
        waypoint_tolerance_m: float = 0.35,
        max_speed_mps: float = 0.5,
        correction_resume_tolerance_m: float = 0.5,
        home_arrival_callback: Optional[Callable[[], None]] = None,
        dock_departure_callback: Optional[Callable[[], None]] = None,
        deck_state_callback: Optional[Callable[[bool, bool], None]] = None,
        time_scale_provider: Optional[Callable[[], float]] = None,
    ) -> None:
        if control_hz <= 0:
            raise ValueError("control_hz must be positive")
        self._executive = executive
        self._hardware = hardware
        self._store = store
        self._period_s = 1.0 / control_hz
        self._waypoint_tolerance_m = waypoint_tolerance_m
        self._max_speed_mps = max_speed_mps
        self._correction_resume_tolerance_m = correction_resume_tolerance_m
        self._home_arrival_callback = home_arrival_callback
        self._dock_departure_callback = dock_departure_callback
        self._deck_state_callback = deck_state_callback
        self._time_scale_provider = time_scale_provider or (lambda: 1.0)
        self._controller = MPCController()
        self._teach_in = TeachIn()
        self._teach_reference = "center"
        self._teach_reference_offset_m = 0.0
        self._lock = threading.RLock()
        self._latest_pose: Optional[Pose] = None
        self._utm_zone_number: Optional[int] = None
        self._utm_zone_letter: Optional[str] = None
        self._route_utm: list[tuple[float, float]] = []
        self._route_waypoints: list[TractorWaypoint] = []
        self._route_wgs84: list[tuple[float, float]] = []  # (lon, lat)
        self._route_visual: dict[str, Any] = {
            "geometry": {"type": "LineString", "coordinates": []},
            "turn_geometry": {"type": "MultiLineString", "coordinates": []},
            "mow_geometry": {"type": "MultiLineString", "coordinates": []},
            "headland_geometry": {"type": "MultiLineString", "coordinates": []},
        }
        self._turn_count = 0
        self._turn_waypoint_count = 0
        self._route_index = 0
        self._trajectory: Optional[ContinuousTrajectory] = None
        self._trajectory_s = 0.0
        self._route_mode = "coverage"
        self._planner_version = "coverage_v2"
        self._active_connection_id: Optional[str] = None
        self._route_strategy = "auto"
        self._route_resolution_cm = 0.5
        self._headland_margin_cm = 5.0
        self._mowing_width_cm = 60.0
        self._lane_spacing_cm = 38.0
        self._estimated_coverage_percent = 0.0
        self._estimated_uncovered_area_m2 = 0.0
        self._estimated_mainland_coverage_percent = 0.0
        self._headland_safety_area_m2 = 0.0
        self._geofence_tracking_reserve_cm = 5.0
        self._headland_passes = 0
        self._mowing_turn_radius_cm = 50.0
        self._coverage_strategy = "interior"
        self._route_tolerance_m = waypoint_tolerance_m
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._commands_active = False
        self._teach_suspended = False
        self._deck_lift_state = (False, False)
        self._deck_transition_until = 0.0
        self._last_gear = 1
        self._cusp_maneuver: Optional[dict[str, float]] = None
        self._last_steering_deg = 0.0
        self._last_control_time: Optional[float] = None
        self._departure_corridor_utm: Optional[Polygon] = None
        self._departure_zone_utm: Optional[Polygon] = None
        self._actual_track_utm: list[tuple[float, float]] = []
        self._actual_mown_segments_utm: list[list[tuple[float, float]]] = []
        self._active_mown_segment: Optional[list[tuple[float, float]]] = None
        self._coverage_grid: Optional[CoverageGrid] = None
        self._mower_decks = mower_decks_from_settings({})
        self._route_metrics: dict[str, float | int] = {}
        self._last_controller_output: dict[str, float] = {}
        self._swath_angle_deg = 0.0
        self._plan_safety: dict[str, Any] = {}

    def start(self) -> None:
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(
            target=self._loop, name="mv2-mission-runtime", daemon=True,
        )
        self._thread.start()

    def stop(self) -> None:
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)
            self._thread = None
        self._stop_outputs()

    def on_gps_fix(self, fix: GpsFix) -> None:
        """Remember the UTM band used for WGS84 conversion."""
        with self._lock:
            self._utm_zone_number = fix.utm_zone_number
            self._utm_zone_letter = fix.utm_zone_letter

    def departure_geofence_allows(self, fix: GpsFix) -> bool:
        """Allow only the short planned corridor from a dock outside the zone.

        The exception expires as soon as the robot enters the mowing polygon.
        It therefore cannot be reused later to leave the geofence again.
        """
        point = Point(fix.utm_x, fix.utm_y)
        with self._lock:
            corridor = self._departure_corridor_utm
            zone = self._departure_zone_utm
            if corridor is None or zone is None:
                return False
            # Keep the corridor until the tractor centre is clearly inside;
            # close-to-boundary RTK noise or the initial steering arc must not
            # consume the one-shot exception prematurely.
            inner_zone = zone.buffer(-0.25)
            if (not inner_zone.is_empty and inner_zone.covers(point)) or (
                inner_zone.is_empty and zone.covers(point)
            ):
                self._departure_corridor_utm = None
                self._departure_zone_utm = None
                return False
            if corridor.covers(point):
                return True
            self._departure_corridor_utm = None
            self._departure_zone_utm = None
            return False

    @staticmethod
    def _zone_polygon_utm(
        zone: dict[str, Any],
        zone_number: int,
        zone_letter: str,
    ) -> Polygon:
        geometry = zone.get("geometry") or {}
        rings = geometry.get("coordinates") or []
        if geometry.get("type") != "Polygon" or not rings:
            raise ValueError("Zone besitzt keine gültige Polygon-Geometrie")
        metric_rings: list[list[tuple[float, float]]] = []
        for ring in rings:
            metric: list[tuple[float, float]] = []
            for lon, lat, *_ in ring:
                x, y, number, letter = utm.from_latlon(float(lat), float(lon))
                if number != zone_number or letter != zone_letter:
                    raise ValueError("Geofence überschreitet eine UTM-Zonengrenze")
                metric.append((x, y))
            if len(metric) >= 4:
                metric_rings.append(metric)
        if not metric_rings:
            raise ValueError("Geofence enthält zu wenige Punkte")
        polygon = Polygon(metric_rings[0], holes=metric_rings[1:])
        if not polygon.is_valid:
            polygon = _valid_coverage_polygon(polygon)
        return polygon

    def footprint_geofence_status(
        self,
        fix: GpsFix,
        zone: dict[str, Any],
        no_go_zones: list[dict[str, Any]] | None = None,
    ) -> dict[str, bool]:
        """Check the complete oriented machine, never only its GPS centre."""
        settings = self._store.settings()
        decks = mower_decks_from_settings(settings)
        with self._lock:
            pose = self._latest_pose
            heading = pose.heading_rad if pose is not None else 0.0
            corridor = self._departure_corridor_utm
            departure_zone = self._departure_zone_utm
        footprint = machine_footprint(
            fix.utm_x,
            fix.utm_y,
            heading,
            settings,
            decks,
        )
        allowed_zone = self._zone_polygon_utm(
            zone, fix.utm_zone_number, fix.utm_zone_letter,
        )
        boundary = allowed_zone.buffer(_GEOFENCE_RTK_TOLERANCE_M)
        inside = boundary.covers(footprint)
        departure_allowed = False
        if not inside and corridor is not None and departure_zone is not None:
            departure_allowed = departure_zone.union(corridor).buffer(
                _GEOFENCE_RTK_TOLERANCE_M,
            ).covers(footprint)

        no_go_hit = False
        for no_go in no_go_zones or []:
            obstacle = self._zone_polygon_utm(
                no_go, fix.utm_zone_number, fix.utm_zone_letter,
            )
            if footprint.intersects(obstacle):
                no_go_hit = True
                break

        # The dock corridor remains a one-shot exception.  Once the whole
        # machine is inside, or once it leaves the corridor, it is discarded.
        if corridor is not None:
            with self._lock:
                if inside or (not departure_allowed and not inside):
                    self._departure_corridor_utm = None
                    self._departure_zone_utm = None
        return {
            "allowed": (inside or departure_allowed) and not no_go_hit,
            "inside": inside,
            "departure_allowed": departure_allowed,
            "no_go_hit": no_go_hit,
        }

    def on_pose(self, pose: Pose) -> None:
        """Receive the authoritative fused pose from ``Localizer``."""
        try:
            outputs = self._hardware.output_snapshot()
        except (AttributeError, TypeError):
            outputs = {}
        if not isinstance(outputs, dict):
            outputs = {}
        state = self._executive.state
        moving_mission = (
            abs(pose.speed_mps) > 0.005
            and state in {
                MowerState.MOWING,
                MowerState.OBSTACLE_AVOIDANCE,
                MowerState.RETURNING,
                MowerState.DOCKING,
            }
        )
        blade_active = moving_mission and state == MowerState.MOWING and outputs.get("blade_enabled") is True
        front_active = blade_active and outputs.get("front_deck_raised") is not True
        rear_active = blade_active and outputs.get("rear_deck_raised") is not True
        mowing_now = front_active or rear_active
        with self._lock:
            self._latest_pose = pose
            point = (pose.utm_x, pose.utm_y)
            if self._coverage_grid is not None and mowing_now:
                self._coverage_grid.mark_pose(
                    pose.utm_x,
                    pose.utm_y,
                    pose.heading_rad,
                    self._mower_decks,
                    front_active=front_active,
                    rear_active=rear_active,
                )
            if moving_mission and (
                not self._actual_track_utm
                or math.dist(self._actual_track_utm[-1], point) >= 0.005
            ):
                previous = self._actual_track_utm[-1] if self._actual_track_utm else None
                self._actual_track_utm.append(point)
                if len(self._actual_track_utm) > 50000:
                    del self._actual_track_utm[:10000]
                if mowing_now:
                    if self._active_mown_segment is None:
                        self._active_mown_segment = []
                        if previous is not None:
                            self._active_mown_segment.append(previous)
                        self._actual_mown_segments_utm.append(self._active_mown_segment)
                    if (
                        not self._active_mown_segment
                        or math.dist(self._active_mown_segment[-1], point) >= 0.005
                    ):
                        self._active_mown_segment.append(point)
            if not mowing_now:
                self._active_mown_segment = None
            if self._executive.state == MowerState.TEACH_IN:
                teach_pose = self._teach_reference_pose_locked(pose)
                if self._teach_suspended and self._teach_in.points:
                    target_x, target_y = self._teach_in.points[-1]
                    if math.hypot(
                        target_x - teach_pose.utm_x,
                        target_y - teach_pose.utm_y,
                    ) <= self._correction_resume_tolerance_m:
                        self._teach_suspended = False
                        self._teach_in.update(teach_pose)
                elif not self._teach_suspended:
                    self._teach_in.update(teach_pose)

    def _teach_reference_pose_locked(self, pose: Pose) -> Pose:
        offset = self._teach_reference_offset_m
        return Pose(
            pose.utm_x - math.sin(pose.heading_rad) * offset,
            pose.utm_y + math.cos(pose.heading_rad) * offset,
            pose.heading_rad,
            pose.speed_mps,
            pose.yaw_rate_rps,
            pose.timestamp,
            pose.fix_quality,
        )

    def start_teach_in(
        self,
        feature_type: str = "mowing",
        reference: str = "center",
    ) -> None:
        if self._executive.state != MowerState.IDLE:
            raise RuntimeError(f"Teach-In kann aus {self._executive.state.name} nicht starten")
        if reference not in {"left", "center", "right"}:
            raise ValueError("Teach-In-Referenz muss links, Mitte oder rechts sein")
        settings = self._store.settings()
        decks = mower_decks_from_settings(settings)
        right_extent, left_extent = machine_lateral_bounds(settings, decks)
        reference_offset = {
            "left": left_extent,
            "center": 0.0,
            "right": right_extent,
        }[reference]
        with self._lock:
            self._teach_in.start(feature_type)
            self._teach_reference = reference
            self._teach_reference_offset_m = reference_offset
            self._teach_suspended = False
        self._executive.start_teach_in()

    def stop_teach_in(self) -> dict[str, Any]:
        if self._executive.state != MowerState.TEACH_IN:
            raise RuntimeError("Kein Teach-In aktiv")
        self._stop_outputs()
        with self._lock:
            points = self._teach_in.points
            geometry = self._teach_geometry_locked(points) if points else None
            closed = self._teach_in.state.name == "CLOSED"
        # Validate first.  If geometry construction raises, TEACH_IN remains
        # active so the operator can undo the affected metres instead of
        # losing the recording.
        self._executive.stop_teach_in()
        with self._lock:
            self._teach_suspended = False
        return {
            "geometry": geometry,
            "point_count": len(points),
            "closed": closed,
            "reference": self._teach_reference,
            "reference_offset_m": round(self._teach_reference_offset_m, 3),
        }

    def teach_in_status(self) -> dict[str, Any]:
        with self._lock:
            points = self._teach_in.points
            coordinates = self._points_wgs84_locked(points) if points else []
            target = coordinates[-1] if self._teach_suspended and coordinates else None
            distance_to_target = None
            if self._teach_suspended and points and self._latest_pose is not None:
                target_x, target_y = points[-1]
                reference_pose = self._teach_reference_pose_locked(self._latest_pose)
                distance_to_target = math.hypot(
                    target_x - reference_pose.utm_x,
                    target_y - reference_pose.utm_y,
                )
            return {
                "state": self._executive.state.name,
                "recorder_state": self._teach_in.state.name,
                "recording": self._executive.state == MowerState.TEACH_IN,
                "suspended": self._teach_suspended,
                "closed": self._teach_in.state.name == "CLOSED",
                "point_count": len(points),
                "length_m": round(self._teach_in.path_length_m, 2),
                "geometry": {"type": "LineString", "coordinates": coordinates},
                "correction_target": target,
                "distance_to_target_m": round(distance_to_target, 2) if distance_to_target is not None else None,
                "reference": self._teach_reference,
                "reference_offset_m": round(self._teach_reference_offset_m, 3),
            }

    def undo_teach_in(self, distance_m: float) -> dict[str, Any]:
        if self._executive.state != MowerState.TEACH_IN:
            raise RuntimeError("Kein Teach-In aktiv")
        self._stop_outputs()
        with self._lock:
            removed = self._teach_in.trim_last_distance(distance_m)
            if removed <= 0:
                raise ValueError("Es sind noch nicht genügend aufgezeichnete Meter vorhanden")
            self._teach_suspended = True
        result = self.teach_in_status()
        result["removed_distance_m"] = round(removed, 2)
        return result

    def continue_teach_in_here(self) -> dict[str, Any]:
        if self._executive.state != MowerState.TEACH_IN:
            raise RuntimeError("Kein Teach-In aktiv")
        with self._lock:
            if not self._teach_suspended:
                return self.teach_in_status()
            if self._latest_pose is None:
                raise RuntimeError("Noch keine aktuelle Roboterposition verfügbar")
            self._teach_suspended = False
            self._teach_in.update(self._teach_reference_pose_locked(self._latest_pose))
        return self.teach_in_status()

    def _plan_cache_key(
        self,
        zone: dict[str, Any],
        settings: dict[str, Any],
        start_pose: Optional[Pose],
    ) -> str:
        """Fingerprint every input that can change an executable route."""
        start_context = None
        if start_pose is not None:
            start_context = {
                # One centimetre matches the live RTK geofence tolerance. The
                # unrounded pose is retained in the cache entry and checked
                # again before reuse.
                "x_m": round(float(start_pose.utm_x), 2),
                "y_m": round(float(start_pose.utm_y), 2),
                "heading_deg": round(math.degrees(start_pose.heading_rad), 1),
            }
        payload = {
            "schema": _PLAN_CACHE_SCHEMA,
            "planner_source": _PLANNER_SOURCE_FINGERPRINT,
            "zone": zone,
            "all_zones": sorted(
                self._store.list_items("zones"),
                key=lambda item: (str(item.get("id", "")), str(item.get("type", ""))),
            ),
            "connections": sorted(
                self._store.list_items("connections"),
                key=lambda item: str(item.get("id", "")),
            ),
            "home": self._store.get_home(),
            "settings": settings,
            "start": start_context,
            "executive_state": self._executive.state.name,
        }
        canonical = json.dumps(
            _canonical_cache_value(payload),
            ensure_ascii=False,
            sort_keys=True,
            separators=(",", ":"),
        ).encode("utf-8")
        return hashlib.sha256(canonical).hexdigest()

    @staticmethod
    def _cached_start_pose(start_pose: Optional[Pose]) -> Optional[list[float]]:
        if start_pose is None:
            return None
        return [
            float(start_pose.utm_x),
            float(start_pose.utm_y),
            float(start_pose.heading_rad),
        ]

    def _serialize_plan_cache(
        self,
        cache_key: str,
        start_pose: Optional[Pose],
    ) -> dict[str, Any]:
        with self._lock:
            return {
                "schema": _PLAN_CACHE_SCHEMA,
                "planner_source": _PLANNER_SOURCE_FINGERPRINT,
                "cache_key": cache_key,
                "created_at_epoch": time.time(),
                "start_pose": self._cached_start_pose(start_pose),
                "utm_zone": [self._utm_zone_number, self._utm_zone_letter],
                "waypoints": [
                    [point.x, point.y, point.operation, point.gear, point.turn_id]
                    for point in self._route_waypoints
                ],
                "route_visual": copy.deepcopy(self._route_visual),
                "departure_corridor": (
                    mapping(self._departure_corridor_utm)
                    if self._departure_corridor_utm is not None else None
                ),
                "departure_zone": (
                    mapping(self._departure_zone_utm)
                    if self._departure_zone_utm is not None else None
                ),
                "metadata": {
                    "route_mode": self._route_mode,
                    "planner_version": self._planner_version,
                    "active_connection_id": self._active_connection_id,
                    "route_strategy": self._route_strategy,
                    "coverage_strategy": self._coverage_strategy,
                    "route_resolution_cm": self._route_resolution_cm,
                    "headland_margin_cm": self._headland_margin_cm,
                    "mowing_width_cm": self._mowing_width_cm,
                    "lane_spacing_cm": self._lane_spacing_cm,
                    "estimated_coverage_percent": self._estimated_coverage_percent,
                    "estimated_uncovered_area_m2": self._estimated_uncovered_area_m2,
                    "estimated_mainland_coverage_percent": self._estimated_mainland_coverage_percent,
                    "headland_safety_area_m2": self._headland_safety_area_m2,
                    "geofence_tracking_reserve_cm": self._geofence_tracking_reserve_cm,
                    "headland_passes": self._headland_passes,
                    "mowing_turn_radius_cm": self._mowing_turn_radius_cm,
                    "route_tolerance_m": self._route_tolerance_m,
                    "route_metrics": dict(self._route_metrics),
                    "swath_angle_deg": self._swath_angle_deg,
                    "plan_safety": dict(self._plan_safety),
                },
            }

    def _restore_plan_cache(
        self,
        cached: dict[str, Any],
        polygon: Polygon,
        settings: dict[str, Any],
        start_pose: Optional[Pose],
    ) -> bool:
        if (
            int(cached.get("schema", -1)) != _PLAN_CACHE_SCHEMA
            or cached.get("planner_source") != _PLANNER_SOURCE_FINGERPRINT
        ):
            return False
        cached_start = cached.get("start_pose")
        if (cached_start is None) != (start_pose is None):
            return False
        if start_pose is not None:
            if not isinstance(cached_start, list) or len(cached_start) != 3:
                return False
            if math.hypot(
                float(cached_start[0]) - start_pose.utm_x,
                float(cached_start[1]) - start_pose.utm_y,
            ) > 0.015:
                return False
            if abs(_normalize_angle(float(cached_start[2]) - start_pose.heading_rad)) > math.radians(0.25):
                return False
        raw_waypoints = cached.get("waypoints")
        metadata = cached.get("metadata")
        utm_zone = cached.get("utm_zone")
        if (
            not isinstance(raw_waypoints, list)
            or len(raw_waypoints) < 2
            or not isinstance(metadata, dict)
            or not isinstance(utm_zone, list)
            or len(utm_zone) != 2
        ):
            return False
        waypoints = [
            TractorWaypoint(
                float(item[0]),
                float(item[1]),
                str(item[2]),
                int(item[3]),
                int(item[4]) if item[4] is not None else None,
            )
            for item in raw_waypoints
            if isinstance(item, list) and len(item) == 5
        ]
        if len(waypoints) != len(raw_waypoints):
            return False
        configured_turn_radius = max(
            0.10, float(settings.get("turn_radius_cm", 25.0)) / 100.0,
        )
        cached_minimum_radius, _ = measure_minimum_turn_radius(waypoints)
        if cached_minimum_radius < configured_turn_radius * 0.995:
            return False
        zone_number = int(utm_zone[0])
        zone_letter = str(utm_zone[1])
        trajectory = ContinuousTrajectory(waypoints)
        cached_visual = cached.get("route_visual")
        if (
            isinstance(cached_visual, dict)
            and isinstance(cached_visual.get("geometry"), dict)
            and cached_visual["geometry"].get("type") == "LineString"
        ):
            route_visual = copy.deepcopy(cached_visual)
        else:
            route_visual = self._build_visual_route(
                waypoints, zone_number, zone_letter,
            )
        decks = mower_decks_from_settings(settings)
        coverage_grid = CoverageGrid(
            polygon,
            resolution_m=max(
                0.01,
                min(0.10, float(settings.get("coverage_grid_resolution_cm", 2)) / 100.0),
            ),
        )
        route_metrics = dict(metadata.get("route_metrics") or {})
        route_metrics["plan_cache_hit"] = True
        route_metrics["plan_cache_key"] = str(cached.get("cache_key", ""))[:12]
        route_metrics["plan_cache_age_s"] = round(max(
            0.0, time.time() - float(cached.get("created_at_epoch", time.time())),
        ), 1)
        departure_corridor = cached.get("departure_corridor")
        departure_zone = cached.get("departure_zone")
        with self._lock:
            self._utm_zone_number = zone_number
            self._utm_zone_letter = zone_letter
            self._route_utm = [(point.x, point.y) for point in waypoints]
            self._route_waypoints = waypoints
            self._route_wgs84 = [
                tuple(point) for point in route_visual["geometry"]["coordinates"]
            ]
            self._route_visual = route_visual
            self._turn_count = max((point.turn_id or 0 for point in waypoints), default=0)
            self._turn_waypoint_count = sum(point.operation == "turn" for point in waypoints)
            self._route_index = 1
            self._trajectory = trajectory
            self._trajectory_s = 0.0
            self._route_mode = str(metadata.get("route_mode", "coverage"))
            self._planner_version = str(metadata.get("planner_version", "coverage_hybrid_v3"))
            self._active_connection_id = metadata.get("active_connection_id")
            self._route_strategy = str(metadata.get("route_strategy", "auto"))
            self._coverage_strategy = str(metadata.get("coverage_strategy", "interior"))
            self._route_resolution_cm = float(metadata.get("route_resolution_cm", 0.5))
            self._headland_margin_cm = float(metadata.get("headland_margin_cm", 5.0))
            self._mowing_width_cm = float(metadata.get("mowing_width_cm", 60.0))
            self._lane_spacing_cm = float(metadata.get("lane_spacing_cm", 38.0))
            self._estimated_coverage_percent = float(metadata.get("estimated_coverage_percent", 0.0))
            self._estimated_uncovered_area_m2 = float(metadata.get("estimated_uncovered_area_m2", 0.0))
            self._estimated_mainland_coverage_percent = float(metadata.get("estimated_mainland_coverage_percent", 0.0))
            self._headland_safety_area_m2 = float(metadata.get("headland_safety_area_m2", 0.0))
            self._geofence_tracking_reserve_cm = float(metadata.get("geofence_tracking_reserve_cm", 5.0))
            self._headland_passes = int(metadata.get("headland_passes", 0))
            self._mowing_turn_radius_cm = float(metadata.get("mowing_turn_radius_cm", 50.0))
            self._departure_corridor_utm = shape(departure_corridor) if departure_corridor else None
            self._departure_zone_utm = shape(departure_zone) if departure_zone else None
            self._route_tolerance_m = float(metadata.get("route_tolerance_m", self._waypoint_tolerance_m))
            self._last_gear = 1
            self._cusp_maneuver = None
            self._last_steering_deg = 0.0
            self._last_control_time = None
            self._mower_decks = decks
            self._coverage_grid = coverage_grid
            self._route_metrics = route_metrics
            self._last_controller_output = {}
            self._swath_angle_deg = float(metadata.get("swath_angle_deg", 0.0))
            self._plan_safety = dict(metadata.get("plan_safety") or {})
        return True

    def plan_zone(self, zone: dict[str, Any]) -> dict[str, Any]:
        """Plan a metric coverage route for a persisted WGS84 polygon."""
        geometry = zone.get("geometry") or {}
        if geometry.get("type") != "Polygon":
            raise ValueError("Die Zone besitzt keine gültige Polygon-Geometrie")
        rings = geometry.get("coordinates") or []
        if not rings or len(rings[0]) < 4:
            raise ValueError("Die Zonengrenze enthält zu wenige Punkte")

        zone_number: Optional[int] = None
        zone_letter: Optional[str] = None
        metric_ring: list[tuple[float, float]] = []
        for coordinate in rings[0]:
            if len(coordinate) < 2:
                raise ValueError("Ungültiger Koordinatenpunkt in der Zone")
            lon, lat = float(coordinate[0]), float(coordinate[1])
            x, y, number, letter = utm.from_latlon(lat, lon)
            if zone_number is None:
                zone_number, zone_letter = number, letter
            elif number != zone_number or letter != zone_letter:
                raise ValueError("Die Zone überschreitet eine UTM-Zonengrenze")
            metric_ring.append((x, y))

        polygon = _valid_coverage_polygon(Polygon(metric_ring))
        settings = self._store.settings()
        mowing_width = max(0.30, float(settings.get("mowing_width_cm", 60)) / 100.0)
        lane_spacing = max(
            0.10,
            min(mowing_width, float(settings.get("lane_width_cm", 38)) / 100.0),
        )
        # Execution waypoints are sampled below one centimetre.  Tracking
        # tolerance remains separate because RTK/control updates cannot stop on
        # every individual 5 mm sample.
        resolution_cm = max(0.2, min(0.9, float(settings.get("route_resolution_cm", 0.5))))
        point_spacing = resolution_cm / 100.0
        turn_strategy = str(settings.get("turn_strategy", "auto"))
        planner_engine = str(settings.get("planner_engine", "mv2"))
        configured_margin_cm = max(
            5.0, min(150.0, float(settings.get("headland_margin_cm", 5.0))),
        )
        boundary_reserve = configured_margin_cm / 100.0
        coverage_strategy = str(settings.get("coverage_strategy", "headland_first"))
        headland_passes = (
            max(1, min(3, int(settings.get("headland_passes", 1))))
            if coverage_strategy == "headland_first"
            else 0
        )
        # A requested outside pass is extra coverage and manoeuvring space.
        # It never removes required parallel swaths from the coverage target.
        headland_width = (
            mowing_width + (headland_passes - 1) * lane_spacing
            if headland_passes else 0.0
        )
        total_turn_space = headland_passes * lane_spacing
        turn_radius = max(
            0.10,
            min(1.0, float(settings.get("turn_radius_cm", 25)) / 100.0),
        )
        mowing_turn_radius = max(
            turn_radius + 0.05,
            min(1.50, float(settings.get("mowing_turn_radius_cm", 50)) / 100.0),
        )
        with self._lock:
            start_pose = self._latest_pose
        cache_key = self._plan_cache_key(zone, settings, start_pose)
        cache_loader = getattr(self._store, "get_plan_cache", None)
        if callable(cache_loader):
            cached_plan = cache_loader(cache_key)
            if cached_plan is not None:
                try:
                    if self._restore_plan_cache(
                        cached_plan, polygon, settings, start_pose,
                    ):
                        logger.info(
                            "Reusing validated mission plan %s for zone %s",
                            cache_key[:12], zone.get("id"),
                        )
                        return self.route_geojson()
                except (KeyError, TypeError, ValueError, OverflowError):
                    logger.warning(
                        "Ignoring invalid persisted mission plan %s",
                        cache_key[:12], exc_info=True,
                    )
                cache_delete = getattr(self._store, "delete_plan_cache", None)
                if callable(cache_delete):
                    cache_delete(cache_key)
        # Coverage topology must not depend on the vehicle's current pose.
        # Even an in-zone pose near an edge can rotate the closed headland so
        # its end lies in a pocket from which the first interior lane is not
        # reachable.  Build the internally consistent field route first and
        # add the pose-to-route transit only after that route is complete.
        planning_start: tuple[float, float, float] | None = None
        # ``headland_margin_cm`` is the one complete physical reserve.  Keep
        # the original polygon as the authoritative geofence and apply this
        # value exactly once to the swept machine contour below.  In
        # particular, do not pre-buffer the zone and then add another hidden
        # tracking/planning margin.
        safe_polygon = polygon
        decks = mower_decks_from_settings(settings)
        right_extent, left_extent = machine_lateral_bounds(settings, decks)
        lateral_clearance = max(abs(right_extent), abs(left_extent))
        rear_extent, front_extent = machine_longitudinal_bounds(settings, decks)
        curved_clearance = machine_curve_clearance(settings, turn_radius, decks)
        planning_polygon = safe_polygon
        # The selected reserve is the complete physical margin.  Do not add a
        # hidden manoeuvre strip merely because one or more outside passes are
        # present: overlap is allowed and full coverage has priority.
        manoeuvre_planning_clearance = boundary_reserve
        no_go_polygons: list[Polygon] = []
        for candidate in self._store.list_items("zones"):
            if candidate.get("type") != "no_go" or not candidate.get("geometry"):
                continue
            try:
                no_go_polygons.append(self._zone_polygon_utm(
                    candidate, int(zone_number), str(zone_letter),
                ))
            except ValueError:
                # A malformed persisted obstacle must never silently weaken the
                # live safety gate, but it should not make an unrelated field
                # impossible to plan when it lies in another UTM zone.
                logger.warning("No-Go-Zone %s could not be used for planning", candidate.get("id"))

        def route_footprint_is_safe(candidate: Sequence[TractorWaypoint]) -> bool:
            if len(candidate) < 2:
                return False
            return bool(validate_swept_machine_route(
                candidate,
                safe_polygon,
                settings,
                decks,
                reserve_m=0.0,
                no_go_zones=no_go_polygons,
                maximum_sample_step_m=min(0.02, max(0.005, point_spacing * 2.0)),
                geometry_tolerance_m=0.001,
            )["valid"])

        def route_has_tracking_reserve(candidate: Sequence[TractorWaypoint]) -> bool:
            if len(candidate) < 2:
                return False
            return bool(validate_swept_machine_route(
                candidate,
                safe_polygon,
                settings,
                decks,
                reserve_m=boundary_reserve,
                no_go_zones=no_go_polygons,
                maximum_sample_step_m=min(0.02, max(0.005, point_spacing * 2.0)),
                geometry_tolerance_m=0.003,
            )["valid"])
        # The selected reserve is measured at the moving machine contour. The
        # curve-only swing component is geometric clearance, not a second
        # configurable reserve.
        mowing_curve_clearance = machine_curve_clearance(
            settings, mowing_turn_radius, decks,
        )
        headland_base_clearance = (
            boundary_reserve
            + max(0.0, mowing_curve_clearance - lateral_clearance)
        )
        headland_polygon = planning_polygon.buffer(
            -headland_base_clearance, join_style="round",
        )
        for obstacle in no_go_polygons:
            headland_polygon = headland_polygon.difference(obstacle.buffer(
                lateral_clearance + boundary_reserve,
                join_style="round",
            ))
        if headland_polygon.is_empty or not isinstance(headland_polygon, Polygon):
            raise ValueError("Die Zone ist zu schmal für die Geofence-Fahrreserve")
        working_polygon = planning_polygon
        lane_geometry = planning_polygon.buffer(
            -(lateral_clearance + manoeuvre_planning_clearance), join_style="round",
        )
        for obstacle in no_go_polygons:
            lane_geometry = lane_geometry.difference(obstacle.buffer(
                lateral_clearance + manoeuvre_planning_clearance,
                join_style="round",
            ))
        turn_geometry = lane_geometry
        if turn_geometry.is_empty:
            raise ValueError(
                "Die Zone ist zu schmal für den breitesten Punkt von Traktor oder Mähwerk"
            )
        if not isinstance(turn_geometry, Polygon):
            parts = _polygon_parts(turn_geometry)
            if not parts:
                raise ValueError(
                    "Die Zone bietet keinen zusammenhängenden sicheren Fahrbereich"
                )
            turn_geometry = max(parts, key=lambda part: part.area)
        turn_boundary = _valid_coverage_polygon(turn_geometry)
        # Outside passes never remove regular swaths and never shorten them.
        # At the two *lateral* field edges, however, the rear-axle line must
        # retain the exact curved implement envelope or the tractor cannot
        # leave that lane without swinging a deck across the geofence.  This
        # clearance depends on the machine/radius only, not on whether one,
        # two or three perimeter passes were requested.  The explicit 3 cm
        # steering-transition band is the distance needed to build curvature
        # from straight travel; it is not another configurable wall reserve.
        steering_transition_m = 0.03
        lane_position_geometry: BaseGeometry = planning_polygon.buffer(
            -(curved_clearance + boundary_reserve + steering_transition_m),
            join_style="round",
        )
        for obstacle in no_go_polygons:
            lane_position_geometry = lane_position_geometry.difference(
                obstacle.buffer(
                    curved_clearance + boundary_reserve + steering_transition_m,
                    join_style="round",
                )
            )
        if lane_position_geometry.is_empty:
            lane_position_geometry = lane_geometry
        # Swath endpoints depend only on the real longitudinal machine
        # contour.  The number of outside passes must not shorten the regular
        # lanes; already mown headland may be crossed again and used to turn.
        lane_start_trim = abs(min(0.0, rear_extent)) + manoeuvre_planning_clearance
        lane_end_trim = max(0.0, front_extent) + manoeuvre_planning_clearance
        lane_endpoint_trim = max(lane_start_trim, lane_end_trim)
        # Plan V2 keeps the actual field topology.  The former global negative
        # buffer pinched concave zones and produced artificial boundaries in
        # open grass.  The headland space is instead removed longitudinally
        # from each individual swath below.
        headland_waypoints: list[TractorWaypoint] = []
        selected_mowing_turn_radius = mowing_turn_radius
        if headland_passes:
            last_headland_error: ValueError | None = None
            # Start from the physical front-swing envelope rather than trying
            # many known-unsafe radii at 5 mm resolution.  Larger fallbacks
            # are retained for unusual/asymmetric implement configurations.
            for candidate_mowing_radius in (
                mowing_turn_radius,
                mowing_turn_radius + 0.15,
                mowing_turn_radius + 0.30,
            ):
                try:
                    candidate_headland = compute_headland_route(
                        headland_polygon,
                        lane_spacing=lane_spacing,
                        mowing_width=mowing_width,
                        point_spacing=point_spacing,
                        turn_radius=candidate_mowing_radius,
                        passes=headland_passes,
                        start_pose=planning_start,
                        connection_boundary=turn_boundary,
                        route_validator=route_footprint_is_safe,
                        corner_clearance_m=max(
                            0.0,
                            machine_curve_clearance(
                                settings, candidate_mowing_radius, decks,
                            ) - lateral_clearance,
                        ),
                    )
                except ValueError as exc:
                    last_headland_error = exc
                    continue
                if route_has_tracking_reserve(candidate_headland):
                    headland_waypoints = candidate_headland
                    selected_mowing_turn_radius = candidate_mowing_radius
                    break
            if not headland_waypoints:
                if last_headland_error is not None:
                    raise last_headland_error
                raise ValueError(
                    "Keine Außenbahn hält den Mähwerküberhang an den Ecken ein"
                )
        coverage_start = planning_start
        preserve_headland_entry = False
        if len(headland_waypoints) >= 2:
            coverage_start = (
                headland_waypoints[-1].x,
                headland_waypoints[-1].y,
                math.atan2(
                    headland_waypoints[-1].y - headland_waypoints[-2].y,
                    headland_waypoints[-1].x - headland_waypoints[-2].x,
                ),
            )
        native_coverage_target = None
        base_waypoints, swath_angle_deg, swath_metrics = compute_best_boustrophedon(
            working_polygon,
            lane_spacing=lane_spacing,
            mowing_width=mowing_width,
            endpoint_trim_m=lane_endpoint_trim,
            endpoint_start_trim_m=lane_endpoint_trim,
            endpoint_end_trim_m=lane_endpoint_trim,
            centerline_zone=lane_geometry,
            lane_position_zone=lane_position_geometry,
        )
        if (
            headland_passes
            and len(base_waypoints) >= 2
            and not preserve_headland_entry
        ):
            desired_entry_heading = math.atan2(
                base_waypoints[1][1] - base_waypoints[0][1],
                base_waypoints[1][0] - base_waypoints[0][0],
            )
            # Rotate the closed outside pass so its natural exit lies beside
            # and points toward the first complete swath. This avoids a
            # separate 180-degree connector after finishing the perimeter.
            aligned_headland = compute_headland_route(
                headland_polygon,
                lane_spacing=lane_spacing,
                mowing_width=mowing_width,
                point_spacing=point_spacing,
                turn_radius=selected_mowing_turn_radius,
                passes=headland_passes,
                start_pose=(
                    base_waypoints[0][0],
                    base_waypoints[0][1],
                    desired_entry_heading,
                ),
                connection_boundary=turn_boundary,
                route_validator=route_footprint_is_safe,
                corner_clearance_m=max(
                    0.0,
                    machine_curve_clearance(
                        settings, selected_mowing_turn_radius, decks,
                    ) - lateral_clearance,
                ),
            )
            if route_has_tracking_reserve(aligned_headland):
                headland_waypoints = aligned_headland
                coverage_start = (
                    headland_waypoints[-1].x,
                    headland_waypoints[-1].y,
                    math.atan2(
                        headland_waypoints[-1].y - headland_waypoints[-2].y,
                        headland_waypoints[-1].x - headland_waypoints[-2].x,
                    ),
                )
        # Keep every geometrically reachable lane.  Previously lanes were
        # removed when their footprint overlapped the outside pass; that saved
        # distance but created the visible holes reported by the operator.
        coverage_waypoints = [] if planner_engine == "fields2cover" else compute_tractor_route(
            working_polygon,
            lane_spacing=lane_spacing,
            mowing_width=mowing_width,
            point_spacing=point_spacing,
            turn_strategy=turn_strategy,
            turn_radius=turn_radius,
            maneuver_passes=int(settings.get("turn_maneuver_passes", 4)),
            boundary_zone=turn_boundary,
            boundary_clearance=total_turn_space,
            start_pose=coverage_start,
            lane_endpoint_trim_m=lane_endpoint_trim,
            base_waypoints=base_waypoints,
            route_validator=route_has_tracking_reserve,
        )
        waypoints = coverage_waypoints

        def attach_headland_route(
            perimeter: list[TractorWaypoint],
            coverage: list[TractorWaypoint],
        ) -> tuple[list[TractorWaypoint], list[TractorWaypoint]]:
            """Connect a safe perimeter pass to either coverage planner."""
            if not perimeter or not coverage:
                raise ValueError("Außen- oder Innenbahnen konnten nicht geplant werden")
            transition_id = max(
                (point.turn_id or 0 for point in perimeter), default=0,
            ) + 1
            first_coverage = coverage[0]
            coverage_heading = math.atan2(
                coverage[1].y - first_coverage.y,
                coverage[1].x - first_coverage.x,
            )
            # A concave field may put the nominal end of a closed headland loop
            # in a pocket from which the first interior lane cannot be reached
            # with the configured radius.  Walk back by at most two metres and
            # choose the latest feasible exit instead of rejecting the whole
            # otherwise valid coverage plan.
            transition: list[TractorWaypoint] | None = None
            transition_cut = len(perimeter) - 1
            sample_step = max(1, int(round(0.05 / point_spacing)))
            maximum_backtrack = max(1, int(round(2.0 / point_spacing)))
            minimum_cut = max(1, transition_cut - maximum_backtrack)
            candidate_cuts = list(range(transition_cut, minimum_cut - 1, -sample_step))
            if candidate_cuts[-1] != minimum_cut:
                candidate_cuts.append(minimum_cut)
            for candidate_cut in candidate_cuts:
                transition_start = perimeter[candidate_cut]
                previous = perimeter[candidate_cut - 1]
                transition_heading = math.atan2(
                    transition_start.y - previous.y,
                    transition_start.x - previous.x,
                )
                try:
                    transition = compute_curvature_bounded_connection(
                        (transition_start.x, transition_start.y, transition_heading),
                        (first_coverage.x, first_coverage.y, coverage_heading),
                        turn_radius=turn_radius,
                        point_spacing=point_spacing,
                        boundary=safe_polygon,
                        allow_reverse=True,
                        operation="turn",
                        turn_id=transition_id,
                        route_validator=route_has_tracking_reserve,
                    )
                    # Validate the actual stitched headings at both seams.
                    # Checking the connector in isolation can assign a
                    # slightly different endpoint heading than the adjacent
                    # perimeter/coverage samples and miss deck swing there.
                    seam_probe = [
                        *perimeter[:candidate_cut + 1],
                        *transition[1:],
                        *coverage[1:min(len(coverage), 40)],
                    ]
                    if not route_has_tracking_reserve(seam_probe):
                        transition = None
                        continue
                    transition_cut = candidate_cut
                    break
                except ValueError:
                    continue
            if transition is None:
                raise ValueError(
                    "Keine radiusbegrenzte Verbindung von Außen- zu Innenbahnen gefunden"
                )
            perimeter = perimeter[:transition_cut + 1]
            combined = list(perimeter)
            combined.extend(transition[1:])
            for point in coverage[1:]:
                combined.append(TractorWaypoint(
                    point.x,
                    point.y,
                    point.operation,
                    point.gear,
                    point.turn_id + transition_id if point.turn_id is not None else None,
                ))
            return combined, perimeter

        if headland_passes and planner_engine != "fields2cover":
            waypoints, headland_waypoints = attach_headland_route(
                headland_waypoints, coverage_waypoints,
            )
        if planner_engine == "fields2cover":
            # Fields2Cover's Robot.width is a *lateral* width.  The former
            # implementation passed the diameter of a diagonal bounding
            # circle, which turned the 60 cm machine into an artificial
            # one-metre-wide robot and inflated the headland.  Keep that
            # diagonal radius only for conservative boundary retries below;
            # the native robot receives the real widest lateral contour.
            native_machine_width = max(0.15, left_extent - right_extent)
            auto_native_headland_width = max(
                native_machine_width,
                turn_radius + curved_clearance + boundary_reserve,
            )
            native_headland_mode = str(
                settings.get("fields2cover_headland_mode", "auto")
            )
            native_headland_width = (
                max(0.40, min(1.50, float(
                    settings.get("fields2cover_headland_width_cm", 70.0)
                ) / 100.0))
                if native_headland_mode == "manual"
                else auto_native_headland_width
            )
            # Native path planning constrains curvature, but it does not test
            # our offset front/rear implements against the field boundary.
            # Ask Fields2Cover again with progressively more headland space
            # until our exact swept machine polygons accept the native path.
            native_plan = None
            native_boundary_inset = boundary_reserve
            candidate_insets = _native_boundary_retry_insets(
                boundary_reserve,
                curved_clearance,
                lateral_clearance,
            )
            for candidate_inset in candidate_insets:
                native_polygon = safe_polygon.buffer(
                    -candidate_inset, join_style="round",
                )
                for obstacle in no_go_polygons:
                    native_polygon = native_polygon.difference(obstacle.buffer(
                        candidate_inset, join_style="round",
                    ))
                if not isinstance(native_polygon, Polygon):
                    native_parts = _polygon_parts(native_polygon)
                    if not native_parts:
                        continue
                    native_polygon = max(native_parts, key=lambda part: part.area)
                candidate_plan = plan_fields2cover(
                    native_polygon,
                    machine_width=native_machine_width,
                    coverage_width=mowing_width,
                    headland_width=native_headland_width,
                    reference_forward_offset=0.0,
                    lane_spacing=lane_spacing,
                    turn_radius=turn_radius,
                    settings=settings,
                )
                raw_native_plan = candidate_plan
                # Keep the native decomposition/coverage target, but use the
                # deterministic maximum-coverage angle evaluator for executable
                # swaths. Native SG may choose a slightly diagonal clipped edge
                # lane that creates an unnecessary field-wide connector.
                native_base_waypoints = list(base_waypoints)
                rebuilt_waypoints: list[TractorWaypoint] | None = None
                if len(native_base_waypoints) >= 2:
                    try:
                        rebuilt_waypoints = compute_tractor_route(
                            working_polygon,
                            lane_spacing=lane_spacing,
                            mowing_width=mowing_width,
                            point_spacing=point_spacing,
                            turn_strategy=turn_strategy,
                            turn_radius=turn_radius,
                            maneuver_passes=int(settings.get("turn_maneuver_passes", 4)),
                            boundary_zone=turn_boundary,
                            boundary_clearance=max(total_turn_space, lane_spacing),
                            start_pose=coverage_start,
                            lane_endpoint_trim_m=0.0,
                            base_waypoints=native_base_waypoints,
                            route_validator=route_has_tracking_reserve,
                        )
                    except ValueError:
                        pass
                # Preference which backend plans the executable turns:
                #   auto   = MV2-rebuilt first, native as fallback (historical
                #            default — native swaths once produced diagonal
                #            edge lanes with field-wide connectors)
                #   native = genuine Fields2Cover path (dubins/reeds_shepp_cc
                #            per fields2cover_path_type) preferred; only this
                #            makes the path-type setting actually effective
                turning_preference = str(
                    settings.get("fields2cover_turning_backend", "auto")
                )
                plan_variants: list[Fields2CoverPlan] = []
                if rebuilt_waypoints is not None:
                    rebuilt_metrics = dict(candidate_plan.metrics)
                    rebuilt_metrics["turning_backend"] = "mv2_kinematic"
                    rebuilt_metrics["native_swath_segments"] = len(native_base_waypoints) // 2
                    rebuilt_metrics["native_suggested_angle_deg"] = round(
                        candidate_plan.swath_angle_deg, 3,
                    )
                    rebuilt_metrics["effective_swath_angle_deg"] = round(
                        swath_angle_deg, 3,
                    )
                    rebuilt_metrics.update({
                        f"coverage_{key}": value for key, value in swath_metrics.items()
                    })
                    plan_variants.append(Fields2CoverPlan(
                        waypoints=rebuilt_waypoints,
                        swath_angle_deg=swath_angle_deg,
                        coverage_geometry=candidate_plan.coverage_geometry,
                        metrics=rebuilt_metrics,
                    ))
                native_fallback_metrics = dict(raw_native_plan.metrics)
                native_fallback_metrics["turning_backend"] = "fields2cover_native_fallback"
                plan_variants.append(Fields2CoverPlan(
                    waypoints=raw_native_plan.waypoints,
                    swath_angle_deg=raw_native_plan.swath_angle_deg,
                    coverage_geometry=raw_native_plan.coverage_geometry,
                    metrics=native_fallback_metrics,
                ))
                if turning_preference == "native":
                    # Native turns explicitly requested: evaluate the genuine
                    # Fields2Cover path first, MV2-rebuilt only as fallback.
                    plan_variants.reverse()
                for plan_variant in plan_variants:
                    route_quality = self._measure_route(plan_variant.waypoints, 0.0)
                    variant_minimum_radius, _ = measure_minimum_turn_radius(
                        plan_variant.waypoints,
                    )
                    if (
                        float(route_quality["longest_reverse_run_m"])
                        > _CUSP_MANEUVER_MAX_LENGTH_M  # follower executes scripted reverse legs up to this length
                        or float(route_quality["maximum_turn_rotation_deg"]) > 285.0
                        or variant_minimum_radius < turn_radius * 0.995
                    ):
                        continue
                    if route_has_tracking_reserve(plan_variant.waypoints):
                        native_plan = plan_variant
                        native_boundary_inset = candidate_inset
                        break
                if native_plan is not None:
                    break
            if native_plan is None:
                raise ValueError(
                    "Fields2Cover konnte mit der vollständigen Traktor-/Mähwerk-"
                    "Kontur keinen geofencesicheren nativen Pfad erzeugen. Bitte "
                    "als Kurvenmodell zunächst Dubins wählen oder die "
                    "Fields2Cover-Vorgewendebreite erhöhen"
                )
            waypoints = native_plan.waypoints
            native_coverage_target = native_plan.coverage_geometry
            coverage_waypoints = waypoints
            swath_angle_deg = native_plan.swath_angle_deg
            swath_metrics = dict(native_plan.metrics)
            swath_metrics["boundary_inset_m"] = round(native_boundary_inset, 3)
            swath_metrics["headland_mode"] = native_headland_mode
            swath_metrics["automatic_headland_width_m"] = round(
                auto_native_headland_width, 3,
            )
            if headland_passes:
                if not preserve_headland_entry:
                    # Align the perimeter exit with the route that was
                    # actually selected. This matters when the native fallback
                    # uses a different first swath than the deterministic
                    # coverage proposal.
                    native_entry = coverage_waypoints[0]
                    native_entry_heading = math.atan2(
                        coverage_waypoints[1].y - native_entry.y,
                        coverage_waypoints[1].x - native_entry.x,
                    )
                    try:
                        native_aligned_headland = compute_headland_route(
                            headland_polygon,
                            lane_spacing=lane_spacing,
                            mowing_width=mowing_width,
                            point_spacing=point_spacing,
                            turn_radius=selected_mowing_turn_radius,
                            passes=headland_passes,
                            start_pose=(
                                native_entry.x,
                                native_entry.y,
                                native_entry_heading,
                            ),
                            connection_boundary=turn_boundary,
                            route_validator=route_footprint_is_safe,
                            corner_clearance_m=max(
                                0.0,
                                machine_curve_clearance(
                                    settings, selected_mowing_turn_radius, decks,
                                ) - lateral_clearance,
                            ),
                        )
                        if route_has_tracking_reserve(native_aligned_headland):
                            headland_waypoints = native_aligned_headland
                    except ValueError:
                        # The previously validated perimeter remains a safe
                        # fallback; only its connector may be less direct.
                        pass
                waypoints, headland_waypoints = attach_headland_route(
                    headland_waypoints, coverage_waypoints,
                )
                coverage_waypoints = waypoints
            coverage_strategy = "fields2cover_native"
        departure_corridor: Optional[Polygon] = None
        departure_waypoint_count = 0
        staged_entry_used = False
        staged_entry_fraction = 0.0
        dock_connection: Optional[dict[str, Any]] = None
        connection_reverse = False
        source_zone_polygon: Optional[Polygon] = None
        if start_pose is not None and waypoints:
            start_point = Point(start_pose.utm_x, start_pose.utm_y)
            start_inside = polygon.buffer(0.05).covers(start_point)
            departing_dock = self._executive.state == MowerState.CHARGING
            first = waypoints[0]
            dx, dy = first.x - start_pose.utm_x, first.y - start_pose.utm_y
            distance = math.hypot(dx, dy)
            target_heading = math.atan2(
                waypoints[1].y - first.y, waypoints[1].x - first.x,
            )
            heading_error = abs(_normalize_angle(target_heading - start_pose.heading_rad))
            dock_guide: list[tuple[float, float]] = []

            start_footprint = machine_footprint(
                start_pose.utm_x,
                start_pose.utm_y,
                start_pose.heading_rad,
                settings,
                decks,
            )
            start_footprint_safe = safe_polygon.buffer(0.003).covers(start_footprint)
            if start_footprint_safe:
                start_footprint_safe = all(
                    obstacle.buffer(0.003).disjoint(start_footprint)
                    for obstacle in no_go_polygons
                )
            if start_inside and not start_footprint_safe:
                raise ValueError(
                    "Startposition nicht sicher: Traktor oder Mähwerk ragt über den "
                    "Geofence oder in eine No-Go-Zone. Bitte das Fahrzeug manuell "
                    "vollständig in einen freien Bereich fahren"
                )

            def add_target_entry_staging(guide: list[tuple[float, float]]) -> None:
                """Extend an operator line only inside the selected safe field."""
                endpoint = guide[-1]
                centre = turn_boundary.representative_point()
                heading = math.atan2(centre.y - endpoint[1], centre.x - endpoint[0])
                base_distance = curved_clearance + boundary_reserve
                for factor in (1.0, 1.5, 2.0, 3.0):
                    candidate = (
                        endpoint[0] + math.cos(heading) * base_distance * factor,
                        endpoint[1] + math.sin(heading) * base_distance * factor,
                    )
                    if turn_boundary.buffer(0.003).covers(Point(candidate)):
                        guide.append(candidate)
                        return
                raise ValueError(
                    "Verbindungsweg erreicht keinen sicheren Eintrittspunkt in der Zielzone"
                )
            if not start_inside and not departing_dock:
                source_zone: Optional[dict[str, Any]] = None
                for candidate in self._store.list_items("zones"):
                    if candidate.get("id") == zone.get("id") or candidate.get("type", "mowing") != "mowing":
                        continue
                    try:
                        candidate_polygon = self._zone_polygon_utm(
                            candidate, zone_number, zone_letter,
                        )
                    except ValueError:
                        continue
                    if candidate_polygon.buffer(0.05).covers(start_point):
                        source_zone = candidate
                        source_zone_polygon = candidate_polygon
                        break
                if source_zone is not None:
                    for connection in self._store.list_items("connections"):
                        if not connection.get("enabled", True) or connection.get("type") != "zone_link":
                            continue
                        forward = (
                            connection.get("from_zone_id") == source_zone.get("id")
                            and connection.get("to_zone_id") == zone.get("id")
                        )
                        reverse = (
                            connection.get("bidirectional", True)
                            and connection.get("from_zone_id") == zone.get("id")
                            and connection.get("to_zone_id") == source_zone.get("id")
                        )
                        if forward or reverse:
                            dock_connection = connection
                            connection_reverse = reverse
                            break
                if dock_connection is None:
                    raise ValueError(
                        "Der Roboter befindet sich außerhalb der ausgewählten Zone. "
                        "Bitte einen Verbindungsweg von der aktuellen Zone anlegen"
                    )
            if distance > self._waypoint_tolerance_m or heading_error > math.radians(5.0) or not start_inside:
                # Do not reject an otherwise safe start merely because the
                # conservative, orientation-independent lane-centre buffer
                # excludes its rear axle.  The transit may use the complete
                # field polygon; the exact swept tractor/deck footprint below
                # remains the authoritative safety constraint.
                allowed_boundary = safe_polygon if start_inside else turn_boundary
                if not start_inside:
                    if dock_connection is not None and dock_connection.get("type") == "zone_link":
                        for lon, lat, *_ in dock_connection.get("geometry", {}).get("coordinates", []):
                            guide_x, guide_y, guide_number, guide_letter = utm.from_latlon(
                                float(lat), float(lon),
                            )
                            if guide_number != zone_number or guide_letter != zone_letter:
                                raise ValueError("Stationsweg überschreitet eine UTM-Zonengrenze")
                            dock_guide.append((guide_x, guide_y))
                        if len(dock_guide) < 2:
                            raise ValueError("Verbindungsweg enthält zu wenige Punkte")
                        if connection_reverse:
                            dock_guide.reverse()
                        add_target_entry_staging(dock_guide)
                        corridor_half_width = max(
                            0.15,
                            min(5.0, float(dock_connection.get("corridor_width_cm", 120)) / 200.0),
                        )
                        connector_corridor = LineString(dock_guide).buffer(
                            corridor_half_width, cap_style="round", join_style="round",
                        )
                        departure_corridor = connector_corridor.union(source_zone_polygon)
                        allowed_boundary = turn_boundary.union(departure_corridor)
                    else:
                        home = self._store.get_home()
                        if not home or home.get("lat") is None or home.get("lon") is None:
                            raise ValueError("Ladestation ist nicht gespeichert")
                        home_x, home_y, home_zone_number, home_zone_letter = utm.from_latlon(
                            float(home["lat"]), float(home["lon"]),
                        )
                        if home_zone_number != zone_number or home_zone_letter != zone_letter:
                            raise ValueError("Ladestation und Mähzone liegen nicht in derselben UTM-Zone")
                        if math.hypot(start_pose.utm_x - home_x, start_pose.utm_y - home_y) > _DOCK_POSITION_TOLERANCE_M:
                            raise ValueError("Roboter ist nicht an der gespeicherten Ladestation")
                        dock_connection = next((
                            connection for connection in self._store.list_items("connections")
                            if connection.get("enabled", True)
                            and connection.get("type") == "dock_link"
                            and connection.get("to_zone_id") == zone.get("id")
                        ), None)
                        if dock_connection is not None:
                            for lon, lat, *_ in dock_connection.get("geometry", {}).get("coordinates", []):
                                guide_x, guide_y, guide_number, guide_letter = utm.from_latlon(
                                    float(lat), float(lon),
                                )
                                if guide_number != zone_number or guide_letter != zone_letter:
                                    raise ValueError("Stationsweg überschreitet eine UTM-Zonengrenze")
                                dock_guide.append((guide_x, guide_y))
                            if len(dock_guide) < 2:
                                raise ValueError("Stationsweg enthält zu wenige Punkte")
                            if math.dist(dock_guide[-1], (home_x, home_y)) < math.dist(dock_guide[0], (home_x, home_y)):
                                dock_guide.reverse()
                            if math.dist(dock_guide[0], (home_x, home_y)) > _DOCK_POSITION_TOLERANCE_M:
                                raise ValueError("Stationsweg beginnt nicht an der Ladestation")
                            add_target_entry_staging(dock_guide)
                            corridor_half_width = max(
                                0.15,
                                min(5.0, float(dock_connection.get("corridor_width_cm", 120)) / 200.0),
                            )
                            departure_corridor = LineString(
                                [(start_pose.utm_x, start_pose.utm_y), *dock_guide]
                            ).buffer(corridor_half_width, cap_style="round", join_style="round")
                            allowed_boundary = turn_boundary.union(departure_corridor)
                        else:
                            if polygon.distance(start_point) > _DOCK_DEPARTURE_MAX_OUTSIDE_M:
                                raise ValueError(
                                    "Ladestation liegt zu weit außerhalb der Zone. "
                                    "Bitte einen Stationsweg in der Arbeitskarte anlegen"
                                )
                            allowed_boundary = turn_boundary.union(
                                start_point.buffer(_DOCK_DEPARTURE_MAX_OUTSIDE_M)
                            )
                transit_route_validator: Optional[Callable[[Sequence[TractorWaypoint]], bool]] = None
                if start_inside or (dock_connection is not None and departure_corridor is not None):
                    transit_validation_zone = safe_polygon
                    if departure_corridor is not None:
                        transit_validation_zone = transit_validation_zone.union(
                            departure_corridor.buffer(_GEOFENCE_RTK_TOLERANCE_M)
                        )

                    def transit_route_validator(candidate: Sequence[TractorWaypoint]) -> bool:
                        return bool(validate_swept_machine_route(
                            candidate,
                            transit_validation_zone,
                            settings,
                            decks,
                            reserve_m=boundary_reserve,
                            no_go_zones=no_go_polygons,
                            maximum_sample_step_m=min(0.02, max(0.005, point_spacing * 2.0)),
                            geometry_tolerance_m=0.001,
                        )["valid"])
                if dock_connection is None:
                    transit_start = (
                        start_pose.utm_x,
                        start_pose.utm_y,
                        start_pose.heading_rad,
                    )
                    transit_target = (first.x, first.y, target_heading)
                    try:
                        if start_inside and heading_error > math.radians(120.0):
                            raise ValueError(
                                "Direkte Anfahrt zeigt vom Routeneinstieg weg"
                            )
                        transit = compute_curvature_bounded_connection(
                            transit_start,
                            transit_target,
                            turn_radius=turn_radius,
                            point_spacing=point_spacing,
                            boundary=allowed_boundary,
                            allow_reverse=True,
                            operation="turn",
                            turn_id=0,
                            route_validator=transit_route_validator,
                        )
                    except ValueError as direct_error:
                        if not start_inside:
                            raise
                        # A safe pose close to the boundary can face almost
                        # exactly away from the first route point. One direct
                        # bounded connection then has no room to reverse its
                        # heading although the field itself is large enough.
                        # Move first toward a safe interior staging pose and
                        # connect from there. This changes only the approach;
                        # the cached coverage route remains untouched.
                        interior = safe_polygon.representative_point()
                        inward_heading = math.atan2(
                            interior.y - start_pose.utm_y,
                            interior.x - start_pose.utm_x,
                        )
                        transit = []
                        inward_distance = math.hypot(
                            interior.x - start_pose.utm_x,
                            interior.y - start_pose.utm_y,
                        )
                        forward_x = math.cos(start_pose.heading_rad)
                        forward_y = math.sin(start_pose.heading_rad)
                        inward_dot = (
                            forward_x * (interior.x - start_pose.utm_x)
                            + forward_y * (interior.y - start_pose.utm_y)
                        )
                        staging_gear = 1 if inward_dot >= 0.0 else -1
                        for staging_distance in (0.25, 0.40, 0.60, 0.80, 1.0):
                            staging_x = (
                                start_pose.utm_x
                                + staging_gear * forward_x * staging_distance
                            )
                            staging_y = (
                                start_pose.utm_y
                                + staging_gear * forward_y * staging_distance
                            )
                            straight_steps = max(
                                1, math.ceil(staging_distance / point_spacing),
                            )
                            straight_leg = [TractorWaypoint(
                                start_pose.utm_x,
                                start_pose.utm_y,
                                "turn",
                                staging_gear,
                                0,
                            )]
                            for step in range(1, straight_steps + 1):
                                fraction = step / straight_steps
                                straight_leg.append(TractorWaypoint(
                                    start_pose.utm_x
                                    + (staging_x - start_pose.utm_x) * fraction,
                                    start_pose.utm_y
                                    + (staging_y - start_pose.utm_y) * fraction,
                                    "turn",
                                    staging_gear,
                                    0,
                                ))
                            for side in (1, -1):
                                for reverse_first in (True, False):
                                    reversal_start = (
                                        staging_x,
                                        staging_y,
                                        start_pose.heading_rad,
                                    )
                                    try:
                                        reversal = compute_compact_heading_reversal(
                                            reversal_start,
                                            turn_radius=turn_radius,
                                            point_spacing=point_spacing,
                                            lateral_offset=min(
                                                lane_spacing,
                                                1.8 * turn_radius,
                                            ),
                                            side=side,
                                            reverse_first=reverse_first,
                                            operation="turn",
                                            turn_id=0,
                                        )
                                        reversal_end = reversal[-1]
                                        entry_leg = compute_curvature_bounded_connection(
                                            (
                                                reversal_end.x,
                                                reversal_end.y,
                                                _normalize_angle(
                                                    start_pose.heading_rad + math.pi,
                                                ),
                                            ),
                                            transit_target,
                                            turn_radius=turn_radius,
                                            point_spacing=point_spacing,
                                            boundary=allowed_boundary,
                                            allow_reverse=True,
                                            allow_multi_point=False,
                                            operation="turn",
                                            turn_id=0,
                                        )
                                    except ValueError:
                                        continue
                                    candidate_transit = (
                                        straight_leg
                                        + reversal[1:]
                                        + entry_leg[1:]
                                    )
                                    if (
                                        transit_route_validator is None
                                        or transit_route_validator(candidate_transit)
                                    ):
                                        transit = candidate_transit
                                        staged_entry_used = True
                                        staged_entry_fraction = round(
                                            staging_distance
                                            / max(0.001, inward_distance),
                                            3,
                                        )
                                        break
                                if transit:
                                    break
                            if transit:
                                break
                        if not transit:
                            if str(direct_error).startswith("Direkte Anfahrt"):
                                raise ValueError(
                                    "Keine sichere Anfahrt über einen Innenpunkt gefunden"
                                ) from direct_error
                            raise direct_error
                else:
                    transit = []
                    current = (start_pose.utm_x, start_pose.utm_y, start_pose.heading_rad)
                    guide_targets = list(dock_guide)
                    if math.dist(guide_targets[0], current[:2]) <= self._waypoint_tolerance_m:
                        guide_targets.pop(0)
                    targets = [*guide_targets, (first.x, first.y)]
                    for index, target in enumerate(targets):
                        final_target = index == len(targets) - 1
                        next_target = targets[index + 1] if not final_target else None
                        target_segment_heading = (
                            target_heading if final_target else math.atan2(
                                next_target[1] - target[1], next_target[0] - target[0],
                            )
                        )
                        segment = compute_curvature_bounded_connection(
                            current,
                            (target[0], target[1], target_segment_heading),
                            turn_radius=turn_radius,
                            point_spacing=point_spacing,
                            boundary=allowed_boundary,
                            allow_reverse=True,
                            operation="turn",
                            turn_id=0,
                            route_validator=transit_route_validator,
                        )
                        if transit:
                            transit.extend(segment[1:])
                        else:
                            transit.extend(segment)
                        current = (target[0], target[1], target_segment_heading)
                transit_line = LineString([(point.x, point.y) for point in transit])
                if not start_inside:
                    if dock_connection is None:
                        outside_length = transit_line.difference(polygon.buffer(0.05)).length
                        if outside_length > _DOCK_DEPARTURE_MAX_OUTSIDE_M:
                            raise ValueError("Radiusbegrenzte Ausfahrt liegt zu lange außerhalb der Zone")
                        departure_corridor = transit_line.buffer(
                            _DOCK_DEPARTURE_CORRIDOR_HALF_WIDTH_M,
                            cap_style="round",
                            join_style="round",
                        )
                waypoints = transit + waypoints[1:]
                departure_waypoint_count = len(transit)
        if len(waypoints) < 2:
            raise ValueError("Für diese Zone konnte keine fahrbare Route berechnet werden")
        measured_turn_radius, radius_violation_index = measure_minimum_turn_radius(
            waypoints,
        )
        if measured_turn_radius < turn_radius * 0.995:
            raise ValueError(
                "Plan verworfen: Eine Kurve unterschreitet den eingestellten "
                f"Mindestwenderadius von {turn_radius * 100.0:.0f} cm "
                f"(gemessen: {measured_turn_radius * 100.0:.1f} cm, "
                f"Wegpunkt {radius_violation_index})"
            )
        validation_zone = safe_polygon
        if departure_corridor is not None:
            # Match the live RTK geofence tolerance at corridor/field seams.
            # Without this, floating-point polygon unions can reject a route
            # for sub-square-millimetre slivers although both adjacent safe
            # regions cover the complete physical machine.
            validation_zone = validation_zone.union(
                departure_corridor.buffer(_GEOFENCE_RTK_TOLERANCE_M)
            )
        plan_safety = validate_swept_machine_route(
            waypoints,
            validation_zone,
            settings,
            decks,
            reserve_m=0.0,
            no_go_zones=no_go_polygons,
            maximum_sample_step_m=min(0.02, max(0.005, point_spacing * 2.0)),
            geometry_tolerance_m=0.001,
        )
        if not plan_safety["valid"]:
            operation = plan_safety.get("operation") or "unbekannt"
            outside_cm2 = float(plan_safety.get("outside_area_m2", 0.0)) * 10_000.0
            violation_index = plan_safety.get("violation_index")
            raise ValueError(
                "Plan verworfen: Traktor oder Mähwerk überschreitet den sicheren "
                f"Geofence im Abschnitt {operation} "
                f"(Wegpunkt {violation_index}, Anfahrt bis {departure_waypoint_count}, "
                f"außerhalb: {outside_cm2:.1f} cm²)"
            )
        reserve_safety = validate_swept_machine_route(
            waypoints,
            validation_zone,
            settings,
            decks,
            reserve_m=boundary_reserve,
            no_go_zones=no_go_polygons,
            maximum_sample_step_m=min(0.02, max(0.005, point_spacing * 2.0)),
            geometry_tolerance_m=0.003,
        )
        plan_safety["tracking_reserve_ok"] = bool(reserve_safety["valid"])
        plan_safety["tracking_reserve_report"] = reserve_safety
        plan_safety["minimum_measured_turn_radius_cm"] = (
            None if not math.isfinite(measured_turn_radius)
            else round(measured_turn_radius * 100.0, 2)
        )
        # Fields2Cover can retry with an inward native planning geometry and
        # therefore treats the selected reserve as a hard acceptance gate.
        # MV2 retains its existing multi-point-turn fallback; its reserve
        # report remains visible until that legacy turn search can perform the
        # same geometry retry without rejecting otherwise usable small fields.
        if planner_engine == "fields2cover" and not reserve_safety["valid"]:
            operation = reserve_safety.get("operation") or "unbekannt"
            outside_cm2 = float(
                reserve_safety.get("outside_area_m2", 0.0)
            ) * 10_000.0
            violation_index = reserve_safety.get("violation_index")
            violation_x = reserve_safety.get("x")
            violation_y = reserve_safety.get("y")
            violation_turn_id = (
                waypoints[int(violation_index)].turn_id
                if violation_index is not None and int(violation_index) < len(waypoints)
                else None
            )
            raise ValueError(
                f"Plan verworfen: Die eingestellte Randreserve von "
                f"{configured_margin_cm:.0f} cm wird im Abschnitt {operation} "
                f"nicht eingehalten (Wegpunkt {violation_index}, "
                f"Wendung {violation_turn_id}, Position {violation_x}, {violation_y}, "
                f"Unterschreitungsfläche: {outside_cm2:.1f} cm²)"
            )
        route = [(point.x, point.y) for point in waypoints]
        # Coverage is assessed over the physically reachable target.  The
        # narrow outer safety strip is intentionally excluded; reporting it as
        # an uncovered planning hole would either reject every small field or
        # encourage a route with zero tracking margin at a wall.
        coverage_target = safe_polygon.buffer(
            -boundary_reserve, join_style="round",
        )
        if coverage_target.is_empty:
            raise ValueError("Die Zone ist zu klein für die Geofence-Fahrreserve")
        coverage_percent, uncovered_area = self._estimate_mowing_coverage(
            coverage_target, waypoints, decks,
        )
        mainland_clearance = max(
            curved_clearance + boundary_reserve,
            lane_endpoint_trim + lateral_clearance + turn_radius,
        )
        mainland_target = safe_polygon.buffer(
            -mainland_clearance, join_style="round",
        )
        if planner_engine == "fields2cover" and native_coverage_target is not None:
            # The native adapter now keeps every decomposed component. Validate
            # that complete target instead of assuming an outer pass may replace
            # missing interior swaths.
            mainland_target = native_coverage_target
        if mainland_target.is_empty:
            mainland_target = lane_geometry
        mainland_coverage_percent, _mainland_uncovered = self._estimate_mowing_coverage(
            mainland_target, waypoints, decks,
        )
        headland_safety_area = max(0.0, coverage_target.area - mainland_target.area)
        minimum_coverage_setting = (
            settings.get("fields2cover_minimum_mainland_coverage_percent", 90.0)
            if planner_engine == "fields2cover"
            else settings.get("minimum_plan_coverage_percent", 98.5)
        )
        minimum_coverage = max(
            80.0 if planner_engine == "fields2cover" else 95.0,
            min(100.0, float(minimum_coverage_setting)),
        )
        swath_metrics["minimum_mainland_coverage_percent"] = round(minimum_coverage, 2)
        swath_metrics["measured_mainland_coverage_percent"] = round(
            mainland_coverage_percent, 2,
        )
        # Coverage is estimated from buffered, discretised deck paths. Accept
        # a tenth of a percentage point numerical tolerance while continuing
        # to report the unrounded measured value.
        if mainland_coverage_percent + 0.10 < minimum_coverage:
            raise ValueError(
                f"Plan verworfen: nur {mainland_coverage_percent:.2f} % statt mindestens "
                f"{minimum_coverage:.2f} % sichere Innenfeld-Abdeckung; Feldzerlegung oder "
                "Mähparameter anpassen"
            )
        trajectory = ContinuousTrajectory(waypoints)
        metrics = self._measure_route(waypoints, uncovered_area)
        metrics["departure_waypoint_count"] = departure_waypoint_count
        metrics["staged_entry_used"] = staged_entry_used
        metrics["staged_entry_fraction"] = staged_entry_fraction
        metrics["plan_cache_hit"] = False
        metrics["plan_cache_key"] = cache_key[:12]
        coverage_grid = CoverageGrid(
            safe_polygon,
            resolution_m=max(0.01, min(0.10, float(settings.get("coverage_grid_resolution_cm", 2)) / 100.0)),
        )

        assert zone_number is not None and zone_letter is not None
        route_visual = self._build_visual_route(waypoints, zone_number, zone_letter)
        with self._lock:
            self._utm_zone_number = zone_number
            self._utm_zone_letter = zone_letter
            self._route_utm = route
            self._route_waypoints = waypoints
            self._route_wgs84 = [tuple(point) for point in route_visual["geometry"]["coordinates"]]
            self._route_visual = route_visual
            self._turn_count = max((point.turn_id or 0 for point in waypoints), default=0)
            self._turn_waypoint_count = sum(point.operation == "turn" for point in waypoints)
            self._route_index = 1
            self._trajectory = trajectory
            self._trajectory_s = 0.0
            self._route_mode = "coverage"
            self._planner_version = (
                "fields2cover_native_2x" if planner_engine == "fields2cover"
                else "coverage_hybrid_v3"
            )
            self._active_connection_id = dock_connection.get("id") if dock_connection else None
            self._route_strategy = (
                str(settings.get("fields2cover_path_type", "dubins"))
                if planner_engine == "fields2cover" else turn_strategy
            )
            self._coverage_strategy = coverage_strategy
            self._route_resolution_cm = resolution_cm
            self._headland_margin_cm = round(boundary_reserve * 100.0, 1)
            self._mowing_width_cm = round(mowing_width * 100.0, 1)
            self._lane_spacing_cm = round(lane_spacing * 100.0, 1)
            self._estimated_coverage_percent = coverage_percent
            self._estimated_uncovered_area_m2 = uncovered_area
            self._estimated_mainland_coverage_percent = mainland_coverage_percent
            self._headland_safety_area_m2 = round(headland_safety_area, 3)
            self._geofence_tracking_reserve_cm = round(
                boundary_reserve * 100.0, 1,
            )
            self._headland_passes = headland_passes
            self._mowing_turn_radius_cm = round(
                selected_mowing_turn_radius * 100.0, 1,
            )
            self._departure_corridor_utm = departure_corridor
            self._departure_zone_utm = polygon if departure_corridor is not None else None
            self._route_tolerance_m = min(
                self._waypoint_tolerance_m, max(0.06, point_spacing * 0.6),
            )
            self._last_gear = 1
            self._cusp_maneuver = None
            self._last_steering_deg = 0.0
            self._last_control_time = None
            self._mower_decks = decks
            self._coverage_grid = coverage_grid
            self._route_metrics = metrics
            self._route_metrics.update(swath_metrics)
            self._last_controller_output = {}
            self._swath_angle_deg = round(swath_angle_deg, 2)
            self._plan_safety = dict(plan_safety)
        cache_writer = getattr(self._store, "put_plan_cache", None)
        if callable(cache_writer):
            try:
                cache_writer(
                    cache_key,
                    self._serialize_plan_cache(cache_key, start_pose),
                )
            except (OSError, TypeError, ValueError):
                # A failed cache write must never prevent a valid mission.
                logger.warning(
                    "Validated mission plan could not be cached",
                    exc_info=True,
                )
        return self.route_geojson()

    @staticmethod
    def _estimate_mowing_coverage(
        target: Polygon,
        waypoints: list[TractorWaypoint],
        decks: tuple[Any, ...],
    ) -> tuple[float, float]:
        """Estimate coverage from each configured physical mower deck path."""
        swaths: list[Any] = []
        for deck in decks:
            active: list[tuple[float, float]] = []
            active_operation: str | None = None
            accumulated = 0.0
            previous: TractorWaypoint | None = None
            for index, point in enumerate(waypoints):
                if previous is not None:
                    accumulated += math.hypot(point.x - previous.x, point.y - previous.y)
                mowing = point.operation in {"mow", "headland"}
                state_changed = active_operation is not None and point.operation != active_operation
                take_sample = (
                    not active or accumulated >= 0.02 or state_changed
                    or index == len(waypoints) - 1
                )
                if mowing and take_sample:
                    heading = route_pose_heading(waypoints, index)
                    active.append((
                        point.x
                        + deck.forward_m * math.cos(heading)
                        - deck.left_m * math.sin(heading),
                        point.y
                        + deck.forward_m * math.sin(heading)
                        + deck.left_m * math.cos(heading),
                    ))
                    active_operation = point.operation
                    accumulated = 0.0
                if (not mowing or state_changed) and active:
                    if len(active) >= 2:
                        swaths.append(LineString(active).buffer(
                            deck.width_m / 2.0,
                            cap_style="square",
                            join_style="round",
                        ))
                    active = []
                    active_operation = point.operation if mowing else None
                previous = point
            if len(active) >= 2:
                swaths.append(LineString(active).buffer(
                    deck.width_m / 2.0,
                    cap_style="square",
                    join_style="round",
                ))
        if not swaths or target.area <= 0.0:
            return 0.0, round(target.area, 3)
        covered_area = unary_union(swaths).intersection(target).area
        uncovered_area = max(0.0, target.area - covered_area)
        return (
            round(min(100.0, covered_area / target.area * 100.0), 2),
            round(uncovered_area, 3),
        )

    @staticmethod
    def _measure_route(
        waypoints: list[TractorWaypoint], uncovered_area_m2: float,
    ) -> dict[str, float | int]:
        mowing_distance = 0.0
        non_mowing_distance = 0.0
        reverse_distance = 0.0
        longest_reverse_run = 0.0
        current_reverse_run = 0.0
        gear_changes = 0
        maximum_turn_rotation = 0.0
        current_turn_rotation = 0.0
        previous_turn_heading: float | None = None
        previous_turn_key: tuple[int | None, int] | None = None
        previous_gear = waypoints[0].gear if waypoints else 1
        for left, right in zip(waypoints, waypoints[1:]):
            distance = math.hypot(right.x - left.x, right.y - left.y)
            if right.operation in {"mow", "headland"}:
                mowing_distance += distance
            else:
                non_mowing_distance += distance
            if right.gear < 0:
                reverse_distance += distance
                current_reverse_run += distance
                longest_reverse_run = max(longest_reverse_run, current_reverse_run)
            else:
                current_reverse_run = 0.0
            if right.gear != previous_gear:
                gear_changes += 1
            previous_gear = right.gear
            if distance > 1e-9 and right.operation == "turn":
                heading = math.atan2(right.y - left.y, right.x - left.x)
                if right.gear < 0:
                    heading = _normalize_angle(heading + math.pi)
                turn_key = (right.turn_id, right.gear)
                if previous_turn_heading is not None and previous_turn_key == turn_key:
                    current_turn_rotation += abs(_normalize_angle(
                        heading - previous_turn_heading,
                    ))
                else:
                    current_turn_rotation = 0.0
                maximum_turn_rotation = max(
                    maximum_turn_rotation, current_turn_rotation,
                )
                previous_turn_heading = heading
                previous_turn_key = turn_key
            else:
                current_turn_rotation = 0.0
                previous_turn_heading = None
                previous_turn_key = None
        # The same transparent cost can later become the reward of a learned
        # high-level route optimiser; safety constraints remain deterministic.
        cost = (
            non_mowing_distance
            + 4.0 * reverse_distance
            + 0.75 * gear_changes
            + 8.0 * max(0.0, uncovered_area_m2)
        )
        return {
            "mowing_distance_m": round(mowing_distance, 2),
            "non_mowing_distance_m": round(non_mowing_distance, 2),
            "reverse_distance_m": round(reverse_distance, 2),
            "longest_reverse_run_m": round(longest_reverse_run, 2),
            "gear_changes": gear_changes,
            "maximum_turn_rotation_deg": round(
                math.degrees(maximum_turn_rotation), 1,
            ),
            "uncovered_area_m2": round(max(0.0, uncovered_area_m2), 3),
            "optimization_cost": round(cost, 2),
        }

    def start_mission(self, zone: dict[str, Any]) -> dict[str, Any]:
        if self._executive.state not in (MowerState.IDLE, MowerState.CHARGING):
            raise RuntimeError(f"Mission kann aus {self._executive.state.name} nicht starten")
        departing_dock = self._executive.state == MowerState.CHARGING
        with self._lock:
            # A new planning request starts a new visual/history session.  Do
            # this before the potentially long native planning call so every
            # client immediately loses the previous planned/actual lines,
            # even when the new plan later fails validation.
            self._actual_track_utm = []
            self._actual_mown_segments_utm = []
            self._active_mown_segment = None
            self._coverage_grid = None
            self._route_utm = []
            self._route_waypoints = []
            self._route_wgs84 = []
            self._trajectory = None
            self._trajectory_s = 0.0
            self._route_index = 0
            self._route_visual = {
                "geometry": {"type": "LineString", "coordinates": []},
                "turn_geometry": {"type": "MultiLineString", "coordinates": []},
                "mow_geometry": {"type": "MultiLineString", "coordinates": []},
                "headland_geometry": {"type": "MultiLineString", "coordinates": []},
            }
        route = self.plan_zone(zone)
        self._executive.start_mission(zone.get("id"))
        if departing_dock and self._dock_departure_callback is not None:
            self._dock_departure_callback()
        return route

    def return_home(self) -> dict[str, Any]:
        """Build and execute a radius-bounded RTK route to the home position."""
        if self._executive.state not in (
            MowerState.MOWING,
            MowerState.PAUSED,
            MowerState.OBSTACLE_AVOIDANCE,
        ):
            raise RuntimeError("Keine aktive Mission kann zur Ladestation zurückkehren")
        home = self._store.get_home()
        if not home or home.get("lat") is None or home.get("lon") is None:
            raise ValueError("Bitte zuerst die Ladestation auf der Karte speichern")
        with self._lock:
            pose = self._latest_pose
        if pose is None:
            raise RuntimeError("Noch keine aktuelle Roboterposition verfügbar")

        home_lat, home_lon = float(home["lat"]), float(home["lon"])
        home_x, home_y, zone_number, zone_letter = utm.from_latlon(home_lat, home_lon)
        direct_heading = math.atan2(home_y - pose.utm_y, home_x - pose.utm_x)
        target_heading = (
            math.radians(90.0 - float(home["heading_deg"]))
            if home.get("heading_deg") is not None else direct_heading
        )
        settings = self._store.settings()
        turn_radius = max(
            0.10,
            min(1.0, float(settings.get("turn_radius_cm", 25)) / 100.0),
        )
        point_spacing = max(
            0.002,
            min(0.009, float(settings.get("route_resolution_cm", 0.5)) / 100.0),
        )
        boundary = None
        zone = (
            self._store.get_item("zones", self._executive.active_zone_id)
            if self._executive.active_zone_id else None
        )
        dock_connection = next((
            connection for connection in self._store.list_items("connections")
            if connection.get("enabled", True)
            and connection.get("type") == "dock_link"
            and connection.get("to_zone_id") == self._executive.active_zone_id
        ), None)
        metric_guide: list[tuple[float, float]] = []
        if zone and zone.get("geometry", {}).get("type") == "Polygon":
            metric_zone = []
            for lon, lat, *_ in zone["geometry"]["coordinates"][0]:
                x, y, number, letter = utm.from_latlon(float(lat), float(lon))
                if number != zone_number or letter != zone_letter:
                    raise ValueError("Ladestation und Mähzone liegen nicht in derselben UTM-Zone")
                metric_zone.append((x, y))
            boundary = Polygon(metric_zone).union(
                Point(home_x, home_y).buffer(_DOCK_DEPARTURE_MAX_OUTSIDE_M)
            )
        if dock_connection is not None:
            if not dock_connection.get("bidirectional", True):
                raise ValueError("Der Stationsweg ist nicht für die Rückfahrt freigegeben")
            for lon, lat, *_ in dock_connection.get("geometry", {}).get("coordinates", []):
                guide_x, guide_y, number, letter = utm.from_latlon(float(lat), float(lon))
                if number != zone_number or letter != zone_letter:
                    raise ValueError("Stationsweg überschreitet eine UTM-Zonengrenze")
                metric_guide.append((guide_x, guide_y))
            if len(metric_guide) < 2:
                raise ValueError("Stationsweg enthält zu wenige Punkte")
            if math.dist(metric_guide[-1], (home_x, home_y)) < math.dist(metric_guide[0], (home_x, home_y)):
                metric_guide.reverse()
            corridor_half_width = max(
                0.15,
                min(5.0, float(dock_connection.get("corridor_width_cm", 120)) / 200.0),
            )
            corridor = LineString(metric_guide).buffer(
                corridor_half_width, cap_style="round", join_style="round",
            )
            boundary = corridor if boundary is None else boundary.union(corridor)
            targets = list(reversed(metric_guide))
            if math.dist(targets[-1], (home_x, home_y)) > self._waypoint_tolerance_m:
                targets.append((home_x, home_y))
            waypoints = []
            current = (pose.utm_x, pose.utm_y, pose.heading_rad)
            for index, target in enumerate(targets):
                final_target = index == len(targets) - 1
                next_target = targets[index + 1] if not final_target else None
                segment_heading = target_heading if final_target else math.atan2(
                    next_target[1] - target[1], next_target[0] - target[0],
                )
                segment = compute_curvature_bounded_connection(
                    current,
                    (target[0], target[1], segment_heading),
                    turn_radius=turn_radius,
                    point_spacing=point_spacing,
                    boundary=boundary,
                    allow_reverse=False,
                    operation="return",
                    turn_id=None,
                )
                waypoints.extend(segment[1:] if waypoints else segment)
                current = (target[0], target[1], segment_heading)
        else:
            waypoints = compute_curvature_bounded_connection(
                (pose.utm_x, pose.utm_y, pose.heading_rad),
                (home_x, home_y, target_heading),
                turn_radius=turn_radius,
                point_spacing=point_spacing,
                boundary=boundary,
                allow_reverse=False,
                operation="return",
                turn_id=None,
            )
        route_visual = self._build_visual_route(waypoints, zone_number, zone_letter)
        with self._lock:
            if (
                self._utm_zone_number is not None
                and self._utm_zone_letter is not None
                and (zone_number != self._utm_zone_number or zone_letter != self._utm_zone_letter)
            ):
                raise ValueError("Ladestation und Roboter liegen nicht in derselben UTM-Zone")
            self._utm_zone_number = zone_number
            self._utm_zone_letter = zone_letter
            self._route_utm = [(point.x, point.y) for point in waypoints]
            self._route_waypoints = waypoints
            self._route_wgs84 = [tuple(point) for point in route_visual["geometry"]["coordinates"]]
            self._route_visual = route_visual
            self._turn_count = 0
            self._turn_waypoint_count = sum(point.operation == "turn" for point in waypoints)
            self._route_index = 1
            self._trajectory = ContinuousTrajectory(waypoints)
            self._trajectory_s = 0.0
            self._route_mode = "return_home"
            self._planner_version = "return_home"
            self._active_connection_id = dock_connection.get("id") if dock_connection else None
            self._last_gear = 1
            self._cusp_maneuver = None
            self._last_steering_deg = 0.0
            self._last_control_time = None
            self._route_metrics = self._measure_route(waypoints, 0.0)
            self._last_controller_output = {}

        self._stop_outputs()
        self._executive.return_to_dock()
        return self.route_geojson()

    @staticmethod
    def _simplify_metric_line(
        points: list[tuple[float, float]], *, minimum_points: int = 3,
    ) -> list[tuple[float, float]]:
        if len(points) < minimum_points:
            return list(points)
        simplified = LineString(points).simplify(
            _VISUAL_PATH_TOLERANCE_M, preserve_topology=False,
        )
        coordinates = list(simplified.coords)
        return coordinates if len(coordinates) >= 2 else [points[0], points[-1]]

    @classmethod
    def _metric_line_wgs84(
        cls,
        points: list[tuple[float, float]],
        zone_number: int,
        zone_letter: str,
        *,
        minimum_points: int = 3,
    ) -> list[list[float]]:
        simplified = cls._simplify_metric_line(points, minimum_points=minimum_points)
        result: list[list[float]] = []
        for x, y in simplified:
            lat, lon = utm.to_latlon(x, y, zone_number, zone_letter)
            result.append([lon, lat])
        return result

    @classmethod
    def _build_visual_route(
        cls,
        waypoints: list[TractorWaypoint],
        zone_number: int,
        zone_letter: str,
    ) -> dict[str, Any]:
        metric = [(point.x, point.y) for point in waypoints]
        turn_lines: list[list[tuple[float, float]]] = []
        mow_lines: list[list[tuple[float, float]]] = []
        headland_lines: list[list[tuple[float, float]]] = []
        active_turn_id: int | None = None
        active_mow: list[tuple[float, float]] = []
        for index, point in enumerate(waypoints):
            if point.operation != "turn":
                if not active_mow and index > 0:
                    active_mow.append(metric[index - 1])
                active_mow.append(metric[index])
            elif active_mow:
                if len(active_mow) >= 2:
                    mow_lines.append(active_mow)
                active_mow = []
            if point.turn_id is None:
                active_turn_id = None
                continue
            if point.turn_id != active_turn_id:
                active_turn_id = point.turn_id
                turn_lines.append([metric[max(0, index - 1)]])
            turn_lines[-1].append(metric[index])
        if len(active_mow) >= 2:
            mow_lines.append(active_mow)

        active_headland: list[tuple[float, float]] = []
        for index, point in enumerate(waypoints):
            if point.operation == "headland":
                if not active_headland and index > 0:
                    active_headland.append(metric[index - 1])
                active_headland.append(metric[index])
            elif active_headland:
                if len(active_headland) >= 2:
                    headland_lines.append(active_headland)
                active_headland = []
        if len(active_headland) >= 2:
            headland_lines.append(active_headland)

        convert = lambda line: cls._metric_line_wgs84(
            line, zone_number, zone_letter,
        )
        return {
            "geometry": {"type": "LineString", "coordinates": convert(metric)},
            "turn_geometry": {
                "type": "MultiLineString",
                "coordinates": [convert(line) for line in turn_lines if len(line) >= 2],
            },
            "mow_geometry": {
                "type": "MultiLineString",
                "coordinates": [convert(line) for line in mow_lines if len(line) >= 2],
            },
            "headland_geometry": {
                "type": "MultiLineString",
                "coordinates": [convert(line) for line in headland_lines if len(line) >= 2],
            },
        }

    def route_geojson(self) -> dict[str, Any]:
        turn_radius_cm = float(self._store.settings().get("turn_radius_cm", 25))
        with self._lock:
            zone_number = self._utm_zone_number
            zone_letter = self._utm_zone_letter
            coverage_grid = self._coverage_grid
            actual_track_utm = list(self._actual_track_utm)
            actual_mown_utm = [
                list(segment) for segment in self._actual_mown_segments_utm
                if len(segment) >= 2
            ]
            visual = self._route_visual
            waypoint_count = len(self._route_waypoints)
            current_operation = (
                self._route_waypoints[self._route_index].operation
                if 0 <= self._route_index < len(self._route_waypoints)
                else None
            )
            deck_geometry = [
                {
                    "name": deck.name,
                    "group": deck.group,
                    "forward_m": deck.forward_m,
                    "left_m": deck.left_m,
                    "width_m": deck.width_m,
                    "depth_m": deck.depth_m,
                }
                for deck in self._mower_decks
            ]
            properties = {
                "waypoint_count": waypoint_count,
                "current_waypoint": self._route_index,
                "mode": self._route_mode,
                "planner_version": self._planner_version,
                "active_connection_id": self._active_connection_id,
                "turn_strategy": self._route_strategy,
                "coverage_strategy": self._coverage_strategy,
                "headland_passes": self._headland_passes,
                "headland_width_cm": round(
                    self._mowing_width_cm
                    + max(0, self._headland_passes - 1) * self._lane_spacing_cm
                    if self._headland_passes else 0.0,
                    1,
                ),
                "mowing_width_cm": self._mowing_width_cm,
                "lane_spacing_cm": self._lane_spacing_cm,
                "overlap_cm": round(
                    max(0.0, self._mowing_width_cm - self._lane_spacing_cm), 1,
                ),
                "estimated_coverage_percent": self._estimated_coverage_percent,
                "estimated_uncovered_area_m2": self._estimated_uncovered_area_m2,
                "estimated_mainland_coverage_percent": self._estimated_mainland_coverage_percent,
                "headland_safety_area_m2": self._headland_safety_area_m2,
                "geofence_tracking_reserve_cm": self._geofence_tracking_reserve_cm,
                "resolution_cm": self._route_resolution_cm,
                "minimum_turn_radius_cm": turn_radius_cm,
                "minimum_mowing_turn_radius_cm": self._mowing_turn_radius_cm,
                "headland_margin_cm": self._headland_margin_cm,
                "turn_count": self._turn_count,
                "turn_waypoint_count": self._turn_waypoint_count,
                "current_operation": current_operation,
                "front_deck_raised": self._deck_lift_state[0],
                "rear_deck_raised": self._deck_lift_state[1],
                "route_metrics": dict(self._route_metrics),
                "swath_angle_deg": self._swath_angle_deg,
                "plan_safety": dict(self._plan_safety),
                "controller": "kinematic_mpc",
                "controller_output": dict(self._last_controller_output),
                "deck_geometry": deck_geometry,
            }

        actual_track: list[list[float]] = []
        actual_mown_lines: list[list[list[float]]] = []
        actual_coverage = {"type": "FeatureCollection", "features": []}
        actual_coverage_percent = 0.0
        if zone_number is not None and zone_letter is not None:
            if actual_track_utm:
                actual_track = self._metric_line_wgs84(
                    actual_track_utm, zone_number, zone_letter, minimum_points=200,
                )
            actual_mown_lines = [
                self._metric_line_wgs84(
                    segment, zone_number, zone_letter, minimum_points=200,
                )
                for segment in actual_mown_utm
            ]
            if coverage_grid is not None:
                def to_wgs84(x: float, y: float) -> tuple[float, float]:
                    lat, lon = utm.to_latlon(x, y, zone_number, zone_letter)
                    return lon, lat

                actual_coverage = coverage_grid.geojson(to_wgs84)
                actual_coverage_percent = coverage_grid.coverage_percent()
        properties["actual_coverage_percent"] = actual_coverage_percent
        return {
            "type": "Feature",
            "properties": properties,
            **visual,
            "actual_geometry": {
                "type": "LineString",
                "coordinates": actual_track,
            },
            "actual_mown_geometry": {
                "type": "MultiLineString",
                "coordinates": actual_mown_lines,
            },
            "actual_coverage": actual_coverage,
        }

    def _teach_geometry_locked(self, points: list[tuple[float, float]]) -> dict[str, Any]:
        ring = list(points)
        if ring and ring[0] != ring[-1]:
            ring.append(ring[0])
        polygon = _valid_coverage_polygon(Polygon(ring))
        coordinates = self._points_wgs84_locked(list(polygon.exterior.coords))
        return {"type": "Polygon", "coordinates": [coordinates]}

    def _points_wgs84_locked(self, points: list[tuple[float, float]]) -> list[list[float]]:
        if self._utm_zone_number is None or self._utm_zone_letter is None:
            raise RuntimeError("UTM-Zone ist noch nicht aus einem GPS-Fix bekannt")
        coordinates: list[list[float]] = []
        for x, y in points:
            lat, lon = utm.to_latlon(
                x, y, self._utm_zone_number, self._utm_zone_letter,
            )
            coordinates.append([lon, lat])
        return coordinates

    def _loop(self) -> None:
        while self._running:
            started = time.monotonic()
            try:
                self._control_step()
            except Exception:
                logger.exception("Mission control step failed")
                self._executive.emergency_stop("Fehler in der autonomen Fahrregelung")
            remaining = self._period_s / self._time_scale() - (time.monotonic() - started)
            if remaining > 0:
                time.sleep(remaining)

    def _time_scale(self) -> float:
        try:
            return max(1.0, min(5.0, float(self._time_scale_provider())))
        except (TypeError, ValueError):
            return 1.0

    def _control_step(self) -> None:
        state = self._executive.state
        if state not in (MowerState.MOWING, MowerState.RETURNING):
            if self._commands_active:
                self._stop_outputs()
            return
        with self._lock:
            pose = self._latest_pose
            route = list(self._route_utm)
            waypoints = list(self._route_waypoints)
            trajectory = self._trajectory
            trajectory_s = self._trajectory_s
            route_index_hint = self._route_index
        if pose is None or len(route) < 2 or trajectory is None:
            self._stop_outputs()
            return

        # Keep route-index compatibility for diagnostics/tests and for a route
        # restored at a known waypoint after a process restart.
        trajectory_s = max(
            trajectory_s,
            trajectory.arc_length_at_waypoint(max(0, route_index_hint - 1)),
        )
        distance_to_end = math.hypot(
            route[-1][0] - pose.utm_x, route[-1][1] - pose.utm_y,
        )
        maneuver = self._cusp_maneuver if state == MowerState.MOWING else None
        if maneuver is not None:
            # --- Distance-bounded cusp maneuver execution -------------------
            # A short post-cusp segment is driven by covered distance from the
            # cusp point, not by trajectory projection: the projection cannot
            # converge on segments that span only a handful of pose updates.
            step_x = pose.utm_x - maneuver["last_x"]
            step_y = pose.utm_y - maneuver["last_y"]
            # Signed along-track projection of the pose delta.  Accumulating
            # |delta| rectifies GPS noise into forward progress, so the
            # reference ran ahead of the machine and every maneuver leg
            # finished early — the driven turns came out wide and mushy
            # instead of matching the planned tight point turns.  Projected
            # onto the motion direction the noise cancels instead.
            step_heading = trajectory.sample(min(
                maneuver["end_s"] - 1e-4,
                maneuver["start_s"] + maneuver["travelled"],
            )).heading_rad
            step = max(
                0.0,
                math.cos(step_heading) * step_x + math.sin(step_heading) * step_y,
            )
            maneuver["travelled"] += step
            maneuver["last_x"] = pose.utm_x
            maneuver["last_y"] = pose.utm_y
            target_distance = maneuver["end_s"] - maneuver["start_s"]
            reference_s = min(
                maneuver["end_s"] - 1e-4,
                maneuver["start_s"] + maneuver["travelled"],
            )
            reference = trajectory.sample(reference_s)
            deviation = math.hypot(
                pose.utm_x - reference.x, pose.utm_y - reference.y,
            )
            _cusp_debug_path = os.environ.get("MV2_CUSP_DEBUG")
            if _cusp_debug_path:
                with open(_cusp_debug_path, "a") as _fh:
                    _fh.write(
                        f"travelled={maneuver['travelled']:.3f}/"
                        f"{maneuver['end_s'] - maneuver['start_s']:.3f} "
                        f"s={maneuver['start_s']:.2f}->{maneuver['end_s']:.2f} "
                        f"dev={deviation:.3f} "
                        f"pose=({pose.utm_x:.2f},{pose.utm_y:.2f},"
                        f"h{math.degrees(pose.heading_rad):.0f}) "
                        f"ref=({reference.x:.2f},{reference.y:.2f},"
                        f"h{math.degrees(reference.heading_rad):.0f},"
                        f"{reference.operation}) "
                        f"gear={self._last_gear:+d} "
                        f"steer={self._last_steering_deg:.1f}\n"
                    )
            if deviation > _CUSP_ABORT_DEVIATION_M:
                self._cusp_maneuver = None
                self._stop_outputs()
                logger.error(
                    "Cusp maneuver aborted: %.0f cm deviation from reference",
                    deviation * 100.0,
                )
                self._executive.pause_mission(
                    f"Wendemanöver abgebrochen: {deviation * 100.0:.0f} cm "
                    "Spurabweichung — Position prüfen und fortsetzen"
                )
                return
            if maneuver["travelled"] >= target_distance - 0.02:
                # Maneuver complete: stop, latch progress at the boundary and
                # let the next tick take a fresh decision (possibly another
                # cusp handover immediately after, e.g. reverse → forward).
                self._cusp_maneuver = None
                with self._lock:
                    self._trajectory_s = maneuver["end_s"] - 1e-4
                    self._route_index = min(
                        len(route) - 1,
                        trajectory.sample(maneuver["end_s"] - 1e-4).segment_index + 1,
                    )
                self._stop_outputs()
                return
            with self._lock:
                self._route_index = min(len(route) - 1, reference.segment_index + 1)
                self._trajectory_s = reference_s
            # A scripted approach leg may still be mowing (operation 'mow' or
            # 'headland' right up to the cusp) — keep the decks down and the
            # blade running there; only actual turn segments lift/stop.
            turning_reference = reference.operation == "turn"
            if not self._prepare_decks(turning_reference):
                return
            # Scripted maneuver control: curvature feedforward + heading hold.
            # The MPC's preview degenerates on segments spanning centimetres
            # and injected large steering into straight reverse shunts
            # (observed: 16° heading drift over a 25 cm shunt, then loops on
            # the following lane).  Segment headings always point along the
            # direction of motion.  Path curvature is κ = sign(v)·tan(δ)/L,
            # so in reverse the steering must be NEGATED — the same convention
            # the MPC path applies below (-output.steering_deg for gear < 0).
            # Without the negation a curved reverse arc amplifies its own
            # heading error (observed: deterministic 36 cm abort on the 67 cm
            # reverse arc while all straight shunts passed).
            settings = self._store.settings()
            time_scale = self._time_scale()
            wheelbase_m = max(
                0.10,
                min(1.0, float(settings.get("vehicle_wheelbase_cm", 25)) / 100.0),
            )
            gear = self._last_gear
            motion_heading = pose.heading_rad + (math.pi if gear < 0 else 0.0)
            heading_error = _normalize_angle(reference.heading_rad - motion_heading)
            # Curvature strictly from inside the maneuver's own gear block:
            # the trajectory's smoothed curvature spans ±3 segments and is
            # polluted by the π heading jump at the cusp itself.
            probe_ds = 0.05
            probe_s = min(
                max(maneuver["start_s"] + 1e-3, reference_s),
                maneuver["end_s"] - 1e-3 - probe_ds,
            )
            local_curvature = 0.0
            if probe_s > maneuver["start_s"]:
                h0 = trajectory.sample(probe_s).heading_rad
                h1 = trajectory.sample(probe_s + probe_ds).heading_rad
                local_curvature = _normalize_angle(h1 - h0) / probe_ds
            feedforward_deg = math.degrees(math.atan(wheelbase_m * local_curvature))
            # Cross-track correction (Stanley form, same sign convention as
            # mower.control.stanley: positive = vehicle right of the path,
            # positive steering = left).  Softening constant 0.3 m/s keeps the
            # term gentle at maneuver speeds.
            offset_x = pose.utm_x - reference.x
            offset_y = pose.utm_y - reference.y
            cross_track_right = (
                math.sin(reference.heading_rad) * offset_x
                - math.cos(reference.heading_rad) * offset_y
            )
            cross_track_deg = math.degrees(math.atan2(cross_track_right, 0.3))
            steering_sign = 1.0 if gear > 0 else -1.0
            desired_steering = max(-45.0, min(45.0, steering_sign * (
                feedforward_deg
                + math.degrees(heading_error) * 0.8
                + cross_track_deg
            )))
            # Divide the maneuver speed by the acceleration factor: GPS runs
            # at wall-clock rate, so at 5× a fix arrives only every ~19 cm of
            # travel — a curved 67 cm reverse arc was driven nearly open-loop
            # and hit the deviation watchdog.  Scaling the ground speed keeps
            # the travel distance per fix constant across time scales (on the
            # real machine time_scale is 1 and nothing changes).
            requested_mps = max(
                0.05,
                float(settings.get("turn_speed_kmh", 0.4)) / 3.6 / time_scale,
            )
            normalized_speed = min(1.0, requested_mps / self._max_speed_mps)
            self._publish_motion_context(requested_mps * 3.6, "turn")
            now = time.monotonic()
            dt = (
                self._period_s
                if self._last_control_time is None
                else max(0.01, min(0.25, now - self._last_control_time))
            )
            maximum_change = _STEERING_RATE_LIMIT_DEG_S * dt * time_scale
            steering = max(
                self._last_steering_deg - maximum_change,
                min(self._last_steering_deg + maximum_change, desired_steering),
            )
            self._last_steering_deg = steering
            self._last_control_time = now
            self._hardware.set_blade(not turning_reference)
            self._hardware.drive(normalized_speed * gear, steering)
            self._commands_active = True
            return
        else:
            if state == MowerState.RETURNING and distance_to_end <= self._waypoint_tolerance_m:
                # Dock arrival is an absolute position condition.  Allow it to
                # complete immediately even after a GPS correction or test pose
                # jump; the forward-progress limiter below is meant for ambiguous
                # coverage hairpins, not for the final home point.
                projected_s = trajectory.length_m
            else:
                projection_speed_mps = abs(pose.speed_mps) / self._time_scale()
                deck_transition_active = time.monotonic() < self._deck_transition_until
                current_reference = trajectory.sample(trajectory_s)
                along_track_catchup = max(
                    0.0,
                    math.cos(current_reference.heading_rad)
                    * (pose.utm_x - current_reference.x)
                    + math.sin(current_reference.heading_rad)
                    * (pose.utm_y - current_reference.y),
                )
                maximum_projection_advance = (
                    0.0
                    if deck_transition_active
                    else min(
                        0.75,
                        max(
                            0.02,
                            projection_speed_mps * self._period_s * 1.8,
                        )
                        + along_track_catchup,
                    )
                )
                projection = trajectory.project(
                    pose.utm_x, pose.utm_y, previous_s=trajectory_s,
                    window_m=max(2.5, projection_speed_mps * 8.0),
                    heading_rad=pose.heading_rad,
                    gear=self._last_gear,
                    max_forward_m=maximum_projection_advance,
                )
                projected_s = projection.sample.s
            # Route progress must be monotonic.  At a reversing cusp, the incoming
            # and outgoing segments occupy almost the same coordinates.  Allowing
            # the projection to move five centimetres backwards made the selected
            # gear and operation alternate on every control tick, so the tractor
            # shuttled back and forth while raising/lowering its decks.  Keep the
            # greatest confirmed arc length and derive the compatibility index
            # from that latched progress rather than from the raw projection.
            raw_projected_s = projected_s
            cusp_s = trajectory.next_gear_change_s(trajectory_s)
            if state == MowerState.MOWING and cusp_s < trajectory.length_m - 1e-6:
                # Progress never crosses a gear boundary on its own — the
                # handover below is the only way past a cusp.  Without this
                # clamp the along-track catch-up could skip a short reverse
                # shunt entirely, leaving the follower driving forward against
                # a reversed reference (observed as a full-lock loop).
                projected_s = min(projected_s, cusp_s - 1e-4)
            trajectory_s = max(trajectory_s, projected_s)
            progress_sample = trajectory.sample(trajectory_s)
            route_index = min(len(route) - 1, progress_sample.segment_index + 1)
            with self._lock:
                self._route_index = route_index
                self._trajectory_s = trajectory_s
            if trajectory.length_m - trajectory_s <= self._route_tolerance_m and distance_to_end <= self._waypoint_tolerance_m:
                self._stop_outputs()
                if state == MowerState.RETURNING:
                    self._executive.on_dock_success()
                    if self._home_arrival_callback is not None:
                        self._home_arrival_callback()
                else:
                    try:
                        self.return_home()
                    except (ValueError, RuntimeError):
                        logger.warning("Mission complete, but no return-home route is available")
                        self._executive.stop_mission()
                return
            if state == MowerState.MOWING and cusp_s < trajectory.length_m - 1e-6:
                cusp_sample = trajectory.sample(cusp_s)
                distance_to_cusp = math.hypot(
                    pose.utm_x - cusp_sample.x, pose.utm_y - cusp_sample.y,
                )
                if cusp_s - trajectory_s <= _CUSP_HANDOVER_WINDOW_M and (
                    distance_to_cusp <= _CUSP_HANDOVER_DISTANCE_M
                    or raw_projected_s >= cusp_s - 1e-3
                ):
                    # Explicit cusp handover: stop, switch gear, and execute a
                    # short post-cusp segment as a distance-bounded maneuver.
                    next_boundary = trajectory.next_gear_change_s(cusp_s + 1e-3)
                    self._publish_motion_context(0.0, "gear_change")
                    self._hardware.drive(0.0, 0.0)
                    self._hardware.set_blade(False)
                    self._commands_active = False
                    self._last_gear = cusp_sample.gear
                    self._last_steering_deg = 0.0
                    self._last_control_time = None
                    with self._lock:
                        self._trajectory_s = cusp_s
                        self._route_index = min(
                            len(route) - 1, cusp_sample.segment_index + 1,
                        )
                    if next_boundary - cusp_s <= _CUSP_MANEUVER_MAX_LENGTH_M:
                        self._cusp_maneuver = {
                            "start_s": cusp_s,
                            "end_s": next_boundary,
                            "travelled": 0.0,
                            "last_x": pose.utm_x,
                            "last_y": pose.utm_y,
                        }
                    return
                if (
                    _CUSP_HANDOVER_WINDOW_M
                    < cusp_s - trajectory_s
                    <= _CUSP_SCRIPT_APPROACH_M
                ):
                    # Script the final approach INTO the cusp as well: the
                    # MPC's preview is clamped at the gear boundary, so its
                    # lookahead collapses on this leg and steering fades out —
                    # the tractor sailed straight past the reversal point.
                    # Same gear, no stop tick; the maneuver branch takes over
                    # on the next control tick.
                    self._cusp_maneuver = {
                        "start_s": trajectory_s,
                        "end_s": cusp_s,
                        "travelled": 0.0,
                        "last_x": pose.utm_x,
                        "last_y": pose.utm_y,
                    }

        waypoint = waypoints[route_index] if route_index < len(waypoints) else TractorWaypoint(*route[route_index])
        turning = state == MowerState.RETURNING or waypoint.operation == "turn"
        # The outer headland is mown with the decks down, but its proximity to
        # the geofence requires the same conservative speed as a turn.  At the
        # normal mowing speed even a small tracking lag can move the machine's
        # widest point outside the polygon on a curved boundary.
        precision_driving = turning or waypoint.operation == "headland"
        if not self._prepare_decks(turning):
            return

        gear = 1 if state == MowerState.RETURNING else waypoint.gear
        if gear != self._last_gear:
            self._publish_motion_context(0.0, "gear_change")
            self._hardware.drive(0.0, 0.0)
            self._hardware.set_blade(False)
            self._commands_active = False
            self._last_gear = gear
            self._last_steering_deg = 0.0
            self._last_control_time = None
            return

        control_pose = pose
        if gear < 0:
            control_pose = Pose(
                pose.utm_x,
                pose.utm_y,
                (pose.heading_rad + math.pi) % (2.0 * math.pi),
                abs(pose.speed_mps),
                pose.yaw_rate_rps,
                pose.timestamp,
                pose.fix_quality,
            )
        settings = self._store.settings()
        time_scale = self._time_scale()
        speed_key = "turn_speed_kmh" if precision_driving else "speed_kmh"
        default_speed = 0.4 if precision_driving else 0.9
        requested_mps = max(0.05, float(settings.get(speed_key, default_speed)) / 3.6)
        normalized_speed = min(1.0, requested_mps / self._max_speed_mps)
        speed_mode = (
            "returning"
            if state == MowerState.RETURNING
            else waypoint.operation
        )
        self._publish_motion_context(requested_mps * 3.6, speed_mode)
        wheelbase_m = max(
            0.10,
            min(1.0, float(settings.get("vehicle_wheelbase_cm", 25)) / 100.0),
        )
        previous_control_steering = -self._last_steering_deg if gear < 0 else self._last_steering_deg
        output = self._controller.compute(
            control_pose,
            trajectory,
            trajectory_s,
            speed_mps=max(requested_mps, abs(pose.speed_mps) / time_scale),
            wheelbase_m=wheelbase_m,
            previous_steering_deg=previous_control_steering,
        )
        self._hardware.set_blade(state == MowerState.MOWING and not turning)
        desired_steering = -output.steering_deg if gear < 0 else output.steering_deg
        now = time.monotonic()
        dt = (
            self._period_s
            if self._last_control_time is None
            else max(0.01, min(0.25, now - self._last_control_time))
        )
        maximum_change = _STEERING_RATE_LIMIT_DEG_S * dt * time_scale
        steering = max(
            self._last_steering_deg - maximum_change,
            min(self._last_steering_deg + maximum_change, desired_steering),
        )
        self._last_steering_deg = steering
        self._last_control_time = now
        with self._lock:
            self._last_controller_output = {
                "steering_command_deg": round(steering, 2),
                "cross_track_error_cm": round(output.cross_track_error_m * 100.0, 2),
                "heading_error_deg": round(math.degrees(output.heading_error_rad), 2),
                "feedforward_deg": round(output.feedforward_deg, 2),
                "predicted_cost": round(output.predicted_cost, 3),
                "trajectory_progress_m": round(trajectory_s, 2),
            }
        self._hardware.drive(normalized_speed * gear, steering)
        self._commands_active = True

    @staticmethod
    def _tracking_geometry(
        route: list[tuple[float, float]],
        waypoints: list[TractorWaypoint],
        route_index: int,
        *,
        preview_m: float,
    ) -> tuple[tuple[float, float], tuple[float, float], float]:
        """Return a stable local tangent and signed planned curvature."""
        route_index = max(1, min(route_index, len(route) - 1))
        gear = waypoints[route_index].gear if route_index < len(waypoints) else 1
        left = route_index - 1
        right = route_index
        travelled = 0.0
        while left > 0 and travelled < preview_m:
            if left < len(waypoints) and waypoints[left].gear != gear:
                break
            travelled += math.hypot(
                route[left][0] - route[left - 1][0],
                route[left][1] - route[left - 1][1],
            )
            left -= 1
        travelled = 0.0
        while right < len(route) - 1 and travelled < preview_m:
            if right + 1 < len(waypoints) and waypoints[right + 1].gear != gear:
                break
            travelled += math.hypot(
                route[right + 1][0] - route[right][0],
                route[right + 1][1] - route[right][1],
            )
            right += 1
        if left == right:
            return route[route_index - 1], route[route_index], 0.0
        middle = max(left + 1, min(route_index, right - 1))
        first, centre, last = route[left], route[middle], route[right]
        side_a = math.dist(centre, last)
        side_b = math.dist(first, last)
        side_c = math.dist(first, centre)
        cross = (
            (centre[0] - first[0]) * (last[1] - first[1])
            - (centre[1] - first[1]) * (last[0] - first[0])
        )
        denominator = side_a * side_b * side_c
        curvature = 0.0 if denominator < 1e-12 else 2.0 * cross / denominator
        return first, last, curvature

    def _waypoint_reached(
        self,
        pose: Pose,
        segment_start: tuple[float, float],
        target: tuple[float, float],
    ) -> bool:
        dx, dy = target[0] - segment_start[0], target[1] - segment_start[1]
        segment_length = math.hypot(dx, dy)
        distance = math.hypot(target[0] - pose.utm_x, target[1] - pose.utm_y)
        if distance <= self._route_tolerance_m:
            return True
        if segment_length < 1e-9:
            return True
        ux, uy = dx / segment_length, dy / segment_length
        from_start_x = pose.utm_x - segment_start[0]
        from_start_y = pose.utm_y - segment_start[1]
        along_track = from_start_x * ux + from_start_y * uy
        cross_track = abs(from_start_x * uy - from_start_y * ux)
        return (
            along_track >= segment_length
            and cross_track <= max(0.25, min(0.50, self._route_tolerance_m * 2.0))
        )

    def _prepare_decks(self, raised_for_manoeuvre: bool) -> bool:
        settings = self._store.settings()
        desired = (
            raised_for_manoeuvre and bool(settings.get("lift_front_on_turn", True)),
            raised_for_manoeuvre and bool(settings.get("lift_rear_on_turn", True)),
        )
        now = time.monotonic()
        if desired != self._deck_lift_state:
            self._publish_motion_context(0.0, "deck_transition")
            self._hardware.drive(0.0, 0.0)
            self._hardware.set_blade(False)
            command = getattr(self._hardware, "set_deck_lift", None)
            if callable(command):
                command(*desired)
            self._deck_lift_state = desired
            if self._deck_state_callback is not None:
                self._deck_state_callback(*desired)
            settle_s = max(
                0.0,
                float(settings.get("deck_lift_settle_ms", 700))
                / 1000.0
                / self._time_scale(),
            )
            self._deck_transition_until = now + settle_s
            self._commands_active = False
            return settle_s <= 0.0
        if now < self._deck_transition_until:
            self._publish_motion_context(0.0, "deck_transition")
            self._hardware.drive(0.0, 0.0)
            self._hardware.set_blade(False)
            self._commands_active = False
            return False
        return True

    def _stop_outputs(self) -> None:
        if self._hardware is not None:
            self._publish_motion_context(0.0, "stopped")
            self._hardware.drive(0.0, 0.0)
            self._hardware.set_blade(False)
        self._commands_active = False
        self._last_steering_deg = 0.0
        self._last_control_time = None

    def _publish_motion_context(self, target_speed_kmh: float, speed_mode: str) -> None:
        command = getattr(self._hardware, "set_motion_context", None)
        if callable(command):
            command(round(max(0.0, float(target_speed_kmh)), 2), speed_mode)
