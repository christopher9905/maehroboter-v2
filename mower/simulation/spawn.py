"""Select a reset pose aligned with a safely inset mowing route."""

from __future__ import annotations

import math
from typing import Any, Iterable

import utm
from shapely.geometry import MultiPolygon, Polygon

from mower.map.coverage_grid import (
    machine_curve_clearance,
    machine_footprint,
    machine_geofence_width,
    machine_lateral_bounds,
    machine_longitudinal_bounds,
    mower_decks_from_settings,
)
from mower.path.boustrophedon import compute_best_boustrophedon
from mower.path.tractor_route import compute_headland_route


# Keep reset-pose validation identical to the runtime's centimetre-level RTK
# boundary tolerance.  A stricter check rejected a valid first headland point
# and made the simulator fall back to an unaligned interior pose; the generated
# approach turn could then drift over the geofence immediately after start.
_RESET_GEOFENCE_TOLERANCE_M = 0.01


def choose_safe_reset_pose(
    zones: Iterable[dict[str, Any]],
    settings: dict[str, Any],
    *,
    origin_x: float,
    origin_y: float,
    clearance_m: float = 0.05,
) -> tuple[float, float, float, str] | None:
    """Return a safe zone-relative pose, preferably aligned to the headland.

    The most recently updated mowing zone is preferred.  If route alignment is
    unavailable, the polygon is eroded by the machine's circumscribed radius;
    that fallback remains safe even if the initial vehicle heading changes.
    """
    decks = mower_decks_from_settings(settings)
    boundary_reserve = max(
        0.05,
        min(1.50, float(settings.get("headland_margin_cm", 5.0)) / 100.0),
    )
    mowing_turn_radius = max(
        0.10,
        min(1.50, float(settings.get("mowing_turn_radius_cm", 50)) / 100.0),
    )
    right_extent, left_extent = machine_lateral_bounds(settings, decks)
    lateral_clearance = max(abs(right_extent), abs(left_extent))
    curve_swing = max(
        0.0,
        machine_curve_clearance(settings, mowing_turn_radius, decks)
        - lateral_clearance,
    )
    footprint = machine_footprint(0.0, 0.0, 0.0, settings, decks)
    radius = max(
        math.hypot(float(x), float(y))
        for x, y in footprint.convex_hull.exterior.coords
    ) + max(0.0, float(clearance_m))
    mowing_zones = sorted(
        (zone for zone in zones if zone.get("type", "mowing") == "mowing"),
        key=lambda zone: str(zone.get("updated_at") or zone.get("created_at") or ""),
        reverse=True,
    )
    for zone in mowing_zones:
        geometry = zone.get("geometry") or {}
        rings = geometry.get("coordinates") or []
        if geometry.get("type") != "Polygon" or not rings:
            continue
        metric_rings: list[list[tuple[float, float]]] = []
        zone_band: tuple[int, str] | None = None
        valid = True
        for ring in rings:
            metric_ring: list[tuple[float, float]] = []
            for coordinate in ring:
                if len(coordinate) < 2:
                    valid = False
                    break
                lon, lat = float(coordinate[0]), float(coordinate[1])
                x, y, number, letter = utm.from_latlon(lat, lon)
                if zone_band is None:
                    zone_band = (number, letter)
                elif zone_band != (number, letter):
                    valid = False
                    break
                metric_ring.append((x, y))
            if not valid:
                break
            if len(metric_ring) >= 4:
                metric_rings.append(metric_ring)
        if not valid or not metric_rings:
            continue
        polygon = Polygon(metric_rings[0], holes=metric_rings[1:])
        if not polygon.is_valid:
            repaired = polygon.buffer(0)
            if not isinstance(repaired, (Polygon, MultiPolygon)) or repaired.is_empty:
                continue
            polygon = repaired
        # Recorded Teach-In rings can contain a centimetre-scale overlap at
        # their closing point.  ``buffer(0)`` then legitimately yields a
        # MultiPolygon with one dominant field and a microscopic artefact.
        # Use that dominant field for both the aligned headland spawn and the
        # conservative fallback instead of silently skipping route alignment.
        if isinstance(polygon, MultiPolygon):
            polygon = max(polygon.geoms, key=lambda item: item.area)
        if str(settings.get("coverage_strategy", "headland_first")) == "headland_first":
            try:
                headland_zone = polygon.buffer(
                    -(boundary_reserve + curve_swing), join_style="round",
                )
                if headland_zone.is_empty or not isinstance(headland_zone, Polygon):
                    raise ValueError("zone too small for simulation tracking reserve")
                lane_spacing = max(
                    0.10,
                    min(
                        float(settings.get("mowing_width_cm", 60)) / 100.0,
                        float(settings.get("lane_width_cm", 38)) / 100.0,
                    ),
                )
                mowing_width = max(
                    0.30,
                    float(settings.get("mowing_width_cm", 60)) / 100.0,
                )
                point_spacing = max(
                    0.002,
                    min(
                        0.009,
                        float(settings.get("route_resolution_cm", 0.5)) / 100.0,
                    ),
                )
                turn_radius = max(
                    0.10,
                    min(
                        1.0,
                        float(settings.get("turn_radius_cm", 25)) / 100.0,
                    ),
                )
                rear_extent, front_extent = machine_longitudinal_bounds(
                    settings, decks,
                )
                endpoint_trim = max(
                    abs(min(0.0, rear_extent)) + boundary_reserve,
                    max(0.0, front_extent) + boundary_reserve,
                )
                lane_geometry = polygon.buffer(
                    -(lateral_clearance + boundary_reserve),
                    join_style="round",
                )
                lane_position_geometry = polygon.buffer(
                    -(
                        machine_curve_clearance(settings, turn_radius, decks)
                        + boundary_reserve
                        + 0.03
                    ),
                    join_style="round",
                )
                base_waypoints, _angle, _metrics = compute_best_boustrophedon(
                    polygon,
                    lane_spacing=lane_spacing,
                    mowing_width=mowing_width,
                    endpoint_trim_m=endpoint_trim,
                    endpoint_start_trim_m=endpoint_trim,
                    endpoint_end_trim_m=endpoint_trim,
                    centerline_zone=lane_geometry,
                    lane_position_zone=(
                        lane_position_geometry
                        if not lane_position_geometry.is_empty
                        else lane_geometry
                    ),
                )
                aligned_start = None
                if len(base_waypoints) >= 2:
                    aligned_start = (
                        base_waypoints[0][0],
                        base_waypoints[0][1],
                        math.atan2(
                            base_waypoints[1][1] - base_waypoints[0][1],
                            base_waypoints[1][0] - base_waypoints[0][0],
                        ),
                    )
                headland = compute_headland_route(
                    headland_zone,
                    lane_spacing=lane_spacing,
                    mowing_width=mowing_width,
                    point_spacing=point_spacing,
                    turn_radius=max(
                        0.10,
                        min(
                            1.5,
                            mowing_turn_radius,
                        ),
                    ),
                    passes=max(1, min(3, int(settings.get("headland_passes", 1)))),
                    start_pose=aligned_start,
                    corner_clearance_m=curve_swing,
                )
                if len(headland) >= 2:
                    first, second = headland[0], headland[1]
                    heading = math.atan2(second.y - first.y, second.x - first.x)
                    width = machine_geofence_width(
                        first.x, first.y, heading, settings, decks,
                    )
                    if polygon.buffer(_RESET_GEOFENCE_TOLERANCE_M).covers(width):
                        return (
                            first.x - float(origin_x),
                            first.y - float(origin_y),
                            heading,
                            str(zone.get("id") or zone.get("name") or "mowing-zone"),
                        )
            except ValueError:
                pass
        safe = polygon.buffer(-radius)
        if safe.is_empty:
            continue
        if isinstance(safe, MultiPolygon):
            safe = max(safe.geoms, key=lambda item: item.area)
        point = safe.representative_point()
        candidate = machine_footprint(point.x, point.y, 0.0, settings, decks)
        if not polygon.buffer(_RESET_GEOFENCE_TOLERANCE_M).covers(candidate):
            continue
        return (
            point.x - float(origin_x),
            point.y - float(origin_y),
            0.0,
            str(zone.get("id") or zone.get("name") or "mowing-zone"),
        )
    return None
