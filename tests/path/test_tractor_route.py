import math

import pytest
from shapely.geometry import LineString, Point, Polygon

from mower.path.tractor_route import (
    compute_curvature_bounded_connection,
    compute_headland_route,
    compute_tractor_route,
    measure_minimum_turn_radius,
)


def _minimum_measured_radius(route):
    radii = []
    for first, middle, last in zip(route, route[1:], route[2:]):
        if not (first.gear == middle.gear == last.gear):
            continue
        side_a = math.hypot(last.x - middle.x, last.y - middle.y)
        side_b = math.hypot(last.x - first.x, last.y - first.y)
        side_c = math.hypot(middle.x - first.x, middle.y - first.y)
        double_area = abs(
            (middle.x - first.x) * (last.y - first.y)
            - (middle.y - first.y) * (last.x - first.x)
        )
        if double_area > 1e-10:
            radii.append(side_a * side_b * side_c / (2.0 * double_area))
    return min(radii) if radii else math.inf


def test_transit_connection_is_dense_connected_and_radius_bounded():
    minimum_radius = 0.25
    route = compute_curvature_bounded_connection(
        (0.0, 0.0, 0.0),
        (1.0, 0.7, math.pi / 2.0),
        turn_radius=minimum_radius,
        point_spacing=0.005,
    )

    assert (route[0].x, route[0].y) == (0.0, 0.0)
    assert route[-1].x == pytest.approx(1.0, abs=1e-8)
    assert route[-1].y == pytest.approx(0.7, abs=1e-8)
    assert max(
        math.hypot(right.x - left.x, right.y - left.y)
        for left, right in zip(route, route[1:])
    ) <= 0.00501
    assert _minimum_measured_radius(route) >= minimum_radius - 2e-5


def test_transit_uses_bounded_shunt_instead_of_forward_loop():
    route = compute_curvature_bounded_connection(
        (0.0, 0.0, 0.0),
        (-0.20, 0.10, math.pi / 2.0),
        turn_radius=0.25,
        point_spacing=0.01,
        allow_reverse=True,
    )

    reverse_runs = []
    current = 0.0
    for left, right in zip(route, route[1:]):
        distance = math.hypot(right.x - left.x, right.y - left.y)
        if right.gear < 0:
            current += distance
        elif current:
            reverse_runs.append(current)
            current = 0.0
    if current:
        reverse_runs.append(current)

    assert any(point.gear < 0 for point in route)
    assert max(reverse_runs) <= 1.000001


def test_small_heading_correction_uses_short_shunt_instead_of_full_circle():
    route = compute_curvature_bounded_connection(
        (0.0, 0.0, 0.0),
        (0.20, -0.10, math.radians(20.0)),
        turn_radius=0.25,
        point_spacing=0.01,
        allow_reverse=True,
    )

    physical_headings = []
    longest_reverse_run = current_reverse_run = 0.0
    for left, right in zip(route, route[1:]):
        distance = math.hypot(right.x - left.x, right.y - left.y)
        if distance <= 1e-9:
            continue
        heading = math.atan2(right.y - left.y, right.x - left.x)
        if right.gear < 0:
            heading += math.pi
            current_reverse_run += distance
            longest_reverse_run = max(longest_reverse_run, current_reverse_run)
        else:
            current_reverse_run = 0.0
        physical_headings.append(heading)
    rotation = sum(
        abs((right - left + math.pi) % (2.0 * math.pi) - math.pi)
        for left, right in zip(physical_headings, physical_headings[1:])
    )

    assert math.degrees(rotation) < 180.0
    assert longest_reverse_run < 0.50


def test_dense_route_uses_requested_point_spacing():
    route = compute_tractor_route(
        Polygon([(0, 0), (5, 0), (5, 3), (0, 3)]),
        lane_spacing=1.0,
        point_spacing=0.20,
        turn_strategy="forward",
        turn_radius=0.40,
    )

    distances = [
        ((right.x - left.x) ** 2 + (right.y - left.y) ** 2) ** 0.5
        for left, right in zip(route, route[1:])
    ]
    assert len(route) > 50
    assert max(distances) <= 0.200001


def test_auto_selects_multi_point_turn_when_lane_is_narrow():
    route = compute_tractor_route(
        Polygon([(0, 0), (5, 0), (5, 3), (0, 3)]),
        lane_spacing=0.5,
        point_spacing=0.20,
        turn_strategy="auto",
        turn_radius=1.2,
        maneuver_passes=4,
    )

    turn_points = [point for point in route if point.operation == "turn"]
    assert turn_points
    assert any(point.gear < 0 for point in turn_points)
    assert len({point.turn_id for point in turn_points}) >= 2


def test_turn_geometry_respects_configured_minimum_radius():
    minimum_radius = 1.2
    route = compute_tractor_route(
        Polygon([(0, 0), (8, 0), (8, 6), (0, 6)]),
        lane_spacing=0.52,
        point_spacing=0.005,
        turn_strategy="multi_point",
        turn_radius=minimum_radius,
        maneuver_passes=6,
    )
    first_turn = [point for point in route if point.turn_id == 1]
    measured_curvatures = []
    for left, middle, right in zip(first_turn, first_turn[1:], first_turn[2:]):
        if not (left.gear == middle.gear == right.gear):
            continue
        first_heading = math.atan2(middle.y - left.y, middle.x - left.x)
        second_heading = math.atan2(right.y - middle.y, right.x - middle.x)
        distance = math.hypot(right.x - left.x, right.y - left.y) / 2.0
        if distance > 1e-6:
            heading_change = abs(
                (second_heading - first_heading + math.pi) % (2.0 * math.pi) - math.pi
            )
            measured_curvatures.append(heading_change / distance)

    assert measured_curvatures
    assert max(measured_curvatures) <= (1.0 / minimum_radius) + 0.01


def test_diagonally_offset_swath_end_uses_oriented_radius_bounded_connector():
    """Clipped lane ends must not shear a U-turn below its physical radius."""
    minimum_radius = 0.25
    route = compute_tractor_route(
        Polygon([(-2, -2), (7, -2), (7, 3), (-2, 3)]),
        lane_spacing=0.48,
        mowing_width=0.60,
        point_spacing=0.005,
        turn_strategy="auto",
        turn_radius=minimum_radius,
        boundary_zone=Polygon([(-2, -2), (7, -2), (7, 3), (-2, 3)]),
        base_waypoints=[(0, 0), (5, 0), (4.2, 0.48), (-0.8, 0.48)],
    )

    measured_radius, violation_index = measure_minimum_turn_radius(route)

    assert violation_index is not None
    assert measured_radius >= minimum_radius * 0.99
    assert route[-1].x == pytest.approx(-0.8)
    assert route[-1].y == pytest.approx(0.48)


def test_headland_route_mows_configured_outside_passes_inside_zone():
    zone = Polygon([(0, 0), (8, 0), (8, 6), (0, 6)])

    route = compute_headland_route(
        zone,
        lane_spacing=0.52,
        point_spacing=0.01,
        turn_radius=1.2,
        passes=2,
    )

    assert route
    assert sum(point.operation == "turn" for point in route) > 0
    assert sum(point.operation == "headland" for point in route) > 100
    assert all(zone.buffer(1e-8).covers(Point(point.x, point.y)) for point in route)


def test_multiple_headland_passes_keep_constant_spacing_through_corners():
    lane_spacing = 0.48
    route = compute_headland_route(
        Polygon([(0, 0), (12, 0), (12, 8), (0, 8)]),
        lane_spacing=lane_spacing,
        mowing_width=0.60,
        point_spacing=0.01,
        turn_radius=0.50,
        passes=3,
    )
    rings = []
    active = []
    for point in route:
        if point.operation == "headland":
            active.append((point.x, point.y))
        elif active:
            rings.append(LineString(active))
            active = []
    if active:
        rings.append(LineString(active))

    assert len(rings) == 3
    for outer, inner in zip(rings, rings[1:]):
        distances = [
            outer.interpolate(index * outer.length / 720).distance(inner)
            for index in range(721)
        ]
        assert min(distances) == pytest.approx(lane_spacing, abs=0.002)
        assert max(distances) == pytest.approx(lane_spacing, abs=0.002)


def test_headland_route_respects_minimum_turn_radius():
    zone = Polygon([(0, 0), (8, 0), (8, 6), (0, 6)])
    minimum_radius = 1.20
    route = compute_headland_route(
        zone,
        lane_spacing=0.52,
        point_spacing=0.005,
        turn_radius=minimum_radius,
        passes=1,
    )
    points = [(point.x, point.y) for point in route if point.operation == "headland"]
    measured_radii = []
    for first, middle, last in zip(points, points[1:], points[2:]):
        side_a = math.dist(middle, last)
        side_b = math.dist(first, last)
        side_c = math.dist(first, middle)
        double_area = abs(
            (middle[0] - first[0]) * (last[1] - first[1])
            - (middle[1] - first[1]) * (last[0] - first[0])
        )
        if double_area > 1e-12:
            measured_radii.append(side_a * side_b * side_c / (2.0 * double_area))

    assert measured_radii
    assert min(measured_radii) >= minimum_radius * 0.999
