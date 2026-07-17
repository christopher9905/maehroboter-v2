import math

from shapely.geometry import Polygon

from mower.map.coverage_grid import (
    CoverageGrid,
    deck_polygon,
    machine_curve_clearance,
    machine_footprint,
    machine_lateral_bounds,
    mower_decks_from_settings,
    route_pose_heading,
    validate_swept_machine_route,
)
from mower.path.tractor_route import TractorWaypoint


def test_three_physical_decks_have_separate_geometry():
    decks = mower_decks_from_settings({
        "mowing_width_cm": 60,
        "rear_mower_gap_cm": 6,
        "vehicle_wheelbase_cm": 25,
        "front_mower_offset_cm": 35,
    })
    assert [deck.name for deck in decks] == ["front", "rear_left", "rear_right"]
    assert decks[0].forward_m == 0.60
    assert decks[1].forward_m < 0
    assert decks[1].left_m == -decks[2].left_m
    assert deck_polygon(0, 0, math.pi / 2, decks[0]).area > 0


def test_coverage_grid_marks_rectangular_decks_without_round_line_caps():
    zone = Polygon([(-2, -2), (2, -2), (2, 2), (-2, 2)])
    grid = CoverageGrid(zone, resolution_m=0.02)
    decks = mower_decks_from_settings({})
    grid.mark_pose(0, 0, 0, decks, front_active=True, rear_active=False)
    assert grid.coverage_percent() > 0
    result = grid.geojson(lambda x, y: (x, y))
    assert {item["properties"]["deck"] for item in result["features"]} == {"front"}


def test_interpolation_does_not_leave_holes_between_pose_updates():
    zone = Polygon([(-1, -1), (2, -1), (2, 1), (-1, 1)])
    grid = CoverageGrid(zone, resolution_m=0.02)
    decks = mower_decks_from_settings({})
    grid.mark_pose(0, 0, 0, decks, front_active=True, rear_active=True)
    grid.mark_pose(1, 0, 0, decks, front_active=True, rear_active=True)
    combined = grid.combined_grid()
    middle_column = round((0.5 - grid.min_x) / grid.resolution_m)
    assert combined[:, middle_column].any()


def test_machine_collision_footprint_uses_widest_tractor_or_mower_part():
    settings = {
        "mowing_width_cm": 60,
        "vehicle_width_cm": 34,
        "rear_mower_gap_cm": 6,
    }
    decks = mower_decks_from_settings(settings)

    right, left = machine_lateral_bounds(settings, decks)
    footprint = machine_footprint(0, 0, 0, settings, decks)

    assert math.isclose(right, -0.30)
    assert math.isclose(left, 0.30)
    assert math.isclose(footprint.bounds[1], -0.30)
    assert math.isclose(footprint.bounds[3], 0.30)


def test_curve_clearance_includes_long_front_mower_swing():
    settings = {"mowing_width_cm": 60, "turn_radius_cm": 25}
    decks = mower_decks_from_settings(settings)
    _right, left = machine_lateral_bounds(settings, decks)

    assert machine_curve_clearance(settings, 0.25, decks) > left + 0.20


def test_pose_heading_does_not_spin_at_forward_reverse_switch():
    route = [
        TractorWaypoint(0.0, 0.0, "turn", 1),
        TractorWaypoint(1.0, 0.0, "turn", 1),
        TractorWaypoint(0.9, 0.0, "turn", -1),
    ]

    assert route_pose_heading(route, 1) == 0.0
    assert route_pose_heading(route, 2) == 0.0


def test_swept_validator_rejects_mower_crossing_with_centre_inside():
    zone = Polygon([(0, 0), (4, 0), (4, 4), (0, 4)])
    route = [
        TractorWaypoint(0.20, 1.0, "mow", 1),
        TractorWaypoint(0.20, 3.0, "mow", 1),
    ]

    report = validate_swept_machine_route(
        route, zone, {}, reserve_m=0.0, maximum_sample_step_m=0.02,
    )

    assert not report["valid"]
    assert report["reason"] == "footprint_outside_geofence"
    assert report["outside_area_m2"] > 0.0
