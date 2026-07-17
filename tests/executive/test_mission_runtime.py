import math
import time
from unittest.mock import MagicMock, patch

import pytest
import utm
from shapely.geometry import Polygon

from mower.api.control_store import ControlStore
from mower.executive.mission_executive import MissionExecutive, MowerState
from mower.executive.mission_runtime import (
    MissionRuntime,
    _mow_swath_endpoints,
    _native_boundary_retry_insets,
    _valid_coverage_polygon,
)
from mower.nav.gps_reader import GpsFix
from mower.nav.localizer import Pose
from mower.path.tractor_route import TractorWaypoint
from mower.path.trajectory import ContinuousTrajectory
from mower.simulation.spawn import choose_safe_reset_pose


def make_fix(lat=48.5, lon=11.0, timestamp=0.0):
    x, y, number, letter = utm.from_latlon(lat, lon)
    return GpsFix(lat, lon, 500.0, x, y, number, letter, 4, 0.7, timestamp)


def make_pose(x, y, timestamp=0.0, heading=0.0, speed=0.2):
    return Pose(x, y, heading, speed, 0.0, timestamp, 4)


def square_zone(fix, size_m=5.0):
    metric = [
        (fix.utm_x, fix.utm_y),
        (fix.utm_x + size_m, fix.utm_y),
        (fix.utm_x + size_m, fix.utm_y + size_m),
        (fix.utm_x, fix.utm_y + size_m),
        (fix.utm_x, fix.utm_y),
    ]
    coordinates = []
    for x, y in metric:
        lat, lon = utm.to_latlon(x, y, fix.utm_zone_number, fix.utm_zone_letter)
        coordinates.append([lon, lat])
    return {
        "id": "zone-1",
        "name": "Testfeld",
        "type": "mowing",
        "geometry": {"type": "Polygon", "coordinates": [coordinates]},
    }


def zone_from_local(fix, local_points, name="Testfeld"):
    coordinates = []
    for dx, dy in local_points:
        lat, lon = utm.to_latlon(
            fix.utm_x + dx,
            fix.utm_y + dy,
            fix.utm_zone_number,
            fix.utm_zone_letter,
        )
        coordinates.append([lon, lat])
    return {
        "id": "zone-local",
        "name": name,
        "type": "mowing",
        "geometry": {"type": "Polygon", "coordinates": [coordinates]},
    }


def test_plans_wgs84_zone_with_existing_coverage_planner():
    fix = make_fix()
    runtime = MissionRuntime(MissionExecutive(), MagicMock(), ControlStore())

    feature = runtime.plan_zone(square_zone(fix))

    assert feature["geometry"]["type"] == "LineString"
    assert feature["properties"]["waypoint_count"] >= 2
    assert len(feature["geometry"]["coordinates"]) < feature["properties"]["waypoint_count"]
    lon, lat = feature["geometry"]["coordinates"][0]
    assert lon == pytest.approx(fix.lon, abs=0.001)
    assert lat == pytest.approx(fix.lat, abs=0.001)


def test_identical_zone_and_settings_reuse_plan_cache_but_changes_invalidate_it():
    fix = make_fix()
    store = ControlStore()
    store.update_settings({
        "coverage_strategy": "interior",
        "route_resolution_cm": 0.9,
    })
    runtime = MissionRuntime(MissionExecutive(), MagicMock(), store)
    zone = square_zone(fix, size_m=5.0)

    first = runtime.plan_zone(zone)
    assert first["properties"]["route_metrics"]["plan_cache_hit"] is False

    with patch(
        "mower.executive.mission_runtime.compute_best_boustrophedon",
        side_effect=AssertionError("planner must not run for a cache hit"),
    ):
        second = runtime.plan_zone(zone)
    assert second["properties"]["route_metrics"]["plan_cache_hit"] is True
    assert second["geometry"] == first["geometry"]

    original_key = runtime._plan_cache_key(zone, store.settings(), None)
    store.update_settings({"lane_width_cm": 37})
    changed_settings_key = runtime._plan_cache_key(zone, store.settings(), None)
    changed_zone = square_zone(fix, size_m=5.1)
    changed_zone_key = runtime._plan_cache_key(changed_zone, store.settings(), None)

    assert changed_settings_key != original_key
    assert changed_zone_key != changed_settings_key


def test_aligned_headland_spawn_is_preserved_instead_of_rotating_route_away():
    fix = make_fix()
    store = ControlStore()
    store.update_settings({
        "planner_engine": "mv2",
        "mowing_width_cm": 60,
        "lane_width_cm": 48,
        "turn_radius_cm": 25,
        "mowing_turn_radius_cm": 50,
        "headland_margin_cm": 10,
        "coverage_strategy": "headland_first",
        "headland_passes": 1,
        "route_resolution_cm": 0.5,
    })
    zone = zone_from_local(fix, [
        (0.0, 0.0),
        (7.232872, 0.139715),
        (5.872565, 4.772753),
        (-0.061292, 5.186291),
        (0.0, 0.0),
    ])
    spawn = choose_safe_reset_pose(
        [zone],
        store.settings(),
        origin_x=fix.utm_x,
        origin_y=fix.utm_y,
    )
    assert spawn is not None
    spawn_x, spawn_y, spawn_heading, _zone_id = spawn
    runtime = MissionRuntime(MissionExecutive(), MagicMock(), store)
    runtime.on_pose(make_pose(
        fix.utm_x + spawn_x,
        fix.utm_y + spawn_y,
        heading=spawn_heading,
        speed=0.0,
    ))

    feature = runtime.plan_zone(zone)

    metrics = feature["properties"]["route_metrics"]
    assert metrics["staged_entry_used"] is False
    assert metrics["departure_waypoint_count"] == 0
    assert math.hypot(
        runtime._route_waypoints[0].x - runtime._latest_pose.utm_x,
        runtime._route_waypoints[0].y - runtime._latest_pose.utm_y,
    ) <= 0.08
    assert feature["properties"]["plan_safety"]["tracking_reserve_ok"] is True


def test_native_fields2cover_is_selectable_and_passes_common_safety_preflight():
    fix = make_fix()
    store = ControlStore()
    store.update_settings({
        "planner_engine": "fields2cover",
        "fields2cover_decomposition": "boustrophedon",
        "fields2cover_route_order": "optimized",
        "fields2cover_path_type": "dubins",
        "fields2cover_optimizer_time_s": 1,
        "minimum_plan_coverage_percent": 95,
        "route_resolution_cm": 0.9,
    })

    feature = MissionRuntime(
        MissionExecutive(), MagicMock(), store,
    ).plan_zone(square_zone(fix, size_m=8.0))

    properties = feature["properties"]
    assert properties["planner_version"] == "fields2cover_native_2x"
    assert properties["coverage_strategy"] == "fields2cover_native"
    assert properties["turn_strategy"] == "dubins"
    assert properties["minimum_mowing_turn_radius_cm"] == 50
    assert properties["plan_safety"]["valid"] is True
    assert properties["plan_safety"]["tracking_reserve_ok"] is True
    assert properties["plan_safety"]["tracking_reserve_report"]["reserve_m"] == 0.05
    assert properties["route_metrics"]["engine"] == "fields2cover_native"
    assert properties["route_metrics"]["machine_envelope_width_m"] == pytest.approx(0.60)
    assert properties["route_metrics"]["coverage_width_m"] == pytest.approx(0.60)
    assert properties["route_metrics"]["headland_width_m"] < 1.0
    assert properties["route_metrics"]["headland_mode"] == "auto"
    assert properties["waypoint_count"] > 100


def test_native_boundary_retries_do_not_use_front_overhang_bounding_circle():
    retries = _native_boundary_retry_insets(
        boundary_reserve=0.05,
        curved_clearance=0.363,
        lateral_clearance=0.30,
    )

    assert retries[:3] == (0.05, 0.113, 0.163)
    assert retries[1] < 0.20
    assert max(retries) < 0.70


def test_native_turns_can_be_removed_without_losing_ordered_swaths():
    points = [
        TractorWaypoint(0.0, 0.0, "mow"),
        TractorWaypoint(1.0, 0.0, "mow"),
        TractorWaypoint(1.1, 0.1, "turn"),
        TractorWaypoint(1.0, 0.5, "mow"),
        TractorWaypoint(0.0, 0.5, "mow"),
    ]

    assert _mow_swath_endpoints(points) == [
        (0.0, 0.0), (1.0, 0.0),
        (1.0, 0.5), (0.0, 0.5),
    ]


def test_simulation_time_scale_is_supplied_and_bounded():
    runtime = MissionRuntime(
        MissionExecutive(), MagicMock(), ControlStore(),
        time_scale_provider=lambda: 5.0,
    )

    assert runtime._time_scale() == 5.0


def test_hybrid_plan_keeps_concave_test1_route_and_footprint_safe():
    fix = make_fix()
    local = [
        (0, 0), (-1.11, 1.89), (-1.12, 2.64), (-0.72, 3.25),
        (-0.04, 3.5), (2.75, 3.52), (5.21, 4.74), (6.69, 4.49),
        (7.65, 3.67), (8.18, 2.53), (7.94, -1.79), (8.58, -3.69),
        (8.45, -4.38), (8.1, -4.62), (7.67, -4.57), (4.56, -2.41),
        (1.4, -3.3), (0.54, -3.27), (-0.13, -2.3), (0.26, -0.64),
        (0, 0),
    ]
    store = ControlStore()
    store.update_settings({
        "mowing_width_cm": 60,
        "lane_width_cm": 50,
        "turn_radius_cm": 25,
        "coverage_strategy": "headland_first",
        "headland_passes": 1,
    })

    feature = MissionRuntime(
        MissionExecutive(), MagicMock(), store,
    ).plan_zone(zone_from_local(fix, local, name="Test1"))

    properties = feature["properties"]
    assert properties["planner_version"] == "coverage_hybrid_v3"
    assert properties["estimated_mainland_coverage_percent"] >= 98.5
    assert properties["estimated_coverage_percent"] >= 90.0
    assert properties["plan_safety"]["valid"] is True
    assert properties["route_metrics"]["candidate_angles"] >= 18
    # The complete target and the safety/headland band stay visible instead
    # of inflating the regular interior-swath coverage value.
    assert properties["estimated_uncovered_area_m2"] > 0.0
    assert properties["headland_safety_area_m2"] > 0.0


def test_repairs_only_negligible_teach_in_self_intersection():
    polygon = Polygon([
        (0.0, 0.0), (8.0, 0.0), (8.0, 8.0), (0.0, 8.0),
        (0.0, 0.0), (0.00001, 0.00001), (0.0, 0.0),
    ])

    repaired = _valid_coverage_polygon(polygon)

    assert repaired.is_valid
    assert repaired.area == pytest.approx(64.0)


def test_rejects_material_figure_eight_zone():
    polygon = Polygon([
        (0.0, 0.0), (4.0, 4.0), (0.0, 4.0), (4.0, 0.0), (0.0, 0.0),
    ])

    with pytest.raises(ValueError, match="schneidet sich selbst"):
        _valid_coverage_polygon(polygon)


def test_route_exposes_actual_track_and_only_blade_on_mown_sections():
    fix = make_fix()
    hardware = MagicMock()
    hardware.output_snapshot.return_value = {
        "blade_enabled": True,
        "front_deck_raised": False,
        "rear_deck_raised": False,
    }
    executive = MissionExecutive()
    executive.start_mission("zone-1")
    runtime = MissionRuntime(executive, hardware, ControlStore())
    runtime.on_gps_fix(fix)

    runtime.on_pose(make_pose(fix.utm_x, fix.utm_y, timestamp=1.0))
    runtime.on_pose(make_pose(fix.utm_x + 0.02, fix.utm_y, timestamp=1.1))
    feature = runtime.route_geojson()

    assert len(feature["actual_geometry"]["coordinates"]) == 2
    assert len(feature["actual_mown_geometry"]["coordinates"]) == 1
    assert len(feature["actual_mown_geometry"]["coordinates"][0]) == 2

    hardware.output_snapshot.return_value["blade_enabled"] = False
    runtime.on_pose(make_pose(fix.utm_x + 0.04, fix.utm_y, timestamp=1.2))
    feature = runtime.route_geojson()
    assert len(feature["actual_geometry"]["coordinates"]) == 3
    assert len(feature["actual_mown_geometry"]["coordinates"][0]) == 2


def test_rendered_mowing_lanes_and_turns_share_exact_endpoints():
    fix = make_fix()
    store = ControlStore()
    store.update_settings({"turn_maneuver_passes": 6})
    runtime = MissionRuntime(MissionExecutive(), MagicMock(), store)

    feature = runtime.plan_zone(square_zone(fix))
    mowing_lines = feature["mow_geometry"]["coordinates"]
    turn_lines = feature["turn_geometry"]["coordinates"]

    assert len(mowing_lines) >= 2
    assert len(turn_lines) == len(mowing_lines) - 1
    for mowing, turn, following_mowing in zip(
        mowing_lines, turn_lines, mowing_lines[1:],
    ):
        assert mowing[-1] == turn[0]
        assert turn[-1] == following_mowing[0]


def test_headland_first_strategy_precedes_inside_coverage():
    fix = make_fix()
    store = ControlStore()
    store.update_settings({
        "coverage_strategy": "headland_first",
        "headland_passes": 1,
    })
    runtime = MissionRuntime(MissionExecutive(), MagicMock(), store)

    feature = runtime.plan_zone(square_zone(fix, size_m=8.0))

    assert feature["properties"]["coverage_strategy"] == "headland_first"
    assert feature["properties"]["headland_passes"] == 1
    assert feature["headland_geometry"]["coordinates"]
    assert runtime._route_waypoints[0].operation == "headland"


def test_route_uses_real_mowing_width_and_reports_interior_and_total_coverage():
    fix = make_fix()
    store = ControlStore()
    store.update_settings({
        "mowing_width_cm": 60,
        "lane_width_cm": 52,
        "coverage_strategy": "headland_first",
        "headland_passes": 1,
        "headland_margin_cm": 5,
    })
    runtime = MissionRuntime(MissionExecutive(), MagicMock(), store)

    feature = runtime.plan_zone(square_zone(fix, size_m=8.0))
    properties = feature["properties"]

    assert properties["mowing_width_cm"] == 60
    assert properties["lane_spacing_cm"] == 52
    assert properties["overlap_cm"] == 8
    assert properties["headland_margin_cm"] == 5
    assert properties["geofence_tracking_reserve_cm"] == 5
    assert properties["estimated_mainland_coverage_percent"] >= 98.5
    assert properties["estimated_coverage_percent"] >= 90.0


def test_runtime_plans_and_starts_mission_while_charging():
    fix = make_fix()
    executive = MissionExecutive()
    executive.start_mission("old-zone")
    executive.stop_mission()
    executive.on_dock_success()
    executive.on_charge_started()
    departure_callback = MagicMock()
    runtime = MissionRuntime(
        executive, MagicMock(), ControlStore(),
        dock_departure_callback=departure_callback,
    )

    feature = runtime.start_mission(square_zone(fix, size_m=8.0))

    assert feature["properties"]["waypoint_count"] >= 2
    assert executive.state == MowerState.MOWING
    assert executive.active_zone_id == "zone-1"
    departure_callback.assert_called_once_with()


def test_charging_robot_uses_single_use_geofence_corridor_from_nearby_dock():
    fix = make_fix()
    zone = square_zone(fix, size_m=8.0)
    dock_x, dock_y = fix.utm_x + 8.8, fix.utm_y + 4.0
    dock_lat, dock_lon = utm.to_latlon(
        dock_x, dock_y, fix.utm_zone_number, fix.utm_zone_letter,
    )
    dock_fix = make_fix(dock_lat, dock_lon)
    inside_lat, inside_lon = utm.to_latlon(
        fix.utm_x + 7.5, fix.utm_y + 4.0,
        fix.utm_zone_number, fix.utm_zone_letter,
    )
    inside_fix = make_fix(inside_lat, inside_lon)
    store = ControlStore()
    store.set_home({"lat": dock_lat, "lon": dock_lon, "name": "Dock"})
    executive = MissionExecutive()
    executive.start_mission("old-zone")
    executive.stop_mission()
    executive.on_dock_success()
    executive.on_charge_started()
    runtime = MissionRuntime(executive, MagicMock(), store)
    runtime.on_pose(make_pose(dock_x, dock_y))

    feature = runtime.start_mission(zone)

    assert feature["geometry"]["coordinates"][0][0] == pytest.approx(dock_lon)
    assert feature["geometry"]["coordinates"][0][1] == pytest.approx(dock_lat)
    assert runtime.departure_geofence_allows(dock_fix) is True
    assert runtime.departure_geofence_allows(inside_fix) is False
    assert runtime.departure_geofence_allows(dock_fix) is False


def test_safe_in_zone_edge_pose_gets_radius_bounded_staged_connection():
    fix = make_fix()
    store = ControlStore()
    store.update_settings({"route_resolution_cm": 0.9})
    runtime = MissionRuntime(MissionExecutive(), MagicMock(), store)
    runtime.on_pose(make_pose(
        fix.utm_x + 0.50,
        fix.utm_y + 4.0,
        heading=0.0,
        speed=0.0,
    ))

    feature = runtime.plan_zone(square_zone(fix, size_m=8.0))

    first_lon, first_lat = feature["geometry"]["coordinates"][0]
    first_x, first_y, _, _ = utm.from_latlon(first_lat, first_lon)
    assert first_x == pytest.approx(fix.utm_x + 0.50, abs=0.01)
    assert first_y == pytest.approx(fix.utm_y + 4.0, abs=0.01)
    assert feature["properties"]["minimum_turn_radius_cm"] == 25
    assert feature["properties"]["plan_safety"]["valid"] is True


def test_charging_departure_rejects_dock_too_far_from_zone():
    fix = make_fix()
    far_x, far_y = fix.utm_x + 14.0, fix.utm_y + 4.0
    far_lat, far_lon = utm.to_latlon(
        far_x, far_y, fix.utm_zone_number, fix.utm_zone_letter,
    )
    store = ControlStore()
    store.set_home({"lat": far_lat, "lon": far_lon, "name": "Dock"})
    executive = MissionExecutive()
    executive.start_mission()
    executive.stop_mission()
    executive.on_dock_success()
    executive.on_charge_started()
    runtime = MissionRuntime(executive, MagicMock(), store)
    runtime.on_pose(make_pose(far_x, far_y))

    with pytest.raises(ValueError, match="zu weit außerhalb"):
        runtime.start_mission(square_zone(fix, size_m=8.0))

    assert executive.state == MowerState.CHARGING


def test_charging_robot_uses_radius_bounded_saved_station_path():
    fix = make_fix()
    zone = square_zone(fix, size_m=8.0)
    dock_x, dock_y = fix.utm_x + 14.0, fix.utm_y + 4.0
    entry_x, entry_y = fix.utm_x + 8.0, fix.utm_y + 4.0
    dock_lat, dock_lon = utm.to_latlon(
        dock_x, dock_y, fix.utm_zone_number, fix.utm_zone_letter,
    )
    entry_lat, entry_lon = utm.to_latlon(
        entry_x, entry_y, fix.utm_zone_number, fix.utm_zone_letter,
    )
    store = ControlStore()
    store.update_settings({"route_resolution_cm": 0.9})
    store.set_home({"lat": dock_lat, "lon": dock_lon, "name": "Dock", "heading_deg": 270})
    connection = store.add_item("connections", {
        "name": "Dockweg", "type": "dock_link", "to_zone_id": zone["id"],
        "corridor_width_cm": 200, "bidirectional": True, "enabled": True,
        "geometry": {"type": "LineString", "coordinates": [
            [dock_lon, dock_lat], [entry_lon, entry_lat],
        ]},
    })
    executive = MissionExecutive()
    executive.start_mission("old-zone")
    executive.stop_mission()
    executive.on_dock_success()
    executive.on_charge_started()
    runtime = MissionRuntime(executive, MagicMock(), store)
    runtime.on_pose(make_pose(dock_x, dock_y, heading=math.pi))

    route = runtime.start_mission(zone)

    assert route["properties"]["active_connection_id"] == connection["id"]
    assert route["geometry"]["coordinates"][0] == pytest.approx([dock_lon, dock_lat])
    assert executive.state == MowerState.MOWING


def test_idle_robot_uses_saved_path_between_two_zones():
    fix = make_fix()
    target = square_zone(fix, size_m=8.0)
    source = zone_from_local(fix, [
        (14.0, 0.0), (22.0, 0.0), (22.0, 8.0), (14.0, 8.0), (14.0, 0.0),
    ], name="Nebenfeld")
    source["id"] = "zone-source"
    start_x, start_y = fix.utm_x + 20.0, fix.utm_y + 4.0
    source_entry = (fix.utm_x + 14.0, fix.utm_y + 4.0)
    target_entry = (fix.utm_x + 8.0, fix.utm_y + 4.0)
    connection_coordinates = []
    for x, y in (source_entry, target_entry):
        lat, lon = utm.to_latlon(x, y, fix.utm_zone_number, fix.utm_zone_letter)
        connection_coordinates.append([lon, lat])
    store = ControlStore()
    store.update_settings({"route_resolution_cm": 0.9})
    store.add_item("zones", source)
    store.add_item("zones", target)
    connection = store.add_item("connections", {
        "name": "Nebenfeld zu Testfeld", "type": "zone_link",
        "from_zone_id": source["id"], "to_zone_id": target["id"],
        "corridor_width_cm": 200, "bidirectional": True, "enabled": True,
        "geometry": {"type": "LineString", "coordinates": connection_coordinates},
    })
    runtime = MissionRuntime(MissionExecutive(), MagicMock(), store)
    runtime.on_pose(make_pose(start_x, start_y, heading=math.pi))

    route = runtime.start_mission(target)

    assert route["properties"]["active_connection_id"] == connection["id"]
    assert route["geometry"]["coordinates"][0] == pytest.approx(
        list(reversed(utm.to_latlon(start_x, start_y, fix.utm_zone_number, fix.utm_zone_letter)))
    )


def test_return_home_uses_bidirectional_saved_station_path():
    fix = make_fix()
    zone = square_zone(fix, size_m=8.0)
    home_x, home_y = fix.utm_x + 14.0, fix.utm_y + 4.0
    entry_x, entry_y = fix.utm_x + 8.0, fix.utm_y + 4.0
    home_lat, home_lon = utm.to_latlon(
        home_x, home_y, fix.utm_zone_number, fix.utm_zone_letter,
    )
    entry_lat, entry_lon = utm.to_latlon(
        entry_x, entry_y, fix.utm_zone_number, fix.utm_zone_letter,
    )
    store = ControlStore()
    store.update_settings({"route_resolution_cm": 0.9})
    store.add_item("zones", zone)
    store.set_home({"lat": home_lat, "lon": home_lon, "name": "Dock", "heading_deg": 90})
    connection = store.add_item("connections", {
        "name": "Dockweg", "type": "dock_link", "to_zone_id": zone["id"],
        "corridor_width_cm": 200, "bidirectional": True, "enabled": True,
        "geometry": {"type": "LineString", "coordinates": [
            [home_lon, home_lat], [entry_lon, entry_lat],
        ]},
    })
    executive = MissionExecutive()
    executive.start_mission(zone["id"])
    runtime = MissionRuntime(executive, MagicMock(), store)
    runtime.on_pose(make_pose(fix.utm_x + 7.0, fix.utm_y + 4.0, heading=0.0))

    route = runtime.return_home()

    assert route["properties"]["active_connection_id"] == connection["id"]
    assert route["geometry"]["coordinates"][-1] == pytest.approx([home_lon, home_lat])


def test_teach_in_returns_server_geometry_in_wgs84():
    fix = make_fix()
    executive = MissionExecutive()
    runtime = MissionRuntime(executive, MagicMock(), ControlStore())
    runtime.on_gps_fix(fix)
    runtime.start_teach_in()
    for index, (dx, dy) in enumerate(((0, 0), (3, 0), (3, 3), (0, 3))):
        runtime.on_pose(make_pose(fix.utm_x + dx, fix.utm_y + dy, timestamp=index * 2.0))

    result = runtime.stop_teach_in()

    assert executive.state == MowerState.IDLE
    assert result["point_count"] == 4
    ring = result["geometry"]["coordinates"][0]
    assert len(ring) == 5
    assert ring[0] == ring[-1]
    assert ring[0][0] == pytest.approx(fix.lon, abs=1e-6)
    assert ring[0][1] == pytest.approx(fix.lat, abs=1e-6)


@pytest.mark.parametrize("reference, expected_offset", [
    ("left", 0.30),
    ("center", 0.0),
    ("right", -0.30),
])
def test_teach_in_records_selected_machine_edge(reference, expected_offset):
    fix = make_fix()
    runtime = MissionRuntime(MissionExecutive(), MagicMock(), ControlStore())
    runtime.on_gps_fix(fix)
    runtime.start_teach_in(reference=reference)

    runtime.on_pose(make_pose(fix.utm_x, fix.utm_y, heading=0.0))

    point = runtime._teach_in.points[0]
    assert point[0] == pytest.approx(fix.utm_x)
    assert point[1] == pytest.approx(fix.utm_y + expected_offset)
    assert runtime.teach_in_status()["reference"] == reference


def test_geofence_checks_complete_machine_footprint_not_gps_centre():
    origin = make_fix()
    zone = square_zone(origin, size_m=5.0)
    runtime = MissionRuntime(MissionExecutive(), MagicMock(), ControlStore())
    runtime.on_pose(make_pose(origin.utm_x + 2.5, origin.utm_y + 0.15, heading=0.0))
    lat, lon = utm.to_latlon(
        origin.utm_x + 2.5,
        origin.utm_y + 0.15,
        origin.utm_zone_number,
        origin.utm_zone_letter,
    )
    near_edge = make_fix(lat=lat, lon=lon)

    status = runtime.footprint_geofence_status(near_edge, zone)

    assert status["inside"] is False
    assert status["allowed"] is False


def test_invalid_teach_in_stays_active_for_undo_correction():
    fix = make_fix()
    executive = MissionExecutive()
    runtime = MissionRuntime(executive, MagicMock(), ControlStore())
    runtime.on_gps_fix(fix)
    runtime.start_teach_in()
    for index, (dx, dy) in enumerate(
        ((0, 0), (4, 4), (0, 4), (4, 0), (0, 0)),
    ):
        runtime.on_pose(make_pose(
            fix.utm_x + dx,
            fix.utm_y + dy,
            timestamp=index * 2.0,
        ))

    with pytest.raises(ValueError, match="schneidet sich selbst"):
        runtime.stop_teach_in()

    assert executive.state == MowerState.TEACH_IN


def test_undo_suspends_recording_until_robot_returns_to_trimmed_endpoint():
    fix = make_fix()
    hardware = MagicMock()
    executive = MissionExecutive(hardware)
    runtime = MissionRuntime(executive, hardware, ControlStore())
    runtime.on_gps_fix(fix)
    runtime.start_teach_in()
    for index, dx in enumerate((0.0, 1.0, 2.0, 3.0, 4.0)):
        runtime.on_pose(make_pose(fix.utm_x + dx, fix.utm_y, timestamp=index * 2.0))

    corrected = runtime.undo_teach_in(2.0)
    runtime.on_pose(make_pose(fix.utm_x + 3.5, fix.utm_y, timestamp=12.0))
    while_away = runtime.teach_in_status()
    runtime.on_pose(make_pose(fix.utm_x + 2.2, fix.utm_y, timestamp=14.0))
    after_return = runtime.teach_in_status()

    assert corrected["removed_distance_m"] == pytest.approx(2.0)
    assert corrected["suspended"] is True
    assert corrected["length_m"] == pytest.approx(2.0)
    assert while_away["point_count"] == corrected["point_count"]
    assert while_away["suspended"] is True
    assert after_return["suspended"] is False
    hardware.drive.assert_called_with(0.0, 0.0)


def test_continue_here_resumes_after_correction_without_returning():
    fix = make_fix()
    runtime = MissionRuntime(MissionExecutive(), MagicMock(), ControlStore())
    runtime.on_gps_fix(fix)
    runtime.start_teach_in()
    runtime.on_pose(make_pose(fix.utm_x, fix.utm_y, timestamp=0.0))
    runtime.on_pose(make_pose(fix.utm_x + 3.0, fix.utm_y, timestamp=2.0))
    runtime.undo_teach_in(1.0)
    runtime.on_pose(make_pose(fix.utm_x + 4.0, fix.utm_y + 1.0, timestamp=4.0))

    status = runtime.continue_teach_in_here()

    assert status["suspended"] is False
    assert status["point_count"] >= 3


def test_control_step_drives_real_hardware_interface_contract():
    fix = make_fix()
    hardware = MagicMock()
    executive = MissionExecutive(hardware)
    runtime = MissionRuntime(executive, hardware, ControlStore())
    route = runtime.start_mission(square_zone(fix))
    first_lon, first_lat = route["geometry"]["coordinates"][0]
    first_x, first_y, _, _ = utm.from_latlon(first_lat, first_lon)
    runtime.on_pose(make_pose(first_x, first_y, heading=0.0))

    runtime._control_step()

    hardware.set_blade.assert_called_with(True)
    speed, steering = hardware.drive.call_args.args
    assert 0.0 < speed <= 1.0
    assert -45.0 <= steering <= 45.0


def test_headland_uses_turn_speed_but_keeps_mower_decks_active():
    fix = make_fix()
    hardware = MagicMock()
    executive = MissionExecutive(hardware)
    store = ControlStore()
    store.update_settings({
        "coverage_strategy": "headland_first",
        "headland_passes": 1,
        "speed_kmh": 1.44,
        "turn_speed_kmh": 0.36,
    })
    runtime = MissionRuntime(executive, hardware, store)
    runtime.start_mission(square_zone(fix))
    assert runtime._route_waypoints[0].operation == "headland"
    start_x, start_y = runtime._route_utm[0]
    runtime.on_pose(make_pose(start_x, start_y, heading=0.0))

    runtime._control_step()

    hardware.set_blade.assert_called_with(True)
    hardware.set_motion_context.assert_called_with(0.36, "headland")
    assert hardware.drive.call_args.args[0] == pytest.approx(0.2)


def test_last_waypoint_finishes_mowing_and_stops_outputs():
    fix = make_fix()
    hardware = MagicMock()
    executive = MissionExecutive(hardware)
    runtime = MissionRuntime(executive, hardware, ControlStore())
    route = runtime.start_mission(square_zone(fix))
    last_lon, last_lat = route["geometry"]["coordinates"][-1]
    last_x, last_y, _, _ = utm.from_latlon(last_lat, last_lon)
    runtime.on_pose(make_pose(last_x, last_y))
    runtime._route_index = len(runtime._route_utm) - 1

    runtime._control_step()

    assert executive.state == MowerState.RETURNING
    hardware.drive.assert_called_with(0.0, 0.0)
    hardware.set_blade.assert_called_with(False)


def test_return_home_builds_route_and_drives_with_blade_off():
    fix = make_fix()
    hardware = MagicMock()
    executive = MissionExecutive(hardware)
    store = ControlStore()
    store.update_settings({"deck_lift_settle_ms": 0})
    home_lat, home_lon = utm.to_latlon(
        fix.utm_x + 5.0, fix.utm_y, fix.utm_zone_number, fix.utm_zone_letter,
    )
    store.set_home({"lat": home_lat, "lon": home_lon, "name": "Dock"})
    runtime = MissionRuntime(executive, hardware, store)
    runtime.on_gps_fix(fix)
    runtime.on_pose(make_pose(fix.utm_x + 1.0, fix.utm_y + 1.0))
    runtime.start_mission(square_zone(fix))

    route = runtime.return_home()
    runtime._control_step()

    assert executive.state == MowerState.RETURNING
    assert route["properties"]["mode"] == "return_home"
    assert route["properties"]["waypoint_count"] > 2
    assert route["properties"]["minimum_turn_radius_cm"] == 25
    hardware.set_blade.assert_called_with(False)
    assert hardware.drive.call_args.args[0] > 0


def test_turn_segment_raises_both_decks_and_reverses_with_blade_off():
    fix = make_fix()
    hardware = MagicMock()
    executive = MissionExecutive(hardware)
    store = ControlStore()
    store.update_settings({
        "turn_strategy": "reverse",
        "route_resolution_cm": 0.5,
        "deck_lift_settle_ms": 0,
        "lift_front_on_turn": True,
        "lift_rear_on_turn": True,
    })
    runtime = MissionRuntime(executive, hardware, store)
    runtime.start_mission(square_zone(fix))
    turn_index = next(
        index for index, point in enumerate(runtime._route_waypoints)
        if point.operation == "turn" and point.gear < 0
    )
    start_x, start_y = runtime._route_utm[turn_index - 1]
    runtime._route_index = turn_index
    runtime.on_pose(make_pose(start_x, start_y))

    runtime._control_step()  # gear-change stop
    runtime._control_step()  # reverse command

    hardware.set_deck_lift.assert_called_with(True, True)
    hardware.set_blade.assert_called_with(False)
    assert hardware.drive.call_args.args[0] < 0


def test_progress_does_not_oscillate_back_across_gear_change_cusp():
    hardware = MagicMock()
    executive = MissionExecutive(hardware)
    store = ControlStore()
    store.update_settings({"deck_lift_settle_ms": 0})
    runtime = MissionRuntime(executive, hardware, store)
    waypoints = [
        TractorWaypoint(0.0, 0.0, "turn", -1),
        TractorWaypoint(1.0, 0.0, "turn", -1),
        TractorWaypoint(0.95, 0.0, "mow", 1),
        TractorWaypoint(2.0, 0.0, "mow", 1),
    ]
    runtime._route_waypoints = waypoints
    runtime._route_utm = [(point.x, point.y) for point in waypoints]
    runtime._trajectory = ContinuousTrajectory(waypoints)
    runtime._trajectory_s = 1.03
    runtime._route_index = 2
    runtime._last_gear = 1
    runtime.on_pose(make_pose(0.98, 0.0, heading=0.0))
    executive.start_mission("test-zone")

    runtime._control_step()

    assert runtime._trajectory_s >= 1.03
    assert runtime._route_index == 2
    assert hardware.drive.call_args.args[0] > 0
    hardware.set_deck_lift.assert_not_called()


def test_progress_is_frozen_while_mower_decks_settle():
    hardware = MagicMock()
    executive = MissionExecutive(hardware)
    runtime = MissionRuntime(executive, hardware, ControlStore())
    waypoints = [
        TractorWaypoint(0.0, 0.0, "mow", 1),
        TractorWaypoint(2.0, 0.0, "mow", 1),
    ]
    runtime._route_waypoints = waypoints
    runtime._route_utm = [(point.x, point.y) for point in waypoints]
    runtime._trajectory = ContinuousTrajectory(waypoints)
    runtime._trajectory_s = 0.0
    runtime._route_index = 1
    runtime._deck_transition_until = time.monotonic() + 1.0
    runtime.on_pose(make_pose(1.0, 0.0, speed=0.0))
    executive.start_mission("test-zone")

    runtime._control_step()

    assert runtime._trajectory_s == 0.0
    assert hardware.drive.call_args.args == (0.0, 0.0)


def test_progress_catches_up_when_vehicle_is_ahead_on_the_same_lane():
    hardware = MagicMock()
    executive = MissionExecutive(hardware)
    runtime = MissionRuntime(executive, hardware, ControlStore())
    waypoints = [
        TractorWaypoint(0.0, 0.0, "mow", 1),
        TractorWaypoint(2.0, 0.0, "mow", 1),
    ]
    runtime._route_waypoints = waypoints
    runtime._route_utm = [(point.x, point.y) for point in waypoints]
    runtime._trajectory = ContinuousTrajectory(waypoints)
    runtime._trajectory_s = 0.0
    runtime._route_index = 1
    runtime.on_pose(make_pose(0.55, 0.0, speed=0.5))
    executive.start_mission("test-zone")

    runtime._control_step()

    assert runtime._trajectory_s >= 0.5


def test_new_mission_clears_previous_lines_before_planning():
    hardware = MagicMock()
    runtime = MissionRuntime(
        MissionExecutive(hardware), hardware, ControlStore(),
    )
    runtime._actual_track_utm = [(1.0, 2.0), (2.0, 3.0)]
    runtime._actual_mown_segments_utm = [[(1.0, 2.0), (2.0, 3.0)]]
    runtime._route_utm = [(1.0, 2.0), (2.0, 3.0)]
    runtime._route_waypoints = [
        TractorWaypoint(1.0, 2.0), TractorWaypoint(2.0, 3.0),
    ]
    runtime._route_visual["geometry"]["coordinates"] = [[11.0, 48.0]]
    runtime.plan_zone = MagicMock(side_effect=ValueError("planning failed"))

    with pytest.raises(ValueError, match="planning failed"):
        runtime.start_mission({"id": "new-zone"})

    assert runtime._actual_track_utm == []
    assert runtime._actual_mown_segments_utm == []
    assert runtime._route_utm == []
    assert runtime._route_waypoints == []
    assert runtime._route_visual["geometry"]["coordinates"] == []


def test_return_home_reaches_docking_state_at_home_position():
    fix = make_fix()
    hardware = MagicMock()
    executive = MissionExecutive(hardware)
    store = ControlStore()
    home_x, home_y = fix.utm_x + 2.0, fix.utm_y
    home_lat, home_lon = utm.to_latlon(
        home_x, home_y, fix.utm_zone_number, fix.utm_zone_letter,
    )
    store.set_home({"lat": home_lat, "lon": home_lon, "name": "Dock"})
    runtime = MissionRuntime(executive, hardware, store)
    runtime.on_gps_fix(fix)
    runtime.on_pose(make_pose(fix.utm_x + 1.0, fix.utm_y + 1.0))
    runtime.start_mission(square_zone(fix))
    runtime.return_home()
    runtime.on_pose(make_pose(home_x, home_y))

    runtime._control_step()

    assert executive.state == MowerState.DOCKING
    hardware.drive.assert_called_with(0.0, 0.0)


def test_return_home_requires_persisted_station():
    fix = make_fix()
    executive = MissionExecutive()
    runtime = MissionRuntime(executive, MagicMock(), ControlStore())
    runtime.on_pose(make_pose(fix.utm_x + 1.0, fix.utm_y + 1.0))
    runtime.start_mission(square_zone(fix))

    with pytest.raises(ValueError, match="Ladestation"):
        runtime.return_home()


def test_dense_waypoint_is_reached_when_crossing_target_plane():
    runtime = MissionRuntime(MissionExecutive(), MagicMock(), ControlStore())
    pose = make_pose(1.05, 0.08)

    assert runtime._waypoint_reached(pose, (0.0, 0.0), (1.0, 0.0)) is True


def test_waypoint_is_not_skipped_when_cross_track_error_is_large():
    runtime = MissionRuntime(MissionExecutive(), MagicMock(), ControlStore())
    pose = make_pose(1.05, 1.0)

    assert runtime._waypoint_reached(pose, (0.0, 0.0), (1.0, 0.0)) is False
