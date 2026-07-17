import json

from mower.api.control_store import ControlStore


def test_store_persists_zones_settings_home_and_events(tmp_path):
    path = tmp_path / "control.json"
    store = ControlStore(path)
    zone = store.add_item("zones", {"name": "Nord", "type": "mowing"})
    connection = store.add_item("connections", {"name": "Weg Nord", "to_zone_id": zone["id"]})
    store.update_settings({"speed_kmh": 1.2})
    store.set_home({"lat": 48.1, "lon": 11.2, "name": "Dock"})
    store.add_event("warning", "test", "Test event")

    loaded = ControlStore(path)
    assert loaded.get_item("zones", zone["id"])["name"] == "Nord"
    assert loaded.get_item("connections", connection["id"])["name"] == "Weg Nord"
    assert loaded.settings()["speed_kmh"] == 1.2
    assert loaded.get_home()["name"] == "Dock"
    assert loaded.list_items("events")[0]["code"] == "test"
    assert json.loads(path.read_text(encoding="utf-8"))["version"] == 1


def test_store_rejects_unknown_setting_keys():
    store = ControlStore()
    result = store.update_settings({"speed_kmh": 1.1, "unknown": "ignored"})
    assert result["speed_kmh"] == 1.1
    assert "unknown" not in result


def test_mower_footprint_visibility_is_persisted():
    store = ControlStore()
    assert store.settings()["show_mower_footprints"] is True
    assert store.update_settings({"show_mower_footprints": False})[
        "show_mower_footprints"
    ] is False


def test_turn_radius_defaults_to_25_cm_and_is_bounded_to_ui_range():
    store = ControlStore()
    assert store.settings()["turn_radius_cm"] == 25
    assert store.update_settings({"turn_radius_cm": 2})["turn_radius_cm"] == 10
    assert store.update_settings({"turn_radius_cm": 180})["turn_radius_cm"] == 100


def test_mowing_turn_radius_defaults_to_50_cm_and_stays_above_turn_radius():
    store = ControlStore()
    assert store.settings()["mowing_turn_radius_cm"] == 50
    assert store.update_settings({"mowing_turn_radius_cm": 20})[
        "mowing_turn_radius_cm"
    ] == 30
    assert store.update_settings({"turn_radius_cm": 60})[
        "mowing_turn_radius_cm"
    ] == 65


def test_headland_margin_is_the_single_physical_reserve_and_has_five_cm_minimum():
    store = ControlStore()
    assert store.settings()["headland_margin_cm"] == 5
    assert store.update_settings({"headland_margin_cm": 0})[
        "headland_margin_cm"
    ] == 5
    assert store.update_settings({"headland_margin_cm": 35})[
        "headland_margin_cm"
    ] == 35


def test_mowing_width_and_lane_spacing_are_separate_and_lane_is_bounded():
    store = ControlStore()
    assert store.settings()["mowing_width_cm"] == 60
    settings = store.update_settings({"mowing_width_cm": 55, "lane_width_cm": 48})
    assert settings["mowing_width_cm"] == 55
    assert settings["lane_width_cm"] == 48
    assert settings["mowing_width_cm"] - settings["lane_width_cm"] == 7

    settings = store.update_settings({"mowing_width_cm": 40})
    assert settings["mowing_width_cm"] == 40
    assert settings["lane_width_cm"] == 40


def test_delete_item_returns_whether_item_existed():
    store = ControlStore()
    zone = store.add_item("zones", {"name": "A"})
    assert store.delete_item("zones", zone["id"]) is True
    assert store.delete_item("zones", zone["id"]) is False


def test_home_can_be_cleared():
    store = ControlStore()
    store.set_home({"lat": 48.1, "lon": 11.2, "name": "Dock"})
    assert store.clear_home() is True
    assert store.get_home() is None
    assert store.clear_home() is False


def test_fields2cover_settings_are_persisted_validated_and_bounded():
    store = ControlStore()

    settings = store.update_settings({
        "planner_engine": "fields2cover",
        "fields2cover_decomposition": "trapezoidal",
        "fields2cover_route_order": "spiral",
        "fields2cover_spiral_size": 99,
        "fields2cover_path_type": "reeds_shepp_cc",
        "fields2cover_headland_mode": "manual",
        "fields2cover_headland_width_cm": 200,
        "fields2cover_headland_width_factor": 0.5,
        "fields2cover_angle_deg": 240,
        "fields2cover_split_angle_deg": -20,
        "fields2cover_max_diff_curvature": 2,
        "fields2cover_minimum_mainland_coverage_percent": 10,
    })

    assert settings["planner_engine"] == "fields2cover"
    assert settings["fields2cover_decomposition"] == "trapezoidal"
    assert settings["fields2cover_route_order"] == "spiral"
    assert settings["fields2cover_spiral_size"] == 20
    assert settings["fields2cover_path_type"] == "reeds_shepp_cc"
    assert settings["fields2cover_headland_mode"] == "manual"
    assert settings["fields2cover_headland_width_cm"] == 150
    assert settings["fields2cover_headland_width_factor"] == 1
    assert settings["fields2cover_angle_deg"] == 180
    assert settings["fields2cover_split_angle_deg"] == 0
    assert settings["fields2cover_max_diff_curvature"] == 1
    assert settings["fields2cover_minimum_mainland_coverage_percent"] == 80

    rejected = store.update_settings({
        "planner_engine": "not-a-planner",
        "fields2cover_path_type": "teleport",
        "fields2cover_headland_mode": "oversized",
    })
    assert rejected["planner_engine"] == "fields2cover"
    assert rejected["fields2cover_path_type"] == "reeds_shepp_cc"
    assert rejected["fields2cover_headland_mode"] == "manual"


def test_plan_cache_is_compressed_persistent_and_separate_from_control_json(tmp_path):
    path = tmp_path / "control.json"
    key = "a" * 64
    payload = {"schema": 1, "waypoints": [[1.0, 2.0, "mow", 1, None]]}

    store = ControlStore(path)
    store.put_plan_cache(key, payload)

    assert ControlStore(path).get_plan_cache(key) == payload
    assert list((tmp_path / "plan-cache").glob("*.json.gz"))
    assert not path.exists() or "waypoints" not in path.read_text(encoding="utf-8")


def test_plan_cache_rejects_invalid_keys_and_can_delete_entries():
    store = ControlStore()
    store.put_plan_cache("b" * 64, {"schema": 1})

    assert store.get_plan_cache("b" * 64) == {"schema": 1}
    assert store.get_plan_cache("../escape") is None
    store.delete_plan_cache("b" * 64)
    assert store.get_plan_cache("b" * 64) is None
