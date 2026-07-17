import asyncio
import threading
from unittest.mock import MagicMock

import pytest
from httpx import ASGITransport, AsyncClient

from mower.api.app import create_app
from mower.api.control_store import ControlStore
from mower.api.runtime_state import RuntimeState
from mower.executive.mission_executive import MissionExecutive


@pytest.fixture
def store():
    return ControlStore()


@pytest.fixture
def runtime():
    state = RuntimeState(hardware_connected=True)
    state.update("gps", {"fix_quality": 4, "label": "RTK Fix"})
    state.update("battery", {"soc_percent": 80})
    state.update("safety", {"raining": False, "lifted": False, "geofence_ok": True})
    return state


def add_zone(store):
    return store.add_item("zones", {
        "name": "Nord", "type": "mowing",
        "geometry": {"type": "Polygon", "coordinates": [[[11.0, 48.0], [11.1, 48.0], [11.1, 48.1], [11.0, 48.0]]]},
    })


async def test_persistent_zone_schedule_settings_and_home_api(store, runtime):
    ex = MissionExecutive()
    app = create_app(ex, store=store, runtime=runtime)
    async with AsyncClient(transport=ASGITransport(app=app), base_url="http://test") as client:
        zone = (await client.post("/api/zones", json={"name": "Nord", "type": "mowing"})).json()
        assert (await client.get("/api/zones")).json()[0]["id"] == zone["id"]
        schedule = (await client.post("/api/schedules", json={
            "zone_id": zone["id"], "days": [0, 2],
            "window_start": "09:00", "window_end": "15:00",
        })).json()
        assert schedule["zone_id"] == zone["id"]
        skipped = (await client.post(f"/api/schedules/{schedule['id']}/skip-next")).json()
        assert skipped["skip_next"] is True
        settings = (await client.put("/api/settings", json={"speed_kmh": 1.3})).json()
        assert settings["speed_kmh"] == 1.3
        home = (await client.put("/api/home", json={"lat": 48.2, "lon": 11.3, "name": "Dock"})).json()
        assert home["name"] == "Dock"


async def test_map_connections_are_persistent_validated_and_reported(store, runtime):
    first = add_zone(store)
    second = store.add_item("zones", {
        "name": "Süd", "type": "mowing",
        "geometry": {"type": "Polygon", "coordinates": [[[11.2, 48.0], [11.3, 48.0], [11.3, 48.1], [11.2, 48.0]]]},
    })
    app = create_app(MissionExecutive(), store=store, runtime=runtime)
    async with AsyncClient(transport=ASGITransport(app=app), base_url="http://test") as client:
        missing_home = await client.post("/api/connections", json={
            "name": "Dock zu Nord", "type": "dock_link", "to_zone_id": first["id"],
            "geometry": {"type": "LineString", "coordinates": [[10.99, 47.99], [11.0, 48.0]]},
        })
        assert missing_home.status_code == 422
        await client.put("/api/home", json={"lat": 47.99, "lon": 10.99, "name": "Dock"})
        dock_link = (await client.post("/api/connections", json={
            "name": "Dock zu Nord", "type": "dock_link", "to_zone_id": first["id"],
            "corridor_width_cm": 150,
            "geometry": {"type": "LineString", "coordinates": [[10.99, 47.99], [11.0, 48.0]]},
        })).json()
        zone_link = (await client.post("/api/connections", json={
            "name": "Nord zu Süd", "type": "zone_link", "from_zone_id": first["id"],
            "to_zone_id": second["id"],
            "geometry": {"type": "LineString", "coordinates": [[11.1, 48.05], [11.2, 48.05]]},
        })).json()
        assert dock_link["corridor_width_cm"] == 150
        assert len((await client.get("/api/connections")).json()) == 2
        topology = (await client.get("/api/map/topology")).json()
        assert topology["valid"] is True
        assert topology["connection_count"] == 2
        blocked = await client.delete(f"/api/zones/{first['id']}")
        assert blocked.status_code == 409
        assert (await client.delete(f"/api/connections/{zone_link['id']}")).json()["deleted"] is True


async def test_zone_api_rejects_self_crossing_polygon(store, runtime):
    app = create_app(MissionExecutive(), store=store, runtime=runtime)
    async with AsyncClient(transport=ASGITransport(app=app), base_url="http://test") as client:
        response = await client.post("/api/zones", json={
            "name": "Kaputt", "type": "mowing",
            "geometry": {"type": "Polygon", "coordinates": [[
                [11.0, 48.0], [11.1, 48.1], [11.1, 48.0], [11.0, 48.1], [11.0, 48.0],
            ]]},
        })
    assert response.status_code == 422
    assert "ungültig" in response.json()["detail"]


async def test_zone_specific_mission_and_interlocks(store, runtime):
    ex = MissionExecutive()
    zone = add_zone(store)
    app = create_app(ex, store=store, runtime=runtime,
                     enforce_mission_safety=True, require_confirmations=True)
    async with AsyncClient(transport=ASGITransport(app=app), base_url="http://test") as client:
        missing = await client.post("/api/mission/start", json={"confirmed": True})
        assert missing.status_code == 409
        started = await client.post("/api/mission/start", json={"zone_id": zone["id"], "confirmed": True})
        assert started.status_code == 200
        assert ex.active_zone_id == zone["id"]
        paused = await client.post("/api/mission/pause")
        assert paused.json()["state"] == "PAUSED"
        resumed = await client.post("/api/mission/resume")
        assert resumed.json()["state"] == "MOWING"
        returned = await client.post("/api/mission/return-home")
        assert returned.json()["state"] == "RETURNING"


async def test_bad_gps_blocks_real_mission(store, runtime):
    zone = add_zone(store)
    runtime.update("gps", {"fix_quality": 5, "label": "RTK Float"})
    app = create_app(MissionExecutive(), store=store, runtime=runtime, enforce_mission_safety=True)
    async with AsyncClient(transport=ASGITransport(app=app), base_url="http://test") as client:
        response = await client.post("/api/mission/start", json={"zone_id": zone["id"]})
    assert response.status_code == 409
    assert "RTK Fix" in response.json()["detail"]


async def test_stale_geofence_result_does_not_block_new_selected_zone(store, runtime):
    zone = add_zone(store)
    runtime.update("safety", {"geofence_ok": False})
    ex = MissionExecutive()
    mission_runtime = MagicMock()
    mission_runtime.start_mission.side_effect = lambda selected: ex.start_mission(selected["id"])
    app = create_app(
        ex, store=store, runtime=runtime, mission_runtime=mission_runtime,
        enforce_mission_safety=True,
    )

    async with AsyncClient(transport=ASGITransport(app=app), base_url="http://test") as client:
        response = await client.post("/api/mission/start", json={"zone_id": zone["id"]})

    assert response.status_code == 200
    assert runtime.snapshot()["safety"]["geofence_ok"] is None


async def test_current_geofence_result_still_blocks_resume(store, runtime):
    zone = add_zone(store)
    ex = MissionExecutive()
    ex.start_mission(zone["id"])
    ex.pause_mission()
    runtime.update("safety", {"geofence_ok": False})
    app = create_app(ex, store=store, runtime=runtime, enforce_mission_safety=True)

    async with AsyncClient(transport=ASGITransport(app=app), base_url="http://test") as client:
        response = await client.post("/api/mission/resume")

    assert response.status_code == 409
    assert response.json()["detail"] == "Mission gesperrt: Geofence nicht gültig"


async def test_confirmed_geofence_override_resumes_soft_stopped_mission(store, runtime):
    zone = add_zone(store)
    ex = MissionExecutive()
    ex.start_mission(zone["id"])
    pose = MagicMock(utm_x=1000.0, utm_y=2000.0)
    ex.on_geofence_violation(pose)
    runtime.update("safety", {"geofence_ok": False})
    app = create_app(
        ex,
        store=store,
        runtime=runtime,
        enforce_mission_safety=True,
        require_confirmations=True,
    )

    async with AsyncClient(transport=ASGITransport(app=app), base_url="http://test") as client:
        unconfirmed = await client.post(
            "/api/mission/resume-geofence-override", json={"confirmed": False},
        )
        resumed = await client.post(
            "/api/mission/resume-geofence-override", json={"confirmed": True},
        )

    assert unconfirmed.status_code == 400
    assert resumed.status_code == 200
    assert resumed.json()["state"] == "MOWING"
    assert resumed.json()["geofence_override_active"] is True
    assert ex.geofence_override_active is True
    assert runtime.snapshot()["safety"]["geofence_override_active"] is True
    assert any(event["code"] == "geofence_override" for event in store.list_items("events"))


async def test_mission_runtime_plans_selected_zone_and_exposes_route(store, runtime):
    zone = add_zone(store)
    ex = MissionExecutive()
    mission_runtime = MagicMock()
    route = {"type": "Feature", "properties": {
                 "waypoint_count": 8,
                 "route_metrics": {"plan_cache_hit": True},
             },
             "geometry": {"type": "LineString", "coordinates": [[11.0, 48.0], [11.1, 48.1]]}}
    mission_runtime.route_geojson.return_value = route
    def start_mission(selected):
        ex.start_mission(selected["id"])
        return route
    mission_runtime.start_mission.side_effect = start_mission
    app = create_app(ex, store=store, runtime=runtime, mission_runtime=mission_runtime,
                     enforce_mission_safety=True)

    async with AsyncClient(transport=ASGITransport(app=app), base_url="http://test") as client:
        started = await client.post("/api/mission/start", json={"zone_id": zone["id"]})
        planned = await client.get("/api/mission/route")

    assert started.status_code == 200
    assert started.json()["plan_cache_hit"] is True
    mission_runtime.start_mission.assert_called_once_with(zone)
    assert planned.json() == route


async def test_status_stays_responsive_while_mission_route_is_planned(store, runtime):
    zone = add_zone(store)
    ex = MissionExecutive()
    planning_started = threading.Event()
    release_planning = threading.Event()
    mission_runtime = MagicMock()

    def slow_start(selected):
        planning_started.set()
        assert release_planning.wait(timeout=2.0)
        ex.start_mission(selected["id"])

    mission_runtime.start_mission.side_effect = slow_start
    app = create_app(ex, store=store, runtime=runtime, mission_runtime=mission_runtime,
                     enforce_mission_safety=True)

    async with AsyncClient(transport=ASGITransport(app=app), base_url="http://test") as client:
        start_task = asyncio.create_task(client.post(
            "/api/mission/start", json={"zone_id": zone["id"]},
        ))
        assert await asyncio.to_thread(planning_started.wait, 1.0)
        status = await asyncio.wait_for(client.get("/api/status"), timeout=0.5)
        assert status.status_code == 200
        assert status.json()["state"] == "IDLE"
        release_planning.set()
        started = await start_task

    assert started.status_code == 200
    assert started.json()["state"] == "MOWING"


async def test_return_home_delegates_to_runtime_route(store, runtime):
    zone = add_zone(store)
    ex = MissionExecutive()
    mission_runtime = MagicMock()
    mission_runtime.start_mission.side_effect = lambda selected: ex.start_mission(selected["id"])
    return_route = {
        "type": "Feature",
        "properties": {"waypoint_count": 2, "mode": "return_home"},
        "geometry": {"type": "LineString", "coordinates": [[11.0, 48.0], [11.1, 48.1]]},
    }
    mission_runtime.return_home.side_effect = lambda: (ex.return_to_dock() or return_route)
    app = create_app(ex, store=store, runtime=runtime, mission_runtime=mission_runtime,
                     enforce_mission_safety=True)

    async with AsyncClient(transport=ASGITransport(app=app), base_url="http://test") as client:
        await client.post("/api/mission/start", json={"zone_id": zone["id"]})
        response = await client.post("/api/mission/return-home")

    assert response.status_code == 200
    assert response.json()["state"] == "RETURNING"
    assert response.json()["route"]["properties"]["mode"] == "return_home"
    mission_runtime.return_home.assert_called_once_with()


async def test_server_teach_in_geometry_is_returned_to_ui(store, runtime):
    ex = MissionExecutive()
    mission_runtime = MagicMock()
    geometry = {"type": "Polygon", "coordinates": [[[11.0, 48.0], [11.1, 48.0],
                                                                [11.0, 48.1], [11.0, 48.0]]]}
    mission_runtime.start_teach_in.side_effect = lambda **_: ex.start_teach_in()
    mission_runtime.stop_teach_in.side_effect = lambda: (
        ex.stop_teach_in() or {"geometry": geometry, "point_count": 4, "closed": True}
    )
    app = create_app(ex, store=store, runtime=runtime, mission_runtime=mission_runtime)

    async with AsyncClient(transport=ASGITransport(app=app), base_url="http://test") as client:
        started = await client.post("/api/teach-in/start", json={"reference": "right"})
        assert started.json()["state"] == "TEACH_IN"
        assert started.json()["reference"] == "right"
        stopped = await client.post("/api/teach-in/stop")

    assert stopped.json()["geometry"] == geometry
    assert stopped.json()["point_count"] == 4
    mission_runtime.start_teach_in.assert_called_once_with(reference="right")


async def test_teach_in_correction_api_delegates_to_shared_runtime(store, runtime):
    ex = MissionExecutive()
    mission_runtime = MagicMock()
    status = {"state": "TEACH_IN", "recording": True, "suspended": True,
              "point_count": 6, "length_m": 7.0,
              "geometry": {"type": "LineString", "coordinates": []},
              "correction_target": [11.0, 48.0], "distance_to_target_m": 3.0}
    mission_runtime.teach_in_status.return_value = status
    mission_runtime.undo_teach_in.return_value = {**status, "removed_distance_m": 3.0}
    mission_runtime.continue_teach_in_here.return_value = {**status, "suspended": False}
    app = create_app(ex, store=store, runtime=runtime, mission_runtime=mission_runtime)

    async with AsyncClient(transport=ASGITransport(app=app), base_url="http://test") as client:
        current = await client.get("/api/teach-in/status")
        undone = await client.post("/api/teach-in/undo", json={"distance_m": 3.0})
        continued = await client.post("/api/teach-in/continue-here")

    assert current.json()["suspended"] is True
    assert undone.json()["removed_distance_m"] == 3.0
    assert continued.json()["suspended"] is False
    mission_runtime.undo_teach_in.assert_called_once_with(3.0)


async def test_manual_drive_is_limited_and_estop_is_latched(store, runtime):
    hw = MagicMock()
    ex = MissionExecutive(hw)
    app = create_app(ex, store=store, runtime=runtime, hardware=hw)
    async with AsyncClient(transport=ASGITransport(app=app), base_url="http://test") as client:
        drive = await client.post("/api/manual/drive", json={"speed": 0.9, "steering": 20})
        assert drive.json()["speed"] == 0.25
        hw.drive.assert_called_with(0.25, 20)
        limited_steering = await client.post(
            "/api/manual/drive", json={"speed": 0.1, "steering": 45},
        )
        assert limited_steering.json()["steering"] == 28
        hw.drive.assert_called_with(0.1, 28)
        stopped = await client.post("/api/estop", json={"confirmed": True})
        assert stopped.json()["state"] == "ERROR"
        hw.estop.assert_called_once()


async def test_manual_implement_controls_decks_and_confirms_blade(store, runtime):
    hw = MagicMock()
    hw.output_snapshot.return_value = {
        "front_deck_raised": False,
        "rear_deck_raised": False,
        "blade_enabled": False,
    }
    app = create_app(MissionExecutive(hw), store=store, runtime=runtime, hardware=hw)

    async with AsyncClient(transport=ASGITransport(app=app), base_url="http://test") as client:
        raised = await client.post(
            "/api/manual/implement", json={"front_raised": True},
        )
        unconfirmed = await client.post(
            "/api/manual/implement", json={"blade_enabled": True},
        )
        hw.output_snapshot.return_value = {
            "front_deck_raised": False,
            "rear_deck_raised": False,
            "blade_enabled": False,
        }
        blade = await client.post(
            "/api/manual/implement",
            json={"blade_enabled": True, "confirmed": True},
        )

    assert raised.status_code == 200
    hw.set_deck_lift.assert_called_with(True, False)
    assert unconfirmed.status_code == 400
    assert blade.status_code == 200
    hw.set_blade.assert_called_with(True)


async def test_manual_implement_lift_stops_running_blade(store, runtime):
    hw = MagicMock()
    hw.output_snapshot.return_value = {
        "front_deck_raised": False,
        "rear_deck_raised": False,
        "blade_enabled": True,
    }
    app = create_app(MissionExecutive(hw), store=store, runtime=runtime, hardware=hw)

    async with AsyncClient(transport=ASGITransport(app=app), base_url="http://test") as client:
        response = await client.post(
            "/api/manual/implement", json={"rear_raised": True},
        )

    assert response.status_code == 200
    hw.drive.assert_called_with(0.0, 0.0)
    hw.set_blade.assert_called_with(False)
    hw.set_deck_lift.assert_called_with(False, True)
    assert response.json()["blade_enabled"] is False


async def test_pin_protects_api(store, runtime):
    app = create_app(MissionExecutive(), store=store, runtime=runtime, pin="2468")
    async with AsyncClient(transport=ASGITransport(app=app), base_url="http://test") as client:
        assert (await client.get("/api/bootstrap")).status_code == 401
        assert (await client.post("/api/auth/login", json={"pin": "0000"})).status_code == 401
        assert (await client.post("/api/auth/login", json={"pin": "2468"})).status_code == 200
        assert (await client.get("/api/bootstrap")).status_code == 200


async def test_firmware_is_staged_but_not_flashed(tmp_path, store, runtime):
    app = create_app(MissionExecutive(), store=store, runtime=runtime,
                     firmware_stage_dir=tmp_path)
    async with AsyncClient(transport=ASGITransport(app=app), base_url="http://test") as client:
        response = await client.post("/api/maintenance/firmware/stage", content=b":10000000",
                                     headers={"X-Filename": "mower.hex", "X-MV2-Confirmed": "true"})
    assert response.status_code == 200
    assert response.json()["flashed"] is False
    assert (tmp_path / "mower.hex").read_bytes() == b":10000000"
