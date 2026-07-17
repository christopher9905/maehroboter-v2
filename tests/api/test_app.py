# tests/api/test_app.py
import pytest
import pytest_asyncio
from unittest.mock import MagicMock
from httpx import AsyncClient, ASGITransport
from mower.executive.mission_executive import MissionExecutive, MowerState
from mower.api.app import create_app


@pytest.fixture
def executive():
    return MissionExecutive()


@pytest.fixture
def app(executive):
    return create_app(executive)


async def test_get_state_returns_idle(app):
    async with AsyncClient(transport=ASGITransport(app=app), base_url="http://test") as client:
        resp = await client.get("/api/state")
    assert resp.status_code == 200
    assert resp.json()["state"] == "IDLE"


async def test_mission_start(app, executive):
    async with AsyncClient(transport=ASGITransport(app=app), base_url="http://test") as client:
        resp = await client.post("/api/mission/start")
    assert resp.status_code == 200
    assert resp.json()["state"] == "MOWING"
    assert executive.state == MowerState.MOWING


async def test_mission_stop(app, executive):
    executive.start_mission()
    async with AsyncClient(transport=ASGITransport(app=app), base_url="http://test") as client:
        resp = await client.post("/api/mission/stop")
    assert resp.status_code == 200
    assert executive.state == MowerState.RETURNING


async def test_mission_start_from_non_idle_returns_409(app, executive):
    executive.start_mission()  # already MOWING
    async with AsyncClient(transport=ASGITransport(app=app), base_url="http://test") as client:
        resp = await client.post("/api/mission/start")
    assert resp.status_code == 409


async def test_mission_start_from_charging_departs_dock(app, executive):
    executive.start_mission()
    executive.stop_mission()
    executive.on_dock_success()
    executive.on_charge_started()
    assert executive.state == MowerState.CHARGING

    async with AsyncClient(transport=ASGITransport(app=app), base_url="http://test") as client:
        resp = await client.post("/api/mission/start")

    assert resp.status_code == 200
    assert resp.json()["state"] == "MOWING"
    assert executive.state == MowerState.MOWING


async def test_teach_in_start(app, executive):
    async with AsyncClient(transport=ASGITransport(app=app), base_url="http://test") as client:
        resp = await client.post("/api/teach-in/start")
    assert resp.status_code == 200
    assert executive.state == MowerState.TEACH_IN


async def test_teach_in_stop(app, executive):
    executive.start_teach_in()
    async with AsyncClient(transport=ASGITransport(app=app), base_url="http://test") as client:
        resp = await client.post("/api/teach-in/stop")
    assert resp.status_code == 200
    assert executive.state == MowerState.IDLE


async def test_reset_from_error(app, executive):
    executive.start_mission()
    executive.on_lift()
    assert executive.state == MowerState.ERROR
    async with AsyncClient(transport=ASGITransport(app=app), base_url="http://test") as client:
        resp = await client.post("/api/reset")
    assert resp.status_code == 200
    assert executive.state == MowerState.IDLE


async def test_reset_from_non_error_is_noop(app, executive):
    async with AsyncClient(transport=ASGITransport(app=app), base_url="http://test") as client:
        resp = await client.post("/api/reset")
    assert resp.status_code == 200
    assert executive.state == MowerState.IDLE


async def test_root_returns_html(app):
    async with AsyncClient(transport=ASGITransport(app=app), base_url="http://test") as client:
        resp = await client.get("/")
    assert resp.status_code == 200
    assert "text/html" in resp.headers["content-type"]


async def test_status_endpoint(app, executive):
    async with AsyncClient(transport=ASGITransport(app=app), base_url="http://test") as client:
        resp = await client.get("/api/status")
    assert resp.status_code == 200
    data = resp.json()
    assert "state" in data
    assert "error_reason" in data


class TestDockingManagerLifecycle:
    """create_app optionally owns a DockingManager's start()/stop() lifecycle."""

    async def test_docking_manager_started_on_lifespan_startup(self, executive):
        docking_manager = MagicMock()
        soc_source = lambda: 50
        charging_source = lambda: False
        app = create_app(
            executive, docking_manager=docking_manager,
            soc_source=soc_source, charging_source=charging_source,
        )
        async with app.router.lifespan_context(app):
            docking_manager.start.assert_called_once_with(soc_source, charging_source)

    async def test_docking_manager_stopped_on_lifespan_shutdown(self, executive):
        docking_manager = MagicMock()
        app = create_app(executive, docking_manager=docking_manager)
        async with app.router.lifespan_context(app):
            docking_manager.stop.assert_not_called()
        docking_manager.stop.assert_called_once()

    async def test_docking_manager_defaults_have_safe_sources(self, executive):
        # No soc_source/charging_source given — must not raise at startup.
        docking_manager = MagicMock()
        app = create_app(executive, docking_manager=docking_manager)
        async with app.router.lifespan_context(app):
            args, _ = docking_manager.start.call_args
            assert args[0]() == 0
            assert args[1]() is False

    async def test_app_works_without_docking_manager(self, app):
        async with app.router.lifespan_context(app):
            async with AsyncClient(transport=ASGITransport(app=app), base_url="http://test") as client:
                resp = await client.get("/api/state")
        assert resp.status_code == 200
