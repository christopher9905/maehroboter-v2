# tests/api/test_app.py
import pytest
import pytest_asyncio
from httpx import AsyncClient, ASGITransport
from mower.executive.mission_executive import MissionExecutive, MowerState
from mower.api.app import create_app


@pytest.fixture
def executive():
    return MissionExecutive()


@pytest.fixture
def app(executive):
    return create_app(executive)


@pytest.mark.asyncio
async def test_get_state_returns_idle(app):
    async with AsyncClient(transport=ASGITransport(app=app), base_url="http://test") as client:
        resp = await client.get("/api/state")
    assert resp.status_code == 200
    assert resp.json()["state"] == "IDLE"


@pytest.mark.asyncio
async def test_mission_start(app, executive):
    async with AsyncClient(transport=ASGITransport(app=app), base_url="http://test") as client:
        resp = await client.post("/api/mission/start")
    assert resp.status_code == 200
    assert resp.json()["state"] == "MOWING"
    assert executive.state == MowerState.MOWING


@pytest.mark.asyncio
async def test_mission_stop(app, executive):
    executive.start_mission()
    async with AsyncClient(transport=ASGITransport(app=app), base_url="http://test") as client:
        resp = await client.post("/api/mission/stop")
    assert resp.status_code == 200
    assert executive.state == MowerState.RETURNING


@pytest.mark.asyncio
async def test_mission_start_from_non_idle_returns_409(app, executive):
    executive.start_mission()  # already MOWING
    async with AsyncClient(transport=ASGITransport(app=app), base_url="http://test") as client:
        resp = await client.post("/api/mission/start")
    assert resp.status_code == 409


@pytest.mark.asyncio
async def test_teach_in_start(app, executive):
    async with AsyncClient(transport=ASGITransport(app=app), base_url="http://test") as client:
        resp = await client.post("/api/teach-in/start")
    assert resp.status_code == 200
    assert executive.state == MowerState.TEACH_IN


@pytest.mark.asyncio
async def test_teach_in_stop(app, executive):
    executive.start_teach_in()
    async with AsyncClient(transport=ASGITransport(app=app), base_url="http://test") as client:
        resp = await client.post("/api/teach-in/stop")
    assert resp.status_code == 200
    assert executive.state == MowerState.IDLE


@pytest.mark.asyncio
async def test_reset_from_error(app, executive):
    executive.start_mission()
    executive.on_lift()
    assert executive.state == MowerState.ERROR
    async with AsyncClient(transport=ASGITransport(app=app), base_url="http://test") as client:
        resp = await client.post("/api/reset")
    assert resp.status_code == 200
    assert executive.state == MowerState.IDLE


@pytest.mark.asyncio
async def test_reset_from_non_error_is_noop(app, executive):
    async with AsyncClient(transport=ASGITransport(app=app), base_url="http://test") as client:
        resp = await client.post("/api/reset")
    assert resp.status_code == 200
    assert executive.state == MowerState.IDLE


@pytest.mark.asyncio
async def test_root_returns_html(app):
    async with AsyncClient(transport=ASGITransport(app=app), base_url="http://test") as client:
        resp = await client.get("/")
    assert resp.status_code == 200
    assert "text/html" in resp.headers["content-type"]


@pytest.mark.asyncio
async def test_status_endpoint(app, executive):
    async with AsyncClient(transport=ASGITransport(app=app), base_url="http://test") as client:
        resp = await client.get("/api/status")
    assert resp.status_code == 200
    data = resp.json()
    assert "state" in data
    assert "error_reason" in data
