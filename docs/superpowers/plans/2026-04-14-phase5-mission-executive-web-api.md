# Phase 5: Mission Executive & Web API — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Implement the Mission Executive state machine (IDLE→MOWING→ERROR→…) and a FastAPI backend with REST endpoints and WebSocket telemetry, served alongside a minimal Leaflet.js frontend.

**Architecture:** `mower/executive/` holds the pure-Python state machine — no HTTP, no hardware dependency (injectable). `mower/api/` holds the FastAPI application factory, a WebSocket connection manager, and a `static/` directory with the HTML/JS frontend. The API wires the executive's `on_state_change` callback to WebSocket broadcasts using `asyncio.run_coroutine_threadsafe` to bridge the sync safety-guard threads to the async API event loop.

**Tech Stack:** Python 3.11+, FastAPI, uvicorn, websockets, httpx (test client), pytest-asyncio, APScheduler (dependency only — scheduling left for Phase 7), HTML/JS + Leaflet.js CDN

**Scope note:** The Mission Executive and the Web API are independent subsystems. If needed, they can be executed as two separate plans (Tasks 1–2 first, then Task 3).

---

## File Structure

```
mower/
├── executive/
│   ├── __init__.py              Create — empty package marker
│   └── mission_executive.py     Create — MowerState enum, MissionExecutive class
└── api/
    ├── __init__.py              Create — empty package marker
    ├── app.py                   Create — FastAPI factory, REST routes, WebSocket endpoint
    ├── ws_manager.py            Create — ConnectionManager for WebSocket broadcast
    └── static/
        ├── index.html           Create — Leaflet.js map + control panel
        └── app.js               Create — WebSocket client + UI logic
tests/
├── executive/
│   ├── __init__.py              Create
│   └── test_mission_executive.py Create — 18 tests
└── api/
    ├── __init__.py              Create
    └── test_app.py              Create — 10 tests
requirements.txt                 Modify — add fastapi, uvicorn, httpx, pytest-asyncio
```

### Interface contracts

**`mission_executive.py`**
```python
LOW_BATTERY_SOC: int = 20
OBSTACLE_TIMEOUT_S: float = 60.0
MAX_AVOIDANCE_ATTEMPTS: int = 3

class MowerState(Enum): IDLE, TEACH_IN, MOWING, OBSTACLE_AVOIDANCE, RETURNING, DOCKING, CHARGING, ERROR

class MissionExecutive:
    # Commands (from Web UI)
    def start_teach_in(self): ...
    def stop_teach_in(self): ...
    def start_mission(self): ...
    def stop_mission(self): ...
    def reset_error(self): ...

    # Safety gate inputs (called by guards/sensors)
    def on_lift(self): ...
    def on_tilt(self, reading): ...
    def on_geofence_violation(self, pose): ...

    # Mission events
    def on_obstacle_detected(self, detections): ...
    def on_obstacle_cleared(self): ...
    def on_battery_low(self, soc: int): ...
    def on_dock_success(self): ...
    def on_charge_started(self): ...
    def on_charge_complete(self): ...

    @property
    def state(self) -> MowerState: ...
    @property
    def error_reason(self) -> str: ...

    on_state_change: Optional[Callable[[MowerState, MowerState], None]]
```

**`app.py`**
```python
def create_app(executive: MissionExecutive) -> FastAPI: ...
# Routes: GET /api/state, POST /api/mission/start, POST /api/mission/stop,
#         POST /api/teach-in/start, POST /api/teach-in/stop, POST /api/reset
# WebSocket: /ws  — broadcasts {"type":"state","state":"MOWING","previous":"IDLE"}
# Static: GET / → index.html
```

---

## Task 1: Mission Executive State Machine

**Files:**
- Create: `mower/executive/__init__.py`
- Create: `mower/executive/mission_executive.py`
- Create: `tests/executive/__init__.py`
- Create: `tests/executive/test_mission_executive.py`

### Background

The state machine has eight states. Thread safety: all transitions run under `_lock`. Callbacks (`on_state_change`) are fired OUTSIDE the lock to prevent deadlock. The hardware interface is injected so tests run without hardware.

State transitions:
```
IDLE        →  TEACH_IN           on start_teach_in()
TEACH_IN    →  IDLE               on stop_teach_in()
IDLE        →  MOWING             on start_mission()
MOWING      →  OBSTACLE_AVOIDANCE on on_obstacle_detected() [attempt ≤ MAX]
MOWING      →  ERROR              on on_obstacle_detected() [attempt > MAX]
OBSTACLE_AVOIDANCE → MOWING       on on_obstacle_cleared()
OBSTACLE_AVOIDANCE → ERROR        on _obstacle_timeout() (60 s)
MOWING / OBSTACLE_AVOIDANCE → RETURNING  on stop_mission() or on_battery_low()
RETURNING   →  DOCKING            on on_dock_success()
DOCKING     →  CHARGING           on on_charge_started()
CHARGING    →  IDLE               on on_charge_complete()
ANY_ACTIVE  →  ERROR              on on_lift() / on_tilt() / on_geofence_violation()
ERROR       →  IDLE               on reset_error()
```

Commands from an invalid state are silently ignored (idempotent).

- [ ] **Step 1: Write the failing tests**

```python
# tests/executive/test_mission_executive.py
import threading
import pytest
from unittest.mock import MagicMock
from mower.executive.mission_executive import (
    MissionExecutive, MowerState,
    LOW_BATTERY_SOC, MAX_AVOIDANCE_ATTEMPTS,
)


def _executive(hw=None):
    return MissionExecutive(hardware_interface=hw)


class TestMowerStateTransitions:
    def test_initial_state_is_idle(self):
        ex = _executive()
        assert ex.state == MowerState.IDLE

    def test_start_teach_in_from_idle(self):
        ex = _executive()
        ex.start_teach_in()
        assert ex.state == MowerState.TEACH_IN

    def test_stop_teach_in_returns_to_idle(self):
        ex = _executive()
        ex.start_teach_in()
        ex.stop_teach_in()
        assert ex.state == MowerState.IDLE

    def test_start_mission_from_idle(self):
        ex = _executive()
        ex.start_mission()
        assert ex.state == MowerState.MOWING

    def test_stop_mission_from_mowing_goes_returning(self):
        ex = _executive()
        ex.start_mission()
        ex.stop_mission()
        assert ex.state == MowerState.RETURNING

    def test_on_obstacle_detected_from_mowing(self):
        ex = _executive()
        ex.start_mission()
        ex.on_obstacle_detected([])
        assert ex.state == MowerState.OBSTACLE_AVOIDANCE

    def test_on_obstacle_cleared_returns_to_mowing(self):
        ex = _executive()
        ex.start_mission()
        ex.on_obstacle_detected([])
        ex.on_obstacle_cleared()
        assert ex.state == MowerState.MOWING

    def test_obstacle_timeout_goes_error(self):
        ex = _executive()
        ex.start_mission()
        ex.on_obstacle_detected([])
        assert ex.state == MowerState.OBSTACLE_AVOIDANCE
        ex._obstacle_timeout()  # simulate 60s expiry
        assert ex.state == MowerState.ERROR

    def test_max_avoidance_attempts_goes_error(self):
        ex = _executive()
        ex.start_mission()
        for _ in range(MAX_AVOIDANCE_ATTEMPTS):
            ex.on_obstacle_detected([])
            ex.on_obstacle_cleared()
        # Next detection after max attempts → ERROR
        ex.on_obstacle_detected([])
        assert ex.state == MowerState.ERROR

    def test_on_battery_low_from_mowing(self):
        ex = _executive()
        ex.start_mission()
        ex.on_battery_low(15)
        assert ex.state == MowerState.RETURNING

    def test_dock_charge_cycle(self):
        ex = _executive()
        ex.start_mission()
        ex.stop_mission()               # → RETURNING
        ex.on_dock_success()            # → DOCKING
        ex.on_charge_started()          # → CHARGING
        ex.on_charge_complete()         # → IDLE
        assert ex.state == MowerState.IDLE

    def test_on_lift_from_mowing_goes_error(self):
        hw = MagicMock()
        ex = _executive(hw)
        ex.start_mission()
        ex.on_lift()
        assert ex.state == MowerState.ERROR
        hw.estop.assert_called_once()
        hw.set_blade.assert_called_once_with(False)

    def test_on_tilt_from_mowing_goes_error(self):
        ex = _executive()
        ex.start_mission()
        reading = MagicMock()
        reading.pitch_deg = 35.0
        reading.roll_deg = 5.0
        ex.on_tilt(reading)
        assert ex.state == MowerState.ERROR

    def test_on_geofence_violation_goes_error(self):
        ex = _executive()
        ex.start_mission()
        pose = MagicMock()
        pose.utm_x, pose.utm_y = 1000.0, 2000.0
        ex.on_geofence_violation(pose)
        assert ex.state == MowerState.ERROR

    def test_reset_error_from_error_goes_idle(self):
        ex = _executive()
        ex.start_mission()
        ex.on_lift()
        assert ex.state == MowerState.ERROR
        ex.reset_error()
        assert ex.state == MowerState.IDLE

    def test_reset_error_from_idle_is_noop(self):
        ex = _executive()
        ex.reset_error()  # no-op, should not raise
        assert ex.state == MowerState.IDLE

    def test_start_mission_when_already_mowing_is_noop(self):
        ex = _executive()
        ex.start_mission()
        ex.start_mission()  # should be ignored
        assert ex.state == MowerState.MOWING

    def test_on_state_change_callback_fires(self):
        ex = _executive()
        transitions = []
        ex.on_state_change = lambda old, new: transitions.append((old, new))
        ex.start_mission()
        assert transitions == [(MowerState.IDLE, MowerState.MOWING)]
```

- [ ] **Step 2: Run tests to verify they fail**

```bash
cd "/Users/christopher/Desktop/Modellbau Projekte/Mähroboter V2"
python3 -m pytest tests/executive/test_mission_executive.py -v
```

Expected: `ModuleNotFoundError: No module named 'mower.executive'`

- [ ] **Step 3: Create package markers**

`mower/executive/__init__.py` — empty  
`tests/executive/__init__.py` — empty

- [ ] **Step 4: Implement `mower/executive/mission_executive.py`**

```python
"""Mission Executive state machine for the autonomous lawn mower.

All state transitions are thread-safe (protected by _lock).
on_state_change callback is always fired OUTSIDE the lock to prevent deadlock.
"""
import logging
import threading
from enum import Enum, auto
from typing import Callable, Optional

logger = logging.getLogger(__name__)

LOW_BATTERY_SOC: int = 20          # percent
OBSTACLE_TIMEOUT_S: float = 60.0   # seconds before avoidance gives up
MAX_AVOIDANCE_ATTEMPTS: int = 3    # per mowing session


class MowerState(Enum):
    IDLE = auto()
    TEACH_IN = auto()
    MOWING = auto()
    OBSTACLE_AVOIDANCE = auto()
    RETURNING = auto()
    DOCKING = auto()
    CHARGING = auto()
    ERROR = auto()


# States considered "active" — safety events trigger ERROR from these
_ACTIVE_STATES = frozenset({
    MowerState.MOWING,
    MowerState.OBSTACLE_AVOIDANCE,
    MowerState.RETURNING,
    MowerState.DOCKING,
    MowerState.CHARGING,
    MowerState.TEACH_IN,
})


class MissionExecutive:
    """State machine for mission management.

    Inject hardware_interface (HardwareInterface) to enable ESTOP on error.
    Leave it None for unit tests.
    """

    def __init__(self, hardware_interface=None):
        self._hw = hardware_interface
        self._state = MowerState.IDLE
        self._error_reason: str = ""
        self._avoidance_attempts: int = 0
        self._obstacle_timer: Optional[threading.Timer] = None
        self._lock = threading.Lock()
        self.on_state_change: Optional[Callable[[MowerState, MowerState], None]] = None

    # ── Properties ──────────────────────────────────────────────────────────

    @property
    def state(self) -> MowerState:
        with self._lock:
            return self._state

    @property
    def error_reason(self) -> str:
        with self._lock:
            return self._error_reason

    # ── Internal helpers ─────────────────────────────────────────────────────

    def _set_state(self, new_state: MowerState, reason: str = "") -> MowerState:
        """Set state and return old state. MUST be called WITH _lock held."""
        old = self._state
        self._state = new_state
        self._error_reason = reason if new_state == MowerState.ERROR else ""
        return old

    def _go_error(self, reason: str) -> MowerState:
        """Transition to ERROR, fire ESTOP. MUST be called WITH _lock held."""
        old = self._set_state(MowerState.ERROR, reason)
        logger.error("→ ERROR: %s", reason)
        if self._hw:
            self._hw.estop()
            self._hw.set_blade(False)
        return old

    def _cancel_obstacle_timer(self):
        """Cancel pending obstacle timeout timer. MUST be called WITH _lock held."""
        if self._obstacle_timer:
            self._obstacle_timer.cancel()
            self._obstacle_timer = None

    def _notify(self, old: MowerState, new: MowerState):
        """Fire on_state_change callback OUTSIDE the lock."""
        if old != new and self.on_state_change:
            self.on_state_change(old, new)

    # ── Commands (from Web UI / scheduler) ───────────────────────────────────

    def start_teach_in(self):
        with self._lock:
            if self._state != MowerState.IDLE:
                return
            old = self._set_state(MowerState.TEACH_IN)
        self._notify(old, MowerState.TEACH_IN)

    def stop_teach_in(self):
        with self._lock:
            if self._state != MowerState.TEACH_IN:
                return
            old = self._set_state(MowerState.IDLE)
        self._notify(old, MowerState.IDLE)

    def start_mission(self):
        with self._lock:
            if self._state != MowerState.IDLE:
                return
            self._avoidance_attempts = 0
            old = self._set_state(MowerState.MOWING)
        self._notify(old, MowerState.MOWING)

    def stop_mission(self):
        with self._lock:
            if self._state not in (MowerState.MOWING, MowerState.OBSTACLE_AVOIDANCE):
                return
            self._cancel_obstacle_timer()
            old = self._set_state(MowerState.RETURNING)
        self._notify(old, MowerState.RETURNING)

    def reset_error(self):
        with self._lock:
            if self._state != MowerState.ERROR:
                return
            old = self._set_state(MowerState.IDLE)
        self._notify(old, MowerState.IDLE)

    # ── Safety gate inputs ───────────────────────────────────────────────────

    def on_lift(self):
        with self._lock:
            if self._state not in _ACTIVE_STATES:
                return
            self._cancel_obstacle_timer()
            old = self._go_error("Deck lift detected")
        self._notify(old, MowerState.ERROR)

    def on_tilt(self, reading):
        with self._lock:
            if self._state not in _ACTIVE_STATES:
                return
            self._cancel_obstacle_timer()
            old = self._go_error(
                f"Tilt limit exceeded: pitch={reading.pitch_deg:.1f}°, roll={reading.roll_deg:.1f}°"
            )
        self._notify(old, MowerState.ERROR)

    def on_geofence_violation(self, pose):
        with self._lock:
            if self._state not in _ACTIVE_STATES:
                return
            self._cancel_obstacle_timer()
            old = self._go_error(
                f"Geofence violation at ({pose.utm_x:.2f}, {pose.utm_y:.2f})"
            )
        self._notify(old, MowerState.ERROR)

    # ── Mission events ───────────────────────────────────────────────────────

    def on_obstacle_detected(self, detections):
        with self._lock:
            if self._state != MowerState.MOWING:
                return
            self._avoidance_attempts += 1
            if self._avoidance_attempts > MAX_AVOIDANCE_ATTEMPTS:
                old = self._go_error(
                    f"Obstacle avoidance failed after {MAX_AVOIDANCE_ATTEMPTS} attempts"
                )
                new = MowerState.ERROR
            else:
                old = self._set_state(MowerState.OBSTACLE_AVOIDANCE)
                new = MowerState.OBSTACLE_AVOIDANCE
                timer = threading.Timer(OBSTACLE_TIMEOUT_S, self._obstacle_timeout)
                timer.daemon = True
                timer.start()
                self._obstacle_timer = timer
        self._notify(old, new)

    def on_obstacle_cleared(self):
        with self._lock:
            if self._state != MowerState.OBSTACLE_AVOIDANCE:
                return
            self._cancel_obstacle_timer()
            old = self._set_state(MowerState.MOWING)
        self._notify(old, MowerState.MOWING)

    def _obstacle_timeout(self):
        """Called by threading.Timer after OBSTACLE_TIMEOUT_S. Callable from tests."""
        with self._lock:
            if self._state != MowerState.OBSTACLE_AVOIDANCE:
                return
            self._obstacle_timer = None
            old = self._go_error(f"Obstacle avoidance timeout ({OBSTACLE_TIMEOUT_S:.0f} s)")
        self._notify(old, MowerState.ERROR)

    def on_battery_low(self, soc: int):
        with self._lock:
            if self._state not in (MowerState.MOWING, MowerState.OBSTACLE_AVOIDANCE):
                return
            self._cancel_obstacle_timer()
            old = self._set_state(MowerState.RETURNING)
        self._notify(old, MowerState.RETURNING)

    def on_dock_success(self):
        with self._lock:
            if self._state != MowerState.RETURNING:
                return
            old = self._set_state(MowerState.DOCKING)
        self._notify(old, MowerState.DOCKING)

    def on_charge_started(self):
        with self._lock:
            if self._state != MowerState.DOCKING:
                return
            old = self._set_state(MowerState.CHARGING)
        self._notify(old, MowerState.CHARGING)

    def on_charge_complete(self):
        with self._lock:
            if self._state != MowerState.CHARGING:
                return
            old = self._set_state(MowerState.IDLE)
        self._notify(old, MowerState.IDLE)
```

- [ ] **Step 5: Run tests to verify they pass**

```bash
python3 -m pytest tests/executive/test_mission_executive.py -v
```

Expected: all 18 tests PASS

- [ ] **Step 6: Run full suite**

```bash
python3 -m pytest tests/ --tb=short -q
```

Expected: 143 + 18 = 161 passed, 0 failed

- [ ] **Step 7: Commit**

```bash
git add mower/executive/__init__.py mower/executive/mission_executive.py \
        tests/executive/__init__.py tests/executive/test_mission_executive.py
git commit -m "feat(executive): Mission Executive state machine with 8 states"
```

---

## Task 2: FastAPI Backend + WebSocket

**Files:**
- Create: `mower/api/__init__.py`
- Create: `mower/api/ws_manager.py`
- Create: `mower/api/app.py`
- Create: `tests/api/__init__.py`
- Create: `tests/api/test_app.py`
- Modify: `requirements.txt` — add fastapi, uvicorn, httpx, pytest-asyncio

### Background

The FastAPI app is created via a factory `create_app(executive)` so the `MissionExecutive` instance is injectable for tests. Routes operate on the executive synchronously (the executive's lock handles thread safety). State change broadcasts to WebSocket clients use `asyncio.run_coroutine_threadsafe` when the executive fires `on_state_change` from a worker thread.

- [ ] **Step 1: Update `requirements.txt` and install**

Append to `requirements.txt`:
```
fastapi>=0.110.0
uvicorn[standard]>=0.27.0
httpx>=0.27.0
pytest-asyncio>=0.23.0
```

Install:
```bash
pip install "fastapi>=0.110.0" "uvicorn[standard]>=0.27.0" "httpx>=0.27.0" "pytest-asyncio>=0.23.0"
```

- [ ] **Step 2: Write the failing tests**

```python
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
```

- [ ] **Step 3: Run tests to verify they fail**

```bash
python3 -m pytest tests/api/test_app.py -v
```

Expected: `ModuleNotFoundError: No module named 'mower.api'`

- [ ] **Step 4: Create package markers**

`mower/api/__init__.py` — empty  
`tests/api/__init__.py` — empty  
`mower/api/static/` — create directory

- [ ] **Step 5: Implement `mower/api/ws_manager.py`**

```python
"""WebSocket connection manager — broadcast messages to all connected clients."""
import json
import logging
from typing import Any

from fastapi import WebSocket

logger = logging.getLogger(__name__)


class ConnectionManager:
    def __init__(self):
        self._connections: list[WebSocket] = []

    async def connect(self, ws: WebSocket):
        await ws.accept()
        self._connections.append(ws)
        logger.info("WebSocket connected (%d total)", len(self._connections))

    def disconnect(self, ws: WebSocket):
        if ws in self._connections:
            self._connections.remove(ws)
        logger.info("WebSocket disconnected (%d remaining)", len(self._connections))

    async def broadcast(self, payload: dict[str, Any]):
        """Send JSON payload to all connected clients; remove broken connections."""
        message = json.dumps(payload)
        dead: list[WebSocket] = []
        for ws in list(self._connections):
            try:
                await ws.send_text(message)
            except Exception as exc:
                logger.warning("WebSocket send failed: %s", exc)
                dead.append(ws)
        for ws in dead:
            self.disconnect(ws)
```

- [ ] **Step 6: Implement `mower/api/app.py`**

```python
"""FastAPI application factory for the Mähroboter V2 control API.

Usage:
    executive = MissionExecutive(hardware_interface=hw)
    app = create_app(executive)
    uvicorn.run(app, host="0.0.0.0", port=8080)
"""
import asyncio
import logging
from pathlib import Path

from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles

from mower.executive.mission_executive import MissionExecutive, MowerState
from mower.api.ws_manager import ConnectionManager

logger = logging.getLogger(__name__)

_STATIC_DIR = Path(__file__).parent / "static"


def create_app(executive: MissionExecutive) -> FastAPI:
    """Create and return the configured FastAPI application.

    Args:
        executive: MissionExecutive instance. Its on_state_change callback is
                   wired to broadcast state updates to all WebSocket clients.
    """
    app = FastAPI(title="Mähroboter V2 Control", version="0.5.0")
    manager = ConnectionManager()

    # ── Wire executive → WebSocket broadcast ─────────────────────────────────

    def _on_state_change(old: MowerState, new: MowerState):
        """Called from a worker thread; bridge to the asyncio event loop."""
        try:
            loop = asyncio.get_event_loop()
        except RuntimeError:
            return
        if loop.is_running():
            asyncio.run_coroutine_threadsafe(
                manager.broadcast({"type": "state", "state": new.name, "previous": old.name}),
                loop,
            )

    executive.on_state_change = _on_state_change

    # ── Static files ─────────────────────────────────────────────────────────

    if _STATIC_DIR.exists():
        app.mount("/static", StaticFiles(directory=str(_STATIC_DIR)), name="static")

    @app.get("/", include_in_schema=False)
    async def root():
        index = _STATIC_DIR / "index.html"
        if index.exists():
            return FileResponse(str(index))
        return FileResponse(str(Path(__file__).parent / "static" / "index.html"))

    # ── REST endpoints ────────────────────────────────────────────────────────

    @app.get("/api/state")
    async def get_state():
        return {"state": executive.state.name}

    @app.get("/api/status")
    async def get_status():
        return {
            "state": executive.state.name,
            "error_reason": executive.error_reason,
        }

    @app.post("/api/mission/start")
    async def mission_start():
        if executive.state != MowerState.IDLE:
            raise HTTPException(
                status_code=409,
                detail=f"Cannot start mission from state {executive.state.name}",
            )
        executive.start_mission()
        return {"state": executive.state.name}

    @app.post("/api/mission/stop")
    async def mission_stop():
        executive.stop_mission()
        return {"state": executive.state.name}

    @app.post("/api/teach-in/start")
    async def teach_in_start():
        executive.start_teach_in()
        return {"state": executive.state.name}

    @app.post("/api/teach-in/stop")
    async def teach_in_stop():
        executive.stop_teach_in()
        return {"state": executive.state.name}

    @app.post("/api/reset")
    async def reset_error():
        executive.reset_error()
        return {"state": executive.state.name}

    # ── WebSocket ─────────────────────────────────────────────────────────────

    @app.websocket("/ws")
    async def websocket_endpoint(ws: WebSocket):
        await manager.connect(ws)
        try:
            # Send current state on connect
            await ws.send_json({"type": "state", "state": executive.state.name})
            while True:
                await ws.receive_text()  # keep connection alive; client sends heartbeat
        except WebSocketDisconnect:
            manager.disconnect(ws)

    return app
```

- [ ] **Step 7: Create minimal `mower/api/static/index.html`**

```html
<!DOCTYPE html>
<html lang="de">
<head>
  <meta charset="UTF-8"/>
  <meta name="viewport" content="width=device-width, initial-scale=1.0"/>
  <title>Mähroboter V2</title>
  <link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css"/>
  <style>
    * { box-sizing: border-box; margin: 0; padding: 0; }
    body { font-family: system-ui, sans-serif; background: #1a1a2e; color: #eee; display: flex; flex-direction: column; height: 100vh; }
    header { background: #16213e; padding: .75rem 1rem; display: flex; align-items: center; gap: 1rem; }
    header h1 { font-size: 1.1rem; }
    #status-badge { padding: .3rem .7rem; border-radius: 1rem; font-size: .85rem; font-weight: bold; background: #444; }
    #status-badge.IDLE      { background: #2d6a4f; }
    #status-badge.MOWING    { background: #1b7f2e; }
    #status-badge.ERROR     { background: #c0392b; }
    #status-badge.RETURNING { background: #e67e22; }
    main { display: flex; flex: 1; overflow: hidden; }
    #map { flex: 1; }
    aside { width: 260px; background: #16213e; padding: 1rem; display: flex; flex-direction: column; gap: .75rem; overflow-y: auto; }
    .btn { padding: .6rem 1rem; border: none; border-radius: .4rem; cursor: pointer; font-size: .9rem; font-weight: bold; width: 100%; }
    .btn-green  { background: #27ae60; color: #fff; }
    .btn-red    { background: #c0392b; color: #fff; }
    .btn-orange { background: #e67e22; color: #fff; }
    .btn-blue   { background: #2980b9; color: #fff; }
    .btn:disabled { opacity: .4; cursor: not-allowed; }
    .section-title { font-size: .75rem; text-transform: uppercase; letter-spacing: .08em; color: #aaa; margin-bottom: .25rem; }
    .telemetry-row { display: flex; justify-content: space-between; font-size: .85rem; padding: .2rem 0; border-bottom: 1px solid #2a2a4a; }
    #error-log { font-size: .8rem; color: #f39c12; white-space: pre-wrap; max-height: 100px; overflow-y: auto; }
  </style>
</head>
<body>
<header>
  <h1>🌿 Mähroboter V2</h1>
  <span id="status-badge">IDLE</span>
  <span id="ws-indicator" title="WebSocket">⚪</span>
</header>
<main>
  <div id="map"></div>
  <aside>
    <div>
      <p class="section-title">Mission</p>
      <button class="btn btn-green" id="btn-start">▶ Start</button>
      <button class="btn btn-red"   id="btn-stop" style="margin-top:.4rem">⏹ Stop</button>
    </div>
    <div>
      <p class="section-title">Teach-In</p>
      <button class="btn btn-blue" id="btn-teach-start">📍 Aufzeichnen</button>
      <button class="btn btn-orange" id="btn-teach-stop" style="margin-top:.4rem">✓ Beenden</button>
    </div>
    <div>
      <p class="section-title">Fehler</p>
      <button class="btn btn-orange" id="btn-reset">↺ Reset</button>
      <div id="error-log" style="margin-top:.4rem"></div>
    </div>
    <div>
      <p class="section-title">Telemetrie</p>
      <div class="telemetry-row"><span>Akku</span><span id="tel-soc">—</span></div>
      <div class="telemetry-row"><span>GPS</span><span id="tel-gps">—</span></div>
      <div class="telemetry-row"><span>Speed</span><span id="tel-speed">—</span></div>
    </div>
  </aside>
</main>
<script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
<script src="/static/app.js"></script>
</body>
</html>
```

- [ ] **Step 8: Create `mower/api/static/app.js`**

```javascript
// Mähroboter V2 — Web UI client
'use strict';

// ── Map setup ─────────────────────────────────────────────────────────────────
const map = L.map('map').setView([48.5, 11.0], 18);
L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
  attribution: '© OpenStreetMap contributors',
  maxZoom: 22,
}).addTo(map);

const robotMarker = L.marker([48.5, 11.0], {
  title: 'Mähroboter',
}).addTo(map);

// ── State ─────────────────────────────────────────────────────────────────────
let currentState = 'IDLE';

function updateUI(state) {
  currentState = state;
  const badge = document.getElementById('status-badge');
  badge.textContent = state;
  badge.className = '';
  badge.classList.add(state);

  document.getElementById('btn-start').disabled       = state !== 'IDLE';
  document.getElementById('btn-stop').disabled        = !['MOWING', 'OBSTACLE_AVOIDANCE'].includes(state);
  document.getElementById('btn-teach-start').disabled = state !== 'IDLE';
  document.getElementById('btn-teach-stop').disabled  = state !== 'TEACH_IN';
  document.getElementById('btn-reset').disabled       = state !== 'ERROR';
}

// ── WebSocket ─────────────────────────────────────────────────────────────────
function connectWS() {
  const ws = new WebSocket(`ws://${location.host}/ws`);
  const indicator = document.getElementById('ws-indicator');

  ws.onopen  = () => { indicator.textContent = '🟢'; };
  ws.onclose = () => { indicator.textContent = '🔴'; setTimeout(connectWS, 3000); };
  ws.onerror = () => { indicator.textContent = '🟠'; };

  ws.onmessage = (evt) => {
    const msg = JSON.parse(evt.data);
    if (msg.type === 'state') updateUI(msg.state);
    if (msg.type === 'pose') {
      robotMarker.setLatLng([msg.lat, msg.lon]);
      document.getElementById('tel-speed').textContent = (msg.speed_mps * 3.6).toFixed(1) + ' km/h';
    }
    if (msg.type === 'soc')  document.getElementById('tel-soc').textContent  = msg.soc_percent + ' %';
    if (msg.type === 'gps')  document.getElementById('tel-gps').textContent  = msg.fix_quality === 4 ? 'RTK Fix' : msg.fix_quality === 5 ? 'RTK Float' : 'No Fix';
    if (msg.type === 'error') {
      const log = document.getElementById('error-log');
      log.textContent = msg.reason + '\n' + log.textContent;
    }
  };
}

connectWS();

// ── Button handlers ───────────────────────────────────────────────────────────
async function apiPost(path) {
  const resp = await fetch(path, { method: 'POST' });
  if (!resp.ok) {
    const err = await resp.json().catch(() => ({ detail: resp.statusText }));
    document.getElementById('error-log').textContent = err.detail || resp.statusText;
  }
  return resp;
}

document.getElementById('btn-start').addEventListener('click',       () => apiPost('/api/mission/start'));
document.getElementById('btn-stop').addEventListener('click',        () => apiPost('/api/mission/stop'));
document.getElementById('btn-teach-start').addEventListener('click', () => apiPost('/api/teach-in/start'));
document.getElementById('btn-teach-stop').addEventListener('click',  () => apiPost('/api/teach-in/stop'));
document.getElementById('btn-reset').addEventListener('click',       () => apiPost('/api/reset'));

updateUI('IDLE');
```

- [ ] **Step 9: Run tests to verify they pass**

```bash
python3 -m pytest tests/api/test_app.py -v
```

Expected: all 10 tests PASS

- [ ] **Step 10: Run full suite**

```bash
python3 -m pytest tests/ --tb=short -q
```

Expected: 161 + 10 = 171 passed, 0 failed

- [ ] **Step 11: Commit**

```bash
git add mower/api/__init__.py mower/api/app.py mower/api/ws_manager.py \
        mower/api/static/index.html mower/api/static/app.js \
        tests/api/__init__.py tests/api/test_app.py \
        requirements.txt
git commit -m "feat(api): FastAPI REST + WebSocket backend with Leaflet.js frontend"
```

---

## Acceptance Criteria

After all tasks are complete:

- [ ] `python3 -m pytest tests/ -q` → **171 passed, 0 failed**
- [ ] `mower/executive/mission_executive.py` — all 8 states, all transitions, thread-safe callbacks
- [ ] `mower/api/app.py` — `create_app(executive)` factory, 6 REST endpoints, WebSocket /ws, static file serving
- [ ] `mower/api/ws_manager.py` — broadcast sends to all connections, removes dead ones
- [ ] `mower/api/static/index.html` + `app.js` — served at `/`, Leaflet.js map, mission control buttons wired to API
- [ ] All new modules importable without hardware
- [ ] `fastapi>=0.110.0`, `uvicorn[standard]>=0.27.0`, `httpx>=0.27.0`, `pytest-asyncio>=0.23.0` in `requirements.txt`
- [ ] Server can be started manually: `uvicorn mower.api.app:app --reload` (after wiring executive in __main__)
