"""FastAPI application factory for the Mähroboter V2 control API.

Usage:
    executive = MissionExecutive(hardware_interface=hw)
    app = create_app(executive)
    uvicorn.run(app, host="0.0.0.0", port=8080)
"""
import asyncio
import contextlib
import logging
from pathlib import Path
from typing import Optional

from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, HTMLResponse
from fastapi.staticfiles import StaticFiles

from mower.executive.mission_executive import MissionExecutive, MowerState
from mower.api.ws_manager import ConnectionManager

logger = logging.getLogger(__name__)

_STATIC_DIR = Path(__file__).parent / "static"


def create_app(executive: MissionExecutive) -> FastAPI:
    """Create and return the configured FastAPI application."""
    manager = ConnectionManager()
    _loop: Optional[asyncio.AbstractEventLoop] = None

    @contextlib.asynccontextmanager
    async def lifespan(app: FastAPI):
        nonlocal _loop
        _loop = asyncio.get_running_loop()   # safe: called from async context
        yield
        _loop = None

    app = FastAPI(title="Mähroboter V2 Control", version="0.5.0", lifespan=lifespan)

    def _on_state_change(old: MowerState, new: MowerState) -> None:
        """Called from a worker thread; bridge to the asyncio event loop."""
        loop = _loop
        if loop is not None and loop.is_running():
            asyncio.run_coroutine_threadsafe(
                manager.broadcast({"type": "state", "state": new.name, "previous": old.name}),
                loop,
            )

    executive.on_state_change = _on_state_change

    if _STATIC_DIR.exists():
        app.mount("/static", StaticFiles(directory=str(_STATIC_DIR)), name="static")

    @app.get("/", include_in_schema=False, response_model=None)
    async def root() -> FileResponse | HTMLResponse:
        index = _STATIC_DIR / "index.html"
        if index.exists():
            return FileResponse(str(index))
        return HTMLResponse("<html><body>Mähroboter V2</body></html>")

    @app.get("/api/state")
    async def get_state() -> dict[str, str]:
        return {"state": executive.state.name}

    @app.get("/api/status")
    async def get_status() -> dict[str, str]:
        return {
            "state": executive.state.name,
            "error_reason": executive.error_reason,
        }

    @app.post("/api/mission/start")
    async def mission_start() -> dict[str, str]:
        if executive.state != MowerState.IDLE:
            raise HTTPException(
                status_code=409,
                detail=f"Cannot start mission from state {executive.state.name}",
            )
        executive.start_mission()
        return {"state": executive.state.name}

    @app.post("/api/mission/stop")
    async def mission_stop() -> dict[str, str]:
        executive.stop_mission()
        return {"state": executive.state.name}

    @app.post("/api/teach-in/start")
    async def teach_in_start() -> dict[str, str]:
        executive.start_teach_in()
        return {"state": executive.state.name}

    @app.post("/api/teach-in/stop")
    async def teach_in_stop() -> dict[str, str]:
        executive.stop_teach_in()
        return {"state": executive.state.name}

    @app.post("/api/reset")
    async def reset_error() -> dict[str, str]:
        executive.reset_error()
        return {"state": executive.state.name}

    @app.websocket("/ws")
    async def websocket_endpoint(ws: WebSocket) -> None:
        await manager.connect(ws)
        try:
            await ws.send_json({"type": "state", "state": executive.state.name})
            while True:
                await ws.receive_text()
        except WebSocketDisconnect:
            manager.disconnect(ws)

    return app
