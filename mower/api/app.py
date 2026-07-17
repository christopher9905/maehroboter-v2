"""FastAPI control plane for the Mähroboter V2."""
from __future__ import annotations

import asyncio
import contextlib
import logging
import os
import secrets
import threading
from datetime import datetime
from pathlib import Path
from typing import Any, Callable, Optional

from fastapi import Body, Depends, FastAPI, HTTPException, Request, Response, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, HTMLResponse, PlainTextResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel, Field
from shapely.geometry import LineString, Polygon, shape
from shapely.validation import explain_validity

from mower.api.control_store import ControlStore
from mower.api.runtime_state import RuntimeState
from mower.api.ws_manager import ConnectionManager
from mower.executive.docking_manager import DockingManager
from mower.executive.mission_executive import MissionExecutive, MowerState
from mower.path.fields2cover_native import native_status as fields2cover_status

logger = logging.getLogger(__name__)

_STATIC_DIR = Path(__file__).parent / "static"
APP_VERSION = "0.6.0"


class MissionStart(BaseModel):
    zone_id: Optional[str] = None
    confirmed: bool = False


class ManualDrive(BaseModel):
    speed: float = Field(ge=-1.0, le=1.0)
    steering: float = Field(ge=-45.0, le=45.0)


class ManualImplement(BaseModel):
    front_raised: Optional[bool] = None
    rear_raised: Optional[bool] = None
    blade_enabled: Optional[bool] = None
    confirmed: bool = False


class ConfirmedAction(BaseModel):
    confirmed: bool = False
    action: Optional[str] = None


class PinLogin(BaseModel):
    pin: str


class HomePosition(BaseModel):
    lat: float = Field(ge=-90, le=90)
    lon: float = Field(ge=-180, le=180)
    name: str = "Ladestation"
    heading_deg: Optional[float] = Field(default=None, ge=0, lt=360)


class TeachUndo(BaseModel):
    distance_m: float = Field(default=3.0, ge=0.5, le=20.0)


class TeachStart(BaseModel):
    reference: str = Field(default="center", pattern="^(left|center|right)$")


def _validated_zone(values: dict[str, Any], *, partial: bool = False) -> dict[str, Any]:
    """Validate and normalise an editable mowing/no-go polygon."""
    clean = dict(values)
    if not partial or "name" in clean:
        name = str(clean.get("name", "")).strip()
        if not name:
            raise HTTPException(status_code=422, detail="Zonenname fehlt")
        clean["name"] = name[:80]
    if not partial or "type" in clean:
        zone_type = clean.get("type", "mowing")
        if zone_type not in ("mowing", "no_go"):
            raise HTTPException(status_code=422, detail="Ungültiger Zonentyp")
        clean["type"] = zone_type
    geometry = clean.get("geometry")
    if geometry is not None:
        try:
            polygon = shape(geometry)
        except (TypeError, ValueError, KeyError) as exc:
            raise HTTPException(status_code=422, detail="Zone enthält keine lesbare Geometrie") from exc
        if not isinstance(polygon, Polygon) or polygon.is_empty:
            raise HTTPException(status_code=422, detail="Zone muss ein gültiges Polygon sein")
        if not polygon.is_valid:
            raise HTTPException(
                status_code=422,
                detail=f"Zonenpolygon ist ungültig: {explain_validity(polygon)}",
            )
        if polygon.area <= 0:
            raise HTTPException(status_code=422, detail="Zone muss eine Fläche größer als null besitzen")
        clean["geometry"] = {
            "type": "Polygon",
            "coordinates": [[list(point) for point in polygon.exterior.coords]],
        }
    clean.setdefault("enabled", True)
    return clean


def _validated_connection(
    values: dict[str, Any],
    zones: list[dict[str, Any]],
    *,
    partial: bool = False,
    existing: Optional[dict[str, Any]] = None,
) -> dict[str, Any]:
    """Validate an operator-defined transit corridor between map objects."""
    clean = dict(values)
    combined = {**(existing or {}), **clean}
    if not partial or "name" in clean:
        name = str(combined.get("name", "")).strip()
        if not name:
            raise HTTPException(status_code=422, detail="Name des Verbindungswegs fehlt")
        clean["name"] = name[:80]
    connection_type = combined.get("type", "zone_link")
    if connection_type not in ("zone_link", "dock_link"):
        raise HTTPException(status_code=422, detail="Ungültiger Verbindungstyp")
    clean["type"] = connection_type
    mowing_ids = {z.get("id") for z in zones if z.get("type", "mowing") == "mowing"}
    from_id = combined.get("from_zone_id")
    to_id = combined.get("to_zone_id")
    if connection_type == "dock_link":
        from_id = None
        clean["from_zone_id"] = None
    elif not from_id or from_id not in mowing_ids:
        raise HTTPException(status_code=422, detail="Startzone des Verbindungswegs ist ungültig")
    if not to_id or to_id not in mowing_ids:
        raise HTTPException(status_code=422, detail="Zielzone des Verbindungswegs ist ungültig")
    if from_id and from_id == to_id:
        raise HTTPException(status_code=422, detail="Start- und Zielzone müssen verschieden sein")
    geometry = combined.get("geometry")
    if geometry is None:
        raise HTTPException(status_code=422, detail="Verbindungsweg benötigt eine Linien-Geometrie")
    try:
        line = shape(geometry)
    except (TypeError, ValueError, KeyError) as exc:
        raise HTTPException(status_code=422, detail="Verbindungsweg enthält keine lesbare Geometrie") from exc
    if not isinstance(line, LineString) or line.is_empty or len(line.coords) < 2 or line.length <= 0:
        raise HTTPException(status_code=422, detail="Verbindungsweg muss aus mindestens zwei Punkten bestehen")
    if not line.is_simple:
        raise HTTPException(status_code=422, detail="Verbindungsweg darf sich nicht selbst kreuzen")
    clean["geometry"] = {"type": "LineString", "coordinates": [list(point) for point in line.coords]}
    width = combined.get("corridor_width_cm", 180)
    try:
        clean["corridor_width_cm"] = max(30.0, min(1000.0, float(width)))
    except (TypeError, ValueError) as exc:
        raise HTTPException(status_code=422, detail="Korridorbreite ist ungültig") from exc
    clean.setdefault("bidirectional", True)
    clean.setdefault("enabled", True)
    return clean


def create_app(
    executive: MissionExecutive,
    docking_manager: Optional[DockingManager] = None,
    soc_source: Optional[Callable[[], int]] = None,
    charging_source: Optional[Callable[[], bool]] = None,
    *,
    store: Optional[ControlStore] = None,
    runtime: Optional[RuntimeState] = None,
    hardware=None,
    pin: Optional[str] = None,
    enforce_mission_safety: bool = False,
    require_confirmations: bool = False,
    tile_dir: Optional[Path | str] = None,
    managed_components: Optional[list[Any]] = None,
    firmware_stage_dir: Optional[Path | str] = None,
    mission_runtime: Any = None,
) -> FastAPI:
    """Build the control API.

    Tests may omit persistence/auth/hardware. The real entrypoint supplies all
    three and enables mission interlocks and confirmations.
    """
    manager = ConnectionManager()
    control_store = store or ControlStore()
    runtime_state = runtime or RuntimeState(hardware_connected=hardware is not None)
    auth_pin = pin if pin is not None else os.environ.get("MV2_UI_PIN", "")
    sessions: set[str] = set()
    tiles = Path(tile_dir) if tile_dir else None
    firmware_staging = Path(firmware_stage_dir) if firmware_stage_dir else None
    _loop: Optional[asyncio.AbstractEventLoop] = None
    _tasks: list[asyncio.Task] = []

    def _auth_required(request: Request) -> None:
        if not auth_pin:
            return
        token = request.cookies.get("mv2_session")
        if not token or token not in sessions:
            raise HTTPException(status_code=401, detail="PIN-Anmeldung erforderlich")

    def _zone_or_404(zone_id: str) -> dict[str, Any]:
        zone = control_store.get_item("zones", zone_id)
        if zone is None:
            raise HTTPException(status_code=404, detail="Zone nicht gefunden")
        return zone

    def _mission_block_reason(
        zone_id: Optional[str], *, check_active_geofence: bool = False,
    ) -> Optional[str]:
        if enforce_mission_safety and not zone_id:
            return "Bitte zuerst eine Mähzone auswählen"
        if zone_id:
            zone = control_store.get_item("zones", zone_id)
            if zone is None or zone.get("type", "mowing") != "mowing":
                return "Die ausgewählte Mähzone ist nicht verfügbar"
            if enforce_mission_safety and not zone.get("geometry"):
                return "Mission gesperrt: Die Zone besitzt noch keine aufgezeichnete Grenze"
        settings = control_store.settings()
        telemetry = runtime_state.snapshot()
        if enforce_mission_safety and not telemetry.get("hardware_connected"):
            return "Mission gesperrt: Fahrhardware nicht verbunden"
        gps = telemetry["gps"]
        safety = telemetry["safety"]
        battery = telemetry["battery"]
        if enforce_mission_safety and settings.get("require_rtk_fix") and gps.get("fix_quality") != 4:
            return "Mission gesperrt: RTK Fix fehlt"
        if settings.get("season_mode") == "off":
            return "Mission gesperrt: Saisonmodus ist ausgeschaltet"
        if not settings.get("weekend_mode", True) and datetime.now().weekday() >= 5:
            return "Mission gesperrt: Wochenendmodus ist deaktiviert"
        if settings.get("rain_enabled") and safety.get("raining") is True:
            return "Mission gesperrt: Regen erkannt"
        resume_at = safety.get("rain_resume_at")
        if settings.get("rain_enabled") and resume_at:
            try:
                if datetime.fromisoformat(resume_at) > datetime.now().astimezone():
                    return f"Mission gesperrt: Regenpause bis {datetime.fromisoformat(resume_at).astimezone().strftime('%H:%M')} Uhr"
            except (TypeError, ValueError):
                pass
        soc = battery.get("soc_percent")
        if soc is not None and soc <= int(settings.get("low_battery_soc", 20)):
            return "Mission gesperrt: Akkustand zu niedrig"
        if safety.get("lifted") is True:
            return "Mission gesperrt: Mähwerk angehoben"
        # geofence_ok describes the last GPS sample of the currently active
        # mission.  In IDLE/CHARGING it can still refer to an old zone and must
        # not prevent planning a newly selected zone.  Resume, on the other
        # hand, deliberately checks the active mission's latest result.
        if check_active_geofence and safety.get("geofence_ok") is False:
            return "Mission gesperrt: Geofence nicht gültig"
        return None

    def _within_quiet_hours(now: datetime, settings: dict[str, Any]) -> bool:
        current = now.strftime("%H:%M")
        start = str(settings.get("quiet_hours_start", "20:00"))
        end = str(settings.get("quiet_hours_end", "08:00"))
        return start <= current < end if start < end else current >= start or current < end

    async def _telemetry_loop() -> None:
        while True:
            await manager.broadcast({"type": "telemetry", **runtime_state.snapshot()})
            # Ten updates per second keep the map and output diagnostics fluid
            # while remaining far below the sensor/serial data rate.
            await asyncio.sleep(0.1)

    async def _scheduler_loop() -> None:
        """Start a due schedule once per local calendar day."""
        while True:
            try:
                now = datetime.now()
                hhmm = now.strftime("%H:%M")
                settings = control_store.settings()
                if _within_quiet_hours(now, settings):
                    await asyncio.sleep(30.0)
                    continue
                for schedule in control_store.list_items("schedules"):
                    if not schedule.get("enabled", True) or now.weekday() not in schedule.get("days", []):
                        continue
                    if schedule.get("last_run_date") == now.date().isoformat():
                        continue
                    start = schedule.get("window_start", schedule.get("start_time", "09:00"))
                    end = schedule.get("window_end", start)
                    if not (start <= hhmm <= end):
                        continue
                    if schedule.get("skip_next"):
                        control_store.update_item("schedules", schedule["id"], {
                            "skip_next": False, "last_run_date": now.date().isoformat(),
                        })
                        control_store.add_event("info", "schedule_skipped", "Geplante Mission übersprungen")
                        continue
                    if executive.state not in (MowerState.IDLE, MowerState.CHARGING):
                        continue
                    reason = _mission_block_reason(schedule.get("zone_id"))
                    if reason:
                        control_store.add_event("warning", "schedule_blocked", reason,
                                                "GPS, Wetter, Akku und Zone prüfen")
                        continue
                    if mission_runtime is not None:
                        zone = _zone_or_404(schedule["zone_id"])
                        mission_runtime.start_mission(zone)
                    else:
                        executive.start_mission(schedule.get("zone_id"))
                    control_store.update_item("schedules", schedule["id"], {
                        "last_run_date": now.date().isoformat(),
                    })
            except Exception:
                logger.exception("Schedule loop failed")
            await asyncio.sleep(30.0)

    @contextlib.asynccontextmanager
    async def lifespan(app: FastAPI):
        nonlocal _loop
        _loop = asyncio.get_running_loop()
        if docking_manager is not None:
            docking_manager.start(soc_source or (lambda: 0), charging_source or (lambda: False))
        _tasks.extend([
            asyncio.create_task(_telemetry_loop()),
            asyncio.create_task(_scheduler_loop()),
        ])
        yield
        for task in _tasks:
            task.cancel()
        await asyncio.gather(*_tasks, return_exceptions=True)
        _tasks.clear()
        if docking_manager is not None:
            docking_manager.stop()
        for component in managed_components or []:
            with contextlib.suppress(Exception):
                component.stop()
        if hardware is not None:
            with contextlib.suppress(Exception):
                hardware.drive(0.0, 0.0)
            with contextlib.suppress(Exception):
                hardware.stop()
        _loop = None

    app = FastAPI(title="Mähroboter V2 Control", version=APP_VERSION, lifespan=lifespan)
    app.state.control_store = control_store
    app.state.runtime_state = runtime_state

    def _on_state_change(old: MowerState, new: MowerState) -> None:
        if new in (MowerState.IDLE, MowerState.CHARGING):
            runtime_state.update("safety", {
                "geofence_ok": None,
                "geofence_override_active": False,
            })
        pause_reason = executive.pause_reason
        geofence_pause = new == MowerState.PAUSED and pause_reason.startswith(
            "Geofence violation"
        )
        level = "error" if new == MowerState.ERROR else "warning" if geofence_pause else "info"
        reason = executive.error_reason
        action = (
            "Ursache prüfen, Gefahrenbereich sichern und Fehler anschließend zurücksetzen"
            if new == MowerState.ERROR else
            "Grenze und Mähwerkskontur prüfen; nur bewusst quittiert fortsetzen"
            if geofence_pause else ""
        )
        message = (
            f"Sicherheitsstopp: {reason}"
            if new == MowerState.ERROR and reason else
            f"Geofence-Stopp: {pause_reason}"
            if geofence_pause else
            f"Status: {old.name} → {new.name}"
        )
        control_store.add_event(level, "state_change", message, action,
                                reason=reason or pause_reason,
                                zone_id=executive.active_zone_id)
        loop = _loop
        if loop is not None and loop.is_running():
            asyncio.run_coroutine_threadsafe(manager.broadcast({
                "type": "state", "state": new.name, "previous": old.name,
                "error_reason": executive.error_reason,
                "pause_reason": executive.pause_reason,
                "geofence_override_active": executive.geofence_override_active,
                "active_zone_id": executive.active_zone_id,
            }), loop)

    executive.on_state_change = _on_state_change

    if _STATIC_DIR.exists():
        app.mount("/static", StaticFiles(directory=str(_STATIC_DIR)), name="static")

    @app.get("/", include_in_schema=False, response_model=None)
    async def root() -> FileResponse | HTMLResponse:
        index = _STATIC_DIR / "index.html"
        return FileResponse(str(index)) if index.exists() else HTMLResponse("<html><body>Mähroboter V2</body></html>")

    # --- Authentication -------------------------------------------------

    @app.get("/api/auth/status")
    async def auth_status(request: Request) -> dict[str, bool]:
        token = request.cookies.get("mv2_session")
        return {"required": bool(auth_pin), "authenticated": not auth_pin or bool(token in sessions)}

    @app.post("/api/auth/login")
    async def auth_login(body: PinLogin, response: Response) -> dict[str, bool]:
        if auth_pin and not secrets.compare_digest(body.pin, auth_pin):
            control_store.add_event("warning", "login_failed", "Fehlgeschlagene PIN-Anmeldung")
            raise HTTPException(status_code=401, detail="PIN ist falsch")
        token = secrets.token_urlsafe(32)
        sessions.add(token)
        response.set_cookie("mv2_session", token, httponly=True, samesite="strict", max_age=86400)
        return {"authenticated": True}

    @app.post("/api/auth/logout")
    async def auth_logout(request: Request, response: Response) -> dict[str, bool]:
        sessions.discard(request.cookies.get("mv2_session", ""))
        response.delete_cookie("mv2_session")
        return {"authenticated": False}

    # --- Bootstrap / telemetry -----------------------------------------

    @app.get("/api/bootstrap")
    async def bootstrap(_: None = Depends(_auth_required)) -> dict[str, Any]:
        return {
            "state": executive.state.name,
            "error_reason": executive.error_reason,
            "pause_reason": executive.pause_reason,
            "geofence_override_active": executive.geofence_override_active,
            "active_zone_id": executive.active_zone_id,
            "telemetry": runtime_state.snapshot(),
            "zones": control_store.list_items("zones"),
            "connections": control_store.list_items("connections"),
            "schedules": control_store.list_items("schedules"),
            "settings": control_store.settings(),
            "home": control_store.get_home(),
            "route": mission_runtime.route_geojson() if mission_runtime is not None else None,
            "version": APP_VERSION,
            "planners": {"fields2cover": fields2cover_status()},
        }

    @app.get("/api/planners/status")
    async def planner_status(_: None = Depends(_auth_required)) -> dict[str, Any]:
        return {
            "mv2": {"available": True, "version": "coverage_hybrid_v3"},
            "fields2cover": fields2cover_status(),
        }

    @app.get("/api/state")
    async def get_state(_: None = Depends(_auth_required)) -> dict[str, Any]:
        return {
            "state": executive.state.name,
            "active_zone_id": executive.active_zone_id,
            "pause_reason": executive.pause_reason,
            "geofence_override_active": executive.geofence_override_active,
        }

    @app.get("/api/status")
    async def get_status(_: None = Depends(_auth_required)) -> dict[str, Any]:
        return {
            "state": executive.state.name,
            "error_reason": executive.error_reason,
            "pause_reason": executive.pause_reason,
            "geofence_override_active": executive.geofence_override_active,
            "active_zone_id": executive.active_zone_id,
            "telemetry": runtime_state.snapshot(),
        }

    # --- Mission and movement ------------------------------------------

    @app.post("/api/mission/start")
    async def mission_start(body: Optional[MissionStart] = Body(default=None),
                            _: None = Depends(_auth_required)) -> dict[str, Any]:
        body = body or MissionStart()
        if require_confirmations and not body.confirmed:
            raise HTTPException(status_code=400, detail="Bestätigung erforderlich")
        if executive.state not in (MowerState.IDLE, MowerState.CHARGING):
            raise HTTPException(status_code=409, detail=f"Mission kann aus {executive.state.name} nicht starten")
        reason = _mission_block_reason(body.zone_id)
        if reason:
            control_store.add_event("warning", "mission_blocked", reason, "Sperrgrund beheben")
            raise HTTPException(status_code=409, detail=reason)
        # Drop any result left by the previously active zone.  The selected
        # zone and (where applicable) its dock corridor are validated by the
        # mission planner and the next live footprint sample.
        runtime_state.update("safety", {
            "geofence_ok": None,
            "geofence_override_active": False,
        })
        planned_route: Optional[dict[str, Any]] = None
        if mission_runtime is not None:
            zone = _zone_or_404(body.zone_id)
            try:
                # Native coverage planning is CPU-heavy and may run several
                # isolated optimizer attempts.  Keep it off the ASGI event
                # loop so telemetry, status, soft-stop and emergency-stop
                # endpoints remain responsive while the route is calculated.
                planned_route = await asyncio.to_thread(
                    mission_runtime.start_mission, zone,
                )
            except (ValueError, RuntimeError) as exc:
                raise HTTPException(status_code=409, detail=str(exc)) from exc
        else:
            executive.start_mission(body.zone_id)
        route_metrics = (
            planned_route.get("properties", {}).get("route_metrics", {})
            if isinstance(planned_route, dict) else {}
        )
        return {
            "state": executive.state.name,
            "zone_id": executive.active_zone_id,
            "plan_cache_hit": bool(route_metrics.get("plan_cache_hit", False)),
        }

    @app.get("/api/mission/route")
    async def mission_route(_: None = Depends(_auth_required)) -> dict[str, Any]:
        if mission_runtime is None:
            return {"type": "Feature", "properties": {"waypoint_count": 0},
                    "geometry": {"type": "LineString", "coordinates": []}}
        return mission_runtime.route_geojson()

    @app.post("/api/mission/pause")
    async def mission_pause(_: None = Depends(_auth_required)) -> dict[str, str]:
        if executive.state not in (MowerState.MOWING, MowerState.OBSTACLE_AVOIDANCE):
            raise HTTPException(status_code=409, detail="Aktive Mission kann nicht pausiert werden")
        executive.pause_mission()
        return {"state": executive.state.name}

    @app.post("/api/mission/resume")
    async def mission_resume(_: None = Depends(_auth_required)) -> dict[str, str]:
        if executive.state != MowerState.PAUSED:
            raise HTTPException(status_code=409, detail="Keine pausierte Mission vorhanden")
        reason = _mission_block_reason(
            executive.active_zone_id, check_active_geofence=True,
        )
        if reason:
            raise HTTPException(status_code=409, detail=reason)
        executive.resume_mission()
        return {"state": executive.state.name}

    @app.post("/api/mission/resume-geofence-override")
    async def mission_resume_geofence_override(
        body: Optional[ConfirmedAction] = Body(default=None),
        _: None = Depends(_auth_required),
    ) -> dict[str, Any]:
        body = body or ConfirmedAction()
        if require_confirmations and not body.confirmed:
            raise HTTPException(
                status_code=400,
                detail="Geofence-Ausnahme muss ausdrücklich bestätigt werden",
            )
        if runtime_state.snapshot().get("safety", {}).get("geofence_ok") is not False:
            raise HTTPException(
                status_code=409,
                detail="Keine aktive Geofence-Verletzung vorhanden",
            )
        if not executive.resume_with_geofence_override():
            raise HTTPException(
                status_code=409,
                detail="Kein quittierbarer Geofence-Stopp vorhanden",
            )
        runtime_state.update("safety", {"geofence_override_active": True})
        control_store.add_event(
            "critical",
            "geofence_override",
            "Geofence-Verletzung durch Bediener ignoriert — Mission fortgesetzt",
            "Fahrzeug beobachten und so schnell wie möglich vollständig in die Zone fahren",
            zone_id=executive.active_zone_id,
        )
        return {
            "state": executive.state.name,
            "geofence_override_active": True,
        }

    @app.post("/api/mission/stop")
    async def mission_stop(_: None = Depends(_auth_required)) -> dict[str, str]:
        executive.stop_mission()
        return {"state": executive.state.name}

    @app.post("/api/mission/return-home")
    async def return_home(_: None = Depends(_auth_required)) -> dict[str, Any]:
        if executive.state not in (MowerState.MOWING, MowerState.PAUSED, MowerState.OBSTACLE_AVOIDANCE):
            raise HTTPException(status_code=409, detail="Keine Mission kann zur Ladestation zurückkehren")
        route = None
        if mission_runtime is not None:
            try:
                route = mission_runtime.return_home()
            except (ValueError, RuntimeError) as exc:
                raise HTTPException(status_code=409, detail=str(exc)) from exc
        else:
            executive.return_to_dock()
        control_store.add_event("info", "return_home", "Rückkehr zur Ladestation angefordert")
        return {"state": executive.state.name, "route": route}

    @app.post("/api/soft-stop")
    async def soft_stop(_: None = Depends(_auth_required)) -> dict[str, str]:
        executive.soft_stop()
        control_store.add_event("warning", "soft_stop", "Soft-Stop durch Bediener", "Umgebung prüfen")
        return {"state": executive.state.name}

    @app.post("/api/estop")
    async def estop(body: Optional[ConfirmedAction] = Body(default=None),
                    _: None = Depends(_auth_required)) -> dict[str, str]:
        body = body or ConfirmedAction()
        if require_confirmations and not body.confirmed:
            raise HTTPException(status_code=400, detail="Not-Aus muss bestätigt werden")
        executive.emergency_stop()
        control_store.add_event("critical", "operator_estop", "Not-Aus durch Bediener",
                                "Gefahrenbereich sichern und Ursache prüfen")
        return {"state": executive.state.name}

    @app.post("/api/manual/drive")
    async def manual_drive(body: ManualDrive, _: None = Depends(_auth_required)) -> dict[str, float]:
        if hardware is None:
            raise HTTPException(status_code=503, detail="Fahrhardware nicht verbunden")
        if executive.state not in (MowerState.IDLE, MowerState.TEACH_IN, MowerState.PAUSED):
            raise HTTPException(status_code=409, detail="Manuelle Fahrt ist in diesem Zustand gesperrt")
        limit = max(0.05, min(0.5, float(control_store.settings().get("manual_speed_limit", 0.25))))
        steering_limit = max(
            10.0,
            min(45.0, float(control_store.settings().get("manual_steering_limit_deg", 28))),
        )
        speed = max(-limit, min(limit, body.speed))
        steering = max(-steering_limit, min(steering_limit, body.steering))
        hardware.drive(speed, steering)
        return {
            "speed": speed,
            "steering": steering,
            "limit": limit,
            "steering_limit": steering_limit,
        }

    @app.post("/api/manual/stop")
    async def manual_stop(_: None = Depends(_auth_required)) -> dict[str, bool]:
        if hardware is not None:
            hardware.drive(0.0, 0.0)
        return {"stopped": True}

    @app.post("/api/manual/implement")
    async def manual_implement(
        body: ManualImplement,
        _: None = Depends(_auth_required),
    ) -> dict[str, Any]:
        if hardware is None:
            raise HTTPException(status_code=503, detail="Fahrhardware nicht verbunden")
        if executive.state not in (MowerState.IDLE, MowerState.TEACH_IN, MowerState.PAUSED):
            raise HTTPException(
                status_code=409,
                detail="Manuelle Mähwerksteuerung ist in diesem Zustand gesperrt",
            )
        if (
            body.front_raised is None
            and body.rear_raised is None
            and body.blade_enabled is None
        ):
            raise HTTPException(status_code=422, detail="Kein Mähwerkbefehl angegeben")

        snapshot = hardware.output_snapshot()
        if not isinstance(snapshot, dict):
            snapshot = {}
        front_raised = (
            bool(snapshot.get("front_deck_raised", False))
            if body.front_raised is None else body.front_raised
        )
        rear_raised = (
            bool(snapshot.get("rear_deck_raised", False))
            if body.rear_raised is None else body.rear_raised
        )
        blade_enabled = (
            bool(snapshot.get("blade_enabled", False))
            if body.blade_enabled is None else body.blade_enabled
        )

        if body.blade_enabled is True:
            if not body.confirmed:
                raise HTTPException(status_code=400, detail="Messerstart muss bestätigt werden")
            if front_raised or rear_raised:
                raise HTTPException(
                    status_code=409,
                    detail="Messer gesperrt: Front- und Heckmähwerk zuerst absenken",
                )

        if body.front_raised is not None or body.rear_raised is not None:
            hardware.drive(0.0, 0.0)
            if blade_enabled and (front_raised or rear_raised):
                hardware.set_blade(False)
                blade_enabled = False
            hardware.set_deck_lift(front_raised, rear_raised)
        if body.blade_enabled is not None:
            hardware.set_blade(body.blade_enabled)
            blade_enabled = body.blade_enabled

        if blade_enabled:
            control_store.add_event(
                "warning",
                "manual_blade_enabled",
                "Messer im manuellen Diagnosebetrieb eingeschaltet",
                "Gefahrenbereich freihalten und Messer nach dem Test ausschalten",
            )
        return {
            "front_deck_raised": front_raised,
            "rear_deck_raised": rear_raised,
            "blade_enabled": blade_enabled,
        }

    @app.post("/api/teach-in/start")
    async def teach_in_start(
        body: Optional[TeachStart] = Body(default=None),
        _: None = Depends(_auth_required),
    ) -> dict[str, Any]:
        reference = body.reference if body is not None else "center"
        if mission_runtime is not None:
            try:
                mission_runtime.start_teach_in(reference=reference)
            except (ValueError, RuntimeError) as exc:
                raise HTTPException(status_code=409, detail=str(exc)) from exc
        else:
            executive.start_teach_in()
        return {"state": executive.state.name, "reference": reference}

    @app.post("/api/teach-in/stop")
    async def teach_in_stop(_: None = Depends(_auth_required)) -> dict[str, Any]:
        if mission_runtime is not None:
            try:
                result = mission_runtime.stop_teach_in()
            except (ValueError, RuntimeError) as exc:
                raise HTTPException(status_code=409, detail=str(exc)) from exc
            return {"state": executive.state.name, **result}
        executive.stop_teach_in()
        return {"state": executive.state.name, "geometry": None, "point_count": 0}

    @app.get("/api/teach-in/status")
    async def teach_in_status(_: None = Depends(_auth_required)) -> dict[str, Any]:
        if mission_runtime is None:
            return {"state": executive.state.name, "recording": executive.state == MowerState.TEACH_IN,
                    "suspended": False, "closed": False, "point_count": 0, "length_m": 0.0,
                    "geometry": {"type": "LineString", "coordinates": []},
                    "correction_target": None, "distance_to_target_m": None}
        return mission_runtime.teach_in_status()

    @app.post("/api/teach-in/undo")
    async def teach_in_undo(body: TeachUndo,
                            _: None = Depends(_auth_required)) -> dict[str, Any]:
        if mission_runtime is None:
            raise HTTPException(status_code=503, detail="Serverseitiges Teach-In ist nicht verfügbar")
        try:
            return mission_runtime.undo_teach_in(body.distance_m)
        except (ValueError, RuntimeError) as exc:
            raise HTTPException(status_code=409, detail=str(exc)) from exc

    @app.post("/api/teach-in/continue-here")
    async def teach_in_continue_here(_: None = Depends(_auth_required)) -> dict[str, Any]:
        if mission_runtime is None:
            raise HTTPException(status_code=503, detail="Serverseitiges Teach-In ist nicht verfügbar")
        try:
            return mission_runtime.continue_teach_in_here()
        except RuntimeError as exc:
            raise HTTPException(status_code=409, detail=str(exc)) from exc

    @app.post("/api/reset")
    async def reset_error(body: Optional[ConfirmedAction] = Body(default=None),
                          _: None = Depends(_auth_required)) -> dict[str, str]:
        body = body or ConfirmedAction()
        if require_confirmations and not body.confirmed:
            raise HTTPException(status_code=400, detail="Zurücksetzen muss bestätigt werden")
        executive.reset_error()
        return {"state": executive.state.name}

    # --- Persistent zones, home, schedules and settings ----------------

    @app.get("/api/zones")
    async def zones_list(_: None = Depends(_auth_required)) -> list[dict[str, Any]]:
        return control_store.list_items("zones")

    @app.post("/api/zones")
    async def zones_add(values: dict[str, Any], _: None = Depends(_auth_required)) -> dict[str, Any]:
        return control_store.add_item("zones", _validated_zone(values))

    @app.put("/api/zones/{zone_id}")
    async def zones_update(zone_id: str, values: dict[str, Any],
                           _: None = Depends(_auth_required)) -> dict[str, Any]:
        if control_store.get_item("zones", zone_id) is None:
            raise HTTPException(status_code=404, detail="Zone nicht gefunden")
        updated = control_store.update_item("zones", zone_id, _validated_zone(values, partial=True))
        if updated is None:
            raise HTTPException(status_code=404, detail="Zone nicht gefunden")
        return updated

    @app.delete("/api/zones/{zone_id}")
    async def zones_delete(zone_id: str, _: None = Depends(_auth_required)) -> dict[str, bool]:
        used = any(s.get("zone_id") == zone_id for s in control_store.list_items("schedules"))
        if used:
            raise HTTPException(status_code=409, detail="Zone wird noch von einem Zeitplan verwendet")
        linked = any(
            connection.get("from_zone_id") == zone_id or connection.get("to_zone_id") == zone_id
            for connection in control_store.list_items("connections")
        )
        if linked:
            raise HTTPException(status_code=409, detail="Zone wird noch von einem Verbindungsweg verwendet")
        return {"deleted": control_store.delete_item("zones", zone_id)}

    @app.get("/api/connections")
    async def connections_list(_: None = Depends(_auth_required)) -> list[dict[str, Any]]:
        return control_store.list_items("connections")

    @app.post("/api/connections")
    async def connections_add(values: dict[str, Any], _: None = Depends(_auth_required)) -> dict[str, Any]:
        clean = _validated_connection(values, control_store.list_items("zones"))
        if clean.get("type") == "dock_link" and control_store.get_home() is None:
            raise HTTPException(status_code=422, detail="Zuerst die Ladestation auf der Karte setzen")
        return control_store.add_item("connections", clean)

    @app.put("/api/connections/{connection_id}")
    async def connections_update(
        connection_id: str,
        values: dict[str, Any],
        _: None = Depends(_auth_required),
    ) -> dict[str, Any]:
        existing = control_store.get_item("connections", connection_id)
        if existing is None:
            raise HTTPException(status_code=404, detail="Verbindungsweg nicht gefunden")
        clean = _validated_connection(
            values,
            control_store.list_items("zones"),
            partial=True,
            existing=existing,
        )
        if clean.get("type") == "dock_link" and control_store.get_home() is None:
            raise HTTPException(status_code=422, detail="Zuerst die Ladestation auf der Karte setzen")
        updated = control_store.update_item("connections", connection_id, clean)
        assert updated is not None
        return updated

    @app.delete("/api/connections/{connection_id}")
    async def connections_delete(connection_id: str, _: None = Depends(_auth_required)) -> dict[str, bool]:
        return {"deleted": control_store.delete_item("connections", connection_id)}

    @app.get("/api/map/topology")
    async def map_topology(_: None = Depends(_auth_required)) -> dict[str, Any]:
        zones = control_store.list_items("zones")
        connections = control_store.list_items("connections")
        mowing_ids = {zone["id"] for zone in zones if zone.get("type", "mowing") == "mowing"}
        linked_ids = {
            zone_id
            for connection in connections if connection.get("enabled", True)
            for zone_id in (connection.get("from_zone_id"), connection.get("to_zone_id"))
            if zone_id in mowing_ids
        }
        warnings: list[str] = []
        if mowing_ids and control_store.get_home() is None:
            warnings.append("Ladestation ist noch nicht auf der Karte gesetzt")
        unlinked = sorted(mowing_ids - linked_ids)
        if len(mowing_ids) > 1 and unlinked:
            warnings.append(f"{len(unlinked)} Mähzone(n) besitzen noch keinen Verbindungsweg")
        return {
            "valid": not warnings,
            "zone_count": len(mowing_ids),
            "no_go_count": sum(zone.get("type") == "no_go" for zone in zones),
            "connection_count": len(connections),
            "unlinked_zone_ids": unlinked,
            "warnings": warnings,
        }

    @app.get("/api/home")
    async def home_get(_: None = Depends(_auth_required)) -> Optional[dict[str, Any]]:
        return control_store.get_home()

    @app.put("/api/home")
    async def home_set(body: HomePosition, _: None = Depends(_auth_required)) -> dict[str, Any]:
        return control_store.set_home(body.model_dump())

    @app.delete("/api/home")
    async def home_delete(_: None = Depends(_auth_required)) -> dict[str, bool]:
        if any(connection.get("type") == "dock_link" for connection in control_store.list_items("connections")):
            raise HTTPException(status_code=409, detail="Ladestation wird noch von einem Stationsweg verwendet")
        return {"deleted": control_store.clear_home()}

    @app.get("/api/schedules")
    async def schedules_list(_: None = Depends(_auth_required)) -> list[dict[str, Any]]:
        return control_store.list_items("schedules")

    @app.post("/api/schedules")
    async def schedules_add(values: dict[str, Any], _: None = Depends(_auth_required)) -> dict[str, Any]:
        zone_id = values.get("zone_id")
        if not zone_id:
            raise HTTPException(status_code=422, detail="Zeitplan benötigt eine Zone")
        _zone_or_404(zone_id)
        values.setdefault("days", [])
        values.setdefault("window_start", "09:00")
        values.setdefault("window_end", "15:00")
        values.setdefault("enabled", True)
        values.setdefault("skip_next", False)
        return control_store.add_item("schedules", values)

    @app.put("/api/schedules/{schedule_id}")
    async def schedules_update(schedule_id: str, values: dict[str, Any],
                               _: None = Depends(_auth_required)) -> dict[str, Any]:
        if values.get("zone_id"):
            _zone_or_404(values["zone_id"])
        updated = control_store.update_item("schedules", schedule_id, values)
        if updated is None:
            raise HTTPException(status_code=404, detail="Zeitplan nicht gefunden")
        return updated

    @app.delete("/api/schedules/{schedule_id}")
    async def schedules_delete(schedule_id: str, _: None = Depends(_auth_required)) -> dict[str, bool]:
        return {"deleted": control_store.delete_item("schedules", schedule_id)}

    @app.post("/api/schedules/{schedule_id}/skip-next")
    async def schedules_skip(schedule_id: str, _: None = Depends(_auth_required)) -> dict[str, Any]:
        updated = control_store.update_item("schedules", schedule_id, {"skip_next": True})
        if updated is None:
            raise HTTPException(status_code=404, detail="Zeitplan nicht gefunden")
        return updated

    @app.get("/api/settings")
    async def settings_get(_: None = Depends(_auth_required)) -> dict[str, Any]:
        return control_store.settings()

    @app.put("/api/settings")
    async def settings_put(values: dict[str, Any], _: None = Depends(_auth_required)) -> dict[str, Any]:
        updated = control_store.update_settings(values)
        simulation = getattr(app.state, "simulation", None)
        if simulation is not None and "vehicle_wheelbase_cm" in values:
            simulation.world.set_wheelbase(float(updated["vehicle_wheelbase_cm"]) / 100.0)
        return updated

    # --- Logs, diagnostics and maintenance ------------------------------

    @app.get("/api/events")
    async def events(limit: int = 100, _: None = Depends(_auth_required)) -> list[dict[str, Any]]:
        return control_store.list_items("events")[:max(1, min(500, limit))]

    @app.get("/api/logs/download", response_class=PlainTextResponse)
    async def log_download(_: None = Depends(_auth_required)) -> str:
        lines = ["timestamp\tlevel\tcode\tmessage\taction"]
        for event in control_store.list_items("events"):
            lines.append("\t".join(str(event.get(k, "")) for k in
                                   ("timestamp", "level", "code", "message", "action")))
        return "\n".join(lines) + "\n"

    @app.get("/api/maintenance/versions")
    async def versions(_: None = Depends(_auth_required)) -> dict[str, Any]:
        return {
            "app": APP_VERSION,
            "firmware": "connected/unknown" if hardware is not None else "not connected",
            "firmware_update": "requires USB/Teensy toolchain",
        }

    @app.post("/api/maintenance/firmware/stage")
    async def firmware_stage(request: Request, _: None = Depends(_auth_required)) -> dict[str, Any]:
        """Safely stage a firmware image; flashing remains a deliberate hardware step."""
        if request.headers.get("x-mv2-confirmed") != "true":
            raise HTTPException(status_code=400, detail="Firmware-Upload muss bestätigt werden")
        if firmware_staging is None:
            raise HTTPException(status_code=503, detail="Firmware-Ablage ist nicht konfiguriert")
        name = Path(request.headers.get("x-filename", "firmware.bin")).name
        if Path(name).suffix.lower() not in (".hex", ".bin"):
            raise HTTPException(status_code=422, detail="Nur .hex- oder .bin-Dateien sind erlaubt")
        payload = await request.body()
        if not payload or len(payload) > 5 * 1024 * 1024:
            raise HTTPException(status_code=413, detail="Firmwaredatei fehlt oder ist größer als 5 MB")
        firmware_staging.mkdir(parents=True, exist_ok=True)
        target = firmware_staging / name
        target.write_bytes(payload)
        control_store.add_event("warning", "firmware_staged", f"Firmware bereitgestellt: {name}",
                                "Vor dem Flashen Hardware, Zieltyp und Prüfsumme kontrollieren",
                                size=len(payload))
        return {"staged": True, "filename": name, "size": len(payload), "flashed": False}

    @app.post("/api/maintenance/calibrate/{kind}")
    async def calibrate(kind: str, body: ConfirmedAction, _: None = Depends(_auth_required)) -> dict[str, Any]:
        steps = {
            "imu": ["Roboter waagerecht stellen", "Langsam um alle Achsen bewegen", "Werte speichern"],
            "compass": ["Freie Fläche wählen", "Roboter zweimal 360° drehen", "Offset prüfen"],
            "rtk": ["Basis/NTRIP verbinden", "RTK Fix abwarten", "Referenzpunkt speichern"],
            "docking": ["Marker ausrichten", "Kamera prüfen", "Langsame Testanfahrt durchführen"],
        }
        if kind not in steps:
            raise HTTPException(status_code=404, detail="Kalibrierung nicht unterstützt")
        if not body.confirmed:
            raise HTTPException(status_code=400, detail="Kalibrierung muss bestätigt werden")
        control_store.add_event("info", "calibration_started", f"Kalibrierung gestartet: {kind}")
        return {"kind": kind, "status": "guided", "steps": steps[kind]}

    @app.post("/api/maintenance/test")
    async def hardware_test(body: ConfirmedAction, _: None = Depends(_auth_required)) -> dict[str, Any]:
        if not body.confirmed:
            raise HTTPException(status_code=400, detail="Hardwaretest muss bestätigt werden")
        if hardware is None:
            raise HTTPException(status_code=503, detail="Hardware nicht verbunden")
        if executive.state != MowerState.IDLE:
            raise HTTPException(status_code=409, detail="Hardwaretests nur im IDLE-Zustand")
        action = body.action or ""
        commands = {
            "motor_forward": (0.12, 0.0),
            "steering_left": (0.0, 25.0),
            "steering_right": (0.0, -25.0),
        }
        if action in commands:
            hardware.drive(*commands[action])
            threading.Timer(0.5, lambda: hardware.drive(0.0, 0.0)).start()
        elif action == "blade_pulse":
            hardware.set_blade(True)
            threading.Timer(0.3, lambda: hardware.set_blade(False)).start()
        else:
            raise HTTPException(status_code=422, detail="Unbekannter Hardwaretest")
        control_store.add_event("warning", "hardware_test", f"Hardwaretest: {action}")
        return {"started": True, "action": action, "auto_stop_seconds": 0.5}

    @app.get("/api/maps/offline/status")
    async def offline_map_status(_: None = Depends(_auth_required)) -> dict[str, Any]:
        count = len(list(tiles.rglob("*.png"))) if tiles and tiles.exists() else 0
        return {"available": count > 0, "tile_count": count, "path": str(tiles) if tiles else None}

    @app.get("/tiles/{z}/{x}/{y}.png", include_in_schema=False)
    async def offline_tile(z: int, x: int, y: int) -> FileResponse:
        if tiles is None:
            raise HTTPException(status_code=404)
        tile = tiles / str(z) / str(x) / f"{y}.png"
        if not tile.is_file() or not tile.resolve().is_relative_to(tiles.resolve()):
            raise HTTPException(status_code=404)
        return FileResponse(tile)

    @app.websocket("/ws")
    async def websocket_endpoint(ws: WebSocket) -> None:
        if auth_pin and ws.cookies.get("mv2_session") not in sessions:
            await ws.close(code=4401)
            return
        await manager.connect(ws)
        try:
            await ws.send_json({
                "type": "state", "state": executive.state.name,
                "error_reason": executive.error_reason,
                "active_zone_id": executive.active_zone_id,
            })
            while True:
                await ws.receive_text()
        except WebSocketDisconnect:
            manager.disconnect(ws)

    return app
