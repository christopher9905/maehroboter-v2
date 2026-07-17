"""Simulation-only API for raw device conditions and deterministic resets."""

from __future__ import annotations

from dataclasses import asdict
from pathlib import Path
from typing import Optional

from fastapi import APIRouter, FastAPI
from pydantic import BaseModel, Field
from fastapi.staticfiles import StaticFiles

from mower.simulation.environment import SimulationEnvironment
from mower.executive.mission_executive import MissionExecutive, MowerState

_CONSOLE_DIR = Path(__file__).parent / "static"


class SimulatedSensorPatch(BaseModel):
    rain_adc: Optional[int] = Field(default=None, ge=0, le=1023)
    lifted: Optional[bool] = None
    error_flags: Optional[int] = Field(default=None, ge=0, le=255)
    charging: Optional[bool] = None
    soc_percent: Optional[float] = Field(default=None, ge=0, le=100)
    gps_fix_quality: Optional[int] = Field(default=None, ge=0, le=8)
    gps_hdop: Optional[float] = Field(default=None, ge=0)
    pitch_deg: Optional[float] = Field(default=None, ge=-180, le=180)
    roll_deg: Optional[float] = Field(default=None, ge=-180, le=180)


class SimulationSpeedPatch(BaseModel):
    factor: float = Field(ge=1.0, le=5.0)


def create_simulation_router(
    environment: SimulationEnvironment,
    executive: Optional[MissionExecutive] = None,
) -> APIRouter:
    router = APIRouter(prefix="/api/simulation", tags=["simulation"])

    @router.get("/state")
    async def simulation_state() -> dict:
        return asdict(environment.world.snapshot())

    @router.put("/sensors")
    async def simulation_sensors(body: SimulatedSensorPatch) -> dict:
        environment.world.set_sensor_state(**body.model_dump(exclude_none=True))
        # Publish through the real device callbacks immediately; the periodic
        # loops continue from the same state afterwards.
        environment.driver.emit_telemetry()
        environment.gps.sample()
        environment.imu.sample()
        return asdict(environment.world.snapshot())

    @router.put("/speed")
    async def simulation_speed(body: SimulationSpeedPatch) -> dict:
        environment.world.set_speed_factor(body.factor)
        return asdict(environment.world.snapshot())

    @router.post("/reset")
    async def simulation_reset() -> dict:
        if executive is not None and executive.state != MowerState.IDLE:
            executive.emergency_stop("Simulation zurückgesetzt")
            executive.reset_error()
        environment.reset()
        # Reset through the production HardwareInterface as well so the
        # observable command outputs match the freshly reset virtual devices.
        environment.hardware.reset_estop()
        environment.hardware.drive(0.0, 0.0)
        environment.hardware.set_blade(False)
        environment.hardware.set_deck_lift(False, False)
        state = environment.world.snapshot()
        environment.driver.emit_telemetry()
        environment.gps.sample()
        environment.imu.sample()
        return asdict(state)

    return router


def mount_simulation_console(app: FastAPI) -> None:
    """Expose the simulator operator console only in simulation mode."""
    app.mount("/simulation", StaticFiles(directory=str(_CONSOLE_DIR), html=True),
              name="simulation-console")
