"""GPS and IMU device adapters driven by the simulation world."""

from __future__ import annotations

import math
import threading
import time
from typing import Callable, Optional

from mower.nav.gps_reader import GpsFix
from mower.nav.imu_reader import ImuReading
from mower.simulation.world import SimulationWorld


class _PeriodicSource:
    def __init__(
        self,
        hz: float,
        name: str,
        *,
        time_scale_provider: Callable[[], float] | None = None,
    ) -> None:
        if hz <= 0:
            raise ValueError("source frequency must be positive")
        self._period = 1.0 / hz
        self._name = name
        self._time_scale_provider = time_scale_provider or (lambda: 1.0)
        self._running = False
        self._thread: threading.Thread | None = None

    def start(self) -> None:
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._loop, name=self._name, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)
            self._thread = None

    def _loop(self) -> None:
        while self._running:
            started = time.monotonic()
            self.sample(started)
            scale = max(1.0, min(5.0, float(self._time_scale_provider())))
            remaining = self._period / scale - (time.monotonic() - started)
            if remaining > 0:
                time.sleep(remaining)

    def sample(self, timestamp: float | None = None) -> object:
        raise NotImplementedError


class SimulatedGpsSource(_PeriodicSource):
    def __init__(self, world: SimulationWorld, *, hz: float = 5.0) -> None:
        super().__init__(hz, "mv2-sim-gps", time_scale_provider=world.time_scale)
        self.world = world
        self.on_fix: Optional[Callable[[GpsFix], None]] = None

    def sample(self, timestamp: float | None = None) -> GpsFix:
        state = self.world.snapshot()
        fix = GpsFix(
            lat=state.lat,
            lon=state.lon,
            alt=500.0,
            utm_x=state.utm_x,
            utm_y=state.utm_y,
            utm_zone_number=state.utm_zone_number,
            utm_zone_letter=state.utm_zone_letter,
            fix_quality=state.gps_fix_quality,
            hdop=state.gps_hdop,
            timestamp=time.monotonic() if timestamp is None else timestamp,
        )
        if self.on_fix:
            self.on_fix(fix)
        return fix


class SimulatedImuSource(_PeriodicSource):
    def __init__(self, world: SimulationWorld, *, hz: float = 20.0) -> None:
        super().__init__(hz, "mv2-sim-imu", time_scale_provider=world.time_scale)
        self.world = world
        self.on_imu: Optional[Callable[[ImuReading], None]] = None

    def sample(self, timestamp: float | None = None) -> ImuReading:
        state = self.world.snapshot()
        reading = ImuReading(
            heading_deg=(90.0 - math.degrees(state.heading_rad)) % 360.0,
            pitch_deg=state.pitch_deg,
            roll_deg=state.roll_deg,
            yaw_rate_dps=math.degrees(state.yaw_rate_rps),
            timestamp=time.monotonic() if timestamp is None else timestamp,
        )
        if self.on_imu:
            self.on_imu(reading)
        return reading
