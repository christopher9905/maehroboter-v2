"""Construction of simulated device adapters for the production runtime."""

from __future__ import annotations

from typing import Callable

from mower.hal.hardware_interface import HardwareInterface
from mower.simulation.serial_driver import SimulatedSerialDriver
from mower.simulation.sources import SimulatedGpsSource, SimulatedImuSource
from mower.simulation.world import SimulationWorld


class SimulationEnvironment:
    def __init__(self, *, origin_lat: float = 48.5, origin_lon: float = 11.0) -> None:
        self.world = SimulationWorld(origin_lat=origin_lat, origin_lon=origin_lon)
        self.driver = SimulatedSerialDriver(self.world)
        self.hardware = HardwareInterface(driver=self.driver)
        # RTK position updates at 10 Hz match the real control/UI cadence and
        # avoid the visible staircase motion of the former 5 Hz source.
        self.gps = SimulatedGpsSource(self.world, hz=10.0)
        self.imu = SimulatedImuSource(self.world)
        self.on_reset: Callable[[], None] | None = None

    def reset(self):
        """Reset world and attached production state consumers together."""
        state = self.world.reset()
        if self.on_reset is not None:
            self.on_reset()
        return state
