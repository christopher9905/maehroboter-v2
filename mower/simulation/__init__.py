"""Hardware-boundary simulation for Mähroboter V2.

The simulation package replaces physical devices only. Mission state,
planning, control, safety, API and UI continue to use the production modules.
"""

from mower.simulation.environment import SimulationEnvironment
from mower.simulation.api import create_simulation_router, mount_simulation_console
from mower.simulation.serial_driver import SimulatedSerialDriver
from mower.simulation.sources import SimulatedGpsSource, SimulatedImuSource
from mower.simulation.world import SimulationWorld, WorldSnapshot

__all__ = [
    "SimulationEnvironment",
    "create_simulation_router",
    "mount_simulation_console",
    "SimulatedGpsSource",
    "SimulatedImuSource",
    "SimulatedSerialDriver",
    "SimulationWorld",
    "WorldSnapshot",
]
