"""SerialDriver-compatible adapter backed by :mod:`mower.simulation.world`."""

from __future__ import annotations

import struct
import threading
import time
from typing import Callable, Optional

from mower.hal.protocol import CmdType, decode_frame
from mower.simulation.world import SimulationWorld


class SimulatedSerialDriver:
    """Processes the real binary command protocol without opening a port."""

    def __init__(self, world: SimulationWorld, *, telemetry_hz: float = 20.0) -> None:
        if telemetry_hz <= 0:
            raise ValueError("telemetry_hz must be positive")
        self.world = world
        self.on_frame: Optional[Callable[[CmdType, bytes], None]] = None
        self._period = 1.0 / telemetry_hz
        self._running = False
        self._thread: threading.Thread | None = None

    def start(self) -> None:
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._loop, name="mv2-sim-hal", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)
            self._thread = None

    def send(self, frame: bytes, priority: bool = False) -> None:
        del priority  # Ordering is synchronous; ESTOP takes effect immediately.
        command, payload = decode_frame(frame)
        if command == CmdType.DRIVE:
            speed, steering = struct.unpack("<ff", payload)
            self.world.set_drive(speed, steering)
        elif command == CmdType.BLADE:
            self.world.set_blade(bool(payload[0]))
        elif command == CmdType.DECK_LIFT:
            self.world.set_deck_lift(bool(payload[0]), bool(payload[1]))
        elif command == CmdType.ESTOP:
            self.world.emergency_stop()
        elif command == CmdType.PING:
            self.world.ping(int.from_bytes(payload, "little"))

    def emit_telemetry(self) -> None:
        callback = self.on_frame
        if callback is None:
            return
        state = self.world.snapshot()
        callback(CmdType.SENSORS, struct.pack("<HBI", state.rain_adc, int(state.lifted), state.encoder_ticks))
        callback(CmdType.SOC, struct.pack("<BH", int(round(state.soc_percent)), state.voltage_mv))
        callback(CmdType.STATUS, struct.pack(
            "<BBBB",
            int(state.watchdog_ok),
            int(state.blade_running),
            state.error_flags,
            int(state.charging),
        ))

    def reset_estop(self) -> None:
        self.world.reset_estop()

    def advance_fixed_step(self) -> None:
        """Advance exactly one simulated device interval.

        Using elapsed wall time made a delayed Python thread catch up in one
        large bicycle-model step.  At 5x that appeared as a lateral teleport
        and let the vehicle skip past controller corrections.  A fixed step
        deliberately slows virtual time under load instead of changing the
        driven geometry.
        """
        scale = self.world.time_scale()
        self.world.step(self._period / scale)
        self.emit_telemetry()

    def _loop(self) -> None:
        while self._running:
            started = time.monotonic()
            self.advance_fixed_step()
            remaining = (
                self._period / self.world.time_scale()
                - (time.monotonic() - started)
            )
            if remaining > 0:
                time.sleep(remaining)
