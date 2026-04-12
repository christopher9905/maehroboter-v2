import time
from dataclasses import dataclass
from typing import Optional

# Physical constants — tune to actual hardware
# 1:14 tractor rear wheel: ~60 mm diameter → circumference ≈ 188 mm
WHEEL_CIRCUMFERENCE_M: float = 0.188
TICKS_PER_REVOLUTION: int = 20
METERS_PER_TICK: float = WHEEL_CIRCUMFERENCE_M / TICKS_PER_REVOLUTION


@dataclass
class OdometryUpdate:
    speed_mps: float            # instantaneous speed (m/s)
    delta_distance_m: float     # distance travelled since last update (m)
    total_distance_m: float     # cumulative distance since reset (m)
    timestamp: float            # time.monotonic() of this update


class Odometry:
    """Converts cumulative encoder tick counts into speed and distance.

    The encoder on the Teensy sends unsigned 32-bit ticks that only increment
    (Phase 1 limitation — direction tracking deferred). Wrap-around at 2^32
    is handled via modular arithmetic.
    """

    def __init__(self, meters_per_tick: float = METERS_PER_TICK):
        self._meters_per_tick = meters_per_tick
        self._prev_ticks: Optional[int] = None
        self._prev_timestamp: Optional[float] = None
        self._total_distance_m: float = 0.0

    def update(self, ticks: int, timestamp: float) -> Optional[OdometryUpdate]:
        """Call with encoder_ticks from HAL SENSORS telemetry.

        Returns None on the first call (no previous state to compute delta).
        """
        if self._prev_ticks is None:
            self._prev_ticks = ticks
            self._prev_timestamp = timestamp
            return None

        dt = timestamp - self._prev_timestamp
        if dt <= 0:
            return None

        # 32-bit unsigned wrap-around: delta = (new - old) mod 2^32
        delta_ticks = (ticks - self._prev_ticks) & 0xFFFFFFFF
        delta_m = delta_ticks * self._meters_per_tick
        speed = delta_m / dt

        self._total_distance_m += delta_m
        self._prev_ticks = ticks
        self._prev_timestamp = timestamp

        return OdometryUpdate(
            speed_mps=speed,
            delta_distance_m=delta_m,
            total_distance_m=self._total_distance_m,
            timestamp=timestamp,
        )

    def reset(self):
        self._prev_ticks = None
        self._prev_timestamp = None
        self._total_distance_m = 0.0
