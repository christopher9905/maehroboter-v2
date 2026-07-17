from dataclasses import dataclass
from collections import deque
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

    def __init__(
        self,
        meters_per_tick: float = METERS_PER_TICK,
        speed_window_s: float = 0.4,
    ):
        if speed_window_s <= 0:
            raise ValueError("speed_window_s must be positive")
        self._meters_per_tick = meters_per_tick
        self._speed_window_s = float(speed_window_s)
        self._prev_ticks: Optional[int] = None
        self._prev_timestamp: Optional[float] = None
        self._total_distance_m: float = 0.0
        self._speed_history: deque[tuple[float, float]] = deque()

    def update(self, ticks: int, timestamp: float) -> Optional[OdometryUpdate]:
        """Call with encoder_ticks from HAL SENSORS telemetry.

        Returns None on the first call (no previous state to compute delta).
        """
        if self._prev_ticks is None:
            self._prev_ticks = ticks
            self._prev_timestamp = timestamp
            self._speed_history.append((timestamp, self._total_distance_m))
            return None

        dt = timestamp - self._prev_timestamp
        if dt <= 0:
            # Duplicate or out-of-order timestamp: discard without updating state
            return None

        # 32-bit unsigned wrap-around: delta = (new - old) mod 2^32
        delta_ticks = (ticks - self._prev_ticks) & 0xFFFFFFFF
        delta_m = delta_ticks * self._meters_per_tick
        self._total_distance_m += delta_m
        self._prev_ticks = ticks
        self._prev_timestamp = timestamp
        self._speed_history.append((timestamp, self._total_distance_m))
        cutoff = timestamp - self._speed_window_s
        # Retain one sample just before the window so the estimate spans a
        # useful interval even when encoder ticks arrive between callbacks.
        while len(self._speed_history) > 1 and self._speed_history[1][0] <= cutoff:
            self._speed_history.popleft()
        oldest_timestamp, oldest_distance = self._speed_history[0]
        window_dt = timestamp - oldest_timestamp
        speed = (
            (self._total_distance_m - oldest_distance) / window_dt
            if window_dt > 0
            else 0.0
        )

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
        self._speed_history.clear()
