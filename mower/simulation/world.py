"""Thread-safe two-dimensional world model used by simulated devices."""

from __future__ import annotations

import math
import threading
from dataclasses import dataclass

import utm

from mower.nav.odometry import METERS_PER_TICK


@dataclass(frozen=True)
class WorldSnapshot:
    x_m: float
    y_m: float
    lat: float
    lon: float
    utm_x: float
    utm_y: float
    utm_zone_number: int
    utm_zone_letter: str
    heading_rad: float
    speed_mps: float
    yaw_rate_rps: float
    steering_deg: float
    encoder_ticks: int
    blade_running: bool
    front_deck_raised: bool
    rear_deck_raised: bool
    estop_latched: bool
    rain_adc: int
    lifted: bool
    error_flags: int
    charging: bool
    watchdog_ok: bool
    soc_percent: float
    voltage_mv: int
    gps_fix_quality: int
    gps_hdop: float
    pitch_deg: float
    roll_deg: float
    simulation_speed_factor: float


class SimulationWorld:
    """Minimal deterministic mower physics plus virtual sensor state.

    Coordinates are maintained in metres relative to a configurable WGS84
    origin. ``step`` is deterministic and can therefore be driven by either a
    real-time thread or unit tests with a fixed delta time.
    """

    def __init__(
        self,
        *,
        origin_lat: float = 48.5,
        origin_lon: float = 11.0,
        max_speed_mps: float = 0.5,
        max_yaw_rate_dps: float | None = None,
        wheelbase_m: float = 0.25,
    ) -> None:
        origin_x, origin_y, zone_number, zone_letter = utm.from_latlon(origin_lat, origin_lon)
        self._lock = threading.RLock()
        self._origin_x = origin_x
        self._origin_y = origin_y
        self._zone_number = zone_number
        self._zone_letter = zone_letter
        self._max_speed_mps = max_speed_mps
        self._max_yaw_rate_rps = (
            math.radians(max_yaw_rate_dps)
            if max_yaw_rate_dps is not None else math.inf
        )
        self._wheelbase_m = max(0.01, float(wheelbase_m))

        self._reset_x_m = 0.0
        self._reset_y_m = 0.0
        self._reset_heading_rad = 0.0
        self._x_m = self._reset_x_m
        self._y_m = self._reset_y_m
        self._heading_rad = self._reset_heading_rad
        self._speed_command = 0.0
        self._steering_deg = 0.0
        self._speed_mps = 0.0
        self._yaw_rate_rps = 0.0
        self._distance_m = 0.0
        self._blade_running = False
        self._front_deck_raised = False
        self._rear_deck_raised = False
        self._estop_latched = False
        self._last_ping_seq: int | None = None

        self._rain_adc = 250
        self._lifted = False
        self._error_flags = 0
        self._charging = False
        self._soc_percent = 80.0
        self._gps_fix_quality = 4
        self._gps_hdop = 0.7
        self._pitch_deg = 0.0
        self._roll_deg = 0.0
        self._simulation_speed_factor = 1.0

    def set_speed_factor(self, factor: float) -> None:
        """Scale virtual motion and operating time for faster test runs."""
        factor = float(factor)
        if not 1.0 <= factor <= 5.0:
            raise ValueError("simulation speed factor must be between 1.0 and 5.0")
        with self._lock:
            self._simulation_speed_factor = factor

    def time_scale(self) -> float:
        """Return the virtual-time factor without copying the full world state."""
        with self._lock:
            return self._simulation_speed_factor

    def set_wheelbase(self, wheelbase_m: float) -> None:
        """Apply the configured physical axle spacing to the bicycle model."""
        wheelbase = float(wheelbase_m)
        if not 0.10 <= wheelbase <= 1.0:
            raise ValueError("simulation wheelbase must be between 0.10 and 1.0 m")
        with self._lock:
            self._wheelbase_m = wheelbase

    def set_reset_pose(
        self,
        x_m: float,
        y_m: float,
        heading_rad: float = 0.0,
        *,
        apply_now: bool = True,
    ) -> None:
        """Configure a safe deterministic pose used at startup and reset."""
        values = (float(x_m), float(y_m), float(heading_rad))
        if not all(math.isfinite(value) for value in values):
            raise ValueError("simulation reset pose must contain finite values")
        with self._lock:
            self._reset_x_m, self._reset_y_m, self._reset_heading_rad = values
            if apply_now:
                self._x_m, self._y_m, self._heading_rad = values
                self._speed_command = 0.0
                self._steering_deg = 0.0
                self._speed_mps = 0.0
                self._yaw_rate_rps = 0.0

    def set_drive(self, speed: float, steering_deg: float) -> None:
        with self._lock:
            if self._estop_latched:
                return
            self._speed_command = max(-1.0, min(1.0, float(speed)))
            self._steering_deg = max(-45.0, min(45.0, float(steering_deg)))

    def set_blade(self, running: bool) -> None:
        with self._lock:
            self._blade_running = bool(running) and not self._estop_latched

    def set_deck_lift(self, front_raised: bool, rear_raised: bool) -> None:
        with self._lock:
            self._front_deck_raised = bool(front_raised)
            self._rear_deck_raised = bool(rear_raised)
            if self._front_deck_raised or self._rear_deck_raised:
                self._blade_running = False

    def emergency_stop(self) -> None:
        with self._lock:
            self._estop_latched = True
            self._speed_command = 0.0
            self._steering_deg = 0.0
            self._speed_mps = 0.0
            self._yaw_rate_rps = 0.0
            self._blade_running = False
            self._front_deck_raised = False
            self._rear_deck_raised = False

    def reset_estop(self) -> None:
        """Reset the virtual hardware latch after an operator reset scenario."""
        with self._lock:
            self._estop_latched = False

    def reset(self) -> WorldSnapshot:
        """Reset vehicle pose, commands and virtual sensors to startup values."""
        with self._lock:
            self._x_m = self._reset_x_m
            self._y_m = self._reset_y_m
            self._heading_rad = self._reset_heading_rad
            self._speed_command = 0.0
            self._steering_deg = 0.0
            self._speed_mps = 0.0
            self._yaw_rate_rps = 0.0
            self._distance_m = 0.0
            self._blade_running = False
            self._estop_latched = False
            self._rain_adc = 250
            self._lifted = False
            self._error_flags = 0
            self._charging = False
            self._soc_percent = 80.0
            self._gps_fix_quality = 4
            self._gps_hdop = 0.7
            self._pitch_deg = 0.0
            self._roll_deg = 0.0
            return self._snapshot_locked()

    def ping(self, sequence: int) -> None:
        with self._lock:
            self._last_ping_seq = int(sequence) & 0xFFFF

    def set_sensor_state(self, **values: object) -> None:
        """Inject virtual raw sensor conditions without touching app state.

        Supported keys intentionally describe device-level values. Existing
        safety callbacks remain responsible for interpreting their meaning.
        """
        allowed = {
            "rain_adc", "lifted", "error_flags", "charging", "soc_percent",
            "gps_fix_quality", "gps_hdop", "pitch_deg", "roll_deg",
        }
        unknown = set(values) - allowed
        if unknown:
            raise ValueError(f"Unknown simulated sensor fields: {', '.join(sorted(unknown))}")
        with self._lock:
            if "rain_adc" in values:
                self._rain_adc = max(0, min(1023, int(values["rain_adc"])))
            if "lifted" in values:
                self._lifted = bool(values["lifted"])
            if "error_flags" in values:
                self._error_flags = max(0, min(255, int(values["error_flags"])))
            if "charging" in values:
                self._charging = bool(values["charging"])
            if "soc_percent" in values:
                self._soc_percent = max(0.0, min(100.0, float(values["soc_percent"])))
            if "gps_fix_quality" in values:
                self._gps_fix_quality = max(0, min(8, int(values["gps_fix_quality"])))
            if "gps_hdop" in values:
                self._gps_hdop = max(0.0, float(values["gps_hdop"]))
            if "pitch_deg" in values:
                self._pitch_deg = float(values["pitch_deg"])
            if "roll_deg" in values:
                self._roll_deg = float(values["roll_deg"])

    def step(self, dt: float) -> WorldSnapshot:
        if dt < 0:
            raise ValueError("Simulation step must not be negative")
        with self._lock:
            target_speed = (
                0.0
                if self._estop_latched
                else self._speed_command * self._max_speed_mps * self._simulation_speed_factor
            )
            self._speed_mps = target_speed
            curvature = math.tan(math.radians(self._steering_deg)) / self._wheelbase_m
            desired_yaw_rate = target_speed * curvature
            self._yaw_rate_rps = max(
                -self._max_yaw_rate_rps,
                min(self._max_yaw_rate_rps, desired_yaw_rate),
            )
            if abs(target_speed) > 1e-9:
                curvature = self._yaw_rate_rps / target_speed
            old_heading = self._heading_rad
            new_heading = old_heading + self._yaw_rate_rps * dt
            if abs(curvature) < 1e-12:
                dx = target_speed * math.cos(old_heading) * dt
                dy = target_speed * math.sin(old_heading) * dt
            else:
                dx = (math.sin(new_heading) - math.sin(old_heading)) / curvature
                dy = (-math.cos(new_heading) + math.cos(old_heading)) / curvature
            self._heading_rad = (new_heading + math.pi) % (2 * math.pi) - math.pi
            self._x_m += dx
            self._y_m += dy
            self._distance_m += math.hypot(dx, dy)

            if self._charging:
                self._soc_percent = min(
                    100.0,
                    self._soc_percent
                    + dt * self._simulation_speed_factor * (25.0 / 3600.0),
                )
            else:
                drain_per_hour = 8.0 + (10.0 if self._blade_running else 0.0)
                drain_per_hour += 5.0 * abs(self._speed_command)
                self._soc_percent = max(
                    0.0,
                    self._soc_percent
                    - dt * self._simulation_speed_factor * (drain_per_hour / 3600.0),
                )
            return self._snapshot_locked()

    def snapshot(self) -> WorldSnapshot:
        with self._lock:
            return self._snapshot_locked()

    def _snapshot_locked(self) -> WorldSnapshot:
        utm_x = self._origin_x + self._x_m
        utm_y = self._origin_y + self._y_m
        lat, lon = utm.to_latlon(utm_x, utm_y, self._zone_number, self._zone_letter)
        voltage_mv = int(round(12000 + self._soc_percent * 30))
        return WorldSnapshot(
            x_m=self._x_m,
            y_m=self._y_m,
            lat=lat,
            lon=lon,
            utm_x=utm_x,
            utm_y=utm_y,
            utm_zone_number=self._zone_number,
            utm_zone_letter=self._zone_letter,
            heading_rad=self._heading_rad,
            speed_mps=self._speed_mps,
            yaw_rate_rps=self._yaw_rate_rps,
            steering_deg=self._steering_deg,
            encoder_ticks=int(self._distance_m / METERS_PER_TICK) & 0xFFFFFFFF,
            blade_running=self._blade_running,
            front_deck_raised=self._front_deck_raised,
            rear_deck_raised=self._rear_deck_raised,
            estop_latched=self._estop_latched,
            rain_adc=self._rain_adc,
            lifted=self._lifted,
            error_flags=self._error_flags,
            charging=self._charging,
            watchdog_ok=self._last_ping_seq is not None,
            soc_percent=self._soc_percent,
            voltage_mv=voltage_mv,
            gps_fix_quality=self._gps_fix_quality,
            gps_hdop=self._gps_hdop,
            pitch_deg=self._pitch_deg,
            roll_deg=self._roll_deg,
            simulation_speed_factor=self._simulation_speed_factor,
        )
