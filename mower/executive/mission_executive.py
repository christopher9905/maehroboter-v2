# mower/executive/mission_executive.py
"""Mission Executive state machine for the autonomous lawn mower.

All state transitions are thread-safe (protected by _lock).
on_state_change callback is always fired OUTSIDE the lock to prevent deadlock.
"""
import logging
import threading
from enum import Enum, auto
from typing import Callable, Optional

logger = logging.getLogger(__name__)

LOW_BATTERY_SOC: int = 20
OBSTACLE_TIMEOUT_S: float = 60.0
MAX_AVOIDANCE_ATTEMPTS: int = 3


class MowerState(Enum):
    IDLE = auto()
    TEACH_IN = auto()
    MOWING = auto()
    OBSTACLE_AVOIDANCE = auto()
    RETURNING = auto()
    DOCKING = auto()
    CHARGING = auto()
    ERROR = auto()


_ACTIVE_STATES = frozenset({
    MowerState.MOWING,
    MowerState.OBSTACLE_AVOIDANCE,
    MowerState.RETURNING,
    MowerState.DOCKING,
    MowerState.CHARGING,
    MowerState.TEACH_IN,
})


class MissionExecutive:
    def __init__(self, hardware_interface=None):
        self._hw = hardware_interface
        self._state = MowerState.IDLE
        self._error_reason: str = ""
        self._avoidance_attempts: int = 0
        self._obstacle_timer: Optional[threading.Timer] = None
        self._lock = threading.Lock()
        self.on_state_change: Optional[Callable[[MowerState, MowerState], None]] = None

    @property
    def state(self) -> MowerState:
        with self._lock:
            return self._state

    @property
    def error_reason(self) -> str:
        with self._lock:
            return self._error_reason

    def _set_state(self, new_state: MowerState, reason: str = "") -> MowerState:
        old = self._state
        self._state = new_state
        self._error_reason = reason if new_state == MowerState.ERROR else ""
        return old

    def _go_error(self, reason: str) -> MowerState:
        """Transition to ERROR. MUST be called WITH _lock held. Returns old state.
        Caller must invoke _hw_estop() AFTER releasing the lock if hardware is present."""
        old = self._set_state(MowerState.ERROR, reason)
        logger.error("→ ERROR: %s", reason)
        return old

    def _hw_estop(self):
        """Call hardware ESTOP. Must be called OUTSIDE the lock."""
        if self._hw:
            self._hw.estop()
            self._hw.set_blade(False)

    def _cancel_obstacle_timer(self):
        if self._obstacle_timer:
            self._obstacle_timer.cancel()
            self._obstacle_timer = None

    def _notify(self, old: MowerState, new: MowerState):
        if old != new:
            logger.info("State: %s → %s", old.name, new.name)
            if self.on_state_change:
                self.on_state_change(old, new)

    def start_teach_in(self):
        with self._lock:
            if self._state != MowerState.IDLE:
                return
            old = self._set_state(MowerState.TEACH_IN)
        self._notify(old, MowerState.TEACH_IN)

    def stop_teach_in(self):
        with self._lock:
            if self._state != MowerState.TEACH_IN:
                return
            old = self._set_state(MowerState.IDLE)
        self._notify(old, MowerState.IDLE)

    def start_mission(self):
        with self._lock:
            if self._state != MowerState.IDLE:
                return
            self._avoidance_attempts = 0
            old = self._set_state(MowerState.MOWING)
        self._notify(old, MowerState.MOWING)

    def stop_mission(self):
        with self._lock:
            if self._state not in (MowerState.MOWING, MowerState.OBSTACLE_AVOIDANCE):
                return
            self._cancel_obstacle_timer()
            old = self._set_state(MowerState.RETURNING)
        self._notify(old, MowerState.RETURNING)

    def reset_error(self):
        with self._lock:
            if self._state != MowerState.ERROR:
                return
            old = self._set_state(MowerState.IDLE)
        self._notify(old, MowerState.IDLE)

    def on_lift(self):
        with self._lock:
            if self._state not in _ACTIVE_STATES:
                return
            self._cancel_obstacle_timer()
            old = self._go_error("Deck lift detected")
        self._hw_estop()
        self._notify(old, MowerState.ERROR)

    def on_tilt(self, reading):
        with self._lock:
            if self._state not in _ACTIVE_STATES:
                return
            self._cancel_obstacle_timer()
            old = self._go_error(
                f"Tilt limit exceeded: pitch={reading.pitch_deg:.1f}°, roll={reading.roll_deg:.1f}°"
            )
        self._hw_estop()
        self._notify(old, MowerState.ERROR)

    def on_geofence_violation(self, pose):
        with self._lock:
            if self._state not in _ACTIVE_STATES:
                return
            self._cancel_obstacle_timer()
            old = self._go_error(
                f"Geofence violation at ({pose.utm_x:.2f}, {pose.utm_y:.2f})"
            )
        self._hw_estop()
        self._notify(old, MowerState.ERROR)

    def on_obstacle_detected(self, detections):
        do_estop = False
        with self._lock:
            if self._state != MowerState.MOWING:
                return
            self._avoidance_attempts += 1
            if self._avoidance_attempts > MAX_AVOIDANCE_ATTEMPTS:
                self._cancel_obstacle_timer()
                old = self._go_error(
                    f"Obstacle avoidance failed after {MAX_AVOIDANCE_ATTEMPTS} attempts"
                )
                new = MowerState.ERROR
                do_estop = True
            else:
                old = self._set_state(MowerState.OBSTACLE_AVOIDANCE)
                new = MowerState.OBSTACLE_AVOIDANCE
                timer = threading.Timer(OBSTACLE_TIMEOUT_S, self._obstacle_timeout)
                timer.daemon = True
                timer.start()
                self._obstacle_timer = timer
        if do_estop:
            self._hw_estop()
        self._notify(old, new)

    def on_obstacle_cleared(self):
        with self._lock:
            if self._state != MowerState.OBSTACLE_AVOIDANCE:
                return
            self._cancel_obstacle_timer()
            old = self._set_state(MowerState.MOWING)
        self._notify(old, MowerState.MOWING)

    def _obstacle_timeout(self):
        with self._lock:
            if self._state != MowerState.OBSTACLE_AVOIDANCE:
                return
            self._obstacle_timer = None
            old = self._go_error(f"Obstacle avoidance timeout ({OBSTACLE_TIMEOUT_S:.0f} s)")
        self._hw_estop()
        self._notify(old, MowerState.ERROR)

    def on_battery_low(self, soc: int):
        with self._lock:
            if self._state not in (MowerState.MOWING, MowerState.OBSTACLE_AVOIDANCE):
                return
            if soc > LOW_BATTERY_SOC:
                return
            self._cancel_obstacle_timer()
            old = self._set_state(MowerState.RETURNING)
        self._notify(old, MowerState.RETURNING)

    def on_dock_success(self):
        with self._lock:
            if self._state != MowerState.RETURNING:
                return
            old = self._set_state(MowerState.DOCKING)
        self._notify(old, MowerState.DOCKING)

    def on_charge_started(self):
        with self._lock:
            if self._state != MowerState.DOCKING:
                return
            old = self._set_state(MowerState.CHARGING)
        self._notify(old, MowerState.CHARGING)

    def on_charge_complete(self):
        with self._lock:
            if self._state != MowerState.CHARGING:
                return
            old = self._set_state(MowerState.IDLE)
        self._notify(old, MowerState.IDLE)
