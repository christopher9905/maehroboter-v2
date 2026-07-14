"""Visual-servoing controller for the final ArUco docking approach.

Consumes a MarkerPose (from mower.cv.aruco_detector) and produces a
DockingCommand for HardwareInterface.drive(speed, steering).

Steering sign matches StanleyOutput / HAL: + = steer LEFT (CCW).
A marker to the RIGHT (bearing_deg > 0) yields NEGATIVE steering.
"""
from dataclasses import dataclass
from enum import Enum, auto
from typing import Optional

from mower.cv.aruco_detector import MarkerPose

BEARING_TOL_DEG: float = 6.0      # within this, considered "centred"
CONTACT_DISTANCE_M: float = 0.05  # marker this close ⇒ docked
SEARCH_STEERING_DEG: float = 12.0 # in-place rotation while searching
STEERING_GAIN: float = 1.2        # deg steering per deg bearing
MAX_STEERING_DEG: float = 45.0
MAX_APPROACH_SPEED: float = 0.25  # forward speed cap (m/s-ish, -1..1 scale)
MIN_APPROACH_SPEED: float = 0.05
TAPER_DISTANCE_M: float = 1.2     # distance at/above which speed is at max


class DockPhase(Enum):
    SEARCHING = auto()
    ALIGNING = auto()
    APPROACHING = auto()
    DOCKED = auto()


@dataclass
class DockingCommand:
    speed: float
    steering_deg: float
    phase: DockPhase
    docked: bool


def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


class DockingController:
    """Four-phase visual-servoing state machine for ArUco docking.

    Each ``compute`` call maps the latest MarkerPose (or its absence) to a
    single DockingCommand; the controller holds no state between calls.

    Phases, in the order a nominal dock progresses through them:
      - SEARCHING:   no marker visible → rotate in place to sweep for it.
      - ALIGNING:    marker off-boresight (|bearing| > tolerance) → steer to
                     centre it with zero forward speed.
      - APPROACHING: marker centred → creep forward, speed tapered by distance.
      - DOCKED:      marker within contact distance, or charge detected →
                     stop (``docked=True``).

    Charge detection and contact distance short-circuit to DOCKED regardless
    of bearing, so a slightly off-centre marker at the pins still docks.
    """

    def __init__(
        self,
        bearing_tol_deg: float = BEARING_TOL_DEG,
        contact_distance_m: float = CONTACT_DISTANCE_M,
        steering_gain: float = STEERING_GAIN,
        max_steering_deg: float = MAX_STEERING_DEG,
        max_speed: float = MAX_APPROACH_SPEED,
        min_speed: float = MIN_APPROACH_SPEED,
        taper_distance_m: float = TAPER_DISTANCE_M,
        search_steering_deg: float = SEARCH_STEERING_DEG,
    ):
        self._bearing_tol = bearing_tol_deg
        self._contact = contact_distance_m
        self._gain = steering_gain
        self._max_steer = max_steering_deg
        self._max_speed = max_speed
        self._min_speed = min_speed
        self._taper = taper_distance_m
        self._search_steer = search_steering_deg

    def compute(self, marker: Optional[MarkerPose], charge_detected: bool) -> DockingCommand:
        if charge_detected:
            return DockingCommand(0.0, 0.0, DockPhase.DOCKED, True)

        if marker is None:
            # Rotate in place to sweep for the marker.
            return DockingCommand(0.0, self._search_steer, DockPhase.SEARCHING, False)

        if marker.distance_m <= self._contact:
            return DockingCommand(0.0, 0.0, DockPhase.DOCKED, True)

        # Marker to the right (bearing +) → steer right (negative).
        steering = _clamp(
            -self._gain * marker.bearing_deg, -self._max_steer, self._max_steer
        )

        if abs(marker.bearing_deg) > self._bearing_tol:
            return DockingCommand(0.0, steering, DockPhase.ALIGNING, False)

        # Centred enough: creep forward, speed tapered by distance.
        frac = _clamp(marker.distance_m / self._taper, 0.0, 1.0)
        speed = self._min_speed + (self._max_speed - self._min_speed) * frac
        return DockingCommand(speed, steering, DockPhase.APPROACHING, False)
