import logging
import math
from typing import Callable, Optional

from mower.nav.imu_reader import ImuReading

logger = logging.getLogger(__name__)

DEFAULT_THRESHOLD_DEG: float = 30.0


class TiltGuard:
    """Monitors IMU tilt and triggers an emergency callback on excessive lean.

    Combined tilt = sqrt(pitch² + roll²). If this exceeds threshold_deg,
    on_tilt(reading) is called. Wire to hardware_interface.estop() in the
    application layer before enabling autonomous driving.
    """

    def __init__(self, threshold_deg: float = DEFAULT_THRESHOLD_DEG):
        self._threshold_deg = threshold_deg
        self.on_tilt: Optional[Callable[[ImuReading], None]] = None

    def check(self, reading: ImuReading):
        tilt = math.sqrt(reading.pitch_deg ** 2 + reading.roll_deg ** 2)
        if tilt >= self._threshold_deg:
            logger.warning(
                "Tilt limit exceeded: pitch=%.1f° roll=%.1f° combined=%.1f°",
                reading.pitch_deg, reading.roll_deg, tilt,
            )
            if self.on_tilt:
                self.on_tilt(reading)
