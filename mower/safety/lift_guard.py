import logging
from typing import Callable, Optional

logger = logging.getLogger(__name__)


class LiftGuard:
    """Fires on_lift once per rising edge of the deck-lift switch.

    The HAL already sends ESTOP + blade-off when lift is detected.
    This guard propagates the event to the mission executive so it can
    transition to ERROR state.

    Call check(sensors_dict) on each SENSORS telemetry frame.
    sensors_dict must contain the 'lift' key (bool) as decoded by protocol.decode_sensors().
    """

    def __init__(self):
        self._was_lifted = False
        self.on_lift: Optional[Callable[[], None]] = None

    def check(self, sensors: dict):
        lifted = bool(sensors.get('lift', False))
        if lifted and not self._was_lifted:
            logger.warning("Deck lift detected — notifying mission executive")
            if self.on_lift:
                self.on_lift()
        self._was_lifted = lifted
