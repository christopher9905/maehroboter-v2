import logging
from typing import Callable, Optional

logger = logging.getLogger(__name__)

RAIN_ADC_THRESHOLD: int = 600
RAIN_DEBOUNCE_READINGS: int = 3
RAIN_DEBOUNCE_INTERVAL_S: float = 0.2
RAIN_RESUME_AFTER_S: float = 1800.0  # 30 minutes


class RainGuard:
    """Fires on_rain_detected after 3 qualified wet readings (>=200 ms apart).

    A wet reading is rain_adc > RAIN_ADC_THRESHOLD (600). Any dry reading resets
    the debounce counter. After rain is declared, fires on_rain_cleared once the
    sensor has been continuously dry for resume_after_s.

    Call check(rain_adc, timestamp) on each SENSORS telemetry frame.
    timestamp should be time.monotonic() -- injectable for unit tests.
    """

    def __init__(
        self,
        threshold: int = RAIN_ADC_THRESHOLD,
        debounce_readings: int = RAIN_DEBOUNCE_READINGS,
        debounce_interval_s: float = RAIN_DEBOUNCE_INTERVAL_S,
        resume_after_s: float = RAIN_RESUME_AFTER_S,
    ):
        self._threshold = threshold
        self._debounce_readings = debounce_readings
        self._debounce_interval_s = debounce_interval_s
        self._resume_after_s = resume_after_s
        self._is_raining = False
        self._wet_times: list = []
        self._dry_since: Optional[float] = None
        self.on_rain_detected: Optional[Callable[[], None]] = None
        self.on_rain_cleared: Optional[Callable[[], None]] = None

    def check(self, rain_adc: int, timestamp: float):
        is_wet = rain_adc > self._threshold

        if is_wet:
            self._dry_since = None  # cancel dry timer
            if not self._is_raining:
                if (not self._wet_times or
                        timestamp - self._wet_times[-1] >= self._debounce_interval_s):
                    self._wet_times.append(timestamp)
                if len(self._wet_times) >= self._debounce_readings:
                    self._is_raining = True
                    self._wet_times.clear()
                    logger.warning("Rain detected (ADC=%d)", rain_adc)
                    if self.on_rain_detected:
                        self.on_rain_detected()
        else:
            self._wet_times.clear()  # reset debounce on any dry reading
            if self._is_raining:
                if self._dry_since is None:
                    self._dry_since = timestamp
                elif timestamp - self._dry_since >= self._resume_after_s:
                    self._is_raining = False
                    self._dry_since = None
                    logger.info("Rain cleared after %.0f s dry", self._resume_after_s)
                    if self.on_rain_cleared:
                        self.on_rain_cleared()
