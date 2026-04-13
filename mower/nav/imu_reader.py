import logging
import math
import threading
import time
from dataclasses import dataclass
from typing import Callable, Optional

logger = logging.getLogger(__name__)

# These are imported at module level for easy patching in tests.
# On non-RPi environments they will fail — tests inject mocks before _init_sensor().
try:
    from adafruit_bno08x.i2c import BNO08X_I2C
    from adafruit_bno08x import BNO_REPORT_EULER, BNO_REPORT_GYROSCOPE
except ImportError:
    BNO08X_I2C = None          # type: ignore[assignment,misc]
    BNO_REPORT_EULER = None    # type: ignore[assignment]
    BNO_REPORT_GYROSCOPE = None  # type: ignore[assignment]


@dataclass
class ImuReading:
    heading_deg: float      # 0–360, 0 = North, clockwise
    pitch_deg: float        # positive = nose up
    roll_deg: float         # positive = right side down
    yaw_rate_dps: float     # degrees/s, CCW positive (ENU frame, standard math convention)
    timestamp: float        # time.monotonic()


class ImuReader:
    """Reads BNO085 9-DoF IMU via I2C at ~100 Hz.

    Pass i2c=<mock> for unit tests. Without i2c, hardware bus is created in start().
    """

    def __init__(self, i2c=None):
        self._i2c = i2c
        self._sensor = None
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self.on_imu: Optional[Callable[[ImuReading], None]] = None

    def start(self):
        if self._i2c is None:
            import board        # only available on RPi
            import busio
            self._i2c = busio.I2C(board.SCL, board.SDA)
        self._init_sensor()
        self._running = True
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)

    def _init_sensor(self):
        if self._i2c is None:
            raise RuntimeError("No I2C bus available — call start() or inject i2c in constructor")
        self._sensor = BNO08X_I2C(self._i2c)
        self._sensor.enable_feature(BNO_REPORT_EULER)
        self._sensor.enable_feature(BNO_REPORT_GYROSCOPE)

    def _read_once(self):
        """Read one sample and fire callback. Called from _read_loop and tests."""
        yaw, pitch, roll = self._sensor.euler  # degrees
        _gx, _gy, gz = self._sensor.gyro       # rad/s
        reading = ImuReading(
            heading_deg=yaw % 360.0,
            pitch_deg=pitch,
            roll_deg=roll,
            yaw_rate_dps=math.degrees(gz),
            timestamp=time.monotonic(),
        )
        if self.on_imu:
            self.on_imu(reading)

    def _read_loop(self):
        while self._running:
            try:
                self._read_once()
                time.sleep(0.01)  # 100 Hz
            except Exception as e:
                logger.error("IMU read error: %s", e)
                time.sleep(0.1)
