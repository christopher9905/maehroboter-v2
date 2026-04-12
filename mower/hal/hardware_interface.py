import logging
import threading
import itertools
from typing import Callable, Optional
from mower.hal.serial_driver import SerialDriver
from mower.hal.protocol import (
    CmdType, encode_drive, encode_blade, encode_estop, encode_ping,
    decode_sensors, decode_soc, decode_status,
)

logger = logging.getLogger(__name__)


class HardwareInterface:
    """High-level hardware API. Only this class is used by upper layers.

    Call start() after construction to begin the serial driver and the
    periodic PING sender (10 Hz) that feeds the Teensy watchdog.
    Call stop() on shutdown.
    """

    def __init__(self, driver: SerialDriver):
        self._driver = driver
        self._driver.on_frame = self._on_frame
        # Callbacks — upper layers assign these
        self.on_sensors: Optional[Callable[[dict], None]] = None
        self.on_soc: Optional[Callable[[dict], None]] = None
        self.on_status: Optional[Callable[[dict], None]] = None
        self._ping_seq = itertools.count()
        self._ping_timer: Optional[threading.Timer] = None
        self._running = False

    def start(self):
        self._running = True
        self._driver.start()
        self._schedule_ping()

    def stop(self):
        self._running = False
        if self._ping_timer:
            self._ping_timer.cancel()
        self._driver.stop()

    def _schedule_ping(self):
        if not self._running:
            return
        self.ping(next(self._ping_seq) & 0xFFFF)
        self._ping_timer = threading.Timer(0.1, self._schedule_ping)  # 10 Hz
        self._ping_timer.daemon = True
        self._ping_timer.start()

    # --- Commands ---

    def drive(self, speed: float, steering: float):
        """speed: -1.0..1.0, steering: -45..45 degrees"""
        self._driver.send(encode_drive(speed, steering))

    def set_blade(self, state: bool):
        self._driver.send(encode_blade(state))

    def estop(self):
        self._driver.send(encode_estop())

    def ping(self, seq: int):
        self._driver.send(encode_ping(seq))

    # --- Telemetry dispatcher ---

    def _on_frame(self, cmd_type: CmdType, payload: bytes):
        try:
            if cmd_type == CmdType.SENSORS:
                data = decode_sensors(payload)
                if data['lift']:
                    logger.warning("Lift detected — sending ESTOP")
                    self.estop()
                    self.set_blade(False)
                if self.on_sensors:
                    self.on_sensors(data)
            elif cmd_type == CmdType.SOC:
                if self.on_soc:
                    self.on_soc(decode_soc(payload))
            elif cmd_type == CmdType.STATUS:
                if self.on_status:
                    self.on_status(decode_status(payload))
        except Exception as e:
            logger.error("Frame dispatch error: %s", e)
