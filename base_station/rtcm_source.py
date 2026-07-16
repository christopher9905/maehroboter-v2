"""Reads raw RTCM3 bytes from the base station's local LC29H (base mode) and
forwards them via callback. No RTCM3 framing/CRC parsing is needed here — the
rover's own RTK receiver interprets the corrections, this class only relays
opaque byte chunks as they arrive.
"""
import logging
import threading
from typing import Callable, Optional

logger = logging.getLogger(__name__)

_READ_BUFSIZE = 1024


def _default_backend(port: str, baud: int):
    import serial
    return serial.Serial(port, baud, timeout=1.0)


class RtcmSerialSource:
    """Reads raw bytes from the base LC29H's serial port and fires on_data.

    Injectable serial_backend (mirrors mower.hal.camera.Camera's capture=None
    pattern, NOT SerialDriver/GpsReader which hardwire the port internally):
    the backend is constructed/opened immediately in __init__ so a missing
    or unavailable port raises right away — no lazy open in start().
    """

    def __init__(self, port: str, baud: int = 115200, serial_backend=None):
        self._port = port
        self._serial = serial_backend if serial_backend is not None else _default_backend(port, baud)
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self.on_data: Optional[Callable[[bytes], None]] = None

    def start(self) -> None:
        self._running = True
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)

    def _read_loop(self) -> None:
        while self._running:
            try:
                data = self._serial.read(_READ_BUFSIZE)
                if data and self.on_data:
                    self.on_data(data)
            except Exception:
                if self._running:
                    logger.error("RTCM read error on %s", self._port, exc_info=True)
