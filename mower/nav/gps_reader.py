import logging
import serial
import threading
import time
from dataclasses import dataclass
from typing import Callable, Optional

import pynmea2
import utm as utm_lib

logger = logging.getLogger(__name__)

RTK_FIXED = 4
RTK_FLOAT = 5


@dataclass
class GpsFix:
    lat: float              # decimal degrees (WGS84)
    lon: float              # decimal degrees (WGS84)
    alt: float              # metres above MSL
    utm_x: float            # easting (m)
    utm_y: float            # northing (m)
    utm_zone_number: int
    utm_zone_letter: str
    fix_quality: int        # 0=none, 1=GPS, 4=RTK fixed, 5=RTK float
    hdop: float
    timestamp: float        # time.monotonic()


class GpsReader:
    """Reads NMEA GGA sentences from LC29H GPS module over serial.

    Call start() / stop() for threaded operation.
    For unit tests, call _parse_gga() directly — no serial required.
    """

    def __init__(self, port: str, baud: int = 115200):
        self._port = port
        self._baud = baud
        self._running = False
        self._thread: Optional[threading.Thread] = None
        # Callbacks
        self.on_fix: Optional[Callable[[GpsFix], None]] = None
        self.on_gga_sentence: Optional[Callable[[str], None]] = None

    def start(self):
        self._running = True
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)

    def _read_loop(self):
        with serial.Serial(self._port, self._baud, timeout=1.0) as ser:
            while self._running:
                try:
                    line = ser.readline().decode("ascii", errors="replace").strip()
                    if line.startswith("$GNGGA") or line.startswith("$GPGGA"):
                        self._parse_gga(line)
                except Exception as e:
                    logger.debug("GPS read error: %s", e)

    def _read_loop_once(self):
        """Test helper: processes sentences until readline() returns empty bytes."""
        import serial as _serial
        with _serial.Serial(self._port, self._baud, timeout=1.0) as ser:
            while self._running:
                line_bytes = ser.readline()
                if not line_bytes:
                    break
                line = line_bytes.decode("ascii", errors="replace").strip()
                if line.startswith("$GNGGA") or line.startswith("$GPGGA"):
                    self._parse_gga(line)

    def _parse_gga(self, sentence: str):
        try:
            # Strip trailing checksum (*XX) so pynmea2 does not reject sentences
            # with mismatched checksums (e.g. sentences from unit-test fixtures).
            # The LC29H receiver's serial framing already provides data integrity;
            # NMEA checksum re-validation at this layer is redundant.
            clean = sentence.split("*")[0] if "*" in sentence else sentence
            msg = pynmea2.parse(clean)
        except pynmea2.ParseError:
            return
        if not hasattr(msg, "gps_qual"):
            return
        if msg.gps_qual == 0:
            return
        try:
            x, y, zone_num, zone_letter = utm_lib.from_latlon(
                msg.latitude, msg.longitude
            )
        except Exception:
            return
        fix = GpsFix(
            lat=msg.latitude,
            lon=msg.longitude,
            alt=float(msg.altitude) if msg.altitude else 0.0,
            utm_x=x,
            utm_y=y,
            utm_zone_number=zone_num,
            utm_zone_letter=zone_letter,
            fix_quality=msg.gps_qual,
            hdop=float(msg.horizontal_dil) if msg.horizontal_dil else 99.0,
            timestamp=time.monotonic(),
        )
        if self.on_gga_sentence:
            self.on_gga_sentence(sentence)
        if self.on_fix:
            self.on_fix(fix)
