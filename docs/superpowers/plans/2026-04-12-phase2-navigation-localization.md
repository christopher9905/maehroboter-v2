# Phase 2: Navigation, Localization & Safety — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** RTK-GPS integration, BNO085 IMU, UKF-based sensor fusion (GPS + IMU + Odometry → UTM pose), Geofencing GATE, Tilt Guard GATE, GeoJSON map storage, and Teach-In map recording — all as prerequisites for Phase 3 autonomous driving.

**Architecture:** Three new Python packages are added under `mower/`: `nav/` (GPS reader, NTRIP client, IMU reader, odometry, UKF localizer), `safety/` (geofence gate, tilt guard gate), and `map/` (GeoJSON map store, teach-in recorder). The localizer subscribes to all three sensor streams and publishes a fused `Pose` at sensor rate. The safety gates subscribe to pose and IMU independently and call `hardware_interface.estop()` on violation. Teach-In subscribes to the localizer's pose and records GPS points. All hardware-dependent classes accept injected dependencies so they are fully unit-testable without hardware.

**Tech Stack:** Python 3.11+, filterpy, pynmea2, utm, shapely, adafruit-circuitpython-bno08x, adafruit-blinka, pytest, pytest-mock

---

## File Structure

```
mower/
├── nav/
│   ├── __init__.py              Create — empty package marker
│   ├── gps_reader.py            Create — NMEA GGA parsing, RTK quality, UTM conversion
│   ├── ntrip_client.py          Create — RTCM corrections via socket to NTRIP caster
│   ├── imu_reader.py            Create — BNO085 via I2C (adafruit library), 100 Hz
│   ├── odometry.py              Create — encoder ticks → speed_mps, delta/total distance
│   └── localizer.py             Create — UKF [x,y,heading,v,yaw_rate] in UTM
├── safety/
│   ├── __init__.py              Create — empty package marker
│   ├── geofence.py              Create — shapely.contains GATE, 10 Hz
│   └── tilt_guard.py            Create — BNO085 tilt >30° GATE
└── map/
    ├── __init__.py              Create — empty package marker
    ├── map_store.py             Create — GeoJSON load/save/feature CRUD
    └── teach_in.py              Create — GPS point recording, auto-close <1m from start
tests/
├── nav/
│   ├── __init__.py              Create
│   ├── test_gps_reader.py       Create
│   ├── test_ntrip_client.py     Create
│   ├── test_imu_reader.py       Create
│   ├── test_odometry.py         Create
│   └── test_localizer.py        Create
├── safety/
│   ├── __init__.py              Create
│   ├── test_geofence.py         Create
│   └── test_tilt_guard.py       Create
└── map/
    ├── __init__.py              Create
    ├── test_map_store.py        Create
    └── test_teach_in.py         Create
requirements.txt                 Modify — add filterpy, pynmea2, utm, shapely, adafruit libs
```

---

## Task 1: GPS Reader

Parses NMEA GGA sentences from the Quectel LC29H serial port. Produces a `GpsFix` dataclass with UTM coordinates and fix quality. Forwards the raw GGA sentence to an optional callback for the NTRIP client.

**Files:**
- Create: `mower/nav/__init__.py`
- Create: `mower/nav/gps_reader.py`
- Create: `tests/nav/__init__.py`
- Create: `tests/nav/test_gps_reader.py`
- Modify: `requirements.txt`

- [ ] **Step 1: Add new dependencies to requirements.txt**

```
pyserial==3.5
crcmod==1.7
pytest==8.3.5
pytest-mock==3.14.0
filterpy==0.1.9
pynmea2==1.19.0
utm==0.7.0
shapely==2.0.6
adafruit-blinka==8.48.0
adafruit-circuitpython-bno08x==1.4.3
```

- [ ] **Step 2: Install new dependencies**

```bash
pip install filterpy==0.1.9 pynmea2==1.19.0 utm==0.7.0 shapely==2.0.6
# Hardware libs: install only if on RPi 5 with actual hardware
# pip install adafruit-blinka adafruit-circuitpython-bno08x
```

- [ ] **Step 3: Create package markers**

`mower/nav/__init__.py` — empty file  
`tests/nav/__init__.py` — empty file

- [ ] **Step 4: Write failing tests**

File: `tests/nav/test_gps_reader.py`

```python
import time
import pytest
from unittest.mock import MagicMock, patch
from mower.nav.gps_reader import GpsReader, GpsFix, RTK_FIXED, RTK_FLOAT


# A valid GGA sentence with RTK Fixed quality (field 6 = 4)
RTK_FIXED_GGA = (
    "$GNGGA,123519.00,4807.038,N,01131.000,E,4,08,0.9,545.4,M,46.9,M,,*47\r\n"
)
# RTK Float (field 6 = 5)
RTK_FLOAT_GGA = (
    "$GNGGA,123519.00,4807.038,N,01131.000,E,5,08,1.5,545.4,M,46.9,M,,*42\r\n"
)
# No fix (field 6 = 0)
NO_FIX_GGA = (
    "$GNGGA,123519.00,4807.038,N,01131.000,E,0,00,99.9,0.0,M,0.0,M,,*4B\r\n"
)


def _make_reader_with_sentences(sentences: list[str]) -> tuple[GpsReader, list[GpsFix]]:
    """Returns a GpsReader whose serial port replays the given sentences."""
    received: list[GpsFix] = []

    reader = GpsReader(port="/dev/ttyFake", baud=115200)
    reader.on_fix = received.append

    # Inject a mock serial that returns sentences line by line
    mock_serial = MagicMock()
    mock_serial.__enter__ = lambda s: s
    mock_serial.__exit__ = MagicMock(return_value=False)
    encoded = [s.encode() for s in sentences] + [b""]  # empty to signal end
    mock_serial.readline.side_effect = encoded

    with patch("mower.nav.gps_reader.serial.Serial", return_value=mock_serial):
        reader._running = True
        reader._read_loop_once()  # drives one iteration per sentence

    return reader, received


class TestGpsFix:
    def test_rtk_fixed_produces_fix(self):
        fixes: list[GpsFix] = []
        reader = GpsReader(port="/dev/fake")
        reader.on_fix = fixes.append
        reader._parse_gga(RTK_FIXED_GGA.strip())
        assert len(fixes) == 1
        fix = fixes[0]
        assert fix.fix_quality == RTK_FIXED
        assert abs(fix.lat - 48.117300) < 0.0001
        assert abs(fix.lon - 11.516667) < 0.0001
        assert fix.utm_x > 0
        assert fix.utm_y > 0

    def test_rtk_float_fix_quality(self):
        fixes: list[GpsFix] = []
        reader = GpsReader(port="/dev/fake")
        reader.on_fix = fixes.append
        reader._parse_gga(RTK_FLOAT_GGA.strip())
        assert fixes[0].fix_quality == RTK_FLOAT

    def test_no_fix_not_published(self):
        fixes: list[GpsFix] = []
        reader = GpsReader(port="/dev/fake")
        reader.on_fix = fixes.append
        reader._parse_gga(NO_FIX_GGA.strip())
        assert len(fixes) == 0

    def test_non_gga_sentence_ignored(self):
        fixes: list[GpsFix] = []
        reader = GpsReader(port="/dev/fake")
        reader.on_fix = fixes.append
        reader._parse_gga("$GNRMC,123519.00,A,4807.038,N,01131.000,E,0.1,0.0,010101,,,A*6E")
        assert len(fixes) == 0

    def test_gga_forwarded_to_ntrip_callback(self):
        forwarded: list[str] = []
        reader = GpsReader(port="/dev/fake")
        reader.on_fix = lambda _: None
        reader.on_gga_sentence = forwarded.append
        reader._parse_gga(RTK_FIXED_GGA.strip())
        assert len(forwarded) == 1
        assert "GNGGA" in forwarded[0]

    def test_utm_zone_populated(self):
        fixes: list[GpsFix] = []
        reader = GpsReader(port="/dev/fake")
        reader.on_fix = fixes.append
        reader._parse_gga(RTK_FIXED_GGA.strip())
        assert fixes[0].utm_zone_number == 32  # Munich is UTM zone 32U
        assert fixes[0].utm_zone_letter == "U"

    def test_malformed_sentence_does_not_raise(self):
        reader = GpsReader(port="/dev/fake")
        reader.on_fix = lambda _: None
        reader._parse_gga("$GNGGA,BAD_DATA,,,,,,,,,,,,,")  # should not raise
```

- [ ] **Step 5: Run tests to verify they fail**

```bash
pytest tests/nav/test_gps_reader.py -v
```

Expected: `ModuleNotFoundError: No module named 'mower.nav.gps_reader'`

- [ ] **Step 6: Implement GPS Reader**

File: `mower/nav/gps_reader.py`

```python
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
        """Test helper: processes sentences until readline() raises StopIteration."""
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
            msg = pynmea2.parse(sentence)
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
```

- [ ] **Step 7: Run tests to verify they pass**

```bash
pytest tests/nav/test_gps_reader.py -v
```

Expected: 7 PASSED

- [ ] **Step 8: Run full suite to check no regressions**

```bash
pytest -v
```

Expected: all previous tests + 7 new = all PASSED

- [ ] **Step 9: Commit**

```bash
git add mower/nav/__init__.py mower/nav/gps_reader.py \
        tests/nav/__init__.py tests/nav/test_gps_reader.py \
        requirements.txt
git commit -m "feat(nav): GPS reader with NMEA GGA parsing and RTK quality flags"
```

---

## Task 2: NTRIP Client

Connects to an NTRIP caster (SAPOS / RTK2go) via TCP socket, sends GGA position sentences for VRS-based corrections, and forwards received RTCM3 data via `on_rtcm` callback. Used by wiring `GpsReader.on_gga_sentence → NtripClient.send_gga`.

**Files:**
- Create: `mower/nav/ntrip_client.py`
- Create: `tests/nav/test_ntrip_client.py`

- [ ] **Step 1: Write failing tests**

File: `tests/nav/test_ntrip_client.py`

```python
import queue
import socket
import threading
import time
import pytest
from unittest.mock import MagicMock, patch, call
from mower.nav.ntrip_client import NtripClient


def _make_client(**kwargs) -> NtripClient:
    defaults = dict(host="ntrip.example.com", port=2101,
                    mountpoint="SAPOS", user="user", password="pass")
    defaults.update(kwargs)
    return NtripClient(**defaults)


class TestNtripClientConnect:
    def test_http_request_contains_mountpoint(self):
        client = _make_client(mountpoint="MY_MOUNT")
        sent_data: list[bytes] = []

        mock_sock = MagicMock()
        mock_sock.recv.side_effect = [b"ICY 200 OK\r\n\r\n", b""]
        mock_sock.sendall.side_effect = lambda d: sent_data.append(d)

        with patch("mower.nav.ntrip_client.socket.create_connection",
                   return_value=mock_sock):
            sock = client._connect()

        combined = b"".join(sent_data).decode()
        assert "GET /MY_MOUNT HTTP/1.0" in combined

    def test_http_request_contains_basic_auth(self):
        import base64
        client = _make_client(user="testuser", password="secret")
        sent_data: list[bytes] = []

        mock_sock = MagicMock()
        mock_sock.recv.return_value = b"ICY 200 OK\r\n\r\n"
        mock_sock.sendall.side_effect = lambda d: sent_data.append(d)

        with patch("mower.nav.ntrip_client.socket.create_connection",
                   return_value=mock_sock):
            client._connect()

        combined = b"".join(sent_data).decode()
        expected = base64.b64encode(b"testuser:secret").decode()
        assert expected in combined

    def test_rejected_response_raises(self):
        client = _make_client()
        mock_sock = MagicMock()
        mock_sock.recv.return_value = b"HTTP/1.1 401 Unauthorized\r\n\r\n"

        with patch("mower.nav.ntrip_client.socket.create_connection",
                   return_value=mock_sock):
            with pytest.raises(ConnectionError):
                client._connect()


class TestNtripClientSendGga:
    def test_send_gga_queued(self):
        client = _make_client()
        client.send_gga("$GNGGA,123519,...,*47")
        assert not client._gga_queue.empty()

    def test_send_gga_queue_does_not_block_when_full(self):
        client = _make_client()
        for _ in range(10):  # overfill the maxsize=4 queue
            client.send_gga("$GNGGA,test")
        # Should not raise or block


class TestNtripClientRtcmCallback:
    def test_on_rtcm_called_with_received_data(self):
        client = _make_client()
        received: list[bytes] = []
        client.on_rtcm = received.append

        rtcm_data = b"\xd3\x00\x13\x3e\xd7\xd3\x02\x02\x98\x0e\xde\xef\x34\xb4"
        mock_sock = MagicMock()
        mock_sock.recv.side_effect = [
            b"ICY 200 OK\r\n\r\n",
            rtcm_data,
            socket.timeout(),
        ]
        mock_sock.sendall = MagicMock()

        call_count = [0]

        def fake_connect(addr, timeout=None):
            call_count[0] += 1
            if call_count[0] > 1:
                raise OSError("stop")
            return mock_sock

        with patch("mower.nav.ntrip_client.socket.create_connection",
                   side_effect=fake_connect), \
             patch("mower.nav.ntrip_client.time.sleep"):
            client._running = True
            try:
                client._run()
            except OSError:
                pass

        assert rtcm_data in received
```

- [ ] **Step 2: Run tests to verify they fail**

```bash
pytest tests/nav/test_ntrip_client.py -v
```

Expected: `ModuleNotFoundError: No module named 'mower.nav.ntrip_client'`

- [ ] **Step 3: Implement NTRIP Client**

File: `mower/nav/ntrip_client.py`

```python
import base64
import logging
import queue
import socket
import threading
import time
from typing import Callable, Optional

logger = logging.getLogger(__name__)


class NtripClient:
    """NTRIP 2.0 client: receives RTCM3 corrections from a caster.

    Usage:
        client = NtripClient(host, port, mountpoint, user, password)
        client.on_rtcm = lambda data: gps_serial.write(data)
        gps_reader.on_gga_sentence = client.send_gga
        client.start()
    """

    def __init__(self, host: str, port: int, mountpoint: str,
                 user: str, password: str):
        self._host = host
        self._port = port
        self._mountpoint = mountpoint
        self._user = user
        self._password = password
        self._gga_queue: queue.Queue = queue.Queue(maxsize=4)
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self.on_rtcm: Optional[Callable[[bytes], None]] = None

    def start(self):
        self._running = True
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=3.0)

    def send_gga(self, sentence: str):
        """Forward a GGA sentence to the caster for VRS-based corrections."""
        line = sentence.strip() + "\r\n"
        try:
            self._gga_queue.put_nowait(line)
        except queue.Full:
            pass

    def _connect(self) -> socket.socket:
        credentials = base64.b64encode(
            f"{self._user}:{self._password}".encode()
        ).decode()
        request = (
            f"GET /{self._mountpoint} HTTP/1.0\r\n"
            f"Host: {self._host}\r\n"
            f"Ntrip-Version: Ntrip/2.0\r\n"
            f"User-Agent: NTRIP MowerClient/1.0\r\n"
            f"Authorization: Basic {credentials}\r\n"
            f"\r\n"
        )
        sock = socket.create_connection((self._host, self._port), timeout=10.0)
        sock.sendall(request.encode())
        resp = sock.recv(1024).decode(errors="replace")
        if "ICY 200 OK" not in resp and "200 OK" not in resp:
            sock.close()
            raise ConnectionError(f"NTRIP refused: {resp[:80]}")
        return sock

    def _run(self):
        while self._running:
            try:
                sock = self._connect()
                sock.settimeout(5.0)
                logger.info("NTRIP connected to %s/%s", self._host, self._mountpoint)
                while self._running:
                    try:
                        gga = self._gga_queue.get_nowait()
                        sock.sendall(gga.encode())
                    except queue.Empty:
                        pass
                    try:
                        data = sock.recv(4096)
                        if data and self.on_rtcm:
                            self.on_rtcm(data)
                    except socket.timeout:
                        pass
            except Exception as e:
                logger.warning("NTRIP error: %s — reconnecting in 5 s", e)
                time.sleep(5.0)
```

- [ ] **Step 4: Run tests to verify they pass**

```bash
pytest tests/nav/test_ntrip_client.py -v
```

Expected: 6 PASSED

- [ ] **Step 5: Run full suite**

```bash
pytest -v
```

Expected: all PASSED

- [ ] **Step 6: Commit**

```bash
git add mower/nav/ntrip_client.py tests/nav/test_ntrip_client.py
git commit -m "feat(nav): NTRIP client for RTK correction delivery"
```

---

## Task 3: IMU Reader

Reads heading, pitch, roll, and yaw rate from the BNO085 over I2C using the Adafruit CircuitPython library. The I2C bus is injected so tests can pass a mock sensor without hardware. Published at ~100 Hz.

**Files:**
- Create: `mower/nav/imu_reader.py`
- Create: `tests/nav/test_imu_reader.py`

- [ ] **Step 1: Write failing tests**

File: `tests/nav/test_imu_reader.py`

```python
import math
import time
import pytest
from unittest.mock import MagicMock, patch
from mower.nav.imu_reader import ImuReader, ImuReading


def _make_mock_sensor(heading=45.0, pitch=2.0, roll=-1.0,
                      gyro_z_rps=0.1) -> MagicMock:
    """Returns a mock that mimics the BNO08X sensor object."""
    sensor = MagicMock()
    sensor.euler = (heading, pitch, roll)
    sensor.gyro = (0.0, 0.0, gyro_z_rps)
    return sensor


def _make_reader_with_mock_sensor(**sensor_kwargs) -> tuple[ImuReader, list[ImuReading]]:
    readings: list[ImuReading] = []
    mock_i2c = MagicMock()
    mock_sensor = _make_mock_sensor(**sensor_kwargs)

    reader = ImuReader(i2c=mock_i2c)
    reader.on_imu = readings.append

    # Patch the adafruit import so no hardware is needed
    with patch("mower.nav.imu_reader.BNO08X_I2C", return_value=mock_sensor), \
         patch("mower.nav.imu_reader.BNO_REPORT_EULER", 1), \
         patch("mower.nav.imu_reader.BNO_REPORT_GYROSCOPE", 2):
        reader._init_sensor()
        reader._read_once()

    return reader, readings


class TestImuReader:
    def test_heading_published(self):
        _, readings = _make_reader_with_mock_sensor(heading=90.0)
        assert len(readings) == 1
        assert abs(readings[0].heading_deg - 90.0) < 0.01

    def test_pitch_published(self):
        _, readings = _make_reader_with_mock_sensor(pitch=5.0)
        assert abs(readings[0].pitch_deg - 5.0) < 0.01

    def test_roll_published(self):
        _, readings = _make_reader_with_mock_sensor(roll=-3.0)
        assert abs(readings[0].roll_deg - (-3.0)) < 0.01

    def test_yaw_rate_converted_to_dps(self):
        gyro_z = 0.5  # rad/s
        _, readings = _make_reader_with_mock_sensor(gyro_z_rps=gyro_z)
        assert abs(readings[0].yaw_rate_dps - math.degrees(gyro_z)) < 0.01

    def test_heading_wraps_to_0_360(self):
        _, readings = _make_reader_with_mock_sensor(heading=370.0)
        assert 0.0 <= readings[0].heading_deg < 360.0

    def test_timestamp_is_recent(self):
        _, readings = _make_reader_with_mock_sensor()
        assert readings[0].timestamp <= time.monotonic()
        assert readings[0].timestamp > time.monotonic() - 1.0
```

- [ ] **Step 2: Run tests to verify they fail**

```bash
pytest tests/nav/test_imu_reader.py -v
```

Expected: `ModuleNotFoundError: No module named 'mower.nav.imu_reader'`

- [ ] **Step 3: Implement IMU Reader**

File: `mower/nav/imu_reader.py`

```python
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
    yaw_rate_dps: float     # degrees/s, positive = clockwise (around Z)
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
```

- [ ] **Step 4: Run tests to verify they pass**

```bash
pytest tests/nav/test_imu_reader.py -v
```

Expected: 6 PASSED

- [ ] **Step 5: Run full suite**

```bash
pytest -v
```

Expected: all PASSED

- [ ] **Step 6: Commit**

```bash
git add mower/nav/imu_reader.py tests/nav/test_imu_reader.py
git commit -m "feat(nav): BNO085 IMU reader with mock-injectable I2C for testing"
```

---

## Task 4: Odometry

Converts cumulative encoder tick counts (from HAL `SENSORS` telemetry) into speed (m/s) and distance (m). Handles 32-bit unsigned tick wrap-around. Physical constants are configurable for hardware calibration.

**Files:**
- Create: `mower/nav/odometry.py`
- Create: `tests/nav/test_odometry.py`

- [ ] **Step 1: Write failing tests**

File: `tests/nav/test_odometry.py`

```python
import pytest
from mower.nav.odometry import Odometry, OdometryUpdate, METERS_PER_TICK


class TestOdometry:
    def test_first_update_returns_none(self):
        odo = Odometry()
        result = odo.update(ticks=100, timestamp=1.0)
        assert result is None

    def test_speed_calculated_from_ticks(self):
        odo = Odometry(meters_per_tick=0.01)
        odo.update(ticks=0, timestamp=0.0)
        result = odo.update(ticks=10, timestamp=1.0)  # 10 ticks in 1 s
        assert result is not None
        assert abs(result.speed_mps - 0.10) < 1e-6

    def test_delta_distance_calculated(self):
        odo = Odometry(meters_per_tick=0.01)
        odo.update(ticks=0, timestamp=0.0)
        result = odo.update(ticks=5, timestamp=0.5)
        assert abs(result.delta_distance_m - 0.05) < 1e-6

    def test_total_distance_accumulates(self):
        odo = Odometry(meters_per_tick=0.01)
        odo.update(ticks=0, timestamp=0.0)
        odo.update(ticks=10, timestamp=1.0)
        result = odo.update(ticks=20, timestamp=2.0)
        assert abs(result.total_distance_m - 0.20) < 1e-6

    def test_32bit_wraparound_handled(self):
        odo = Odometry(meters_per_tick=0.01)
        odo.update(ticks=0xFFFFFF00, timestamp=0.0)
        # Wraps: new_ticks=0x00000010 → delta = 0x110 = 272 ticks
        result = odo.update(ticks=0x10, timestamp=1.0)
        assert result is not None
        assert result.delta_distance_m > 0  # wrap handled, not negative

    def test_zero_dt_returns_none(self):
        odo = Odometry()
        odo.update(ticks=0, timestamp=1.0)
        result = odo.update(ticks=100, timestamp=1.0)  # same timestamp
        assert result is None

    def test_reset_clears_state(self):
        odo = Odometry(meters_per_tick=0.01)
        odo.update(ticks=100, timestamp=1.0)
        odo.reset()
        result = odo.update(ticks=200, timestamp=2.0)
        assert result is None  # first update after reset
```

- [ ] **Step 2: Run tests to verify they fail**

```bash
pytest tests/nav/test_odometry.py -v
```

Expected: `ModuleNotFoundError: No module named 'mower.nav.odometry'`

- [ ] **Step 3: Implement Odometry**

File: `mower/nav/odometry.py`

```python
import time
from dataclasses import dataclass
from typing import Optional

# Physical constants — tune to actual hardware
# 1:14 tractor rear wheel: ~60 mm diameter → circumference ≈ 188 mm
WHEEL_CIRCUMFERENCE_M: float = 0.188
TICKS_PER_REVOLUTION: int = 20
METERS_PER_TICK: float = WHEEL_CIRCUMFERENCE_M / TICKS_PER_REVOLUTION


@dataclass
class OdometryUpdate:
    speed_mps: float            # instantaneous speed (m/s)
    delta_distance_m: float     # distance travelled since last update (m)
    total_distance_m: float     # cumulative distance since reset (m)
    timestamp: float            # time.monotonic() of this update


class Odometry:
    """Converts cumulative encoder tick counts into speed and distance.

    The encoder on the Teensy sends unsigned 32-bit ticks that only increment
    (Phase 1 limitation — direction tracking deferred). Wrap-around at 2^32
    is handled via modular arithmetic.
    """

    def __init__(self, meters_per_tick: float = METERS_PER_TICK):
        self._meters_per_tick = meters_per_tick
        self._prev_ticks: Optional[int] = None
        self._prev_timestamp: Optional[float] = None
        self._total_distance_m: float = 0.0

    def update(self, ticks: int, timestamp: float) -> Optional[OdometryUpdate]:
        """Call with encoder_ticks from HAL SENSORS telemetry.

        Returns None on the first call (no previous state to compute delta).
        """
        if self._prev_ticks is None:
            self._prev_ticks = ticks
            self._prev_timestamp = timestamp
            return None

        dt = timestamp - self._prev_timestamp
        if dt <= 0:
            return None

        # 32-bit unsigned wrap-around: delta = (new - old) mod 2^32
        delta_ticks = (ticks - self._prev_ticks) & 0xFFFFFFFF
        delta_m = delta_ticks * self._meters_per_tick
        speed = delta_m / dt

        self._total_distance_m += delta_m
        self._prev_ticks = ticks
        self._prev_timestamp = timestamp

        return OdometryUpdate(
            speed_mps=speed,
            delta_distance_m=delta_m,
            total_distance_m=self._total_distance_m,
            timestamp=timestamp,
        )

    def reset(self):
        self._prev_ticks = None
        self._prev_timestamp = None
        self._total_distance_m = 0.0
```

- [ ] **Step 4: Run tests to verify they pass**

```bash
pytest tests/nav/test_odometry.py -v
```

Expected: 7 PASSED

- [ ] **Step 5: Run full suite**

```bash
pytest -v
```

Expected: all PASSED

- [ ] **Step 6: Commit**

```bash
git add mower/nav/odometry.py tests/nav/test_odometry.py
git commit -m "feat(nav): odometry module — encoder ticks to speed and distance"
```

---

## Task 5: EKF Localizer

Fuses GPS, IMU, and odometry measurements into a continuous pose estimate using an Unscented Kalman Filter (filterpy). State: `[utm_x, utm_y, heading_rad, speed_mps, yaw_rate_rps]`. GPS and IMU provide independent correction updates; the UKF predict step runs between any two sensor updates.

**Files:**
- Create: `mower/nav/localizer.py`
- Create: `tests/nav/test_localizer.py`

- [ ] **Step 1: Write failing tests**

File: `tests/nav/test_localizer.py`

```python
import math
import time
import pytest
from mower.nav.localizer import Localizer, Pose
from mower.nav.gps_reader import GpsFix, RTK_FIXED
from mower.nav.imu_reader import ImuReading
from mower.nav.odometry import OdometryUpdate


def _make_fix(utm_x=500000.0, utm_y=5320000.0, fix_quality=RTK_FIXED,
              ts=1.0) -> GpsFix:
    return GpsFix(
        lat=48.0, lon=11.5,
        alt=500.0,
        utm_x=utm_x, utm_y=utm_y,
        utm_zone_number=32, utm_zone_letter="U",
        fix_quality=fix_quality,
        hdop=0.9,
        timestamp=ts,
    )


def _make_imu(heading_deg=0.0, yaw_rate_dps=0.0, pitch=0.0, roll=0.0,
              ts=1.0) -> ImuReading:
    return ImuReading(
        heading_deg=heading_deg,
        pitch_deg=pitch,
        roll_deg=roll,
        yaw_rate_dps=yaw_rate_dps,
        timestamp=ts,
    )


def _make_odo(speed=0.5, delta=0.5, total=0.5, ts=1.0) -> OdometryUpdate:
    return OdometryUpdate(
        speed_mps=speed,
        delta_distance_m=delta,
        total_distance_m=total,
        timestamp=ts,
    )


class TestLocalizerInit:
    def test_not_initialized_before_first_gps(self):
        loc = Localizer()
        assert not loc.initialized

    def test_initialized_after_first_gps_fix(self):
        loc = Localizer()
        loc.update_gps(_make_fix())
        assert loc.initialized

    def test_first_gps_sets_position(self):
        poses: list[Pose] = []
        loc = Localizer()
        loc.on_pose = poses.append
        loc.update_gps(_make_fix(utm_x=500000.0, utm_y=5320000.0))
        assert len(poses) == 0  # first fix only initializes, no output
        loc.update_gps(_make_fix(utm_x=500001.0, utm_y=5320001.0, ts=2.0))
        assert len(poses) == 1
        assert abs(poses[0].utm_x - 500001.0) < 1.0
        assert abs(poses[0].utm_y - 5320001.0) < 1.0


class TestLocalizerUpdates:
    def test_gps_update_shifts_position(self):
        poses: list[Pose] = []
        loc = Localizer()
        loc.on_pose = poses.append
        loc.update_gps(_make_fix(utm_x=500000.0, utm_y=5320000.0, ts=0.0))
        loc.update_gps(_make_fix(utm_x=500010.0, utm_y=5320000.0, ts=1.0))
        # After GPS correction, x should be close to 500010
        assert abs(poses[-1].utm_x - 500010.0) < 2.0

    def test_imu_update_requires_initialization_first(self):
        poses: list[Pose] = []
        loc = Localizer()
        loc.on_pose = poses.append
        loc.update_imu(_make_imu(ts=1.0))  # before GPS init — must not raise
        assert len(poses) == 0

    def test_imu_update_after_init_publishes_pose(self):
        poses: list[Pose] = []
        loc = Localizer()
        loc.on_pose = poses.append
        loc.update_gps(_make_fix(ts=0.0))
        loc.update_imu(_make_imu(heading_deg=90.0, ts=1.0))
        assert len(poses) == 1

    def test_odometry_update_after_init_publishes_pose(self):
        poses: list[Pose] = []
        loc = Localizer()
        loc.on_pose = poses.append
        loc.update_gps(_make_fix(ts=0.0))
        loc.update_odometry(_make_odo(ts=1.0))
        assert len(poses) == 1

    def test_fix_quality_propagated_to_pose(self):
        from mower.nav.gps_reader import RTK_FLOAT
        poses: list[Pose] = []
        loc = Localizer()
        loc.on_pose = poses.append
        loc.update_gps(_make_fix(fix_quality=RTK_FLOAT, ts=0.0))
        loc.update_gps(_make_fix(fix_quality=RTK_FLOAT, ts=1.0))
        assert poses[-1].fix_quality == RTK_FLOAT

    def test_pose_timestamp_matches_sensor(self):
        poses: list[Pose] = []
        loc = Localizer()
        loc.on_pose = poses.append
        loc.update_gps(_make_fix(ts=0.0))
        loc.update_odometry(_make_odo(ts=42.5))
        assert abs(poses[-1].timestamp - 42.5) < 0.01
```

- [ ] **Step 2: Run tests to verify they fail**

```bash
pytest tests/nav/test_localizer.py -v
```

Expected: `ModuleNotFoundError: No module named 'mower.nav.localizer'`

- [ ] **Step 3: Implement EKF Localizer**

File: `mower/nav/localizer.py`

```python
import logging
import math
import threading
from dataclasses import dataclass
from typing import Callable, Optional

import numpy as np
from filterpy.kalman import UnscentedKalmanFilter
from filterpy.kalman import MerweScaledSigmaPoints

from mower.nav.gps_reader import GpsFix
from mower.nav.imu_reader import ImuReading
from mower.nav.odometry import OdometryUpdate

logger = logging.getLogger(__name__)

# State vector: [utm_x, utm_y, heading_rad, speed_mps, yaw_rate_rps]
# heading: East = 0, counter-clockwise positive (standard math/UTM convention)
_DIM_X = 5


def _fx(x: np.ndarray, dt: float) -> np.ndarray:
    """Process model: constant velocity + yaw rate (kinematic bicycle)."""
    heading = x[2]
    v = x[3]
    yaw_rate = x[4]
    return np.array([
        x[0] + v * math.cos(heading) * dt,
        x[1] + v * math.sin(heading) * dt,
        x[2] + yaw_rate * dt,
        v,
        yaw_rate,
    ])


def _hx_gps(x: np.ndarray) -> np.ndarray:
    return x[:2]  # [utm_x, utm_y]


def _hx_imu(x: np.ndarray) -> np.ndarray:
    return np.array([x[2], x[4]])  # [heading_rad, yaw_rate_rps]


def _hx_odo(x: np.ndarray) -> np.ndarray:
    return np.array([x[3]])  # [speed_mps]


# Measurement noise covariances (tuned empirically in Phase 2)
_R_GPS = np.diag([0.05 ** 2, 0.05 ** 2])          # 5 cm RTK accuracy
_R_IMU = np.diag([math.radians(2.0) ** 2,          # 2° heading noise
                  math.radians(1.0) ** 2])           # 1°/s gyro noise
_R_ODO = np.array([[0.05 ** 2]])                    # 5 cm/s speed noise


def _compass_to_rad(heading_deg: float) -> float:
    """Convert compass heading (North=0, CW) to math angle (East=0, CCW)."""
    return math.radians(90.0 - heading_deg)


@dataclass
class Pose:
    utm_x: float
    utm_y: float
    heading_rad: float      # 0 = East, CCW positive
    speed_mps: float
    yaw_rate_rps: float
    timestamp: float
    fix_quality: int        # from latest GPS frame


class Localizer:
    """UKF sensor fusion: GPS + IMU + Odometry → Pose at sensor rate.

    Call update_gps / update_imu / update_odometry from their respective
    reader callbacks. Each update call triggers a predict step (using elapsed
    dt) followed by a measurement correction, then publishes a Pose via on_pose.
    """

    def __init__(self):
        points = MerweScaledSigmaPoints(n=_DIM_X, alpha=0.1, beta=2.0, kappa=-2.0)
        self._ukf = UnscentedKalmanFilter(
            dim_x=_DIM_X, dim_z=2,     # dim_z=2 for GPS (default hx)
            dt=0.1, fx=_fx, hx=_hx_gps,
            points=points,
        )
        self._ukf.Q = np.diag([0.02, 0.02, 0.002, 0.5, 0.05])
        self._ukf.P = np.eye(_DIM_X) * 500.0   # large initial uncertainty
        self._initialized = False
        self._latest_fix_quality: int = 0
        self._last_predict_time: Optional[float] = None
        self._lock = threading.Lock()
        self.on_pose: Optional[Callable[[Pose], None]] = None

    @property
    def initialized(self) -> bool:
        return self._initialized

    def update_gps(self, fix: GpsFix):
        with self._lock:
            self._latest_fix_quality = fix.fix_quality
            if not self._initialized:
                self._ukf.x = np.array([
                    fix.utm_x, fix.utm_y, 0.0, 0.0, 0.0
                ])
                self._last_predict_time = fix.timestamp
                self._initialized = True
                return  # first fix only seeds state — no pose published
            self._predict(fix.timestamp)
            self._ukf.update(
                z=np.array([fix.utm_x, fix.utm_y]),
                hx=_hx_gps, R=_R_GPS,
            )
            self._publish(fix.timestamp)

    def update_imu(self, reading: ImuReading):
        with self._lock:
            if not self._initialized:
                return
            self._predict(reading.timestamp)
            heading_rad = _compass_to_rad(reading.heading_deg)
            yaw_rate_rps = math.radians(reading.yaw_rate_dps)
            self._ukf.update(
                z=np.array([heading_rad, yaw_rate_rps]),
                hx=_hx_imu, R=_R_IMU,
            )
            self._publish(reading.timestamp)

    def update_odometry(self, odo: OdometryUpdate):
        with self._lock:
            if not self._initialized:
                return
            self._predict(odo.timestamp)
            self._ukf.update(
                z=np.array([odo.speed_mps]),
                hx=_hx_odo, R=_R_ODO,
            )
            self._publish(odo.timestamp)

    def _predict(self, timestamp: float):
        dt = timestamp - self._last_predict_time
        if dt > 0:
            self._ukf.predict(dt=dt)
        self._last_predict_time = timestamp

    def _publish(self, timestamp: float):
        x = self._ukf.x
        pose = Pose(
            utm_x=float(x[0]),
            utm_y=float(x[1]),
            heading_rad=float(x[2]),
            speed_mps=float(x[3]),
            yaw_rate_rps=float(x[4]),
            timestamp=timestamp,
            fix_quality=self._latest_fix_quality,
        )
        if self.on_pose:
            self.on_pose(pose)
```

- [ ] **Step 4: Run tests to verify they pass**

```bash
pytest tests/nav/test_localizer.py -v
```

Expected: 8 PASSED

- [ ] **Step 5: Run full suite**

```bash
pytest -v
```

Expected: all PASSED

- [ ] **Step 6: Commit**

```bash
git add mower/nav/localizer.py tests/nav/test_localizer.py
git commit -m "feat(nav): UKF localizer fusing GPS + IMU + odometry into UTM pose"
```

---

## Task 6: Geofencing GATE

Checks the robot's current UTM position against the boundary polygon from the GeoJSON map at 10 Hz. On violation, calls `on_violation` callback (upper layer triggers ESTOP). This is a **safety gate** — it must pass before Phase 3 autonomous driving.

**Files:**
- Create: `mower/safety/__init__.py`
- Create: `mower/safety/geofence.py`
- Create: `tests/safety/__init__.py`
- Create: `tests/safety/test_geofence.py`

- [ ] **Step 1: Create package markers**

`mower/safety/__init__.py` — empty  
`tests/safety/__init__.py` — empty

- [ ] **Step 2: Write failing tests**

File: `tests/safety/test_geofence.py`

```python
import math
import threading
import time
import pytest
from mower.safety.geofence import Geofence
from mower.nav.localizer import Pose
from mower.nav.gps_reader import RTK_FIXED


def _make_pose(utm_x: float, utm_y: float) -> Pose:
    return Pose(
        utm_x=utm_x, utm_y=utm_y,
        heading_rad=0.0, speed_mps=0.0, yaw_rate_rps=0.0,
        timestamp=time.monotonic(), fix_quality=RTK_FIXED,
    )


# A simple 10 m × 10 m square centered at (500000, 5320000)
BOUNDARY_COORDS = [
    [500000.0 - 5, 5320000.0 - 5],
    [500000.0 + 5, 5320000.0 - 5],
    [500000.0 + 5, 5320000.0 + 5],
    [500000.0 - 5, 5320000.0 + 5],
    [500000.0 - 5, 5320000.0 - 5],  # closed
]


class TestGeofence:
    def test_point_inside_no_violation(self):
        violations: list[Pose] = []
        gf = Geofence(boundary_utm_coords=BOUNDARY_COORDS)
        gf.on_violation = violations.append
        gf.check(_make_pose(500000.0, 5320000.0))
        assert len(violations) == 0

    def test_point_outside_triggers_violation(self):
        violations: list[Pose] = []
        gf = Geofence(boundary_utm_coords=BOUNDARY_COORDS)
        gf.on_violation = violations.append
        gf.check(_make_pose(500100.0, 5320100.0))  # far outside
        assert len(violations) == 1

    def test_point_on_boundary_no_violation(self):
        """Shapely: polygon.contains() returns False for points exactly on boundary.
        We use covers() so that boundary points are considered safe."""
        violations: list[Pose] = []
        gf = Geofence(boundary_utm_coords=BOUNDARY_COORDS)
        gf.on_violation = violations.append
        gf.check(_make_pose(500000.0 - 5, 5320000.0))  # exactly on west edge
        assert len(violations) == 0

    def test_from_geojson_feature(self):
        """Geofence can be constructed from a GeoJSON boundary Feature."""
        feature = {
            "type": "Feature",
            "properties": {"type": "boundary"},
            "geometry": {
                "type": "Polygon",
                "coordinates": [BOUNDARY_COORDS],
            },
        }
        gf = Geofence.from_geojson_feature(feature)
        violations: list[Pose] = []
        gf.on_violation = violations.append
        gf.check(_make_pose(500100.0, 5320100.0))
        assert len(violations) == 1

    def test_violation_pose_is_the_offending_pose(self):
        violations: list[Pose] = []
        gf = Geofence(boundary_utm_coords=BOUNDARY_COORDS)
        gf.on_violation = violations.append
        bad_pose = _make_pose(600000.0, 6000000.0)
        gf.check(bad_pose)
        assert violations[0] is bad_pose
```

- [ ] **Step 3: Run tests to verify they fail**

```bash
pytest tests/safety/test_geofence.py -v
```

Expected: `ModuleNotFoundError: No module named 'mower.safety.geofence'`

- [ ] **Step 4: Implement Geofencing**

File: `mower/safety/geofence.py`

```python
import logging
from typing import Callable, Optional

from shapely.geometry import Point, Polygon

from mower.nav.localizer import Pose

logger = logging.getLogger(__name__)


class Geofence:
    """Checks UTM position against a boundary polygon.

    Call check(pose) at 10 Hz. If the position falls outside the boundary,
    on_violation(pose) is called immediately.

    Uses shapely.covers() so points exactly on the boundary edge are safe.
    """

    def __init__(self, boundary_utm_coords: list):
        """
        boundary_utm_coords: list of [utm_x, utm_y] pairs forming a closed ring.
        """
        self._polygon = Polygon([(c[0], c[1]) for c in boundary_utm_coords])
        self.on_violation: Optional[Callable[[Pose], None]] = None

    @classmethod
    def from_geojson_feature(cls, feature: dict) -> "Geofence":
        """Construct from a GeoJSON Feature with geometry.type == 'Polygon'."""
        coords = feature["geometry"]["coordinates"][0]  # outer ring
        return cls(boundary_utm_coords=coords)

    def check(self, pose: Pose):
        """Check if pose is within the geofence. Call at 10 Hz."""
        point = Point(pose.utm_x, pose.utm_y)
        if not self._polygon.covers(point):
            logger.warning(
                "Geofence violation at UTM (%.2f, %.2f)", pose.utm_x, pose.utm_y
            )
            if self.on_violation:
                self.on_violation(pose)
```

- [ ] **Step 5: Run tests to verify they pass**

```bash
pytest tests/safety/test_geofence.py -v
```

Expected: 5 PASSED

- [ ] **Step 6: Run full suite**

```bash
pytest -v
```

Expected: all PASSED

- [ ] **Step 7: Commit**

```bash
git add mower/safety/__init__.py mower/safety/geofence.py \
        tests/safety/__init__.py tests/safety/test_geofence.py
git commit -m "feat(safety): geofencing GATE — shapely polygon boundary check"
```

---

## Task 7: Tilt Guard GATE

Monitors IMU pitch and roll. If the combined tilt angle exceeds 30° (configurable), calls `on_tilt` callback — upper layer sends ESTOP and stops the blade. This is a **safety gate** — must be wired to `hardware_interface.estop()` before Phase 3.

**Files:**
- Create: `mower/safety/tilt_guard.py`
- Create: `tests/safety/test_tilt_guard.py`

- [ ] **Step 1: Write failing tests**

File: `tests/safety/test_tilt_guard.py`

```python
import math
import pytest
from mower.safety.tilt_guard import TiltGuard
from mower.nav.imu_reader import ImuReading


def _make_imu(pitch: float = 0.0, roll: float = 0.0) -> ImuReading:
    return ImuReading(
        heading_deg=0.0,
        pitch_deg=pitch,
        roll_deg=roll,
        yaw_rate_dps=0.0,
        timestamp=0.0,
    )


class TestTiltGuard:
    def test_flat_no_trigger(self):
        triggered: list[ImuReading] = []
        tg = TiltGuard(threshold_deg=30.0)
        tg.on_tilt = triggered.append
        tg.check(_make_imu(pitch=0.0, roll=0.0))
        assert len(triggered) == 0

    def test_pitch_over_threshold_triggers(self):
        triggered: list[ImuReading] = []
        tg = TiltGuard(threshold_deg=30.0)
        tg.on_tilt = triggered.append
        tg.check(_make_imu(pitch=35.0))
        assert len(triggered) == 1

    def test_roll_over_threshold_triggers(self):
        triggered: list[ImuReading] = []
        tg = TiltGuard(threshold_deg=30.0)
        tg.on_tilt = triggered.append
        tg.check(_make_imu(roll=-31.0))
        assert len(triggered) == 1

    def test_combined_tilt_uses_vector_magnitude(self):
        """pitch=25°, roll=25° → combined ≈ 35.4° > 30° → trigger."""
        triggered: list[ImuReading] = []
        tg = TiltGuard(threshold_deg=30.0)
        tg.on_tilt = triggered.append
        tg.check(_make_imu(pitch=25.0, roll=25.0))
        assert len(triggered) == 1

    def test_just_below_threshold_no_trigger(self):
        triggered: list[ImuReading] = []
        tg = TiltGuard(threshold_deg=30.0)
        tg.on_tilt = triggered.append
        tg.check(_make_imu(pitch=29.9))
        assert len(triggered) == 0

    def test_triggered_reading_passed_to_callback(self):
        triggered: list[ImuReading] = []
        tg = TiltGuard(threshold_deg=30.0)
        tg.on_tilt = triggered.append
        reading = _make_imu(pitch=45.0)
        tg.check(reading)
        assert triggered[0] is reading

    def test_custom_threshold(self):
        triggered: list[ImuReading] = []
        tg = TiltGuard(threshold_deg=15.0)
        tg.on_tilt = triggered.append
        tg.check(_make_imu(pitch=16.0))
        assert len(triggered) == 1
```

- [ ] **Step 2: Run tests to verify they fail**

```bash
pytest tests/safety/test_tilt_guard.py -v
```

Expected: `ModuleNotFoundError: No module named 'mower.safety.tilt_guard'`

- [ ] **Step 3: Implement Tilt Guard**

File: `mower/safety/tilt_guard.py`

```python
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
        """Call from ImuReader.on_imu callback (100 Hz)."""
        tilt = math.sqrt(reading.pitch_deg ** 2 + reading.roll_deg ** 2)
        if tilt >= self._threshold_deg:
            logger.warning(
                "Tilt limit exceeded: pitch=%.1f° roll=%.1f° combined=%.1f°",
                reading.pitch_deg, reading.roll_deg, tilt,
            )
            if self.on_tilt:
                self.on_tilt(reading)
```

- [ ] **Step 4: Run tests to verify they pass**

```bash
pytest tests/safety/test_tilt_guard.py -v
```

Expected: 7 PASSED

- [ ] **Step 5: Run full suite**

```bash
pytest -v
```

Expected: all PASSED

- [ ] **Step 6: Commit**

```bash
git add mower/safety/tilt_guard.py tests/safety/test_tilt_guard.py
git commit -m "feat(safety): tilt guard GATE — combined pitch+roll >30° triggers ESTOP"
```

---

## Task 8: Map Store

Loads and saves GeoJSON feature collections. Manages features by type (boundary, zone, no-go, transit-path, charging-station). Validated against the schema defined in the design spec.

**Files:**
- Create: `mower/map/__init__.py`
- Create: `mower/map/map_store.py`
- Create: `tests/map/__init__.py`
- Create: `tests/map/test_map_store.py`

- [ ] **Step 1: Create package markers**

`mower/map/__init__.py` — empty  
`tests/map/__init__.py` — empty

- [ ] **Step 2: Write failing tests**

File: `tests/map/test_map_store.py`

```python
import json
import os
import tempfile
import pytest
from mower.map.map_store import MapStore, MapFeatureType

SAMPLE_GEOJSON = {
    "type": "FeatureCollection",
    "features": [
        {
            "type": "Feature",
            "properties": {"type": "boundary"},
            "geometry": {
                "type": "Polygon",
                "coordinates": [[[0, 0], [10, 0], [10, 10], [0, 10], [0, 0]]],
            },
        },
        {
            "type": "Feature",
            "properties": {"type": "zone", "id": "zone_1", "order": 1},
            "geometry": {
                "type": "Polygon",
                "coordinates": [[[1, 1], [9, 1], [9, 9], [1, 9], [1, 1]]],
            },
        },
        {
            "type": "Feature",
            "properties": {"type": "charging-station"},
            "geometry": {"type": "Point", "coordinates": [5.0, 5.0]},
        },
    ],
}


class TestMapStore:
    def test_load_from_dict(self):
        store = MapStore()
        store.load(SAMPLE_GEOJSON)
        assert len(store.features) == 3

    def test_get_boundary(self):
        store = MapStore()
        store.load(SAMPLE_GEOJSON)
        boundary = store.get_by_type(MapFeatureType.BOUNDARY)
        assert len(boundary) == 1
        assert boundary[0]["properties"]["type"] == "boundary"

    def test_get_zones(self):
        store = MapStore()
        store.load(SAMPLE_GEOJSON)
        zones = store.get_by_type(MapFeatureType.ZONE)
        assert len(zones) == 1
        assert zones[0]["properties"]["id"] == "zone_1"

    def test_add_feature(self):
        store = MapStore()
        store.load(SAMPLE_GEOJSON)
        new_feature = {
            "type": "Feature",
            "properties": {"type": "no-go"},
            "geometry": {
                "type": "Polygon",
                "coordinates": [[[2, 2], [4, 2], [4, 4], [2, 4], [2, 2]]],
            },
        }
        store.add_feature(new_feature)
        assert len(store.get_by_type(MapFeatureType.NO_GO)) == 1

    def test_save_and_reload(self):
        store = MapStore()
        store.load(SAMPLE_GEOJSON)
        with tempfile.NamedTemporaryFile(suffix=".geojson", delete=False,
                                         mode="w") as f:
            path = f.name
        try:
            store.save(path)
            store2 = MapStore()
            store2.load_from_file(path)
            assert len(store2.features) == 3
        finally:
            os.unlink(path)

    def test_to_geojson_roundtrip(self):
        store = MapStore()
        store.load(SAMPLE_GEOJSON)
        exported = store.to_geojson()
        assert exported["type"] == "FeatureCollection"
        assert len(exported["features"]) == 3

    def test_clear_removes_all(self):
        store = MapStore()
        store.load(SAMPLE_GEOJSON)
        store.clear()
        assert len(store.features) == 0
```

- [ ] **Step 3: Run tests to verify they fail**

```bash
pytest tests/map/test_map_store.py -v
```

Expected: `ModuleNotFoundError: No module named 'mower.map.map_store'`

- [ ] **Step 4: Implement Map Store**

File: `mower/map/map_store.py`

```python
import json
import logging
from enum import Enum
from typing import Any

logger = logging.getLogger(__name__)


class MapFeatureType(str, Enum):
    BOUNDARY = "boundary"
    ZONE = "zone"
    NO_GO = "no-go"
    TRANSIT_PATH = "transit-path"
    CHARGING_STATION = "charging-station"


class MapStore:
    """In-memory GeoJSON FeatureCollection with file persistence.

    The schema matches the design spec:
      boundary     — Polygon defining the outer mowing area
      zone         — Polygon sub-area with id and order
      no-go        — Polygon exclusion zone
      transit-path — LineString between zones (blade: false)
      charging-station — Point at dock location
    """

    def __init__(self):
        self._features: list[dict] = []

    @property
    def features(self) -> list[dict]:
        return list(self._features)

    def load(self, geojson: dict):
        """Load from a GeoJSON FeatureCollection dict."""
        self._features = list(geojson.get("features", []))

    def load_from_file(self, path: str):
        with open(path, encoding="utf-8") as f:
            self.load(json.load(f))

    def save(self, path: str):
        with open(path, "w", encoding="utf-8") as f:
            json.dump(self.to_geojson(), f, indent=2)

    def to_geojson(self) -> dict:
        return {"type": "FeatureCollection", "features": list(self._features)}

    def get_by_type(self, feature_type: MapFeatureType) -> list[dict]:
        return [
            f for f in self._features
            if f.get("properties", {}).get("type") == feature_type.value
        ]

    def add_feature(self, feature: dict):
        self._features.append(feature)

    def clear(self):
        self._features.clear()
```

- [ ] **Step 5: Run tests to verify they pass**

```bash
pytest tests/map/test_map_store.py -v
```

Expected: 7 PASSED

- [ ] **Step 6: Run full suite**

```bash
pytest -v
```

Expected: all PASSED

- [ ] **Step 7: Commit**

```bash
git add mower/map/__init__.py mower/map/map_store.py \
        tests/map/__init__.py tests/map/test_map_store.py
git commit -m "feat(map): GeoJSON map store with feature CRUD and file persistence"
```

---

## Task 9: Teach-In Recorder

Records GPS poses during manual driving and builds a GeoJSON polygon. A new point is recorded every **0.5 m** of travel OR every **1 s** (whichever comes first). The polygon is automatically closed when the robot returns to within **1 m** of the start point. Incomplete recordings are saved as draft features.

**Files:**
- Create: `mower/map/teach_in.py`
- Create: `tests/map/test_teach_in.py`

- [ ] **Step 1: Write failing tests**

File: `tests/map/test_teach_in.py`

```python
import math
import time
import pytest
from mower.map.teach_in import TeachIn, TeachInState
from mower.nav.localizer import Pose
from mower.nav.gps_reader import RTK_FIXED


def _make_pose(utm_x: float, utm_y: float, ts: float = None) -> Pose:
    return Pose(
        utm_x=utm_x, utm_y=utm_y,
        heading_rad=0.0, speed_mps=0.5, yaw_rate_rps=0.0,
        timestamp=ts if ts is not None else time.monotonic(),
        fix_quality=RTK_FIXED,
    )


class TestTeachInBasic:
    def test_initial_state_is_idle(self):
        ti = TeachIn()
        assert ti.state == TeachInState.IDLE

    def test_start_transitions_to_recording(self):
        ti = TeachIn()
        ti.start("boundary")
        assert ti.state == TeachInState.RECORDING

    def test_cannot_start_twice(self):
        ti = TeachIn()
        ti.start("boundary")
        with pytest.raises(RuntimeError):
            ti.start("boundary")

    def test_update_before_start_ignored(self):
        ti = TeachIn()
        ti.update(_make_pose(0.0, 0.0))  # must not raise

    def test_first_pose_sets_start_point(self):
        ti = TeachIn()
        ti.start("boundary")
        ti.update(_make_pose(500000.0, 5320000.0))
        assert len(ti.points) == 1


class TestTeachInSampling:
    def test_point_recorded_every_half_meter(self):
        ti = TeachIn(distance_threshold_m=0.5, time_threshold_s=999.0)
        ti.start("boundary")
        ti.update(_make_pose(0.0, 0.0, ts=0.0))    # point 1
        ti.update(_make_pose(0.3, 0.0, ts=0.1))    # 0.3 m — skip
        ti.update(_make_pose(0.6, 0.0, ts=0.2))    # 0.6 m — record
        assert len(ti.points) == 2

    def test_point_recorded_every_second_regardless_of_distance(self):
        ti = TeachIn(distance_threshold_m=999.0, time_threshold_s=1.0)
        ti.start("boundary")
        ti.update(_make_pose(0.0, 0.0, ts=0.0))    # point 1
        ti.update(_make_pose(0.01, 0.0, ts=0.5))   # 0.5 s — skip
        ti.update(_make_pose(0.02, 0.0, ts=1.1))   # 1.1 s — record
        assert len(ti.points) == 2

    def test_both_thresholds_checked(self):
        """Either threshold triggers recording, whichever comes first."""
        ti = TeachIn(distance_threshold_m=0.5, time_threshold_s=1.0)
        ti.start("boundary")
        ti.update(_make_pose(0.0, 0.0, ts=0.0))
        # Distance < 0.5 m, time < 1 s → skip
        ti.update(_make_pose(0.2, 0.0, ts=0.5))
        assert len(ti.points) == 1
        # Time > 1 s → record
        ti.update(_make_pose(0.25, 0.0, ts=1.2))
        assert len(ti.points) == 2


class TestTeachInAutoClose:
    def test_auto_close_when_near_start(self):
        ti = TeachIn(close_threshold_m=1.0, min_points=4)
        ti.start("boundary")
        # Record enough points to satisfy min_points
        ti.update(_make_pose(0.0, 0.0, ts=0.0))
        ti.update(_make_pose(10.0, 0.0, ts=2.0))
        ti.update(_make_pose(10.0, 10.0, ts=4.0))
        ti.update(_make_pose(0.0, 10.0, ts=6.0))
        # Return within 1 m of start
        ti.update(_make_pose(0.5, 0.0, ts=8.0))
        assert ti.state == TeachInState.CLOSED

    def test_not_closed_before_min_points(self):
        ti = TeachIn(close_threshold_m=1.0, min_points=5)
        ti.start("boundary")
        ti.update(_make_pose(0.0, 0.0, ts=0.0))
        ti.update(_make_pose(0.5, 0.0, ts=2.0))   # return near start, but min_points=5
        assert ti.state == TeachInState.RECORDING

    def test_to_geojson_feature_on_close(self):
        ti = TeachIn(close_threshold_m=1.0, min_points=4)
        ti.start("boundary")
        ti.update(_make_pose(0.0, 0.0, ts=0.0))
        ti.update(_make_pose(10.0, 0.0, ts=2.0))
        ti.update(_make_pose(10.0, 10.0, ts=4.0))
        ti.update(_make_pose(0.0, 10.0, ts=6.0))
        ti.update(_make_pose(0.5, 0.0, ts=8.0))
        feature = ti.to_geojson_feature()
        assert feature["type"] == "Feature"
        assert feature["properties"]["type"] == "boundary"
        assert feature["geometry"]["type"] == "Polygon"

    def test_cancel_saves_draft(self):
        ti = TeachIn()
        ti.start("zone")
        ti.update(_make_pose(0.0, 0.0, ts=0.0))
        ti.update(_make_pose(5.0, 0.0, ts=2.0))
        ti.cancel()
        assert ti.state == TeachInState.CANCELLED
        draft = ti.to_geojson_feature()
        assert draft["properties"].get("draft") is True
```

- [ ] **Step 2: Run tests to verify they fail**

```bash
pytest tests/map/test_teach_in.py -v
```

Expected: `ModuleNotFoundError: No module named 'mower.map.teach_in'`

- [ ] **Step 3: Implement Teach-In**

File: `mower/map/teach_in.py`

```python
import logging
import math
import time
from enum import Enum, auto
from typing import Optional

from mower.nav.localizer import Pose

logger = logging.getLogger(__name__)

# Defaults — configurable per instance
DEFAULT_DISTANCE_THRESHOLD_M: float = 0.5
DEFAULT_TIME_THRESHOLD_S: float = 1.0
DEFAULT_CLOSE_THRESHOLD_M: float = 1.0
DEFAULT_MIN_POINTS: int = 4


class TeachInState(Enum):
    IDLE = auto()
    RECORDING = auto()
    CLOSED = auto()
    CANCELLED = auto()


def _distance(x1: float, y1: float, x2: float, y2: float) -> float:
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)


class TeachIn:
    """Records GPS poses during manual driving to build a GeoJSON polygon.

    Sampling rule: record a new point if distance since last point >= 0.5 m
    OR time since last point >= 1 s (whichever comes first).
    Auto-close: if robot returns within 1 m of start AND min_points reached.
    """

    def __init__(
        self,
        distance_threshold_m: float = DEFAULT_DISTANCE_THRESHOLD_M,
        time_threshold_s: float = DEFAULT_TIME_THRESHOLD_S,
        close_threshold_m: float = DEFAULT_CLOSE_THRESHOLD_M,
        min_points: int = DEFAULT_MIN_POINTS,
    ):
        self._dist_thresh = distance_threshold_m
        self._time_thresh = time_threshold_s
        self._close_thresh = close_threshold_m
        self._min_points = min_points
        self._state = TeachInState.IDLE
        self._feature_type: Optional[str] = None
        self._points: list[tuple[float, float]] = []  # (utm_x, utm_y)
        self._last_point: Optional[tuple[float, float]] = None
        self._last_time: Optional[float] = None

    @property
    def state(self) -> TeachInState:
        return self._state

    @property
    def points(self) -> list[tuple[float, float]]:
        return list(self._points)

    def start(self, feature_type: str):
        """Begin recording. feature_type: 'boundary', 'zone', 'no-go', etc."""
        if self._state == TeachInState.RECORDING:
            raise RuntimeError("TeachIn already recording")
        self._feature_type = feature_type
        self._points.clear()
        self._last_point = None
        self._last_time = None
        self._state = TeachInState.RECORDING

    def update(self, pose: Pose):
        """Feed current pose. Call at localizer update rate (GPS rate ~5–10 Hz)."""
        if self._state != TeachInState.RECORDING:
            return

        x, y, ts = pose.utm_x, pose.utm_y, pose.timestamp

        # First point
        if self._last_point is None:
            self._record(x, y, ts)
            return

        # Sampling decision
        dist = _distance(self._last_point[0], self._last_point[1], x, y)
        elapsed = ts - self._last_time
        if dist >= self._dist_thresh or elapsed >= self._time_thresh:
            self._record(x, y, ts)

        # Auto-close check
        if len(self._points) >= self._min_points:
            start_x, start_y = self._points[0]
            if _distance(start_x, start_y, x, y) < self._close_thresh:
                logger.info("TeachIn auto-closed after %d points", len(self._points))
                self._state = TeachInState.CLOSED

    def cancel(self):
        self._state = TeachInState.CANCELLED

    def to_geojson_feature(self) -> dict:
        """Build a GeoJSON Feature from recorded points.

        Returns a draft feature if not yet closed.
        The polygon ring is auto-closed (first point repeated as last).
        """
        is_draft = self._state != TeachInState.CLOSED
        coords = [[x, y] for x, y in self._points]
        if coords and coords[0] != coords[-1]:
            coords.append(coords[0])  # close the ring

        props = {"type": self._feature_type}
        if is_draft:
            props["draft"] = True

        return {
            "type": "Feature",
            "properties": props,
            "geometry": {
                "type": "Polygon",
                "coordinates": [coords],
            },
        }

    def _record(self, x: float, y: float, ts: float):
        self._points.append((x, y))
        self._last_point = (x, y)
        self._last_time = ts
```

- [ ] **Step 4: Run tests to verify they pass**

```bash
pytest tests/map/test_teach_in.py -v
```

Expected: 13 PASSED

- [ ] **Step 5: Run full suite**

```bash
pytest -v
```

Expected: all PASSED

- [ ] **Step 6: Commit**

```bash
git add mower/map/teach_in.py tests/map/test_teach_in.py
git commit -m "feat(map): teach-in recorder — GPS polygon recording with auto-close"
```

---

## Integration Summary

After all 9 tasks, the Phase 2 wiring in the application layer will connect:

```python
# Sensor pipeline (run in application startup)
gps_reader = GpsReader(port="/dev/ttyGPS", baud=115200)
ntrip = NtripClient(host=..., port=2101, mountpoint=..., user=..., password=...)
imu = ImuReader()                        # hardware I2C on RPi 5
odo = Odometry()
loc = Localizer()

# Sensor → Localizer
gps_reader.on_fix = loc.update_gps
gps_reader.on_gga_sentence = ntrip.send_gga
imu.on_imu = loc.update_imu
hw.on_sensors = lambda s: odo_update_and_forward(s, odo, loc)

# Safety gates → HAL
map_store = MapStore()
map_store.load_from_file("map.geojson")
[boundary] = map_store.get_by_type(MapFeatureType.BOUNDARY)
geofence = Geofence.from_geojson_feature(boundary)
tilt_guard = TiltGuard(threshold_deg=30.0)

geofence.on_violation = lambda _: (hw.estop(), hw.set_blade(False))
tilt_guard.on_tilt = lambda _: (hw.estop(), hw.set_blade(False))

loc.on_pose = geofence.check     # 10 Hz geofence check
imu.on_imu = tilt_guard.check    # 100 Hz tilt check

# Start all
ntrip.start(); gps_reader.start(); imu.start(); hw.start()
```

**Safety Gate Verification Checklist (before Phase 3):**
1. Drive robot to RTK-Fixed GPS position — confirm loc.on_pose fires with valid UTM coordinates
2. Load boundary GeoJSON — confirm geofence.check() does not fire within boundary
3. Move robot outside boundary — confirm ESTOP is sent within one check cycle (100 ms)
4. Tilt robot past 30° by hand — confirm ESTOP + blade off triggered
5. Record a Teach-In boundary (> 10 m perimeter) — confirm GeoJSON saved correctly
