# Phase 1: Hardware Foundation — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Teensy 4.1 Firmware + Python Serial Driver — bidirektionale Kommunikation mit binärem Protokoll, Motorsteuerung, Safety-Watchdog und vollständige manuelle Fahrkontrolle vom RPi 5 aus.

**Architecture:** Teensy 4.1 läuft als dedizierter Hardware-Proxy (C++/Arduino). Der RPi 5 sendet Kommandos und empfängt Telemetrie über USB-Serial mit einem binären Frame-Protokoll (CRC-8/MAXIM). Python-seitig gibt es drei Schichten: `protocol.py` (Frame-Encoding/Decoding), `serial_driver.py` (Threading, Send/Receive-Loop), `hardware_interface.py` (High-Level-API für den Rest des Systems).

**Tech Stack:** Python 3.11+, pyserial, crcmod, pytest; C++/Arduino auf Teensy 4.1, PlatformIO (empfohlen) oder Arduino IDE

---

## Dateistruktur

```
mower/
├── firmware/
│   └── mower_firmware/
│       ├── mower_firmware.ino     # Main loop, setup
│       ├── protocol.h             # Frame-Definitionen, CMD-Typen, CRC8
│       ├── motor_control.h        # Fahrantrieb via Cytron MDD10A
│       ├── motor_control.cpp
│       ├── servo_control.h        # Lenkservo
│       ├── servo_control.cpp
│       ├── blade_control.h        # Mähwerk-ESC
│       ├── blade_control.cpp
│       ├── sensor_reader.h        # Encoder, Regensensor, Lift, BMS-SOC
│       ├── sensor_reader.cpp
│       ├── watchdog.h             # Safety-Watchdog (500ms Timeout)
│       └── watchdog.cpp
├── mower/
│   ├── __init__.py
│   └── hal/
│       ├── __init__.py
│       ├── protocol.py            # Frame-Encoding/Decoding, CRC8, Konstanten
│       ├── serial_driver.py       # Serial-Thread, Send/Receive-Queue
│       └── hardware_interface.py  # High-Level-API: drive(), blade(), estop()
├── tests/
│   └── hal/
│       ├── __init__.py
│       ├── test_protocol.py       # Unit-Tests: Encoding, Decoding, CRC, Fehlerbehandlung
│       └── test_hardware_interface.py  # Tests mit Mock-SerialDriver
├── setup/
│   └── 99-mower-serial.rules      # udev-Regel für low-latency USB-Serial
└── requirements.txt
```

---

## Task 1: Projektstruktur & Dependencies

**Files:**
- Create: `requirements.txt`
- Create: `mower/__init__.py`
- Create: `mower/hal/__init__.py`
- Create: `tests/__init__.py`
- Create: `tests/hal/__init__.py`

- [ ] **Step 1: Verzeichnisse anlegen**

```bash
mkdir -p mower/hal tests/hal firmware/mower_firmware setup
touch mower/__init__.py mower/hal/__init__.py tests/__init__.py tests/hal/__init__.py
```

- [ ] **Step 2: requirements.txt erstellen**

```
pyserial==3.5
crcmod==1.7
pytest==8.3.5
pytest-mock==3.14.0
```

- [ ] **Step 3: Dependencies installieren**

```bash
pip install -r requirements.txt
```

Erwartete Ausgabe: alle 4 Pakete installiert ohne Fehler.

- [ ] **Step 4: Commit**

```bash
git init
git add requirements.txt mower/ tests/ setup/
git commit -m "chore: initial project structure for Phase 1"
```

---

## Task 2: Protokoll-Definitionen (Python)

**Files:**
- Create: `mower/hal/protocol.py`
- Create: `tests/hal/test_protocol.py`

Das Protokoll definiert den Frame-Aufbau und alle Kommando-/Telemetrie-Typen. Dies ist das Herzstück des HAL — beide Seiten (Python und Teensy) müssen identische Definitionen verwenden.

**Frame-Format:** `[0xAA][CMD_TYPE 1B][PAYLOAD_LEN 1B][PAYLOAD nB][CRC8 1B]`
**CRC:** CRC-8/MAXIM (Polynom 0x31, Initial 0x00, XOR-Out 0x00, Reflect In/Out: True)

- [ ] **Step 1: Failing Tests schreiben**

`tests/hal/test_protocol.py`:
```python
import pytest
from mower.hal.protocol import (
    CmdType, encode_frame, decode_frame, FrameError,
    encode_drive, encode_ping, encode_estop, encode_blade,
    decode_sensors, decode_soc, decode_status,
)

START_BYTE = 0xAA


class TestCrcAndFraming:
    def test_encode_frame_starts_with_start_byte(self):
        frame = encode_frame(CmdType.PING, b'\x01')
        assert frame[0] == START_BYTE

    def test_encode_frame_embeds_cmd_type(self):
        frame = encode_frame(CmdType.PING, b'\x01')
        assert frame[1] == CmdType.PING

    def test_encode_frame_embeds_payload_length(self):
        payload = b'\x01\x02\x03'
        frame = encode_frame(CmdType.PING, payload)
        assert frame[2] == len(payload)

    def test_encode_frame_crc_is_last_byte(self):
        frame = encode_frame(CmdType.PING, b'\x01')
        assert len(frame) == 5  # header(3) + payload(1) + crc(1)
        # Flipping the payload must change the CRC (last byte)
        frame2 = encode_frame(CmdType.PING, b'\x02')
        assert frame[-1] != frame2[-1]

    def test_decode_valid_frame(self):
        frame = encode_frame(CmdType.PING, b'\x07')
        cmd_type, payload = decode_frame(frame)
        assert cmd_type == CmdType.PING
        assert payload == b'\x07'

    def test_decode_wrong_start_byte_raises(self):
        frame = bytearray(encode_frame(CmdType.PING, b'\x01'))
        frame[0] = 0xBB
        with pytest.raises(FrameError, match="start byte"):
            decode_frame(bytes(frame))

    def test_decode_bad_crc_raises(self):
        frame = bytearray(encode_frame(CmdType.PING, b'\x01'))
        frame[-1] ^= 0xFF  # flip CRC
        with pytest.raises(FrameError, match="CRC"):
            decode_frame(bytes(frame))

    def test_decode_truncated_frame_raises(self):
        with pytest.raises(FrameError):
            decode_frame(b'\xAA\x01')  # too short


class TestCommandEncoding:
    def test_encode_drive(self):
        frame = encode_drive(speed=0.5, steering=15.0)
        cmd, payload = decode_frame(frame)
        assert cmd == CmdType.DRIVE
        assert len(payload) == 8  # two float32 = 8 bytes

    def test_encode_drive_speed_clipped(self):
        # speed must be clamped to [-1.0, 1.0]
        frame = encode_drive(speed=2.5, steering=0.0)
        _, payload = decode_frame(frame)
        import struct
        speed, _ = struct.unpack('<ff', payload)
        assert speed == pytest.approx(1.0)

    def test_encode_blade_on(self):
        frame = encode_blade(True)
        cmd, payload = decode_frame(frame)
        assert cmd == CmdType.BLADE
        assert payload == b'\x01'

    def test_encode_blade_off(self):
        frame = encode_blade(False)
        _, payload = decode_frame(frame)
        assert payload == b'\x00'

    def test_encode_estop(self):
        frame = encode_estop()
        cmd, payload = decode_frame(frame)
        assert cmd == CmdType.ESTOP
        assert payload == b''

    def test_encode_ping(self):
        frame = encode_ping(seq=42)
        cmd, payload = decode_frame(frame)
        assert cmd == CmdType.PING
        assert int.from_bytes(payload, 'little') == 42


class TestTelemetryDecoding:
    def test_decode_sensors(self):
        import struct
        payload = struct.pack('<HBI', 512, 0, 1024)  # rain_adc, lift, encoder
        data = decode_sensors(payload)
        assert data['rain_adc'] == 512
        assert data['lift'] is False
        assert data['encoder_ticks'] == 1024

    def test_decode_sensors_lift_true(self):
        import struct
        payload = struct.pack('<HBI', 700, 1, 0)
        data = decode_sensors(payload)
        assert data['lift'] is True

    def test_decode_soc(self):
        import struct
        payload = struct.pack('<BH', 75, 14800)  # 75%, 14.8V
        data = decode_soc(payload)
        assert data['soc_percent'] == 75
        assert data['voltage_mv'] == 14800

    def test_decode_status(self):
        import struct
        payload = struct.pack('<BBB', 1, 0, 0)  # watchdog_ok, blade_running, error_flags
        data = decode_status(payload)
        assert data['watchdog_ok'] is True
        assert data['blade_running'] is False
        assert data['error_flags'] == 0
```

- [ ] **Step 2: Tests ausführen — müssen alle FAIL**

```bash
pytest tests/hal/test_protocol.py -v
```

Erwartete Ausgabe: `ImportError` oder alle `FAILED` — `protocol.py` existiert noch nicht.

- [ ] **Step 3: `mower/hal/protocol.py` implementieren**

```python
import struct
import crcmod
from enum import IntEnum


class CmdType(IntEnum):
    DRIVE = 0x01
    BLADE = 0x02
    ESTOP = 0x03
    PING  = 0x04
    # Telemetry (Teensy → RPi)
    SENSORS = 0x10
    SOC     = 0x11
    STATUS  = 0x12


START_BYTE = 0xAA

# CRC-8/MAXIM: poly=0x31, initCrc=0x00, rev=True, xorOut=0x00
_crc8 = crcmod.predefined.mkCrcFun('crc-8-maxim')


class FrameError(Exception):
    pass


def _compute_crc(data: bytes) -> int:
    return _crc8(data)


def encode_frame(cmd_type: CmdType, payload: bytes) -> bytes:
    header = bytes([START_BYTE, int(cmd_type), len(payload)])
    body = header + payload
    crc = _compute_crc(body)
    return body + bytes([crc])


def decode_frame(data: bytes) -> tuple[CmdType, bytes]:
    if len(data) < 4:
        raise FrameError("Frame too short")
    if data[0] != START_BYTE:
        raise FrameError(f"Invalid start byte: 0x{data[0]:02X}")
    payload_len = data[2]
    expected_len = 3 + payload_len + 1
    if len(data) < expected_len:
        raise FrameError("Frame truncated")
    body = data[:3 + payload_len]
    received_crc = data[3 + payload_len]
    expected_crc = _compute_crc(body)
    if received_crc != expected_crc:
        raise FrameError(f"CRC mismatch: got 0x{received_crc:02X}, expected 0x{expected_crc:02X}")
    cmd_type = CmdType(data[1])
    payload = bytes(data[3:3 + payload_len])
    return cmd_type, payload


def encode_drive(speed: float, steering: float) -> bytes:
    speed = max(-1.0, min(1.0, speed))
    steering = max(-45.0, min(45.0, steering))
    payload = struct.pack('<ff', speed, steering)
    return encode_frame(CmdType.DRIVE, payload)


def encode_blade(state: bool) -> bytes:
    return encode_frame(CmdType.BLADE, bytes([1 if state else 0]))


def encode_estop() -> bytes:
    return encode_frame(CmdType.ESTOP, b'')


def encode_ping(seq: int) -> bytes:
    return encode_frame(CmdType.PING, seq.to_bytes(2, 'little'))


def decode_sensors(payload: bytes) -> dict:
    rain_adc, lift_raw, encoder_ticks = struct.unpack('<HBI', payload)
    return {
        'rain_adc': rain_adc,
        'lift': bool(lift_raw),
        'encoder_ticks': encoder_ticks,
    }


def decode_soc(payload: bytes) -> dict:
    soc_percent, voltage_mv = struct.unpack('<BH', payload)
    return {
        'soc_percent': soc_percent,
        'voltage_mv': voltage_mv,
    }


def decode_status(payload: bytes) -> dict:
    watchdog_ok, blade_running, error_flags = struct.unpack('<BBB', payload)
    return {
        'watchdog_ok': bool(watchdog_ok),
        'blade_running': bool(blade_running),
        'error_flags': error_flags,
    }
```

- [ ] **Step 4: Tests ausführen — müssen alle PASS**

```bash
pytest tests/hal/test_protocol.py -v
```

Erwartete Ausgabe: alle `PASSED` (17 Tests).

- [ ] **Step 5: Commit**

```bash
git add mower/hal/protocol.py tests/hal/test_protocol.py
git commit -m "feat(hal): binary serial protocol with CRC-8/MAXIM encoding"
```

---

## Task 3: Python Serial Driver

**Files:**
- Create: `mower/hal/serial_driver.py`
- Create: `tests/hal/test_hardware_interface.py` (Teil davon)

Der Serial Driver läuft mit zwei Threads: ein Send-Thread (Queue-basiert) und ein Receive-Thread (kontinuierliches Frame-Lesen). Beide sind Daemon-Threads und stoppen wenn der Hauptprozess endet.

- [ ] **Step 1: Failing Test schreiben**

`tests/hal/test_hardware_interface.py` (erster Block):
```python
import threading
import time
import pytest
from unittest.mock import MagicMock, patch, call
from mower.hal.protocol import encode_frame, CmdType, encode_drive
from mower.hal.serial_driver import SerialDriver


class TestSerialDriver:
    def test_send_puts_frame_on_serial(self, mock_serial):
        driver = SerialDriver(port='/dev/ttyACM0', baud=921600)
        driver._serial = mock_serial
        driver.start()
        frame = encode_drive(0.5, 0.0)
        driver.send(frame)
        time.sleep(0.05)
        driver.stop()
        mock_serial.write.assert_called_with(frame)

    def test_receive_callback_called_on_valid_frame(self, mock_serial):
        import struct
        from mower.hal.protocol import encode_frame, CmdType
        payload = struct.pack('<HBI', 300, 0, 500)
        frame = encode_frame(CmdType.SENSORS, payload)
        # Serial returns one frame then blocks
        call_count = 0
        def read_side_effect(n):
            nonlocal call_count
            call_count += 1
            if call_count == 1:
                return bytes([0xAA])
            elif call_count == 2:
                return frame[1:2]  # CMD_TYPE
            elif call_count == 3:
                return frame[2:3]  # PAYLOAD_LEN
            elif call_count == 4:
                return frame[3:-1]  # PAYLOAD
            elif call_count == 5:
                return frame[-1:]   # CRC
            else:
                time.sleep(1)
                return b''
        mock_serial.read.side_effect = read_side_effect
        received = []
        driver = SerialDriver(port='/dev/ttyACM0', baud=921600)
        driver._serial = mock_serial
        driver.on_frame = lambda cmd, payload: received.append((cmd, payload))
        driver.start()
        time.sleep(0.2)
        driver.stop()
        assert len(received) == 1
        assert received[0][0] == CmdType.SENSORS

    def test_stop_terminates_threads(self, mock_serial):
        driver = SerialDriver(port='/dev/ttyACM0', baud=921600)
        driver._serial = mock_serial
        driver.start()
        time.sleep(0.05)
        driver.stop()
        assert not driver._send_thread.is_alive()
        assert not driver._recv_thread.is_alive()


@pytest.fixture
def mock_serial():
    s = MagicMock()
    s.read.return_value = b''
    s.write.return_value = None
    s.in_waiting = 0
    return s
```

- [ ] **Step 2: Tests ausführen — müssen FAIL**

```bash
pytest tests/hal/test_hardware_interface.py::TestSerialDriver -v
```

Erwartete Ausgabe: `ImportError` — `serial_driver.py` fehlt.

- [ ] **Step 3: `mower/hal/serial_driver.py` implementieren**

```python
import threading
import queue
import logging
from typing import Callable, Optional
import serial
from mower.hal.protocol import decode_frame, FrameError, CmdType

logger = logging.getLogger(__name__)


class SerialDriver:
    """Thread-safe serial driver. Sends frames via queue, receives frames via callback."""

    def __init__(self, port: str, baud: int = 921600):
        self._port = port
        self._baud = baud
        self._serial: Optional[serial.Serial] = None
        self._send_queue: queue.Queue = queue.Queue(maxsize=64)
        self._running = False
        self._send_thread: Optional[threading.Thread] = None
        self._recv_thread: Optional[threading.Thread] = None
        self.on_frame: Optional[Callable[[CmdType, bytes], None]] = None

    def start(self):
        if self._serial is None:
            self._serial = serial.Serial(
                self._port, self._baud, timeout=0.1
            )
        self._running = True
        self._send_thread = threading.Thread(target=self._send_loop, daemon=True)
        self._recv_thread = threading.Thread(target=self._recv_loop, daemon=True)
        self._send_thread.start()
        self._recv_thread.start()
        logger.info("SerialDriver started on %s @ %d baud", self._port, self._baud)

    def stop(self):
        self._running = False
        if self._send_thread:
            self._send_thread.join(timeout=1.0)
        if self._recv_thread:
            self._recv_thread.join(timeout=1.0)
        if self._serial and self._serial.is_open:
            self._serial.close()
        logger.info("SerialDriver stopped")

    def send(self, frame: bytes):
        try:
            self._send_queue.put_nowait(frame)
        except queue.Full:
            logger.warning("Send queue full — dropping frame")

    def _send_loop(self):
        while self._running:
            try:
                frame = self._send_queue.get(timeout=0.05)
                self._serial.write(frame)
            except queue.Empty:
                pass
            except Exception as e:
                logger.error("Send error: %s", e)

    def _recv_loop(self):
        """Read frames byte-by-byte: wait for start byte, then read header + payload + CRC."""
        while self._running:
            try:
                b = self._serial.read(1)
                if not b or b[0] != 0xAA:
                    continue
                cmd_byte = self._serial.read(1)
                len_byte = self._serial.read(1)
                if not cmd_byte or not len_byte:
                    continue
                payload_len = len_byte[0]
                payload = self._serial.read(payload_len)
                crc_byte = self._serial.read(1)
                if len(payload) != payload_len or not crc_byte:
                    continue
                frame = bytes([0xAA]) + cmd_byte + len_byte + payload + crc_byte
                cmd_type, decoded_payload = decode_frame(frame)
                if self.on_frame:
                    self.on_frame(cmd_type, decoded_payload)
            except FrameError as e:
                logger.warning("Frame error: %s", e)
            except Exception as e:
                if self._running:
                    logger.error("Recv error: %s", e)
```

- [ ] **Step 4: Tests ausführen — müssen PASS**

```bash
pytest tests/hal/test_hardware_interface.py::TestSerialDriver -v
```

Erwartete Ausgabe: `3 passed`.

- [ ] **Step 5: Commit**

```bash
git add mower/hal/serial_driver.py tests/hal/test_hardware_interface.py
git commit -m "feat(hal): threaded serial driver with send queue and frame receiver"
```

---

## Task 4: High-Level Hardware Interface

**Files:**
- Create: `mower/hal/hardware_interface.py`
- Modify: `tests/hal/test_hardware_interface.py` (zweiter Block hinzufügen)

`HardwareInterface` ist die einzige API, die der Rest des Systems (Mission Executive, Planner etc.) kennt. Sie abstrahiert Protokoll und Serial Driver vollständig.

- [ ] **Step 1: Tests hinzufügen**

An `tests/hal/test_hardware_interface.py` anhängen:
```python
from mower.hal.hardware_interface import HardwareInterface


class TestHardwareInterface:
    def test_drive_sends_drive_frame(self, mock_driver):
        hw = HardwareInterface(driver=mock_driver)
        hw.drive(speed=0.3, steering=10.0)
        mock_driver.send.assert_called_once()
        frame = mock_driver.send.call_args[0][0]
        from mower.hal.protocol import decode_frame, CmdType
        cmd, payload = decode_frame(frame)
        assert cmd == CmdType.DRIVE

    def test_estop_sends_estop_frame(self, mock_driver):
        hw = HardwareInterface(driver=mock_driver)
        hw.estop()
        frame = mock_driver.send.call_args[0][0]
        from mower.hal.protocol import decode_frame, CmdType
        cmd, _ = decode_frame(frame)
        assert cmd == CmdType.ESTOP

    def test_blade_on_sends_blade_frame(self, mock_driver):
        hw = HardwareInterface(driver=mock_driver)
        hw.set_blade(True)
        frame = mock_driver.send.call_args[0][0]
        from mower.hal.protocol import decode_frame, CmdType
        cmd, payload = decode_frame(frame)
        assert cmd == CmdType.BLADE
        assert payload == b'\x01'

    def test_telemetry_callback_on_sensors(self, mock_driver):
        import struct
        from mower.hal.protocol import encode_frame, CmdType
        hw = HardwareInterface(driver=mock_driver)
        received = []
        hw.on_sensors = lambda d: received.append(d)
        payload = struct.pack('<HBI', 300, 0, 1000)
        frame = encode_frame(CmdType.SENSORS, payload)
        # Simulate incoming frame via on_frame callback
        from mower.hal.protocol import decode_frame
        cmd, p = decode_frame(frame)
        hw._on_frame(cmd, p)
        assert len(received) == 1
        assert received[0]['rain_adc'] == 300
        assert received[0]['encoder_ticks'] == 1000

    def test_lift_triggers_estop(self, mock_driver):
        import struct
        from mower.hal.protocol import encode_frame, CmdType, decode_frame
        hw = HardwareInterface(driver=mock_driver)
        payload = struct.pack('<HBI', 100, 1, 0)  # lift=True
        frame = encode_frame(CmdType.SENSORS, payload)
        cmd, p = decode_frame(frame)
        hw._on_frame(cmd, p)
        # estop must have been sent
        frames_sent = [mock_driver.send.call_args_list[i][0][0]
                       for i in range(mock_driver.send.call_count)]
        cmds = [decode_frame(f)[0] for f in frames_sent]
        assert CmdType.ESTOP in cmds


@pytest.fixture
def mock_driver():
    d = MagicMock()
    return d
```

- [ ] **Step 2: Tests ausführen — müssen FAIL**

```bash
pytest tests/hal/test_hardware_interface.py::TestHardwareInterface -v
```

- [ ] **Step 3: `mower/hal/hardware_interface.py` implementieren**

```python
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
```

- [ ] **Step 4: Tests ausführen — müssen PASS**

```bash
pytest tests/hal/test_hardware_interface.py -v
```

Erwartete Ausgabe: alle Tests `PASSED`.

- [ ] **Step 5: Commit**

```bash
git add mower/hal/hardware_interface.py tests/hal/test_hardware_interface.py
git commit -m "feat(hal): high-level hardware interface with lift-triggered estop"
```

---

## Task 5: udev-Regel für Low-Latency Serial

**Files:**
- Create: `setup/99-mower-serial.rules`

- [ ] **Step 1: udev-Regel erstellen**

`setup/99-mower-serial.rules`:
```
# Mähroboter Teensy 4.1 — Low-latency USB-Serial
# Teensy vendor ID: 16C0, product varies — adjust idProduct if needed
SUBSYSTEM=="tty", ATTRS{idVendor}=="16c0", ATTRS{idProduct}=="0483", \
    SYMLINK+="mower", MODE="0666", \
    RUN+="/bin/setserial /dev/%k low_latency"
```

- [ ] **Step 2: Regel installieren (auf dem RPi 5 ausführen)**

```bash
# setserial ist auf Raspberry Pi OS nicht vorinstalliert
sudo apt install -y setserial
sudo cp setup/99-mower-serial.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```

Nach Teensy-Anstecken: `ls -la /dev/mower` sollte den Symlink zeigen.

- [ ] **Step 3: Commit**

```bash
git add setup/99-mower-serial.rules
git commit -m "chore: udev rule for Teensy low-latency serial symlink"
```

---

## Task 6: Teensy Firmware — Protokoll & Safety-Watchdog

**Files:**
- Create: `firmware/mower_firmware/protocol.h`
- Create: `firmware/mower_firmware/watchdog.h`
- Create: `firmware/mower_firmware/watchdog.cpp`

Die Firmware wird mit Arduino IDE oder PlatformIO auf den Teensy 4.1 geflasht. Tests sind manuell (Serial Monitor).

- [ ] **Step 1: `protocol.h` erstellen**

```cpp
#pragma once
#include <stdint.h>

// Frame structure: [0xAA][CMD_TYPE][PAYLOAD_LEN][PAYLOAD...][CRC8]
static const uint8_t FRAME_START = 0xAA;

enum CmdType : uint8_t {
  CMD_DRIVE   = 0x01,
  CMD_BLADE   = 0x02,
  CMD_ESTOP   = 0x03,
  CMD_PING    = 0x04,
  // Telemetry (Teensy → RPi)
  CMD_SENSORS = 0x10,
  CMD_SOC     = 0x11,
  CMD_STATUS  = 0x12,
};

// CRC-8/MAXIM (Dallas 1-Wire, poly=0x31, reflect in/out)
inline uint8_t crc8_maxim(const uint8_t* data, uint8_t len) {
  uint8_t crc = 0x00;
  for (uint8_t i = 0; i < len; i++) {
    uint8_t byte = data[i];
    for (uint8_t j = 0; j < 8; j++) {
      uint8_t mix = (crc ^ byte) & 0x01;
      crc >>= 1;
      if (mix) crc ^= 0x8C;  // reflected poly 0x31
      byte >>= 1;
    }
  }
  return crc;
}

inline void send_frame(HardwareSerial& port, CmdType cmd, const uint8_t* payload, uint8_t len) {
  uint8_t header[3] = { FRAME_START, (uint8_t)cmd, len };
  // CRC over header + payload. Static buffer avoids VLA (not valid C++).
  static uint8_t buf[3 + 255];  // max payload 255 bytes
  memcpy(buf, header, 3);
  memcpy(buf + 3, payload, len);
  uint8_t crc = crc8_maxim(buf, 3 + len);
  port.write(header, 3);
  port.write(payload, len);
  port.write(&crc, 1);
}
```

- [ ] **Step 2: `watchdog.h` und `watchdog.cpp` erstellen**

`watchdog.h`:
```cpp
#pragma once
#include <Arduino.h>

class Watchdog {
public:
  Watchdog(uint32_t timeout_ms = 500);
  void feed();            // Call when PING received
  bool is_triggered();    // Returns true if timeout elapsed
  void reset();
private:
  uint32_t _timeout_ms;
  uint32_t _last_feed_ms;
  bool _triggered;
};
```

`watchdog.cpp`:
```cpp
#include "watchdog.h"

Watchdog::Watchdog(uint32_t timeout_ms)
  : _timeout_ms(timeout_ms), _last_feed_ms(0), _triggered(false) {}

void Watchdog::feed() {
  _last_feed_ms = millis();
  _triggered = false;
}

bool Watchdog::is_triggered() {
  if (!_triggered && (millis() - _last_feed_ms > _timeout_ms)) {
    _triggered = true;
  }
  return _triggered;
}

void Watchdog::reset() {
  _last_feed_ms = millis();
  _triggered = false;
}
```

- [ ] **Step 3: Commit**

```bash
git add firmware/
git commit -m "feat(firmware): CRC-8/MAXIM protocol header and safety watchdog"
```

---

## Task 7: Teensy Firmware — Motor, Servo, Blade, Sensoren

**Files:**
- Create: `firmware/mower_firmware/motor_control.h/.cpp`
- Create: `firmware/mower_firmware/servo_control.h/.cpp`
- Create: `firmware/mower_firmware/blade_control.h/.cpp`
- Create: `firmware/mower_firmware/sensor_reader.h/.cpp`
- Create: `firmware/mower_firmware/mower_firmware.ino`

- [ ] **Step 1: `motor_control.h/.cpp` erstellen**

`motor_control.h`:
```cpp
#pragma once
#include <Arduino.h>

// Cytron MDD10A: DIR pin + PWM pin per channel
class MotorControl {
public:
  MotorControl(uint8_t dir_pin, uint8_t pwm_pin);
  void begin();
  void set_speed(float speed);  // -1.0 (reverse) to 1.0 (forward)
  void stop();
private:
  uint8_t _dir_pin, _pwm_pin;
};
```

`motor_control.cpp`:
```cpp
#include "motor_control.h"

MotorControl::MotorControl(uint8_t dir_pin, uint8_t pwm_pin)
  : _dir_pin(dir_pin), _pwm_pin(pwm_pin) {}

void MotorControl::begin() {
  pinMode(_dir_pin, OUTPUT);
  pinMode(_pwm_pin, OUTPUT);
  stop();
}

void MotorControl::set_speed(float speed) {
  speed = constrain(speed, -1.0f, 1.0f);
  digitalWrite(_dir_pin, speed >= 0 ? HIGH : LOW);
  analogWrite(_pwm_pin, (uint8_t)(abs(speed) * 255));
}

void MotorControl::stop() {
  digitalWrite(_dir_pin, LOW);
  analogWrite(_pwm_pin, 0);
}
```

- [ ] **Step 2: `servo_control.h/.cpp` erstellen**

`servo_control.h`:
```cpp
#pragma once
#include <Servo.h>

class ServoControl {
public:
  ServoControl(uint8_t pin, int center_us = 1500, int range_us = 500);
  void begin();
  void set_angle(float degrees);  // -45 to +45
private:
  uint8_t _pin;
  int _center_us, _range_us;
  Servo _servo;
};
```

`servo_control.cpp`:
```cpp
#include "servo_control.h"

ServoControl::ServoControl(uint8_t pin, int center_us, int range_us)
  : _pin(pin), _center_us(center_us), _range_us(range_us) {}

void ServoControl::begin() {
  _servo.attach(_pin);
  set_angle(0);
}

void ServoControl::set_angle(float degrees) {
  degrees = constrain(degrees, -45.0f, 45.0f);
  int us = _center_us + (int)(degrees / 45.0f * _range_us);
  _servo.writeMicroseconds(us);
}
```

- [ ] **Step 3: `blade_control.h/.cpp` erstellen**

`blade_control.h`:
```cpp
#pragma once
#include <Arduino.h>

// Brushless ESC controlled via PWM signal (1000–2000µs)
class BladeControl {
public:
  BladeControl(uint8_t esc_pin);
  void begin();       // Arms ESC with idle signal
  void set_on(bool state);
  bool is_running();
private:
  uint8_t _esc_pin;
  bool _running;
};
```

`blade_control.cpp`:
```cpp
#include "blade_control.h"

BladeControl::BladeControl(uint8_t esc_pin) : _esc_pin(esc_pin), _running(false) {}

void BladeControl::begin() {
  pinMode(_esc_pin, OUTPUT);
  // Teensy 4.1 default PWM resolution is 12-bit (0–4095).
  // At 50 Hz: 1000µs idle = 1000/20000 * 4095 ≈ 205
  analogWriteResolution(12);
  analogWriteFrequency(_esc_pin, 50);
  analogWrite(_esc_pin, 205);  // 1000µs idle — arms ESC
  _running = false;
}

void BladeControl::set_on(bool state) {
  if (state) {
    analogWrite(_esc_pin, 307);  // 1500µs — half throttle for mowing (12-bit @ 50Hz)
  } else {
    analogWrite(_esc_pin, 205);  // 1000µs idle
  }
  _running = state;
}

bool BladeControl::is_running() { return _running; }
```

- [ ] **Step 4: `sensor_reader.h/.cpp` erstellen**

`sensor_reader.h`:
```cpp
#pragma once
#include <Arduino.h>

class SensorReader {
public:
  SensorReader(uint8_t rain_pin, uint8_t lift_pin, uint8_t encoder_pin_a);
  void begin();
  uint16_t read_rain_adc();
  bool read_lift();
  int32_t read_encoder_ticks();
  void encoder_isr();  // Call from interrupt
private:
  uint8_t _rain_pin, _lift_pin, _encoder_pin_a;
  volatile int32_t _encoder_ticks;
};

extern SensorReader* g_sensor_reader;  // For ISR linkage
```

`sensor_reader.cpp`:
```cpp
#include "sensor_reader.h"

SensorReader* g_sensor_reader = nullptr;

SensorReader::SensorReader(uint8_t rain_pin, uint8_t lift_pin, uint8_t encoder_pin_a)
  : _rain_pin(rain_pin), _lift_pin(lift_pin), _encoder_pin_a(encoder_pin_a), _encoder_ticks(0) {}

void SensorReader::begin() {
  pinMode(_rain_pin, INPUT);
  pinMode(_lift_pin, INPUT_PULLUP);  // Active LOW
  pinMode(_encoder_pin_a, INPUT_PULLUP);
  g_sensor_reader = this;
  attachInterrupt(digitalPinToInterrupt(_encoder_pin_a), [](){
    if (g_sensor_reader) g_sensor_reader->encoder_isr();
  }, RISING);
}

uint16_t SensorReader::read_rain_adc() {
  return analogRead(_rain_pin);
}

bool SensorReader::read_lift() {
  return digitalRead(_lift_pin) == LOW;  // Active LOW
}

int32_t SensorReader::read_encoder_ticks() {
  return _encoder_ticks;
}

void SensorReader::encoder_isr() {
  // Phase 1: single-channel encoder, always increments.
  // TODO Phase 2: use motor direction state for signed ticks (bidirectional odometry).
  _encoder_ticks++;
}
```

- [ ] **Step 5: Hauptsketch `mower_firmware.ino` erstellen**

```cpp
#include "protocol.h"
#include "motor_control.h"
#include "servo_control.h"
#include "blade_control.h"
#include "sensor_reader.h"
#include "watchdog.h"
#include <Wire.h>

// Pin-Belegung (anpassen nach Verdrahtung)
#define MOTOR_DIR_PIN   2
#define MOTOR_PWM_PIN   3
#define SERVO_PIN       4
#define BLADE_ESC_PIN   5
#define RAIN_ADC_PIN    A0
#define LIFT_PIN        6
#define ENCODER_PIN_A   7

MotorControl motor(MOTOR_DIR_PIN, MOTOR_PWM_PIN);
ServoControl steering(SERVO_PIN);
BladeControl blade(BLADE_ESC_PIN);
SensorReader sensors(RAIN_ADC_PIN, LIFT_PIN, ENCODER_PIN_A);
Watchdog watchdog(500);

uint32_t last_sensors_tx = 0;
uint32_t last_soc_tx = 0;
uint32_t last_status_tx = 0;
uint8_t recv_buf[64];
uint8_t recv_idx = 0;

void handle_lift_interrupt() {
  if (sensors.read_lift()) {
    blade.set_on(false);
    motor.stop();
  }
}

void process_command(CmdType cmd, const uint8_t* payload, uint8_t len) {
  watchdog.feed();
  switch (cmd) {
    case CMD_DRIVE: {
      float speed, steer;
      memcpy(&speed, payload, 4);
      memcpy(&steer, payload + 4, 4);
      motor.set_speed(speed);
      steering.set_angle(steer);
      break;
    }
    case CMD_BLADE:
      blade.set_on(payload[0] != 0);
      break;
    case CMD_ESTOP:
      motor.stop();
      blade.set_on(false);
      break;
    case CMD_PING:
      // Just feeding watchdog is enough
      break;
    default:
      break;
  }
}

void setup() {
  Serial.begin(921600);  // USB-Serial to RPi 5
  motor.begin();
  steering.begin();
  blade.begin();
  sensors.begin();
  watchdog.reset();
  // Lift interrupt
  attachInterrupt(digitalPinToInterrupt(LIFT_PIN), handle_lift_interrupt, CHANGE);
}

void loop() {
  // Read incoming frames
  while (Serial.available()) {
    uint8_t b = Serial.read();
    if (recv_idx == 0 && b != FRAME_START) continue;
    recv_buf[recv_idx++] = b;
    if (recv_idx >= 3) {
      uint8_t payload_len = recv_buf[2];
      uint8_t expected = 3 + payload_len + 1;
      if (recv_idx >= expected) {
        // Validate CRC
        uint8_t calc_crc = crc8_maxim(recv_buf, 3 + payload_len);
        if (calc_crc == recv_buf[3 + payload_len]) {
          process_command((CmdType)recv_buf[1], recv_buf + 3, payload_len);
        }
        recv_idx = 0;
      }
    }
    if (recv_idx >= sizeof(recv_buf)) recv_idx = 0;
  }

  // Watchdog check
  if (watchdog.is_triggered()) {
    motor.stop();
    blade.set_on(false);
  }

  uint32_t now = millis();

  // Send SENSORS at 50 Hz
  if (now - last_sensors_tx >= 20) {
    last_sensors_tx = now;
    uint16_t rain = sensors.read_rain_adc();
    uint8_t lift = sensors.read_lift() ? 1 : 0;
    int32_t enc = sensors.read_encoder_ticks();
    uint8_t payload[7];
    memcpy(payload, &rain, 2);
    payload[2] = lift;
    memcpy(payload + 3, &enc, 4);
    send_frame(Serial, CMD_SENSORS, payload, 7);
    if (lift) { blade.set_on(false); }
  }

  // Send SOC at 1 Hz
  // TODO Phase 1: Voltage divider fallback (A1). Phase 4 replaces with BMS I2C (BQ76940).
  // Linear approximation: 12000mV=0%, 16800mV=100% → 48mV per percent
  if (now - last_soc_tx >= 1000) {
    last_soc_tx = now;
    uint16_t raw = analogRead(A1);
    // Teensy 4.1 ADC is 12-bit (0–4095), Vref=3.3V, voltage divider: R1=100k, R2=10k → factor=11
    uint16_t voltage_mv = (uint16_t)(raw * (3300.0f / 4095.0f) * 11.0f);
    uint8_t soc = (uint8_t)constrain((voltage_mv - 12000) / 48, 0, 100);
    uint8_t payload[3];
    payload[0] = soc;
    memcpy(payload + 1, &voltage_mv, 2);
    send_frame(Serial, CMD_SOC, payload, 3);
  }

  // Send STATUS at 10 Hz
  if (now - last_status_tx >= 100) {
    last_status_tx = now;
    uint8_t payload[3] = {
      (uint8_t)(!watchdog.is_triggered()),
      (uint8_t)blade.is_running(),
      0x00
    };
    send_frame(Serial, CMD_STATUS, payload, 3);
  }
}
```

- [ ] **Step 6: Firmware kompilieren und flashen**

Mit Arduino IDE:
1. Board: "Teensy 4.1" auswählen
2. Tools → USB Type: "Serial"
3. `mower_firmware.ino` öffnen
4. Verify (Ctrl+R) → keine Kompilierungsfehler erwartet
5. Upload (Ctrl+U) → Teensy flashen

Mit PlatformIO (empfohlen):
```bash
cd firmware/mower_firmware
pio run --target upload
```

- [ ] **Step 7: Manuellem Kommunikationstest durchführen**

```python
# test_manual.py — auf dem RPi 5 ausführen
from mower.hal.protocol import encode_drive, encode_ping, encode_estop
from mower.hal.serial_driver import SerialDriver
from mower.hal.hardware_interface import HardwareInterface
import time

driver = SerialDriver(port='/dev/mower', baud=921600)
hw = HardwareInterface(driver=driver)
hw.on_sensors = lambda d: print("Sensors:", d)
hw.on_soc = lambda d: print("SOC:", d)
hw.on_status = lambda d: print("Status:", d)
hw.start()  # startet Serial-Driver + automatischen PING-Sender (10 Hz)

time.sleep(1.0)  # Teensy-Verbindung stabilisieren

# Fahrt vorwärts 2 Sekunden
hw.drive(speed=0.3, steering=0.0)
time.sleep(2.0)
hw.drive(speed=0.0, steering=0.0)
time.sleep(0.5)

# ESTOP und sauber beenden
hw.estop()
hw.stop()
```

```bash
python test_manual.py
```

Erwartetes Ergebnis: Sensors/SOC/Status werden ausgegeben, Motor läuft 2 Sekunden, stoppt dann.

- [ ] **Step 8: Commit**

```bash
git add firmware/
git commit -m "feat(firmware): full Teensy firmware with motor/servo/blade/sensor/watchdog"
```

---

## Task 8: Gesamttest & Abschluss Phase 1

- [ ] **Step 1: Alle Python-Tests ausführen**

```bash
pytest tests/ -v --tb=short
```

Erwartete Ausgabe: alle Tests `PASSED`, 0 failures.

- [ ] **Step 2: Watchdog-Verhalten verifizieren**

Teensy angesteckt, RPi 5 läuft:
1. Python-Script starten (sendet PING alle 100 ms)
2. Python-Script abbrechen (Ctrl+C)
3. Beobachten: nach 500 ms soll Motor stoppen (Watchdog greift)

- [ ] **Step 3: Lift-Erkennung verifizieren**

1. Motor läuft mit `drive(0.3, 0.0)`
2. Lift-Schalter manuell betätigen
3. Motor und Blade müssen sofort stoppen (interrupt-basiert auf Teensy)
4. RPi 5 empfängt Sensors-Frame mit `lift=True`

- [ ] **Step 4: Final Commit**

```bash
git add .
git commit -m "feat: Phase 1 complete — HAL, serial protocol, Teensy firmware operational"
```

---

## Zusammenfassung Phase 1

Nach Abschluss dieses Plans:
- Binäres CRC-8/MAXIM Protokoll implementiert und getestet (Python + Teensy)
- Teensy 4.1 steuert Motor, Servo, Blade-ESC, liest Encoder/Regen/Lift
- Safety-Watchdog auf Teensy aktiv (500 ms Timeout)
- Lift-Interrupt stoppt Blade sofort (hardwareseitig)
- Python HAL vollständig unit-getestet (Mock-basiert)
- Manuelle Fahrkontrolle vom RPi 5 funktioniert

**Nächster Plan:** Phase 2 — Navigation, RTK-GPS, EKF-Sensorfusion, Geofencing
