# Eigene RTK-Basisstation — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Ein neues, eigenständiges Top-Level-Paket `base_station/`, das rohe RTCM3-Korrekturdaten vom lokalen (stationären) LC29H über einen minimalen NTRIP-Caster an beliebig viele Rover ausliefert — kompatibel zum bestehenden `mower/nav/ntrip_client.py`, ohne dass sich am Rover-Code etwas ändert.

**Architecture:** Zwei kleine, unabhängig testbare Klassen — `RtcmSerialSource` (liest rohe Bytes vom Basis-LC29H, injizierbarer Serial-Backend nach dem Muster von `mower/hal/camera.py`s `Camera`) und `NtripServer` (TCP-Caster, handshake-kompatibel zu `NtripClient`, Thread-sicherer Verbindungs-Broadcast). `base_station/main.py` verdrahtet beide und liest Konfiguration aus Umgebungsvariablen — ohne Soft-Fallback, im Gegensatz zu `mower/main.py`.

**Tech Stack:** Python 3.10, stdlib `socket`/`threading`/`base64` (keine neue Dependency), `pytest`. Tests für `NtripServer` laufen gegen echte Loopback-Sockets (`127.0.0.1:0`) statt Mocks.

**Spec:** `docs/superpowers/specs/2026-07-15-rtk-base-station-design.md` (gelesen, approved — enthält die vollständige Architektur- und Verhaltensbeschreibung; dieser Plan setzt sie in TDD-Schritte um).

---

## File Structure

- Create: `base_station/__init__.py` — leer, macht `base_station` zu einem importierbaren Package (mirrort `mower/__init__.py`).
- Create: `base_station/rtcm_source.py` — `RtcmSerialSource`: liest rohe Bytes vom Basis-LC29H, feuert `on_data`.
- Create: `base_station/ntrip_server.py` — `NtripServer`: TCP-Caster, Handshake + Broadcast.
- Create: `base_station/main.py` — Verdrahtung + Env-Var-Konfiguration, kein Soft-Fallback.
- Create: `tests/base_station/__init__.py` — leer (mirrort `tests/hal/__init__.py`).
- Create: `tests/base_station/test_rtcm_source.py`
- Create: `tests/base_station/test_ntrip_server.py`

### Interface contracts

```python
# base_station/rtcm_source.py
class RtcmSerialSource:
    def __init__(self, port: str, baud: int = 115200, serial_backend=None): ...
    def start(self) -> None: ...
    def stop(self) -> None: ...
    on_data: Optional[Callable[[bytes], None]]   # set by caller

# base_station/ntrip_server.py
class NtripServer:
    def __init__(self, host: str, port: int, mountpoint: str, user: str, password: str): ...
    def start(self) -> None: ...
    def stop(self) -> None: ...
    def broadcast(self, data: bytes) -> None: ...
```

---

## Task 1: `RtcmSerialSource`

### Background

Liest rohe Bytes vom seriellen Port des Basis-LC29H (im Base-Mode) und feuert sie per `on_data`-Callback — kein RTCM3-Framing/CRC-Parsing nötig, die Bytes werden als opaker Strom durchgereicht.

**Wichtig — Injection-Muster:** Anders als `mower/hal/serial_driver.py`s `SerialDriver` (öffnet den Port erst in `start()`, kein Injection-Punkt) mirrort dies **`mower/hal/camera.py`s `Camera`**: der `serial_backend`-Parameter wird im Konstruktor sofort verwendet (Default baut einen echten `serial.Serial(...)`, sofort geöffnet). Das ist bewusst so — es bedeutet, dass ein fehlender/nicht vorhandener Port **sofort beim Konstruieren** eine klare Exception wirft, was genau zum "fail loudly beim Start, kein Soft-Fallback"-Verhalten passt, das `base_station/main.py` in Task 3 braucht.

**Laufzeit-Fehler:** Ein einzelner fehlschlagender `read()` (z. B. USB-Wackelkontakt) darf den Lese-Thread nicht crashen — Exception pro Iteration abfangen, auf ERROR loggen, weiterlesen. Exakt das Muster aus `SerialDriver._recv_loop()` (`mower/hal/serial_driver.py`).

**Files:**
- Create: `base_station/__init__.py`
- Create: `base_station/rtcm_source.py`
- Create: `tests/base_station/__init__.py`
- Test: `tests/base_station/test_rtcm_source.py`

- [ ] **Step 1: Package-Marker anlegen**

```bash
touch "base_station/__init__.py" "tests/base_station/__init__.py"
```

(Beide Dateien bleiben leer.)

- [ ] **Step 2: Failing Tests schreiben**

```python
# tests/base_station/test_rtcm_source.py
import time
import pytest

from base_station.rtcm_source import RtcmSerialSource


class _FakeSerial:
    """Mimics pyserial's Serial.read(size) -> bytes. A small sleep on empty
    reads mirrors real pyserial's blocking timeout behaviour, so the read
    loop doesn't busy-spin the CPU while a test waits for it."""

    def __init__(self, chunks):
        self._chunks = list(chunks)

    def read(self, size=1024):
        if self._chunks:
            return self._chunks.pop(0)
        time.sleep(0.01)
        return b""


class _FlakySerial:
    """Raises once on the first read(), then behaves like _FakeSerial."""

    def __init__(self, chunks):
        self._chunks = list(chunks)
        self._raised = False

    def read(self, size=1024):
        if not self._raised:
            self._raised = True
            raise OSError("simulated USB hiccup")
        if self._chunks:
            return self._chunks.pop(0)
        time.sleep(0.01)
        return b""


def test_on_data_fires_with_bytes_read():
    received = []
    src = RtcmSerialSource(port="/dev/ttyUSB0", serial_backend=_FakeSerial([b"\x01\x02\x03"]))
    src.on_data = lambda data: received.append(data)
    src.start()
    time.sleep(0.1)
    src.stop()
    assert received == [b"\x01\x02\x03"]


def test_empty_read_does_not_fire_callback():
    received = []
    src = RtcmSerialSource(port="/dev/ttyUSB0", serial_backend=_FakeSerial([]))
    src.on_data = lambda data: received.append(data)
    src.start()
    time.sleep(0.05)
    src.stop()
    assert received == []


def test_stop_terminates_thread():
    src = RtcmSerialSource(port="/dev/ttyUSB0", serial_backend=_FakeSerial([]))
    src.start()
    time.sleep(0.05)
    src.stop()
    assert not src._thread.is_alive()


def test_transient_read_error_does_not_kill_thread():
    received = []
    src = RtcmSerialSource(port="/dev/ttyUSB0", serial_backend=_FlakySerial([b"\xaa\xbb"]))
    src.on_data = lambda data: received.append(data)
    src.start()
    time.sleep(0.1)  # first read raises, thread must survive and read again
    src.stop()
    assert received == [b"\xaa\xbb"]
    assert not src._thread.is_alive()
```

- [ ] **Step 3: Tests ausführen — müssen FAIL**

```bash
python3 -m pytest tests/base_station/test_rtcm_source.py -v
```

Erwartung: `ModuleNotFoundError: No module named 'base_station.rtcm_source'` (oder `base_station` insgesamt).

- [ ] **Step 4: `base_station/rtcm_source.py` implementieren**

```python
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
```

- [ ] **Step 5: Tests ausführen — müssen PASS**

```bash
python3 -m pytest tests/base_station/test_rtcm_source.py -v
```

Erwartung: 4 passed.

- [ ] **Step 6: Volle Suite laufen lassen**

```bash
python3 -m pytest tests/ -q
```

Erwartung: alle bisherigen Tests weiterhin grün, plus die 4 neuen (231 + 4 = 235).

- [ ] **Step 7: Commit**

```bash
git add base_station/__init__.py base_station/rtcm_source.py tests/base_station/__init__.py tests/base_station/test_rtcm_source.py
git commit -m "feat(base-station): add RtcmSerialSource — raw RTCM3 relay from base LC29H"
```

---

## Task 2: `NtripServer`

### Background

Minimaler NTRIP-Caster: TCP-Server, der den exakten Handshake akzeptiert, den `NtripClient._connect()` (`mower/nav/ntrip_client.py:52-70`) bereits sendet:

```
GET /{mountpoint} HTTP/1.0\r\n
Host: {host}\r\n
Ntrip-Version: Ntrip/2.0\r\n
User-Agent: NTRIP MowerClient/1.0\r\n
Authorization: Basic {base64(user:password)}\r\n
\r\n
```

`NtripClient._connect()` prüft die Antwort auf `"ICY 200 OK"` oder `"200 OK"` als Teilstring der ersten 1024 gelesenen Bytes — die Erfolgsantwort muss das enthalten.

**Thread-Sicherheit ist hier der kritische Punkt:** die Liste aktiver Verbindungen wird von drei verschiedenen Threads verändert — Accept-Thread (fügt hinzu), `broadcast()` (aufgerufen aus `RtcmSerialSource`s Lese-Thread, entfernt tote Verbindungen), `stop()` (iteriert, schließt alle). Alle Zugriffe auf die Liste laufen durch einen `threading.Lock`; `broadcast()` kopiert die Liste unter Lock, bevor außerhalb des Locks geschrieben wird (ein langsamer Client-Socket darf den Accept-Thread nicht blockieren).

**Files:**
- Create: `base_station/ntrip_server.py`
- Test: `tests/base_station/test_ntrip_server.py`

- [ ] **Step 1: Failing Tests schreiben**

Die Tests binden den Server an `127.0.0.1:0` (Port 0 = OS vergibt einen freien Port), lesen den tatsächlich vergebenen Port über `server._listener.getsockname()[1]` aus. Ein Test verwendet den **echten, unveränderten `NtripClient`** aus `mower/nav/ntrip_client.py` als Gegenstelle — das ist der stärkste Kompatibilitätsbeweis, weil er die tatsächliche Produktionsklasse gegen den neuen Server testet, statt das Protokoll im Test nachzubauen.

```python
# tests/base_station/test_ntrip_server.py
import base64
import socket
import time
import pytest

from base_station.ntrip_server import NtripServer
from mower.nav.ntrip_client import NtripClient

HOST = "127.0.0.1"
MOUNTPOINT = "MV2BASE"
USER = "rover"
PASSWORD = "s3cret"


@pytest.fixture
def server():
    srv = NtripServer(host=HOST, port=0, mountpoint=MOUNTPOINT, user=USER, password=PASSWORD)
    srv.start()
    yield srv
    srv.stop()


def _bound_port(srv):
    return srv._listener.getsockname()[1]


def _handshake_request(mountpoint, user, password):
    creds = base64.b64encode(f"{user}:{password}".encode()).decode()
    return (
        f"GET /{mountpoint} HTTP/1.0\r\n"
        f"Host: {HOST}\r\n"
        f"Ntrip-Version: Ntrip/2.0\r\n"
        f"User-Agent: NTRIP MowerClient/1.0\r\n"
        f"Authorization: Basic {creds}\r\n"
        f"\r\n"
    ).encode()


def test_valid_handshake_returns_success(server):
    sock = socket.create_connection((HOST, _bound_port(server)), timeout=2.0)
    sock.sendall(_handshake_request(MOUNTPOINT, USER, PASSWORD))
    resp = sock.recv(1024)
    sock.close()
    assert b"ICY 200 OK" in resp


def test_wrong_mountpoint_rejected(server):
    sock = socket.create_connection((HOST, _bound_port(server)), timeout=2.0)
    sock.sendall(_handshake_request("WRONGPOINT", USER, PASSWORD))
    resp = sock.recv(1024)
    sock.close()
    assert b"ICY 200 OK" not in resp


def test_wrong_credentials_rejected(server):
    sock = socket.create_connection((HOST, _bound_port(server)), timeout=2.0)
    sock.sendall(_handshake_request(MOUNTPOINT, USER, "wrong-password"))
    resp = sock.recv(1024)
    sock.close()
    assert b"ICY 200 OK" not in resp


def test_broadcast_delivers_to_connected_client(server):
    sock = socket.create_connection((HOST, _bound_port(server)), timeout=2.0)
    sock.sendall(_handshake_request(MOUNTPOINT, USER, PASSWORD))
    sock.recv(1024)  # consume handshake response
    server.broadcast(b"\x01\x02\x03rtcm")
    data = sock.recv(1024)
    sock.close()
    assert data == b"\x01\x02\x03rtcm"


def test_broadcast_reaches_multiple_clients(server):
    port = _bound_port(server)
    s1 = socket.create_connection((HOST, port), timeout=2.0)
    s2 = socket.create_connection((HOST, port), timeout=2.0)
    for s in (s1, s2):
        s.sendall(_handshake_request(MOUNTPOINT, USER, PASSWORD))
        s.recv(1024)
    server.broadcast(b"corrections")
    d1 = s1.recv(1024)
    d2 = s2.recv(1024)
    s1.close()
    s2.close()
    assert d1 == b"corrections"
    assert d2 == b"corrections"


def test_disconnected_client_does_not_break_broadcast(server):
    port = _bound_port(server)
    s1 = socket.create_connection((HOST, port), timeout=2.0)
    s2 = socket.create_connection((HOST, port), timeout=2.0)
    for s in (s1, s2):
        s.sendall(_handshake_request(MOUNTPOINT, USER, PASSWORD))
        s.recv(1024)
    s1.close()  # s1 goes away before the broadcast
    time.sleep(0.05)
    server.broadcast(b"still-works")  # must not raise
    data = s2.recv(1024)
    s2.close()
    assert data == b"still-works"


def test_real_ntrip_client_receives_broadcast(server):
    """End-to-end interop proof against the actual, unmodified NtripClient."""
    received = []
    client = NtripClient(host=HOST, port=_bound_port(server), mountpoint=MOUNTPOINT,
                         user=USER, password=PASSWORD)
    client.on_rtcm = lambda data: received.append(data)
    client.start()
    time.sleep(0.3)  # allow the client's connect + first recv loop to run
    server.broadcast(b"real-client-rtcm")
    time.sleep(0.2)
    client.stop()
    assert b"real-client-rtcm" in received
```

- [ ] **Step 2: Tests ausführen — müssen FAIL**

```bash
python3 -m pytest tests/base_station/test_ntrip_server.py -v
```

Erwartung: `ModuleNotFoundError: No module named 'base_station.ntrip_server'`.

- [ ] **Step 3: `base_station/ntrip_server.py` implementieren**

```python
"""Minimal NTRIP caster: TCP server compatible with
mower.nav.ntrip_client.NtripClient's handshake. Broadcasts RTCM3 bytes (fed
via broadcast()) to all connected rovers.

No RTCM3 parsing, no VRS/GGA handling — a single stationary base station
serves the same corrections to every connected rover regardless of position.
"""
import base64
import logging
import socket
import threading
from typing import Optional

logger = logging.getLogger(__name__)

_RECV_BUFSIZE = 2048
_SUCCESS_RESPONSE = b"ICY 200 OK\r\n\r\n"
_REJECT_RESPONSE = b"ERROR - Bad Mountpoint or Credentials\r\n"


class NtripServer:
    def __init__(self, host: str, port: int, mountpoint: str, user: str, password: str):
        self._host = host
        self._port = port
        self._mountpoint = mountpoint
        self._user = user
        self._password = password
        self._listener: Optional[socket.socket] = None
        self._accept_thread: Optional[threading.Thread] = None
        self._running = False
        self._clients: list[socket.socket] = []
        self._lock = threading.Lock()

    def start(self) -> None:
        self._listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._listener.bind((self._host, self._port))
        self._listener.listen(5)
        self._running = True
        self._accept_thread = threading.Thread(target=self._accept_loop, daemon=True)
        self._accept_thread.start()

    def stop(self) -> None:
        self._running = False
        if self._listener:
            self._listener.close()
        if self._accept_thread:
            self._accept_thread.join(timeout=2.0)
        with self._lock:
            for c in self._clients:
                try:
                    c.close()
                except OSError:
                    pass
            self._clients.clear()

    def broadcast(self, data: bytes) -> None:
        with self._lock:
            clients = list(self._clients)
        dead = []
        for c in clients:
            try:
                c.sendall(data)
            except OSError:
                dead.append(c)
        if dead:
            with self._lock:
                for c in dead:
                    if c in self._clients:
                        self._clients.remove(c)
                    try:
                        c.close()
                    except OSError:
                        pass

    def _accept_loop(self) -> None:
        while self._running:
            try:
                conn, _addr = self._listener.accept()
            except OSError:
                break  # listener closed -> stop() was called
            threading.Thread(target=self._handle_client, args=(conn,), daemon=True).start()

    def _handle_client(self, conn: socket.socket) -> None:
        try:
            request = conn.recv(_RECV_BUFSIZE)
            if self._is_authorized(request):
                conn.sendall(_SUCCESS_RESPONSE)
                with self._lock:
                    self._clients.append(conn)
            else:
                conn.sendall(_REJECT_RESPONSE)
                conn.close()
        except OSError:
            try:
                conn.close()
            except OSError:
                pass

    def _is_authorized(self, request: bytes) -> bool:
        try:
            lines = request.decode(errors="replace").split("\r\n")
            request_line = lines[0]
            _method, path, _proto = request_line.split(" ", 2)
            mountpoint = path.lstrip("/")
            if mountpoint != self._mountpoint:
                return False
            auth_header = next(
                (l for l in lines[1:] if l.lower().startswith("authorization:")), None
            )
            if auth_header is None:
                return False
            b64 = auth_header.split(" ", 2)[-1]
            decoded = base64.b64decode(b64).decode(errors="replace")
            user, _, password = decoded.partition(":")
            return user == self._user and password == self._password
        except Exception:
            return False
```

- [ ] **Step 4: Tests ausführen — müssen PASS**

```bash
python3 -m pytest tests/base_station/test_ntrip_server.py -v
```

Erwartung: 7 passed. (Falls `test_real_ntrip_client_receives_broadcast` flaky wirkt wegen der `time.sleep`-Werte: leicht erhöhen, nicht das Timing aus dem Design werfen — echte Sockets auf localhost sind normalerweise sehr schnell, 0.3s/0.2s sollten großzügig reichen.)

- [ ] **Step 5: Volle Suite laufen lassen**

```bash
python3 -m pytest tests/ -q
```

Erwartung: 235 + 7 = 242 passed, 0 failed.

- [ ] **Step 6: Commit**

```bash
git add base_station/ntrip_server.py tests/base_station/test_ntrip_server.py
git commit -m "feat(base-station): add NtripServer — minimal NTRIP caster, NtripClient-compatible"
```

---

## Task 3: `base_station/main.py`

### Background

Verdrahtet `RtcmSerialSource.on_data → NtripServer.broadcast`, liest Konfiguration aus Umgebungsvariablen. **Kein Soft-Fallback** wie bei `mower/main.py`: fehlt der serielle Port oder fehlen die Pflicht-Zugangsdaten, muss der Start klar fehlschlagen. Da `RtcmSerialSource` (Task 1) den Port bereits im Konstruktor öffnet, propagiert eine fehlende Hardware die Exception automatisch — hier wird nichts abgefangen.

Dünnes Verdrahtungsskript, keine dedizierten Tests (mirrort `mower/main.py`) — die Substanz steckt in den bereits getesteten Modulen aus Task 1/2.

**Files:**
- Create: `base_station/main.py`

- [ ] **Step 1: `base_station/main.py` implementieren**

```python
"""Base station entrypoint: reads local RTCM3 corrections from the base
LC29H and serves them to rovers via NtripServer.

Configuration via environment variables (BASE_NTRIP_USER/PASSWORD are
required — no default):
  BASE_SERIAL_PORT    — serial port to the base LC29H (default: /dev/ttyUSB0)
  BASE_NTRIP_PORT     — TCP port for the caster (default: 2101)
  BASE_MOUNTPOINT     — NTRIP mountpoint name (default: MV2BASE)
  BASE_NTRIP_USER     — required, no default
  BASE_NTRIP_PASSWORD — required, no default

Unlike mower/main.py, this entrypoint does NOT degrade gracefully: without a
working GNSS module this device has no job, so a missing serial port or
missing credentials must fail loudly at startup rather than running a
silently useless process.
"""
import logging
import os
import time

from base_station.rtcm_source import RtcmSerialSource
from base_station.ntrip_server import NtripServer

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

DEFAULT_SERIAL_PORT = "/dev/ttyUSB0"
DEFAULT_NTRIP_PORT = 2101
DEFAULT_MOUNTPOINT = "MV2BASE"


def _require_env(name: str) -> str:
    value = os.environ.get(name)
    if not value:
        raise RuntimeError(f"{name} must be set (no default — see base_station/main.py)")
    return value


def main() -> None:
    serial_port = os.environ.get("BASE_SERIAL_PORT", DEFAULT_SERIAL_PORT)
    ntrip_port = int(os.environ.get("BASE_NTRIP_PORT", DEFAULT_NTRIP_PORT))
    mountpoint = os.environ.get("BASE_MOUNTPOINT", DEFAULT_MOUNTPOINT)
    user = _require_env("BASE_NTRIP_USER")
    password = _require_env("BASE_NTRIP_PASSWORD")

    server = NtripServer(host="0.0.0.0", port=ntrip_port, mountpoint=mountpoint,
                         user=user, password=password)
    server.start()
    logger.info("NTRIP caster listening on 0.0.0.0:%d/%s", ntrip_port, mountpoint)

    # Raises immediately if the serial port is unavailable — intentional,
    # see module docstring: no soft fallback for a device with no other job.
    source = RtcmSerialSource(port=serial_port)
    source.on_data = server.broadcast
    source.start()
    logger.info("Relaying RTCM3 from %s", serial_port)

    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        source.stop()
        server.stop()


if __name__ == "__main__":
    main()
```

- [ ] **Step 2: Fail-loud-Verhalten manuell verifizieren**

Auf dieser Dev-Maschine ist kein `/dev/ttyUSB0` vorhanden — das ist die Gelegenheit, das gewünschte Fail-Loud-Verhalten direkt zu beweisen, ohne echte Hardware zu brauchen:

```bash
BASE_NTRIP_USER=rover BASE_NTRIP_PASSWORD=secret python3 -m base_station.main
```

Erwartung: Der Prozess startet den `NtripServer` erfolgreich (Log-Zeile "NTRIP caster listening…"), bricht dann beim Konstruieren von `RtcmSerialSource` mit einer klaren, nicht abgefangenen Exception ab (z. B. `serial.serialutil.SerialException: ... No such file or directory: '/dev/ttyUSB0'`). Das ist das **korrekte, gewünschte** Verhalten — kein stiller Fallback.

Zusätzlich, fehlende Pflicht-Env-Vars:

```bash
python3 -m base_station.main
```

Erwartung: `RuntimeError: BASE_NTRIP_USER must be set (no default — see base_station/main.py)` — bricht ab, bevor überhaupt ein Socket geöffnet wird.

- [ ] **Step 3: Import-Sanity-Check**

```bash
python3 -c "from base_station.main import main; print('import ok')"
```

Erwartung: `import ok` — der Import selbst darf nicht fehlschlagen (nur der Aufruf von `main()` ohne Hardware).

- [ ] **Step 4: Volle Suite ein letztes Mal laufen lassen**

```bash
python3 -m pytest tests/ -q
```

Erwartung: weiterhin 242 passed, 0 failed (dieser Task fügt keine neuen Tests hinzu).

- [ ] **Step 5: Commit**

```bash
git add base_station/main.py
git commit -m "feat(base-station): add main.py entrypoint — wires RtcmSerialSource to NtripServer"
```

---

## Acceptance Criteria

Nach Abschluss aller Tasks:

- [x] `python3 -m pytest tests/ -q` → **242 passed, 0 failed** (231 bestehend + 11 neu: 4 `RtcmSerialSource` + 7 `NtripServer`)
- [x] `base_station/rtcm_source.py` — `RtcmSerialSource` liest rohe Bytes injizierbar (Backend-Injection wie `Camera`, nicht wie `SerialDriver`), übersteht transiente Lesefehler ohne den Thread zu beenden; `stop()` schließt den seriellen Handle (im Review nachgebessert)
- [x] `base_station/ntrip_server.py` — `NtripServer` ist handshake-kompatibel zum bestehenden, unveränderten `NtripClient` (bewiesen durch `test_real_ntrip_client_receives_broadcast`), Thread-sichere Verbindungsliste, mehrere gleichzeitige Rover unterstützt; tote Verbindungen werden aus dem Broadcast entfernt, abgelehnte Handshakes landen nie in der Liste, Shutdown-Race gegen einen gerade akzeptierten Client geschlossen, Logging ergänzt (alles im Review empirisch per Mutation-Testing nachgewiesen)
- [x] `base_station/main.py` — verdrahtet beide, kein Soft-Fallback (manuell verifiziert: fehlender Serial-Port und fehlende Pflicht-Env-Vars brechen jeweils klar ab)
- [x] `mower/nav/ntrip_client.py` **unverändert** — bestätigt: `git diff` gegen den Rover-Client ist leer, keine Rover-seitigen Code-Änderungen nötig, nur Konfiguration (host/port/mountpoint/Zugangsdaten) beim Deployment
- [x] Alle neuen Module ohne echte Hardware importierbar und testbar

Finale Gesamt-Review (Verdikt: **merge-bereit**, keine kritischen/wichtigen Funde) ergänzend geprüft: Threading-Modell unter Last analysiert (Accept-Thread, Handler-Threads, Broadcast vom RTCM-Reader-Thread, Shutdown), keine Deadlocks/Races gefunden.

### Nicht Teil dieses Plans (Hardware-Bring-up, siehe Spec Abschnitt 7)

- LC29H #2 in den Base-/Survey-in-Modus versetzen
- Antennenposition vermessen/fixieren
- RPi Zero + LC29H physisch aufbauen, Netzwerkanbindung zum Rover
- Prozess-Supervision (systemd-Unit) für `base_station/main.py`
- Feldtest der tatsächlichen Korrektur-Qualität/Fix-Zeit

### Deferred Hardening (aus der finalen Review, nicht blockierend — trivial verifizierbar unschädlich, da `NtripClient` aktuell nirgends mit GGA-Versand verdrahtet ist)

- [ ] **GGA/Client-Bytes werden nie gelesen.** `NtripServer` liest den Handshake einmal und danach nie wieder vom Client-Socket — sendet ein künftig verdrahteter Rover GGA-Sätze, sammeln die sich unread im Kernel-Empfangspuffer, bis TCP-Flusskontrolle nach Stunden zum Reconnect-Zyklus zwingt. Fix: Client-Socket weiter drainen (verwerfen) ODER dokumentieren, dass Rover gegen diese Basis kein `on_gga_sentence` verdrahten dürfen.
- [ ] **Keine Socket-Timeouts serverseitig.** `conn.recv()` beim Handshake und `sendall()` beim Broadcast blockieren unbegrenzt — ein Client, der verbindet und nie sendet, blockiert einen Handler-Thread für immer (unbounded, da Handler-Threads nicht getrackt werden); ein hängender Rover kann `broadcast()` für alle anderen kurzzeitig blockieren. Auf dem vorgesehenen vertrauenswürdigen LAN mit wenigen Rovern geringes Risiko, aber `settimeout()` auf beiden wäre die saubere Härtung.
- [ ] **Lese-Loop-Logging in `RtcmSerialSource` etwas lauter als das zitierte Vorbild** — loggt `exc_info=True` (voller Traceback) pro fehlgeschlagener Iteration statt `SerialDriver`s knapperem `logger.error("Recv error: %s", e)`. Bei dauerhaft gezogenem USB-Gerät flutet das die Logs. Kosmetisch, kein Korrektheitsproblem.

---

## Notes

- **DRY:** `RtcmSerialSource` folgt `Camera`s Injection-Pattern; Testkonventionen (Fake-Backend, Loopback-Sockets) folgen bestehenden Mustern (`test_camera.py`, `TestSerialDriver` in `test_hardware_interface.py`).
- **YAGNI:** kein RTCM3-Frame-Parsing (nicht nötig, reiner Byte-Relay), kein GGA/VRS-Handling im Server (eine stationäre Basis braucht das nicht), kein Reconnect-Backoff über das bereits etablierte Muster hinaus.
- **Kompatibilität ist der Kern-Risikopunkt dieses Plans** — deshalb testet Task 2 explizit gegen den echten `NtripClient`, nicht nur gegen einen im Test nachgebauten Handshake.
