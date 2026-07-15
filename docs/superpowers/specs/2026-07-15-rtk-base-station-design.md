# Eigene RTK-Basisstation — Design Spec

## 1. Ziel

Der Mähroboter bezieht RTK-Korrekturdaten heute per NTRIP-Client (`mower/nav/ntrip_client.py`) von einem externen Dienst (SAPOS oder RTK2go) — abhängig von einer Internetverbindung und ggf. kostenpflichtig. Dieses Projekt baut eine **eigene, stationäre RTK-Basisstation**: ein zweites Quectel-LC29H-Modul an einem RPi Zero, das lokal RTCM3-Korrekturen erzeugt und per eigenem NTRIP-Caster an den Rover (und potenziell weitere Rover) ausliefert — unabhängig von Internet und externem Dienst.

Dies ist Teil von Phase 7 (Tuning & Erweiterungen) laut [Design-Spec](2026-04-12-maehroboter-v2-design.md), Abschnitt 6 ("Phase 2 (optional): Eigene Basisstation").

## 2. Scope

**Physisch zwei getrennte Geräte:**
- **Rover** (bestehend, RPi 5 auf dem Mähroboter) — `mower/nav/ntrip_client.py` bleibt unverändert; zeigt statt auf SAPOS/RTK2go künftig auf die eigene Basisstation (host/port/mountpoint/Zugangsdaten sind ohnehin Konstruktor-Parameter).
- **Basisstation** (neu, RPi Zero + zweites LC29H, stationär) — neues Top-Level-Verzeichnis `base_station/`.

**In Scope:**
- Ein minimaler NTRIP-Caster-Server, kompatibel zum bestehenden `NtripClient`-Handshake.
- Eine serielle RTCM3-Quelle, die Bytes vom lokalen LC29H (im Base-Mode) abgreift und weiterreicht.
- Verdrahtung + Konfiguration über Umgebungsvariablen (`base_station/main.py`).

**Explizit außerhalb des Scopes (Hardware-Bring-up, separat zu tracken):**
- Das LC29H-Modul der Basisstation in den Base-/Survey-in-Modus versetzen (modul-spezifische Konfiguration, einmalig, außerhalb dieses Codes — es existiert kein Datenblatt/keine Hardware in dieser Umgebung, um das zu verifizieren).
- Physischer Aufbau/Standort der Basisstation, Antennenmontage.
- Netzwerk zwischen Basisstation und Rover (WLAN/Ethernet) einrichten.
- Feldtest der tatsächlichen Korrektur-Qualität/Fix-Zeit.

## 3. Architektur

```
LC29H (Base-Mode, stationär)
        │ UART (rohe RTCM3-Bytes)
        ▼
RtcmSerialSource.on_data(bytes)
        │
        ▼
NtripServer.broadcast(bytes)
        │
   ┌────┴────┬─────────────┐
   ▼         ▼             ▼
Rover 1   Rover 2  …   Rover N
(NtripClient, bestehend, unverändert)
```

### 3.1 `base_station/rtcm_source.py` — `RtcmSerialSource`

Liest rohe Bytes vom seriellen Port des Basis-LC29H und feuert sie per Callback. Kein RTCM3-Framing/CRC-Parsing nötig — die Bytes werden als opaker Strom durchgereicht (der Rover-seitige RTK-Empfänger interpretiert RTCM3, nicht wir).

```python
class RtcmSerialSource:
    def __init__(self, port: str, baud: int = 115200, serial_backend=None):
        ...
        self.on_data: Optional[Callable[[bytes], None]] = None

    def start(self) -> None: ...
    def stop(self) -> None: ...
```

Injizierbarer `serial_backend` (mirrort `SerialDriver`/`GpsReader`-Muster) — testbar ohne echte Hardware. `start()` öffnet den Port und startet einen Lese-Thread, der in einer Schleife `read()` aufruft und bei nicht-leeren Chunks `on_data` feuert. `stop()` beendet den Thread sauber (mirrort `SerialDriver.stop()`).

### 3.2 `base_station/ntrip_server.py` — `NtripServer`

Minimaler NTRIP-Caster: TCP-Server, der den exakten Handshake akzeptiert, den `NtripClient._connect()` bereits sendet:

```
GET /{mountpoint} HTTP/1.0\r\n
Host: {host}\r\n
Ntrip-Version: Ntrip/2.0\r\n
User-Agent: NTRIP MowerClient/1.0\r\n
Authorization: Basic {base64(user:password)}\r\n
\r\n
```

```python
class NtripServer:
    def __init__(self, host: str, port: int, mountpoint: str,
                 user: str, password: str):
        ...

    def start(self) -> None: ...
    def stop(self) -> None: ...
    def broadcast(self, data: bytes) -> None: ...
```

**Verhalten:**
- `start()` bindet einen TCP-Listener und startet einen Accept-Loop-Thread. Jede eingehende Verbindung wird in einem eigenen Handler behandelt: Request-Zeile + Header parsen, Mountpoint gegen den konfigurierten Wert prüfen, `Authorization`-Header gegen `user`/`password` prüfen. Bei Erfolg: `ICY 200 OK\r\n\r\n` senden, Socket zur Liste aktiver Verbindungen hinzufügen. Bei Mountpoint- oder Auth-Fehler: Fehlerantwort senden, Socket schließen, nicht zur Liste hinzufügen.
- `broadcast(data)` schreibt `data` an alle aktiven Sockets. Schlägt ein Schreibversuch fehl (Exception), wird der betroffene Socket aus der Liste entfernt — kein Crash, andere Verbindungen unberührt.
- `stop()` schließt den Listener und alle aktiven Client-Sockets, beendet den Accept-Thread.
- Mehrere gleichzeitige Rover werden unterstützt (Liste statt Einzelverbindung) — kostet im Design praktisch nichts extra, kein YAGNI-Verstoß.
- Kein GGA-Handling nötig: eine einzelne stationäre Basisstation sendet dieselben Korrekturen unabhängig von der Rover-Position (kein VRS-Modus). Vom Client evtl. gesendete GGA-Zeilen werden ignoriert.

### 3.3 `base_station/main.py`

Verdrahtet `RtcmSerialSource.on_data → NtripServer.broadcast`, liest Konfiguration aus Umgebungsvariablen:

| Variable | Default | Bedeutung |
|---|---|---|
| `BASE_SERIAL_PORT` | `/dev/ttyUSB0` | Serieller Port zum Basis-LC29H |
| `BASE_NTRIP_PORT` | `2101` | TCP-Port des Casters (NTRIP-Standard) |
| `BASE_MOUNTPOINT` | `MV2BASE` | Mountpoint-Name |
| `BASE_NTRIP_USER` | — (Pflicht) | Zugangsdaten für Rover-Verbindungen |
| `BASE_NTRIP_PASSWORD` | — (Pflicht) | Zugangsdaten für Rover-Verbindungen |

Anders als `mower/main.py` gibt es **keinen** Soft-Fallback: fehlt der serielle Port oder fehlen die Pflicht-Zugangsdaten, bricht der Start mit einer klaren Fehlermeldung ab. Ohne GNSS-Modul hat dieses Gerät keine sinnvolle Aufgabe — ein still funktionsloser Prozess wäre irreführender als ein klarer Fehler beim Start.

## 4. Fehlerbehandlung

| Fall | Verhalten |
|---|---|
| Falscher Mountpoint | Verbindung abgelehnt, Socket geschlossen |
| Falsche Zugangsdaten | Verbindung abgelehnt, Socket geschlossen |
| Rover trennt sich | Erkannt beim nächsten `broadcast()` (Schreibfehler), Socket aus Liste entfernt |
| Kein Rover verbunden | `broadcast()` ist No-op |
| Serieller Port zum LC29H nicht erreichbar | `main.py` bricht beim Start mit klarer Fehlermeldung ab |

## 5. Testing

Alles ohne echte Hardware testbar:

- **`RtcmSerialSource`**: injizierbarer Fake-Serial-Backend (mirrort `_FakeCapture` bei `Camera`). Tests: `on_data` feuert mit gelesenen Bytes; sauberer Thread-Start/-Stop (mirrort `SerialDriver`s bestehende Tests).
- **`NtripServer`**: echte Loopback-Sockets (`127.0.0.1:0`, OS vergibt freien Port) statt Mocks — die Klasse *ist* Socket-Protokoll-Handling, ein echter Client-Socket ist hier der treffendere Test als ein gemocktes `socket`-Modul. Tests:
  - Korrekter Handshake → Client empfängt `ICY 200 OK`
  - Falscher Mountpoint → Verbindung abgelehnt
  - Falsche Zugangsdaten → Verbindung abgelehnt
  - `broadcast(data)` kommt beim verbundenen Client an (exakte Bytes)
  - Mehrere gleichzeitige Clients erhalten denselben Broadcast
  - Ein getrennter Client lässt nachfolgende `broadcast()`-Aufrufe nicht crashen
- **`base_station/main.py`**: dünnes Verdrahtungsskript, keine dedizierten Tests (mirrort `mower/main.py`) — die Substanz steckt in den getesteten Modulen.

## 6. Kompatibilität mit dem Rover

Keine Code-Änderung am Rover nötig. `NtripClient(host, port, mountpoint, user, password)` wird beim Deployment einfach mit den Werten der eigenen Basisstation statt SAPOS/RTK2go konfiguriert — reine Konfigurationsfrage, keine Architekturänderung an `mower/nav/ntrip_client.py`.

## 7. Nicht automatisierbares Hardware-Bring-up (separat zu tracken)

- Zweites LC29H in den Base-/Survey-in-Modus versetzen (modulspezifische Einmalkonfiguration).
- Antennenposition der Basisstation vermessen/fixieren (Survey-in-Genauigkeit hängt davon ab).
- RPi Zero + LC29H physisch aufbauen, Stromversorgung, Netzwerkanbindung zum Rover.
- Feldtest: tatsächliche Fix-Zeit und Korrektur-Qualität gegenüber SAPOS/RTK2go vergleichen.
