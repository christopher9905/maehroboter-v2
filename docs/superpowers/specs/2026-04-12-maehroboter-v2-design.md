# Mähroboter V2 — Design Spec
**Datum:** 2026-04-12  
**Status:** Draft v2

---

## 1. Projektziel

Bau eines autonomen Mähroboters auf Basis eines 1:14 RC-Traktors mit originalgetreuem Mähwerk. Keine Außenkantenkabel — Navigation ausschließlich über RTK-GPS und Sensorfusion. Teach-In-basierte Zonendefinition, zonenbasierte Coverage mit adaptiver Bahnplanung, supervisierter Autonomiebetrieb mit Web UI.

---

## 2. Anforderungen

### Funktional
- Rasenfläche: ~2000 m², komplex (Bäume, Zäune, Hänge, Teilflächen über Wege verbunden)
- Positionsgenauigkeit: <3–5 cm (RTK-GPS)
- Teach-In: Manuelles Abfahren der Grenzen und Zonen
- Mähmuster: Zonenbasiert, pro Zone adaptiver Boustrophedon (optimaler Winkel je Zonengeometrie)
- Betriebsmodus: Supervisierte Autonomie — autonom mähend, per Web UI überwachbar, jederzeit manuell übersteuerbar
- Automatische Rückkehr zur Ladestation (RTK-gesteuert + ArUco-Fein-Docking)
- Ladestation Phase 1: Roboter fährt präzise in Ladebereich, Kabel wird manuell angesteckt

### Sicherheit
- Lift-Erkennung (Reed-/Hallschalter am Mähwerk, via Teensy): sofortiger Klingenstopp
- Tilt-Erkennung (BNO085 IMU, via RPi 5): Notbremsung bei Kippgefahr
- GPS-Geofencing: Notbremsung bei Verlassen der definierten Außengrenze
- Regensensor (kapazitiv, via Teensy-ADC): Mähbetrieb pausieren bei ADC-Wert >600 (uint16, 12-bit ADC), Entprellzeit 3 s (3 aufeinanderfolgende Messungen über 200 ms), Wiederaufnahme nach 30 min Trockenheit (konfigurierbar)
- Kamera-basierte Hinderniserkennung: Stopp und Umfahrung mit Timeout
- Hardware-Watchdog auf Teensy: Notbremsung bei Kommunikationsausfall zum RPi 5 (>500 ms ohne Heartbeat-Ping)

### Nicht-Funktional
- Kein Außenkantenkabel
- Kein ROS2 (reines Python-Stack)
- Web UI statt ROS-Visualisierung
- Modulare Architektur: Hailo-8L und LiDAR nachrüstbar ohne Umbau
- Betrieb ohne Simulation — direkte Hardware-Iteration

---

## 3. Hardware-Stack

### Hauptrechner & Echtzeit-Controller
| Komponente | Rolle |
|---|---|
| Raspberry Pi 5 (8 GB) | Hauptrechner: Navigation, CV, Web UI, Sensorfusion, Mission Executive |
| Teensy 4.1 | Echtzeit-Controller: Motorsteuerung, Servos, Sensorauslese, Safety-Watchdog |
| USB-Serial | Kommunikation RPi 5 ↔ Teensy 4.1 (binäres Protokoll, 921600 baud) |

### Navigation & Sensorik
| Komponente | Bus / Anschluss | Rolle |
|---|---|---|
| Quectel LC29H | UART → RPi 5 (GPIO) | RTK-GPS, NMEA + RTCM |
| BNO085 | I2C → RPi 5 direkt | 9-DoF IMU: Orientierung, Lage, Tilt. Direkt am RPi um Latenz in EKF zu minimieren |
| RPi Camera Module 3 | CSI → RPi 5 | Hinderniserkennung, ArUco-Docking |
| Kapazitiver Regensensor | ADC → Teensy | Mähpause bei Nässe |
| Reed-/Hallschalter | Digital-IO → Teensy | Mähdeckhebung → sofortiger Klingenstopp |
| Encoder am Antriebsmotor | Digital-IO → Teensy | Odometrie für EKF-Sensorfusion |

### Antrieb & Aktorik
| Komponente | Rolle |
|---|---|
| Brushed DC Motor + Cytron MDD10A | Fahrantrieb (Vor/Rück, Geschwindigkeit) |
| Lenkservo | Vorderradlenkung, Teensy-gesteuert |
| Brushless ESC + Motor (je Rotor) | Mähwerk-Antrieb (kompatibel mit 4S: 16.8V max) |

### Stromversorgung
**Entscheidung: 4S2P Li-Ion** (21700-Zellen, 14.4V nominal, 16.8V voll geladen)

Gründe: Cytron MDD10A kompatibel bis 25V ✓, Brushless ESCs für Mähwerk als 4S-kompatibel wählen ✓, niedrigere Spannung = geringeres Sicherheitsrisiko, ausreichende Energie für ~2000 m².

| Komponente | Rolle |
|---|---|
| 4S2P 21700 Li-Ion Pack | Hauptakku (~60–80 Wh je nach Zellkapazität) |
| BMS mit I2C-Interface (z.B. basierend auf BQ76940) | Schutz, Balancing, SOC-Schätzung via I2C → Teensy |
| DC-DC Buck Converter | 5V/5A → RPi 5; 3.3V → Logik |
| Spannungsteiler Backup | Teensy ADC als Fallback für grobe SOC-Schätzung |
| Ladeanschluss | Präzisions-Docking (ArUco) + manuelles Kabel (Phase 1) |

**Spannungsschienen:**
- Hauptpack (14.4V): Brushless ESC / Mähwerk, Cytron MDD10A direkt
- 5V Buck: RPi 5, Teensy 4.1
- 3.3V LDO: Logik, Sensoren

### Spätere Erweiterungen (vorbereitet, nicht sofort)
- Hailo-8L HAT (RPi 5 HAT-Slot) — CV-Beschleunigung
- Zweiter LC29H + RPi Zero — eigene RTK-Basisstation
- LiDAR (z.B. RPLidar A1) — Fallback bei schlechtem RTK

---

## 4. Software-Architektur

Reines Python-Stack auf RPi 5. Klare Schichtenarchitektur — jede Schicht hat eine Verantwortung und kommuniziert über definierte Interfaces. Schichten 4 und 5 sind parallele Peer-Schichten (gleichzeitig aktiv, nicht sequentiell).

```
┌──────────────────────────────────────────────────┐
│            Schicht 6: Web UI                     │
│            (FastAPI + Leaflet.js)                │
├──────────────────────────────────────────────────┤
│            Schicht 5: Mission Executive          │
│            (State Machine)                       │
├─────────────────────┬────────────────────────────┤
│  Schicht 3:         │  Schicht 4:                │
│  Coverage Path      │  Computer Vision           │
│  Planner            │  (OpenCV + MobileNet-SSD)  │
├─────────────────────┴────────────────────────────┤
│            Schicht 2: Karte & Zonen              │
│            (GeoJSON, shapely)                    │
├──────────────────────────────────────────────────┤
│            Schicht 1: Lokalisierung & EKF        │
│            (filterpy, GPS + IMU + Odometrie)     │
├──────────────────────────────────────────────────┤
│            Schicht 0: Hardware Abstraction       │
│            (Teensy 4.1 + pyserial)               │
└──────────────────────────────────────────────────┘
```

---

### Schicht 0 — Hardware Abstraction Layer

**Teensy-Firmware (C++):**
- Motorkommandos empfangen und ausführen (Antrieb, Lenkservo, Mähwerk-ESC)
- Sensorwerte lesen: Encoder, Regensensor (ADC), Lift-Schalter, BMS SOC (I2C)
- Safety-Watchdog: kein Heartbeat-Ping in 500 ms → autonome Notbremsung (unabhängig von RPi)
- Interrupt-basierte Lift-Erkennung: sofortiger Klingenstopp ohne Warten auf Kommando

**Python Serial Driver (RPi 5):**
- Binäres Protokoll über USB-Serial, 921600 baud
- Byte-Budget: Kommandos 20 Hz × ~12 Byte = ~240 B/s; Telemetrie 50 Hz × ~40 Byte = ~2 kB/s; gesamt <<10 kB/s → ausreichend Headroom
- `pyserial` mit `low_latency`-Flag (via `udev`-Regel) für <1 ms Latenz
- Kommandos: `{DRIVE: speed, steering}`, `{BLADE: state}`, `{ESTOP}`, `{PING}`
- Telemetrie: `{SENSORS: rain_adc, lift_bool, soc_percent, encoder_ticks}`, `{STATUS}`

**Hinweis:** BNO085 ist direkt per I2C am RPi 5 angeschlossen (nicht via Teensy), da IMU-Daten mit 100 Hz direkt in den EKF fließen und minimale Latenz erfordern.

---

### Schicht 1 — Lokalisierung & EKF-Sensorfusion

**NTRIP-Client:** RTK-Korrekturdaten vom Dienst (SAPOS oder RTK2go) → LC29H → RTK-Fix

**EKF (filterpy):**

State-Vektor: `[x, y, heading, v, yaw_rate]` (Position in UTM, Geschwindigkeit, Gierrate)

| Sensor | Update-Rate | Messmodell |
|---|---|---|
| RTK-GPS (LC29H) | 5–10 Hz | Absolute Position (x, y) |
| BNO085 (IMU) | 100 Hz | Heading, Yaw-Rate |
| Encoder (Odometrie) | 20–50 Hz | Geschwindigkeit, relative Positionsänderung |

- Koordinatenrahmen: UTM (Zone bei Session-Start fixiert, keine Zonenübergänge erwartet)
- Rauschmodell: Prozessrauschen empirisch getuned in Phase 2
- Qualitäts-Flags: RTK-Fix / Float / kein Fix

**Verhalten je GPS-Qualität:**
| GPS-Status | Mission Executive Verhalten |
|---|---|
| RTK-Fix | Normalbetrieb |
| RTK-Float | Reduzierte Geschwindigkeit (50%), keine neuen Zonen starten |
| Kein Fix < 10 s | Stopp an Stelle, warten |
| Kein Fix > 10 s | → RETURNING (Heimfahrt auf letzter bekannter Position) |
| Geofence-Verletzung | Sofort → ERROR + ESTOP |

---

### Schicht 2 — Karte & Zonen

**GeoJSON-Schema:**
```json
{
  "type": "FeatureCollection",
  "features": [
    {"type": "Feature", "properties": {"type": "boundary"}, "geometry": {"type": "Polygon"}},
    {"type": "Feature", "properties": {"type": "zone", "id": "zone_1", "order": 1}, "geometry": {"type": "Polygon"}},
    {"type": "Feature", "properties": {"type": "no-go"}, "geometry": {"type": "Polygon"}},
    {"type": "Feature", "properties": {"type": "transit-path", "blade": false}, "geometry": {"type": "LineString"}},
    {"type": "Feature", "properties": {"type": "charging-station"}, "geometry": {"type": "Point"}}
  ]
}
```

**Teach-In-Modus:**
- Operator steuert Roboter manuell (Web UI Touch-Joystick oder RC-Fernbedienung)
- GPS-Punkte werden alle 0.5 m Fahrweg oder alle 1 s aufgezeichnet (je nachdem was zuerst eintritt)
- Grenz-Schließung: automatisch wenn Roboter auf < 1 m zum Startpunkt kommt
- Zonen-Wechsel: per Web UI Taste während Teach-In
- Transit-Pfade: als eigene Teach-In-Sequenz zwischen Zonen (Blade-Flag automatisch `false`)
- Unvollständige Zonen werden als Draft gespeichert und können in Web UI editiert werden

**Geofencing:** Laufende Enthaltheitsprüfung (`shapely.contains`) gegen Außengrenze, 10 Hz

---

### Schicht 3 — Coverage Path Planner

Pro Zone wird ein optimaler Boustrophedon-Pfad berechnet:

1. Zonenpolygon einlesen (shapely)
2. Optimalen Mähwinkel berechnen (längste Achse des Minimum-Bounding-Rectangle → minimale Wendeanzahl)
3. Parallele Bahnen mit 38 cm Abstand (10% Überlapp auf 40 cm Mähbreite) erzeugen
4. Bahnen als geordnete Waypoint-Liste ausgeben
5. Zonenreihenfolge optimieren (kürzeste Überfahrt, Nearest-Neighbor TSP-Näherung)
6. Transit-Pfade zwischen Teilflächen: Blade `off` während Traversierung

Bibliotheken: `shapely`, `numpy`, `scipy`

---

### Schicht 4 — Computer Vision

**Hinderniserkennung (Mähbetrieb):**
- Modell: MobileNet-SSD v2 (SSD Lite Variante), Input 300×300, OpenCV DNN Backend
- Ziel-FPS: ~10–15 FPS auf RPi 5 CPU (ausreichend bei 0.3–0.5 m/s)
- Relevante COCO-Klassen: `person`, `dog`, `cat`, `bird` (sicherheitskritisch) + `chair`, `potted plant`, `bench` (Hindernis)
- Mindest-Konfidenz: 0.5
- Erkanntes Hindernis → Mission Executive → `OBSTACLE_AVOIDANCE`

**Docking (Ladestation):**
- ArUco-Marker (Dictionary DICT_4X4_50, Größe 15×15 cm) an Ladestations-Eingang
- Kamera am Roboter: ca. 20 cm Montagehöhe, leicht nach unten geneigt
- Verlässliche Detektionsreichweite: ~1.5 m bei gewählter Marker-Größe und RPi Camera Module 3
- Handoff-Logik: RTK führt Roboter bis Marker erkannt UND Abstand < 1.2 m → Übergabe an visuelle Servoregelung
- Visuelle Servoregelung (`cv2.aruco`): Letzte 1.2 m bis Zielposition (Ladekontakt)

**Später:** Hailo-8L beschleunigt auf >30 FPS, gleiche OpenCV-API — kein Code-Umbau

---

### Schicht 5 — Mission Executive (State Machine)

```
IDLE ──→ TEACH_IN ──→ IDLE
IDLE ──→ MOWING ──→ OBSTACLE_AVOIDANCE ──→ MOWING
                 OBSTACLE_AVOIDANCE ──→ ERROR  (Timeout 60s oder unauflösbar)
MOWING ──→ RETURNING ──→ DOCKING ──→ CHARGING ──→ IDLE
* ──→ ERROR  (Lift, Tilt, Geofence-Verletzung, GPS-Verlust >10s)
ERROR ──→ IDLE  (nach manuellem Reset via Web UI)
```

- Akkustand-Schwelle für RETURNING: konfigurierbar, default 20% SOC
- Zeitplan-Unterstützung: cron-artig via `APScheduler`
- Obstacle Avoidance: max. 60 s oder 3 Umfahrungsversuche → ERROR wenn nicht auflösbar
- Tilt-Schwelle: >30° → ERROR + ESTOP (konfigurierbar)

---

### Schicht 6 — Web UI

- **Backend:** FastAPI (async), WebSocket für Echtzeit-Telemetrie, `asyncio` + `threading` (Serial-I/O in eigenem Thread)
- **Frontend:** HTML/JS + Leaflet.js

Features:
- Live-Karte mit Roboterposition, Mähpfad, Zonen
- Zonen-Editor (Polygone zeichnen, bearbeiten, löschen)
- Mission starten / stoppen / pausieren
- Manuelle Steuerung (Touch-Joystick)
- Telemetrie-Dashboard: Akkustand (SOC%), GPS-Qualität, Status, Regen, Geschwindigkeit
- Zeitplan-Konfiguration
- Teach-In-Modus aktivieren / Zone wechseln
- Fehlerlog und Reset

---

## 5. Kommunikationsprotokoll RPi 5 ↔ Teensy 4.1

**Frame-Format:**
```
[0xAA][CMD_TYPE 1B][PAYLOAD_LEN 1B][PAYLOAD nB][CRC8 1B]
```

- Start-Byte: `0xAA`
- CRC-Polynom: **CRC-8/MAXIM** (Dallas 1-Wire, Polynom 0x31) — unterstützt in Arduino (`OneWire`-Library) und Python (`crcmod`)
- Baud-Rate: **921600**
- `pyserial` mit low-latency USB-Serial (udev-Regel: `ATTR{bInterfaceClass}=="02" ... setserial /dev/ttyACM0 low_latency`)
- Watchdog basiert auf ausbleibendem `PING`-Kommando (nicht auf Datagrammen)

| Typ | Richtung | Rate | Payload |
|---|---|---|---|
| `DRIVE` | RPi→Teensy | 20 Hz | speed (float), steering_angle (float) |
| `BLADE` | RPi→Teensy | On-demand | state (bool) |
| `ESTOP` | RPi→Teensy | On-demand | — |
| `PING` | RPi→Teensy | 10 Hz | sequence_nr |
| `SENSORS` | Teensy→RPi | 50 Hz | rain_adc (uint16), lift (bool), encoder_ticks (int32) |
| `SOC` | Teensy→RPi | 1 Hz | soc_percent (uint8), voltage_mv (uint16) |
| `STATUS` | Teensy→RPi | 10 Hz | watchdog_ok, blade_running, error_flags |

---

## 6. RTK-Setup

- **Phase 1:** NTRIP-Client auf RPi 5 nutzt externen Korrekturdienst (SAPOS Deutschland ~200€/Jahr, oder kostenlos RTK2go)
- **Phase 2 (optional):** Eigene Basisstation (zweiter LC29H + RPi Zero, stationär), lokaler NTRIP-Server → unabhängig von Internetverbindung

---

## 7. Sicherheits-Prereqs vor autonomem Betrieb

**Geofencing und Tilt-Erkennung sind Voraussetzung (Gate) bevor Phase 3 (autonome Fahrten) beginnt.** Reihenfolge innerhalb Phase 2:
1. EKF-Fusion und RTK-Fix verifizieren
2. Geofencing implementieren und testen
3. Tilt-Erkennung implementieren und testen
4. Erst dann: Teach-In und erste autonome Wegpunktfahrten

---

## 8. Entwicklungsphasen

### Phase 1 — Mechanik & Hardware-Basis
- RC-Traktor-Unterbau aufbauen, Teensy 4.1 einbauen
- Motoren, Lenkservo, Mähwerk-ESC verdrahten und testen
- Spannungsversorgung (4S2P Pack, BMS, DC-DC) aufbauen
- USB-Serial Protokoll zwischen Teensy und RPi 5 implementieren und testen
- Grundlegende Fahrt- und Lenktests (manuell per Serial)

### Phase 2 — Navigation, Lokalisierung & Sicherheit
- LC29H RTK-GPS integrieren, NTRIP-Client einrichten, RTK-Fix verifizieren
- BNO085 IMU (I2C, RPi 5) einbinden
- EKF-Fusion implementieren und tunen (GPS + IMU + Encoder)
- **Geofencing implementieren und testen (GATE)**
- **Tilt-Erkennung implementieren und testen (GATE)**
- Teach-In-Modus implementieren, Karten speichern/laden

### Phase 3 — Coverage Path Planning & Waypoint-Folge
- Boustrophedon-Pfadplanung pro Zone implementieren (shapely)
- Waypoint-Folge-Regler: **Stanley Controller** (primär — robuste Querfehler-Kompensation bei niedrigen Geschwindigkeiten mit Ackermann-Lenkung; Pure Pursuit als Fallback)
- Erste autonome Fahrten ohne Mähwerk
- Zonenreihenfolge und Transit-Pfade

### Phase 4 — Computer Vision
- MobileNet-SSD Hinderniserkennung integrieren
- Obstacle-Avoidance Logik im Mission Executive
- Lift-Erkennung (Reed/Hall via Teensy), Regensensor-Logik

### Phase 5 — Web UI & Mission Executive
- FastAPI Backend + Leaflet.js Frontend
- State Machine vollständig implementieren
- Zeitplan, manuelle Übersteuerung, Fehlerlog

### Phase 6 — Docking & Laden
- ArUco-Marker an Ladestation montieren
- Visuelle Servoregelung für präzises Docking implementieren
- Mähwerk in Betrieb nehmen (echte Mähfahrten)

### Phase 7 — Tuning & Erweiterungen
- Hailo-8L HAT bei Bedarf
- Eigene RTK-Basisstation (Phase 2 RTK)
- LiDAR-Integration als RTK-Fallback bei schlechtem Empfang

---

## 9. Technologie-Stack

| Bereich | Technologie | Rolle |
|---|---|---|
| Sprache RPi 5 | Python 3.11+ | Gesamter Software-Stack |
| Sprache Teensy | C++ (Arduino-Framework) | Echtzeit-Hardware-Controller |
| Sensorfusion | `filterpy` | EKF (GPS + IMU + Encoder) |
| Geometrie / Pfadplanung | `shapely`, `numpy`, `scipy` | Zonen, Boustrophedon, TSP |
| Computer Vision | `opencv-python`, OpenCV DNN | Hinderniserkennung (MobileNet-SSD) |
| ArUco-Docking | `cv2.aruco` (Teil von OpenCV) | Marker-Detektion, visuelle Servo |
| GPS / NMEA | `pyserial`, `pynmea2` | LC29H-Kommunikation, NMEA-Parsing |
| Web Backend | `FastAPI`, `websockets` | REST API + Echtzeit-Telemetrie |
| Web Frontend | HTML/JS, Leaflet.js | Kartenansicht, Steuerung |
| Kartendaten | GeoJSON | Zonen, Grenzen, Pfade |
| Serial-Kommunikation | `pyserial` | RPi 5 ↔ Teensy 4.1 |
| Concurrency | `asyncio` + `threading` | FastAPI async + Serial-Thread |
| Missionszeitplan | `APScheduler` | Cron-artige Mähzeitpläne |
| CRC-Prüfung | `crcmod` (Python), OneWire (Teensy) | Protokoll-Integrität (CRC-8/MAXIM) |
