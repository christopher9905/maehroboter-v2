# Mähroboter-V2-Simulation

Dieser Ordner enthält ausschließlich Ersatzgeräte und das virtuelle
Umgebungsmodell. Zustandsmaschine, Teach-In, Planung, Regler, Sicherheitslogik,
API und UI bleiben Produktionscode.

Start:

```bash
python -m mower.simulation
```

Optionale Variablen:

```text
MV2_SIM_HOST=127.0.0.1
MV2_SIM_PORT=8090
MV2_DATA_PATH=mower/data/simulation-control.json
```

Der erste Ausbauschritt stellt bereit:

- zweidimensionale Fahrzeugbewegung in metrischen UTM-Koordinaten,
- Verarbeitung der echten DRIVE-, BLADE-, ESTOP- und PING-Frames,
- SENSORS-, SOC- und STATUS-Telemetrie über `HardwareInterface`,
- simulierte GPS- und IMU-Geräte mit den produktiven Datenklassen,
- Rohsensor-Injektion über `SimulationWorld.set_sensor_state()`,
- Simulations-API unter `/api/simulation/state`, `/sensors`, `/speed` und `/reset`.

Zusätzlich ist jetzt die vollständige gemeinsame Missionspipeline aktiv:

- GPS, IMU und Encoder werden vom produktiven `Localizer` fusioniert,
- Teach-In wird serverseitig in UTM aufgezeichnet und als WGS84-Polygon gespeichert,
- der Coverage-V2-Planer erzeugt Randbahnen, vollständige Innenbahnen und
  kinematisch fahrbare Verbindungen auf dem echten Flächenpolygon,
- der prädiktive MPC-Regler fährt die Wegpunkte über `HardwareInterface` ab,
- Pause/Fortsetzen, Soft-Stop, Not-Aus und Sicherheitsfehler wirken unverändert,
- die geplante Route wird in der normalen Produktions-UI dargestellt.

Der Teach-In-Assistent verwendet ebenfalls diese gemeinsame Pipeline. Sein
Joystick sendet die normalen manuellen Fahrbefehle, während die Live-Karte die
vom produktiven `TeachIn` aufgezeichnete Geometrie zeigt. Fehlerhafte letzte
Meter können verworfen werden; die Aufzeichnung pausiert dann während der
Rückfahrt zum letzten gültigen Punkt und setzt dort automatisch fort.
Als Grenzreferenz kann links, Mitte oder rechts gewählt werden. Links und
rechts verwenden automatisch die jeweils äußerste Kante von Traktor und
Mähwerken statt nur die GPS-/Fahrzeugmitte.

Die produktive Geofence-Prüfung verwendet eine mit dem Kurs gedrehte Linie
zwischen der äußersten rechten und linken Kante von Traktor und Mähwerken. Die
GPS-Mitte allein reicht daher weder an der Zonengrenze noch an einer No-Go-Zone
als Freigabe; auch der physisch breiteste Punkt muss innerhalb der erlaubten
Fläche bleiben. Bahnenden berücksichtigen zusätzlich die Längsausdehnung der
Maschine.

Die Traktorroute enthält dichte Planpunkte und explizite Wendemanöver. Je nach
Einstellung werden Vorwärts-, Rücksetz- oder mehrzügige Wenden erzeugt. Die
produktive Missionslaufzeit stoppt dabei das Messer, hebt Front- und/oder
Heckmähwerk über das echte `DECK_LIFT`-Protokollkommando an und führt die
geplanten Fahrtrichtungswechsel aus. Die Simulation bildet die beiden
Hubzustände und den Ladekontakt an der Home-Position mit ab.

Simulationskonsole:

```text
http://127.0.0.1:8090/simulation/
```

Dort lassen sich RTK Fix/Float/Verlust, Regen, Lift, Laden, Akku, Neigung,
Bumper-Fehler, RTK-Hardwarefehler und Messerüberstrom gezielt einspeisen. Die
Konsole verändert ausschließlich virtuelle Gerätewerte. Die Anwendung unter
`http://127.0.0.1:8090/` bleibt die normale Produktions-UI und ist das System
unter Test.

Der Slider "Simulationsgeschwindigkeit" beschleunigt virtuelle Bewegung,
Ladezeit und Akkuverbrauch von 1x bis 5x. Sensor-, Hardware- und Regelzyklen
werden dabei im selben Verhältnis skaliert, sodass die geometrische Spur auch
bei 5x repräsentativ bleibt. Der Faktor bleibt bei einem Reset erhalten, damit
mehrere Testszenarien mit derselben Geschwindigkeit wiederholt werden können.

Beim Start und bei jedem Reset wird der Simulationsroboter bevorzugt am ersten
Punkt der sicheren Außenbahn der zuletzt aktualisierten Mähzone platziert und
bereits in Fahrtrichtung ausgerichtet. Die Außenbahn hält für den breitesten
Maschinenpunkt fünf Zentimeter Spurführungsreserve; die separate
Geofence-Toleranz für RTK-Rauschen beträgt einen Zentimeter. Ist keine gültige
Außenbahn verfügbar, wird konservativ eine Position gewählt, deren vollständige
Maschinenkontur in die Zone passt. Ist auch das nicht möglich, bleibt der
gegebene Simulationsursprung erhalten und die normale Geofence-Sicherung
verhindert einen unsicheren Missionsstart.
