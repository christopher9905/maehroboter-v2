"""Robot entrypoint: wires HardwareInterface, DockingManager and the web API.

Configuration via environment variables (all optional, sensible defaults):
  MV2_MODE          — real (default) or simulation
  MV2_SERIAL_PORT   — Teensy serial port (default: /dev/ttyACM0)
  MV2_CAMERA_DEVICE — OpenCV camera index (default: 0)
  MV2_CALIBRATION   — path to the camera calibration .npz file
                       (default: mower/config/camera_calibration.npz)

Hardware, camera and calibration are all best-effort: if any is unavailable
(dev machine, camera not yet mounted, calibration not yet run) this logs a
warning and falls back to running the web API without that piece, exactly
like _dev_server.py — the mower stays remotely controllable while bring-up
is incomplete.
"""
import dataclasses
import logging
import math
import os
import time
from datetime import datetime, timedelta
from pathlib import Path

import uvicorn

from mower.executive.mission_executive import MissionExecutive, MowerState
from mower.executive.docking_runtime import LatestSoc, LatestCharging, build_docking_manager
from mower.api.app import create_app
from mower.api.control_store import ControlStore
from mower.api.runtime_state import RuntimeState

logger = logging.getLogger(__name__)

DEFAULT_SERIAL_PORT = "/dev/ttyACM0"
DEFAULT_CAMERA_DEVICE = 0
DEFAULT_CALIBRATION_PATH = "mower/config/camera_calibration.npz"
DEFAULT_DATA_PATH = "mower/data/control.json"
DEFAULT_SIM_DATA_PATH = "mower/data/simulation-control.json"
DEFAULT_TILE_PATH = "mower/data/tiles"


def _try_build_hardware(port: str):
    from mower.hal.serial_driver import SerialDriver
    from mower.hal.hardware_interface import HardwareInterface
    try:
        driver = SerialDriver(port=port)
        hw = HardwareInterface(driver=driver)
        hw.start()
        return hw
    except Exception:
        logger.warning("No Teensy hardware on %s — running without drive/blade control", port, exc_info=True)
        return None


def _try_build_docking_manager(hardware, executive, camera_device, calibration_path):
    if hardware is None:
        return None, None
    from mower.hal.camera import Camera, load_camera_calibration
    try:
        camera_matrix, dist_coeffs = load_camera_calibration(calibration_path)
        camera = Camera(device=camera_device)
        if not camera.is_opened:
            raise RuntimeError(f"Camera device {camera_device} did not open")
        docking_manager = build_docking_manager(
            hardware=hardware, executive=executive, camera=camera,
            camera_matrix=camera_matrix, dist_coeffs=dist_coeffs,
        )
        return docking_manager, camera
    except Exception:
        logger.warning("Docking camera/calibration unavailable — precision docking disabled", exc_info=True)
        return None, None


def build_app():
    mode = os.environ.get("MV2_MODE", "real").strip().lower()
    if mode not in {"real", "simulation"}:
        raise ValueError("MV2_MODE must be 'real' or 'simulation'")
    serial_port = os.environ.get("MV2_SERIAL_PORT", DEFAULT_SERIAL_PORT)
    camera_device = int(os.environ.get("MV2_CAMERA_DEVICE", DEFAULT_CAMERA_DEVICE))
    calibration_path = os.environ.get("MV2_CALIBRATION", DEFAULT_CALIBRATION_PATH)
    default_data_path = DEFAULT_SIM_DATA_PATH if mode == "simulation" else DEFAULT_DATA_PATH
    data_path = os.environ.get("MV2_DATA_PATH", default_data_path)
    tile_path = os.environ.get("MV2_TILE_PATH", DEFAULT_TILE_PATH)

    components = []
    simulation = None
    if mode == "simulation":
        from mower.simulation import SimulationEnvironment
        simulation = SimulationEnvironment(
            origin_lat=float(os.environ.get("MV2_SIM_ORIGIN_LAT", "48.5")),
            origin_lon=float(os.environ.get("MV2_SIM_ORIGIN_LON", "11.0")),
        )
        hardware = simulation.hardware
        hardware.start()
        logger.info("Simulation hardware active")
    else:
        hardware = _try_build_hardware(serial_port)
    executive = MissionExecutive(hardware_interface=hardware)
    store = ControlStore(Path(data_path))
    if mode == "simulation":
        store.update_settings({"demo_data": True})
        simulation.world.set_wheelbase(
            float(store.settings().get("vehicle_wheelbase_cm", 25)) / 100.0,
        )
        from mower.simulation.spawn import choose_safe_reset_pose
        origin = simulation.world.snapshot()
        safe_spawn = choose_safe_reset_pose(
            store.list_items("zones"),
            store.settings(),
            origin_x=origin.utm_x,
            origin_y=origin.utm_y,
        )
        if safe_spawn is not None:
            spawn_x, spawn_y, spawn_heading, spawn_zone = safe_spawn
            simulation.world.set_reset_pose(spawn_x, spawn_y, spawn_heading)
            logger.info(
                "Simulation reset pose placed safely inside zone %s at %.2f/%.2f m",
                spawn_zone, spawn_x, spawn_y,
            )
        else:
            logger.warning(
                "No mowing zone can contain the complete configured machine; "
                "simulation keeps the configured origin"
            )
    runtime = RuntimeState(hardware_connected=hardware is not None)
    from mower.safety.lift_guard import LiftGuard
    from mower.safety.rain_guard import RainGuard
    lift_guard = LiftGuard()
    rain_guard = RainGuard(
        threshold=int(store.settings().get("rain_threshold_adc", 600)),
        resume_after_s=float(store.settings().get("rain_wait_minutes", 30)) * 60.0,
    )
    from mower.nav.odometry import Odometry
    from mower.nav.localizer import Localizer
    from mower.executive.mission_runtime import MissionRuntime
    odometry = Odometry()
    localizer = Localizer()
    if simulation is not None:
        def reset_navigation_state():
            odometry.reset()
            localizer.reset()
            runtime.update("pose", {"speed_mps": 0.0, "heading_deg": 90.0})
            runtime.update("safety", {
                "front_deck_raised": False,
                "rear_deck_raised": False,
                "geofence_ok": None,
                "geofence_override_active": False,
            })

        simulation.on_reset = reset_navigation_state
    def on_home_arrival():
        # Physical hardware waits for the charge contact. In simulation the
        # virtual station closes that contact as soon as the RTK home point is
        # reached, exercising the same STATUS callback used on the robot.
        if simulation is not None:
            simulation.world.set_sensor_state(charging=True)

    mission_runtime = MissionRuntime(
        executive,
        hardware,
        store,
        control_hz=20.0 if simulation is not None else 10.0,
        home_arrival_callback=on_home_arrival,
        dock_departure_callback=(
            (lambda: simulation.world.set_sensor_state(charging=False))
            if simulation is not None else None
        ),
        time_scale_provider=simulation.world.time_scale if simulation is not None else None,
        deck_state_callback=lambda front, rear: runtime.update("safety", {
            "front_deck_raised": front,
            "rear_deck_raised": rear,
        }),
    )
    rain_dry_started = None
    last_error_flags = 0

    def on_pose(pose):
        runtime.update("pose", {
            "speed_mps": round(pose.speed_mps, 3),
            "heading_deg": round((90.0 - math.degrees(pose.heading_rad)) % 360.0, 1),
        })
        mission_runtime.on_pose(pose)

    localizer.on_pose = on_pose

    lift_guard.on_lift = lambda: executive.on_lift()

    def on_rain_detected():
        runtime.update("safety", {"raining": True, "rain_resume_at": None})
        store.add_event("warning", "rain_detected", "Regen erkannt — Mission pausiert",
                        "Trockenphase abwarten; nächste mögliche Wiederaufnahme wird angezeigt")
        executive.pause_mission()

    def on_rain_cleared():
        runtime.update("safety", {"raining": False, "rain_resume_at": None})
        store.add_event("info", "rain_cleared", "Regenpause beendet",
                        "Mission kann nach Sicherheitsprüfung fortgesetzt werden")

    rain_guard.on_rain_detected = on_rain_detected
    rain_guard.on_rain_cleared = on_rain_cleared

    latest_soc = LatestSoc()
    latest_charging = LatestCharging()
    if hardware is not None:
        def on_outputs(data):
            runtime.update("outputs", data)

        hardware.on_outputs = on_outputs
        runtime.update("outputs", hardware.output_snapshot())

        def on_soc(data):
            latest_soc.update(data["soc_percent"])
            runtime.update("battery", {
                "soc_percent": data["soc_percent"],
                "voltage_v": round(data["voltage_mv"] / 1000.0, 2),
            })
            executive.on_battery_low(data["soc_percent"])
            if executive.state == MowerState.CHARGING and data["soc_percent"] >= 95:
                executive.on_charge_complete()
                if simulation is not None:
                    simulation.world.set_sensor_state(charging=False)

        def on_sensors(data):
            nonlocal rain_dry_started
            timestamp = time.monotonic()
            wet_raw = data["rain_adc"] > int(store.settings().get("rain_threshold_adc", 600))
            rain_guard.check(data["rain_adc"], timestamp)
            if rain_guard.is_raining and not wet_raw:
                if rain_dry_started is None:
                    rain_dry_started = datetime.now().astimezone()
                resume_at = rain_dry_started + timedelta(
                    minutes=float(store.settings().get("rain_wait_minutes", 30)))
            else:
                rain_dry_started = None
                resume_at = None
            runtime.update("safety", {
                "rain_adc": data["rain_adc"],
                "raining": rain_guard.is_raining,
                "rain_resume_at": resume_at.isoformat() if resume_at else None,
                "lifted": data["lift"],
            })
            odometry_update = odometry.update(data["encoder_ticks"], timestamp)
            if odometry_update is not None:
                # The encoder is unsigned; sign the measured speed with the
                # commanded drive direction so the localizer's UKF predicts
                # backwards motion during reverse maneuvers instead of being
                # dragged forward against the GPS fixes.
                direction = getattr(hardware, "last_drive_direction", 1)
                if direction < 0:
                    odometry_update = dataclasses.replace(
                        odometry_update,
                        speed_mps=-odometry_update.speed_mps,
                        delta_distance_m=-odometry_update.delta_distance_m,
                    )
                runtime.update("pose", {"speed_mps": round(odometry_update.speed_mps, 3)})
                localizer.update_odometry(odometry_update)
            lift_guard.check(data)

        def on_status(data):
            nonlocal last_error_flags
            latest_charging.update(data["charging"])
            runtime.update("safety", {
                "watchdog_ok": data["watchdog_ok"],
                "blade_running": data["blade_running"],
                "error_flags": data["error_flags"],
                "bumper_left": bool(data["error_flags"] & 0x01),
                "bumper_right": bool(data["error_flags"] & 0x02),
            })
            runtime.update("battery", {"charging": data["charging"]})
            if data["charging"] and executive.state == MowerState.DOCKING:
                executive.on_charge_started()
            new_flags = data["error_flags"] & ~last_error_flags
            error_messages = {
                0x01: ("Bumper links blockiert", "Bumper und Fahrweg links prüfen"),
                0x02: ("Bumper rechts blockiert", "Bumper und Fahrweg rechts prüfen"),
                0x04: ("RTK-Verbindung verloren", "GPS-Antenne und NTRIP-Verbindung prüfen"),
                0x08: ("Messerstrom zu hoch", "Mähwerk spannungsfrei auf Blockade prüfen"),
            }
            for bit, (message, action) in error_messages.items():
                if new_flags & bit:
                    store.add_event("critical", "hardware_fault", message, action, flag=bit)
                    executive.emergency_stop(message)
            last_error_flags = data["error_flags"]

        hardware.on_soc = on_soc
        hardware.on_sensors = on_sensors
        # 'charging' reflects the STATUS frame's charge-detect byte — real
        # signal, but the sensing pin is still a hardware bring-up item (see
        # CHARGE_DETECT_PIN in firmware/mower_firmware/mower_firmware.ino).
        hardware.on_status = on_status

    if simulation is None:
        docking_manager, _camera = _try_build_docking_manager(
            hardware, executive, camera_device, calibration_path,
        )
    else:
        docking_manager, _camera = None, None
    runtime.update("diagnostics", {
        "camera": "available" if _camera is not None else "unavailable",
    })

    # Navigation readers are opt-in until their real device paths are known.
    # Once enabled, their data feeds the live map and safety interlocks.
    gps_port = os.environ.get("MV2_GPS_PORT")
    if simulation is not None or gps_port:
        try:
            if simulation is not None:
                gps = simulation.gps
            else:
                from mower.nav.gps_reader import GpsReader
                gps = GpsReader(gps_port)
            previous_fix_quality = 0

            def on_fix(fix):
                nonlocal previous_fix_quality
                if simulation is not None:
                    truth = simulation.world.snapshot()
                    runtime.update("simulation_truth", {
                        "lat": truth.lat,
                        "lon": truth.lon,
                        "utm_x": truth.utm_x,
                        "utm_y": truth.utm_y,
                        "speed_mps": round(truth.speed_mps, 4),
                        "heading_deg": round((90.0 - math.degrees(truth.heading_rad)) % 360.0, 2),
                    })
                mission_runtime.on_gps_fix(fix)
                localizer.update_gps(fix)
                label = {4: "RTK Fix", 5: "RTK Float", 1: "GPS"}.get(fix.fix_quality, "Kein Fix")
                runtime.update("gps", {
                    "fix_quality": fix.fix_quality, "label": label, "hdop": fix.hdop,
                })
                runtime.update("pose", {"lat": fix.lat, "lon": fix.lon})
                runtime.update("diagnostics", {"rtk": label})
                if previous_fix_quality == 4 and fix.fix_quality != 4 and executive.state == MowerState.MOWING:
                    executive.pause_mission()
                    store.add_event("warning", "rtk_lost", "RTK Fix verloren — Mission pausiert",
                                    "Antenne und NTRIP prüfen; erst bei RTK Fix fortsetzen")
                previous_fix_quality = fix.fix_quality

                if executive.state in (MowerState.MOWING, MowerState.PAUSED) and store.settings().get("geofence_enabled", True):
                    zone = store.get_item("zones", executive.active_zone_id) if executive.active_zone_id else None
                    if zone and zone.get("geometry"):
                        try:
                            geofence = mission_runtime.footprint_geofence_status(
                                fix,
                                zone,
                                [
                                    item for item in store.list_items("zones")
                                    if item.get("type") == "no_go" and item.get("geometry")
                                ],
                            )
                            allowed = bool(geofence["allowed"])
                            if allowed:
                                if executive.on_geofence_recovered():
                                    store.add_event(
                                        "info",
                                        "geofence_rearmed",
                                        "Fahrzeug wieder vollständig in der Zone — Geofence-Schutz erneut aktiv",
                                        zone_id=executive.active_zone_id,
                                    )
                            else:
                                executive.on_geofence_violation(fix)
                            runtime.update("safety", {
                                "geofence_ok": allowed,
                                "geofence_override_active": executive.geofence_override_active,
                            })
                        except Exception:
                            logger.exception("Geofence check failed")

            gps.on_fix = on_fix
            gps.start()
            components.append(gps)
        except Exception:
            logger.warning("GPS unavailable — live map disabled", exc_info=True)

    if simulation is not None or os.environ.get("MV2_ENABLE_IMU", "0") == "1":
        try:
            from mower.safety.tilt_guard import TiltGuard
            if simulation is not None:
                imu = simulation.imu
            else:
                from mower.nav.imu_reader import ImuReader
                imu = ImuReader()
            tilt_guard = TiltGuard(float(store.settings().get("tilt_limit_deg", 30)))
            tilt_guard.on_tilt = executive.on_tilt

            def on_imu(reading):
                localizer.update_imu(reading)
                runtime.update("pose", {"heading_deg": reading.heading_deg})
                runtime.update("safety", {"tilt_deg": max(abs(reading.pitch_deg), abs(reading.roll_deg))})
                runtime.update("diagnostics", {"imu": "available"})
                tilt_guard.check(reading)

            imu.on_imu = on_imu
            imu.start()
            components.append(imu)
        except Exception:
            logger.warning("IMU unavailable", exc_info=True)

    mission_runtime.start()
    components.insert(0, mission_runtime)

    web_app = create_app(
        executive,
        docking_manager=docking_manager,
        soc_source=latest_soc.get,
        charging_source=latest_charging.get,
        store=store,
        runtime=runtime,
        hardware=hardware,
        pin=os.environ.get("MV2_UI_PIN", ""),
        enforce_mission_safety=True,
        require_confirmations=True,
        tile_dir=tile_path,
        managed_components=components,
        firmware_stage_dir="mower/data/firmware-staged",
        mission_runtime=mission_runtime,
    )
    web_app.state.mode = mode
    if simulation is not None:
        from mower.simulation import create_simulation_router, mount_simulation_console
        web_app.state.simulation = simulation
        web_app.include_router(create_simulation_router(simulation, executive))
        mount_simulation_console(web_app)
    return web_app


app = build_app()

if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8080)
