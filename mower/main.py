"""Robot entrypoint: wires HardwareInterface, DockingManager and the web API.

Configuration via environment variables (all optional, sensible defaults):
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
import logging
import os

import uvicorn

from mower.executive.mission_executive import MissionExecutive
from mower.executive.docking_runtime import LatestSoc, build_docking_manager
from mower.api.app import create_app

logger = logging.getLogger(__name__)

DEFAULT_SERIAL_PORT = "/dev/ttyACM0"
DEFAULT_CAMERA_DEVICE = 0
DEFAULT_CALIBRATION_PATH = "mower/config/camera_calibration.npz"


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
    serial_port = os.environ.get("MV2_SERIAL_PORT", DEFAULT_SERIAL_PORT)
    camera_device = int(os.environ.get("MV2_CAMERA_DEVICE", DEFAULT_CAMERA_DEVICE))
    calibration_path = os.environ.get("MV2_CALIBRATION", DEFAULT_CALIBRATION_PATH)

    hardware = _try_build_hardware(serial_port)
    executive = MissionExecutive(hardware_interface=hardware)

    latest_soc = LatestSoc()
    if hardware is not None:
        hardware.on_soc = lambda data: latest_soc.update(data["soc_percent"])

    docking_manager, _camera = _try_build_docking_manager(
        hardware, executive, camera_device, calibration_path,
    )

    # No real charge-contact sensor exists in the serial protocol yet (see
    # Phase 6 plan bring-up checklist) — DockingManager falls back to its
    # documented distance-only DOCKED trigger until that lands.
    return create_app(
        executive,
        docking_manager=docking_manager,
        soc_source=latest_soc.get,
        charging_source=lambda: False,
    )


app = build_app()

if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8080)
