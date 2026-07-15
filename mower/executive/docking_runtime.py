"""Runtime wiring: builds a DockingManager from real hardware + camera.

Kept separate from mower.api.app so the FastAPI app factory stays
hardware-agnostic and directly testable without a camera or serial port.
"""
import threading

from mower.cv.aruco_detector import ArucoDetector, TARGET_MARKER_ID
from mower.control.docking_controller import DockingController
from mower.executive.docking_manager import DockingManager
from mower.executive.mission_executive import MissionExecutive


class LatestSoc:
    """Thread-safe holder for the most recently reported SOC percent (0 if unknown).

    Wire HardwareInterface.on_soc to call update(); pass get() as the
    DockingManager soc_source.
    """

    def __init__(self):
        self._value: int = 0
        self._lock = threading.Lock()

    def update(self, soc_percent: int) -> None:
        with self._lock:
            self._value = soc_percent

    def get(self) -> int:
        with self._lock:
            return self._value


class LatestCharging:
    """Thread-safe holder for the most recently reported charge-contact state.

    Wire HardwareInterface.on_status to call update(data['charging']); pass
    get() as the DockingManager charging_source.
    """

    def __init__(self):
        self._value: bool = False
        self._lock = threading.Lock()

    def update(self, charging: bool) -> None:
        with self._lock:
            self._value = charging

    def get(self) -> bool:
        with self._lock:
            return self._value


def build_docking_manager(
    hardware,
    executive: MissionExecutive,
    camera,
    camera_matrix,
    dist_coeffs,
    target_marker_id: int = TARGET_MARKER_ID,
) -> DockingManager:
    """Construct a DockingManager wired to a real camera and hardware interface.

    camera_matrix / dist_coeffs come from the hardware bring-up calibration
    step (see mower.hal.camera.load_camera_calibration) — never hard-coded.
    """
    detector = ArucoDetector(camera_matrix, dist_coeffs, target_id=target_marker_id)
    controller = DockingController()
    return DockingManager(
        detector=detector,
        controller=controller,
        executive=executive,
        hardware=hardware,
        frame_source=camera.read,
    )
