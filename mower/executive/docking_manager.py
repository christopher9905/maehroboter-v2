"""Docking manager: RTK→visual handoff and DOCKING/CHARGING orchestration.

Bridges ArucoDetector + DockingController + MissionExecutive + HardwareInterface.
Mirrors ObstacleDetector: a synchronous _tick() unit (tested directly) plus a
background thread loop for the real system.
"""
import logging
import threading
from typing import Callable, Optional

from mower.executive.mission_executive import MissionExecutive, MowerState

logger = logging.getLogger(__name__)

FULL_SOC: int = 95          # SOC at/above which charging is considered complete
HANDOFF_DISTANCE_M: float = 1.2
LOOP_HZ: float = 10.0


class DockingManager:
    def __init__(
        self,
        detector,
        controller,
        executive: MissionExecutive,
        hardware,
        frame_source: Callable[[], object],
        handoff_distance_m: float = HANDOFF_DISTANCE_M,
        full_soc: int = FULL_SOC,
    ):
        self._detector = detector
        self._controller = controller
        self._executive = executive
        self._hw = hardware
        self._frame_source = frame_source
        self._handoff = handoff_distance_m
        self._full_soc = full_soc
        self._stop_event = threading.Event()
        self._thread: Optional[threading.Thread] = None

    def _tick(self, frame, soc: int, charging: bool) -> None:
        state = self._executive.state

        if state == MowerState.RETURNING:
            marker = self._detector.detect(frame)
            if marker is not None and marker.distance_m <= self._handoff:
                logger.info("Marker acquired at %.2f m — handoff to visual docking",
                            marker.distance_m)
                self._executive.on_dock_success()   # → DOCKING
            return

        if state == MowerState.DOCKING:
            marker = self._detector.detect(frame)
            cmd = self._controller.compute(marker, charge_detected=charging)
            if cmd.docked:
                if self._hw:
                    self._hw.drive(0.0, 0.0)
                    self._hw.set_blade(False)
                self._executive.on_charge_started()  # → CHARGING
            elif self._hw:
                self._hw.drive(cmd.speed, cmd.steering_deg)
            return

        if state == MowerState.CHARGING:
            if self._hw:
                self._hw.drive(0.0, 0.0)
            if soc >= self._full_soc:
                logger.info("Charge complete (SOC %d%%)", soc)
                self._executive.on_charge_complete()  # → IDLE
            return

        # IDLE / MOWING / TEACH_IN / OBSTACLE_AVOIDANCE / ERROR: nothing to do.

    # --- Background loop (real system) ---

    def start(self, soc_source: Callable[[], int], charging_source: Callable[[], bool]):
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop_event.clear()
        self._thread = threading.Thread(
            target=self._run_loop, args=(soc_source, charging_source), daemon=True)
        self._thread.start()

    def stop(self):
        self._stop_event.set()
        if self._thread:
            self._thread.join(timeout=2.0)

    def _run_loop(self, soc_source, charging_source):
        interval = 1.0 / LOOP_HZ
        while not self._stop_event.is_set():
            try:
                frame = self._frame_source()
                if frame is not None:
                    self._tick(frame, soc_source(), charging_source())
            except Exception:
                logger.exception("Docking manager error")
            self._stop_event.wait(timeout=interval)
