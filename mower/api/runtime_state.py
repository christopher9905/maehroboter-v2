"""Latest real robot telemetry shared by hardware callbacks and the API."""
from __future__ import annotations

import copy
import threading
import time
from typing import Any


class RuntimeState:
    def __init__(self, hardware_connected: bool = False):
        self._lock = threading.Lock()
        self._updated_monotonic = 0.0
        self._data: dict[str, Any] = {
            "hardware_connected": hardware_connected,
            "pose": {"lat": None, "lon": None, "speed_mps": None, "heading_deg": None},
            "simulation_truth": {
                "lat": None, "lon": None, "utm_x": None, "utm_y": None,
                "speed_mps": None, "heading_deg": None,
            },
            "gps": {"fix_quality": 0, "label": "Kein Fix", "satellites": None, "hdop": None},
            "safety": {
                "rain_adc": None,
                "raining": None,
                "rain_resume_at": None,
                "tilt_deg": None,
                "bumper_left": None,
                "bumper_right": None,
                "lifted": None,
                "geofence_ok": None,
                "geofence_override_active": False,
                "watchdog_ok": None,
                "blade_running": None,
                "front_deck_raised": False,
                "rear_deck_raised": False,
                "error_flags": 0,
            },
            "battery": {
                "soc_percent": None,
                "voltage_v": None,
                "current_a": None,
                "temperature_c": None,
                "charging": None,
            },
            "outputs": {
                "speed_command": 0.0,
                "target_speed_kmh": 0.0,
                "speed_mode": "stopped",
                "steering_deg": 0.0,
                "blade_enabled": False,
                "front_deck_raised": False,
                "rear_deck_raised": False,
                "estop_active": False,
            },
            "diagnostics": {
                "imu": "unavailable",
                "bumper": "unavailable",
                "motor_current": "unavailable",
                "blade_current": "unavailable",
                "camera": "unknown",
                "rtk": "unavailable",
            },
        }

    def update(self, section: str, values: dict[str, Any]) -> None:
        with self._lock:
            if section in self._data and isinstance(self._data[section], dict):
                self._data[section].update(values)
            else:
                self._data[section] = copy.deepcopy(values)
            self._updated_monotonic = time.monotonic()

    def set_hardware_connected(self, connected: bool) -> None:
        with self._lock:
            self._data["hardware_connected"] = bool(connected)
            self._updated_monotonic = time.monotonic()

    def snapshot(self) -> dict[str, Any]:
        with self._lock:
            data = copy.deepcopy(self._data)
            age = time.monotonic() - self._updated_monotonic if self._updated_monotonic else None
        data["age_seconds"] = round(age, 2) if age is not None else None
        data["stale"] = age is None or age > 3.0
        return data
