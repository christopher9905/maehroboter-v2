"""Thread-safe, atomic persistence for UI configuration and operational data."""
from __future__ import annotations

import copy
import gzip
import json
import os
import re
import threading
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Optional
from uuid import uuid4


DEFAULT_SETTINGS: dict[str, Any] = {
    "robot_name": "MV2-Alpha",
    "mowing_width_cm": 60,
    "front_mower_width_percent": 60,
    "mower_deck_depth_cm": 14,
    "front_mower_offset_cm": 35,
    "rear_mower_offset_cm": 22,
    "rear_mower_gap_cm": 6,
    "show_mower_footprints": True,
    "coverage_grid_resolution_cm": 2,
    "minimum_plan_coverage_percent": 98.5,
    "vehicle_wheelbase_cm": 25,
    "vehicle_width_cm": 34,
    "vehicle_front_overhang_cm": 12,
    "vehicle_rear_overhang_cm": 12,
    "lane_width_cm": 38,
    "route_resolution_cm": 0.5,
    "speed_kmh": 0.9,
    "planner_engine": "mv2",
    "coverage_strategy": "headland_first",
    "headland_passes": 1,
    "turn_strategy": "auto",
    "turn_radius_cm": 25,
    "mowing_turn_radius_cm": 50,
    "headland_margin_cm": 5,
    "turn_maneuver_passes": 6,
    "turn_speed_kmh": 0.4,
    "lift_front_on_turn": True,
    "lift_rear_on_turn": True,
    "deck_lift_settle_ms": 700,
    "fields2cover_decomposition": "boustrophedon",
    "fields2cover_headland_mode": "auto",
    "fields2cover_headland_width_cm": 70.0,
    "fields2cover_headland_width_factor": 3.0,
    "fields2cover_swath_objective": "n_swath_modified",
    "fields2cover_route_order": "optimized",
    "fields2cover_path_type": "dubins",
    "fields2cover_turning_backend": "auto",
    "fields2cover_optimizer_time_s": 2,
    "fields2cover_search_optimum": False,
    "fields2cover_spiral_size": 6,
    "fields2cover_angle_mode": "best",
    "fields2cover_angle_deg": 0.0,
    "fields2cover_split_angle_deg": 90.0,
    "fields2cover_max_diff_curvature": 0.1,
    "fields2cover_minimum_mainland_coverage_percent": 90.0,
    "manual_speed_limit": 0.25,
    "manual_steering_limit_deg": 28,
    "tilt_limit_deg": 30,
    "rain_enabled": True,
    "rain_threshold_adc": 600,
    "rain_wait_minutes": 30,
    "geofence_enabled": True,
    "require_rtk_fix": True,
    "low_battery_soc": 20,
    "quiet_hours_start": "20:00",
    "quiet_hours_end": "08:00",
    "season_mode": "auto",
    "weekend_mode": True,
    "demo_data": False,
}

_PLAN_CACHE_KEY_RE = re.compile(r"^[0-9a-f]{64}$")
_PLAN_CACHE_FILE_LIMIT = 12


def _now() -> str:
    return datetime.now(timezone.utc).isoformat()


class ControlStore:
    """Small JSON document store.

    ``path=None`` creates an in-memory store for tests and development. A real
    path persists every mutation through an atomic ``os.replace``.
    """

    def __init__(self, path: Optional[Path | str] = None):
        self._path = Path(path) if path else None
        self._plan_cache_dir = (
            self._path.parent / "plan-cache" if self._path is not None else None
        )
        self._memory_plan_cache: dict[str, dict[str, Any]] = {}
        self._lock = threading.RLock()
        self._data: dict[str, Any] = {
            "version": 1,
            "zones": [],
            "connections": [],
            "schedules": [],
            "settings": copy.deepcopy(DEFAULT_SETTINGS),
            "home": None,
            "events": [],
            "missions": [],
        }
        self._load()

    def _load(self) -> None:
        if self._path is None or not self._path.exists():
            return
        with self._lock:
            loaded = json.loads(self._path.read_text(encoding="utf-8"))
            for key in ("zones", "connections", "schedules", "events", "missions"):
                if isinstance(loaded.get(key), list):
                    self._data[key] = loaded[key]
            if isinstance(loaded.get("settings"), dict):
                self._data["settings"].update(loaded["settings"])
                # Since July 2026 this value is the complete physical minimum
                # distance from the widest machine contour to the geofence.
                # Older stores used zero here and added hidden planner margins.
                self._data["settings"]["headland_margin_cm"] = max(
                    5.0,
                    min(150.0, float(
                        self._data["settings"].get("headland_margin_cm", 5.0)
                    )),
                )
            if loaded.get("home") is None or isinstance(loaded.get("home"), dict):
                self._data["home"] = loaded.get("home")

    def _save(self) -> None:
        if self._path is None:
            return
        self._path.parent.mkdir(parents=True, exist_ok=True)
        tmp = self._path.with_suffix(self._path.suffix + ".tmp")
        tmp.write_text(json.dumps(self._data, ensure_ascii=False, indent=2), encoding="utf-8")
        os.replace(tmp, self._path)

    def snapshot(self) -> dict[str, Any]:
        with self._lock:
            return copy.deepcopy(self._data)

    def settings(self) -> dict[str, Any]:
        with self._lock:
            return copy.deepcopy(self._data["settings"])

    def update_settings(self, values: dict[str, Any]) -> dict[str, Any]:
        allowed = set(DEFAULT_SETTINGS)
        clean = {key: value for key, value in values.items() if key in allowed}
        if "turn_radius_cm" in clean:
            clean["turn_radius_cm"] = max(
                10.0, min(100.0, float(clean["turn_radius_cm"])),
            )
        numeric_bounds = {
            "headland_margin_cm": (5.0, 150.0),
            "mowing_turn_radius_cm": (15.0, 150.0),
            "front_mower_width_percent": (35.0, 100.0),
            "mower_deck_depth_cm": (5.0, 40.0),
            "front_mower_offset_cm": (10.0, 80.0),
            "rear_mower_offset_cm": (5.0, 60.0),
            "rear_mower_gap_cm": (0.0, 30.0),
            "coverage_grid_resolution_cm": (1.0, 10.0),
            "vehicle_wheelbase_cm": (10.0, 100.0),
            "vehicle_width_cm": (15.0, 100.0),
            "vehicle_front_overhang_cm": (5.0, 100.0),
            "vehicle_rear_overhang_cm": (5.0, 100.0),
            "minimum_plan_coverage_percent": (95.0, 100.0),
            "manual_speed_limit": (0.05, 0.50),
            "manual_steering_limit_deg": (10.0, 45.0),
            "fields2cover_headland_width_factor": (1.0, 6.0),
            "fields2cover_headland_width_cm": (40.0, 150.0),
            "fields2cover_optimizer_time_s": (1.0, 30.0),
            "fields2cover_spiral_size": (2.0, 20.0),
            "fields2cover_angle_deg": (-180.0, 180.0),
            "fields2cover_split_angle_deg": (0.0, 180.0),
            "fields2cover_max_diff_curvature": (0.01, 1.0),
            "fields2cover_minimum_mainland_coverage_percent": (80.0, 100.0),
        }
        for key, (minimum, maximum) in numeric_bounds.items():
            if key in clean:
                clean[key] = max(minimum, min(maximum, float(clean[key])))
        enum_values = {
            "planner_engine": {"mv2", "fields2cover"},
            "fields2cover_decomposition": {"none", "trapezoidal", "boustrophedon"},
            "fields2cover_swath_objective": {"n_swath_modified", "n_swath", "swath_length"},
            "fields2cover_route_order": {"optimized", "boustrophedon", "snake", "spiral"},
            "fields2cover_path_type": {"dubins", "dubins_cc", "reeds_shepp", "reeds_shepp_cc"},
            "fields2cover_turning_backend": {"auto", "native"},
            "fields2cover_angle_mode": {"best", "fixed"},
            "fields2cover_headland_mode": {"auto", "manual"},
        }
        for key, allowed_values in enum_values.items():
            if key in clean and str(clean[key]) not in allowed_values:
                clean.pop(key)
        with self._lock:
            mowing_width = max(
                30.0,
                min(120.0, float(clean.get(
                    "mowing_width_cm", self._data["settings"].get("mowing_width_cm", 60),
                ))),
            )
            if "mowing_width_cm" in clean:
                clean["mowing_width_cm"] = mowing_width
            if "lane_width_cm" in clean:
                clean["lane_width_cm"] = max(
                    10.0, min(mowing_width, float(clean["lane_width_cm"])),
                )
            elif "mowing_width_cm" in clean:
                current_lane = float(self._data["settings"].get("lane_width_cm", 38))
                if current_lane > mowing_width:
                    clean["lane_width_cm"] = mowing_width
            if "turn_radius_cm" in clean or "mowing_turn_radius_cm" in clean:
                effective_turn_radius = float(clean.get(
                    "turn_radius_cm",
                    self._data["settings"].get("turn_radius_cm", 25),
                ))
                requested_mowing_radius = float(clean.get(
                    "mowing_turn_radius_cm",
                    self._data["settings"].get("mowing_turn_radius_cm", 50),
                ))
                clean["mowing_turn_radius_cm"] = min(
                    150.0,
                    max(effective_turn_radius + 5.0, requested_mowing_radius),
                )
            self._data["settings"].update(clean)
            self._save()
            return copy.deepcopy(self._data["settings"])

    def list_items(self, collection: str) -> list[dict[str, Any]]:
        with self._lock:
            return copy.deepcopy(self._data[collection])

    def get_item(self, collection: str, item_id: str) -> Optional[dict[str, Any]]:
        with self._lock:
            item = next((x for x in self._data[collection] if x.get("id") == item_id), None)
            return copy.deepcopy(item) if item else None

    def add_item(self, collection: str, values: dict[str, Any]) -> dict[str, Any]:
        item = copy.deepcopy(values)
        item.setdefault("id", uuid4().hex)
        item.setdefault("created_at", _now())
        item["updated_at"] = _now()
        with self._lock:
            self._data[collection].append(item)
            self._save()
        return copy.deepcopy(item)

    def update_item(self, collection: str, item_id: str, values: dict[str, Any]) -> Optional[dict[str, Any]]:
        with self._lock:
            item = next((x for x in self._data[collection] if x.get("id") == item_id), None)
            if item is None:
                return None
            immutable = {"id", "created_at"}
            item.update({k: copy.deepcopy(v) for k, v in values.items() if k not in immutable})
            item["updated_at"] = _now()
            self._save()
            return copy.deepcopy(item)

    def delete_item(self, collection: str, item_id: str) -> bool:
        with self._lock:
            before = len(self._data[collection])
            self._data[collection] = [x for x in self._data[collection] if x.get("id") != item_id]
            changed = len(self._data[collection]) != before
            if changed:
                self._save()
            return changed

    def get_home(self) -> Optional[dict[str, Any]]:
        with self._lock:
            return copy.deepcopy(self._data["home"])

    def set_home(self, home: dict[str, Any]) -> dict[str, Any]:
        value = copy.deepcopy(home)
        value["updated_at"] = _now()
        with self._lock:
            self._data["home"] = value
            self._save()
        return copy.deepcopy(value)

    def clear_home(self) -> bool:
        with self._lock:
            changed = self._data["home"] is not None
            self._data["home"] = None
            if changed:
                self._save()
            return changed

    def add_event(self, level: str, code: str, message: str, action: str = "", **details: Any) -> dict[str, Any]:
        event = {
            "id": uuid4().hex,
            "timestamp": _now(),
            "level": level,
            "code": code,
            "message": message,
            "action": action,
            "details": details,
        }
        with self._lock:
            self._data["events"].insert(0, event)
            del self._data["events"][500:]
            self._save()
        return copy.deepcopy(event)

    def clear_events(self) -> None:
        with self._lock:
            self._data["events"] = []
            self._save()

    def get_plan_cache(self, cache_key: str) -> Optional[dict[str, Any]]:
        """Load one validated-route snapshot from memory or compressed disk."""
        key = str(cache_key).lower()
        if _PLAN_CACHE_KEY_RE.fullmatch(key) is None:
            return None
        with self._lock:
            if self._plan_cache_dir is None:
                value = self._memory_plan_cache.get(key)
                return copy.deepcopy(value) if value is not None else None
            path = self._plan_cache_dir / f"{key}.json.gz"
            if not path.exists():
                return None
            try:
                with gzip.open(path, "rt", encoding="utf-8") as stream:
                    value = json.load(stream)
                if not isinstance(value, dict):
                    raise ValueError("plan cache root must be an object")
                os.utime(path, None)
                return copy.deepcopy(value)
            except (OSError, ValueError, json.JSONDecodeError):
                path.unlink(missing_ok=True)
                return None

    def put_plan_cache(self, cache_key: str, value: dict[str, Any]) -> None:
        """Persist a route snapshot atomically without bloating control.json."""
        key = str(cache_key).lower()
        if _PLAN_CACHE_KEY_RE.fullmatch(key) is None:
            raise ValueError("invalid plan cache key")
        snapshot = copy.deepcopy(value)
        with self._lock:
            if self._plan_cache_dir is None:
                self._memory_plan_cache[key] = snapshot
                while len(self._memory_plan_cache) > _PLAN_CACHE_FILE_LIMIT:
                    self._memory_plan_cache.pop(next(iter(self._memory_plan_cache)))
                return
            self._plan_cache_dir.mkdir(parents=True, exist_ok=True)
            path = self._plan_cache_dir / f"{key}.json.gz"
            temporary = self._plan_cache_dir / f".{key}.json.gz.tmp"
            with gzip.open(temporary, "wt", encoding="utf-8", compresslevel=6) as stream:
                json.dump(snapshot, stream, ensure_ascii=False, separators=(",", ":"))
            os.replace(temporary, path)
            cached = sorted(
                self._plan_cache_dir.glob("*.json.gz"),
                key=lambda candidate: candidate.stat().st_mtime,
                reverse=True,
            )
            for obsolete in cached[_PLAN_CACHE_FILE_LIMIT:]:
                obsolete.unlink(missing_ok=True)

    def delete_plan_cache(self, cache_key: str) -> None:
        key = str(cache_key).lower()
        if _PLAN_CACHE_KEY_RE.fullmatch(key) is None:
            return
        with self._lock:
            self._memory_plan_cache.pop(key, None)
            if self._plan_cache_dir is not None:
                (self._plan_cache_dir / f"{key}.json.gz").unlink(missing_ok=True)
