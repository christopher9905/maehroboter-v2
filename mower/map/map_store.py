import json
import logging
from enum import Enum

logger = logging.getLogger(__name__)


class MapFeatureType(str, Enum):
    BOUNDARY = "boundary"
    ZONE = "zone"
    NO_GO = "no-go"
    TRANSIT_PATH = "transit-path"
    CHARGING_STATION = "charging-station"


class MapStore:
    """In-memory GeoJSON FeatureCollection with file persistence."""

    def __init__(self):
        self._features: list[dict] = []

    @property
    def features(self) -> list[dict]:
        return list(self._features)

    def load(self, geojson: dict):
        """Replace all current features with those from a GeoJSON FeatureCollection dict."""
        self._features = list(geojson.get("features", []))

    def load_from_file(self, path: str):
        with open(path, encoding="utf-8") as f:
            self.load(json.load(f))

    def save(self, path: str):
        with open(path, "w", encoding="utf-8") as f:
            json.dump(self.to_geojson(), f, indent=2)

    def to_geojson(self) -> dict:
        return {"type": "FeatureCollection", "features": list(self._features)}

    def get_by_type(self, feature_type: MapFeatureType) -> list[dict]:
        return [
            f for f in self._features
            if f.get("properties", {}).get("type") == feature_type.value
        ]

    def add_feature(self, feature: dict):
        self._features.append(feature)

    def clear(self):
        self._features.clear()
