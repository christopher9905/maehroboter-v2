import json
import os
import tempfile
import pytest
from mower.map.map_store import MapStore, MapFeatureType

SAMPLE_GEOJSON = {
    "type": "FeatureCollection",
    "features": [
        {
            "type": "Feature",
            "properties": {"type": "boundary"},
            "geometry": {
                "type": "Polygon",
                "coordinates": [[[0, 0], [10, 0], [10, 10], [0, 10], [0, 0]]],
            },
        },
        {
            "type": "Feature",
            "properties": {"type": "zone", "id": "zone_1", "order": 1},
            "geometry": {
                "type": "Polygon",
                "coordinates": [[[1, 1], [9, 1], [9, 9], [1, 9], [1, 1]]],
            },
        },
        {
            "type": "Feature",
            "properties": {"type": "charging-station"},
            "geometry": {"type": "Point", "coordinates": [5.0, 5.0]},
        },
    ],
}


class TestMapStore:
    def test_load_from_dict(self):
        store = MapStore()
        store.load(SAMPLE_GEOJSON)
        assert len(store.features) == 3

    def test_get_boundary(self):
        store = MapStore()
        store.load(SAMPLE_GEOJSON)
        boundary = store.get_by_type(MapFeatureType.BOUNDARY)
        assert len(boundary) == 1
        assert boundary[0]["properties"]["type"] == "boundary"

    def test_get_zones(self):
        store = MapStore()
        store.load(SAMPLE_GEOJSON)
        zones = store.get_by_type(MapFeatureType.ZONE)
        assert len(zones) == 1
        assert zones[0]["properties"]["id"] == "zone_1"

    def test_add_feature(self):
        store = MapStore()
        store.load(SAMPLE_GEOJSON)
        new_feature = {
            "type": "Feature",
            "properties": {"type": "no-go"},
            "geometry": {
                "type": "Polygon",
                "coordinates": [[[2, 2], [4, 2], [4, 4], [2, 4], [2, 2]]],
            },
        }
        store.add_feature(new_feature)
        assert len(store.get_by_type(MapFeatureType.NO_GO)) == 1

    def test_save_and_reload(self):
        store = MapStore()
        store.load(SAMPLE_GEOJSON)
        with tempfile.NamedTemporaryFile(suffix=".geojson", delete=False,
                                         mode="w") as f:
            path = f.name
        try:
            store.save(path)
            store2 = MapStore()
            store2.load_from_file(path)
            assert len(store2.features) == 3
        finally:
            os.unlink(path)

    def test_to_geojson_roundtrip(self):
        store = MapStore()
        store.load(SAMPLE_GEOJSON)
        exported = store.to_geojson()
        assert exported["type"] == "FeatureCollection"
        assert len(exported["features"]) == 3

    def test_clear_removes_all(self):
        store = MapStore()
        store.load(SAMPLE_GEOJSON)
        store.clear()
        assert len(store.features) == 0
