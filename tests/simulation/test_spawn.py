import utm
from shapely.geometry import Polygon

from mower.api.control_store import DEFAULT_SETTINGS
from mower.map.coverage_grid import machine_geofence_width, mower_decks_from_settings
from mower.simulation.spawn import choose_safe_reset_pose


def test_safe_reset_pose_moves_complete_machine_away_from_recorded_boundary():
    origin_x, origin_y, zone_number, zone_letter = utm.from_latlon(48.5, 11.0)
    metric_ring = [
        (origin_x, origin_y),
        (origin_x + 6.0, origin_y),
        (origin_x + 6.0, origin_y + 4.0),
        (origin_x, origin_y + 4.0),
        (origin_x, origin_y),
    ]
    wgs_ring = [
        tuple(reversed(utm.to_latlon(x, y, zone_number, zone_letter)))
        for x, y in metric_ring
    ]
    zone = {
        "id": "recorded-zone",
        "type": "mowing",
        "updated_at": "2026-07-15T20:00:00+00:00",
        "geometry": {"type": "Polygon", "coordinates": [wgs_ring]},
    }

    result = choose_safe_reset_pose(
        [zone], DEFAULT_SETTINGS, origin_x=origin_x, origin_y=origin_y,
    )

    assert result is not None
    x_m, y_m, heading, zone_id = result
    width = machine_geofence_width(
        origin_x + x_m,
        origin_y + y_m,
        heading,
        DEFAULT_SETTINGS,
        mower_decks_from_settings(DEFAULT_SETTINGS),
    )
    assert zone_id == "recorded-zone"
    assert Polygon(metric_ring).buffer(0.01).covers(width)
    # A horizontal optimized swath may legitimately have zero heading. The
    # aligned spawn must lie on the perimeter route rather than at the generic
    # polygon-centre fallback.
    assert abs(x_m - 3.0) > 0.2 or abs(y_m - 2.0) > 0.2


def test_safe_reset_pose_uses_dominant_polygon_after_teach_in_repair():
    origin_x, origin_y, zone_number, zone_letter = utm.from_latlon(48.5, 11.0)
    # A retraced closing section can make buffer(0) return a MultiPolygon,
    # matching the topology seen in the recorded Test1 Teach-In ring.
    local_ring = [
        (0, 0), (4, 4), (0, 8), (4, 12),
        (8, 8), (4, 4), (8, 0), (0, 0),
    ]
    wgs_ring = [
        tuple(reversed(utm.to_latlon(
            origin_x + x, origin_y + y, zone_number, zone_letter,
        )))
        for x, y in local_ring
    ]
    zone = {
        "id": "repaired-zone",
        "type": "mowing",
        "updated_at": "2026-07-15T21:00:00+00:00",
        "geometry": {"type": "Polygon", "coordinates": [wgs_ring]},
    }

    result = choose_safe_reset_pose(
        [zone], DEFAULT_SETTINGS, origin_x=origin_x, origin_y=origin_y,
    )

    assert result is not None
    _x_m, _y_m, heading, zone_id = result
    assert zone_id == "repaired-zone"
    assert abs(heading) > 0.1
