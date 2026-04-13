import time
import pytest
from mower.safety.geofence import Geofence
from mower.nav.localizer import Pose
from mower.nav.gps_reader import RTK_FIXED


def _make_pose(utm_x: float, utm_y: float) -> Pose:
    return Pose(
        utm_x=utm_x, utm_y=utm_y,
        heading_rad=0.0, speed_mps=0.0, yaw_rate_rps=0.0,
        timestamp=time.monotonic(), fix_quality=RTK_FIXED,
    )


# A simple 10 m × 10 m square centered at (500000, 5320000)
BOUNDARY_COORDS = [
    [500000.0 - 5, 5320000.0 - 5],
    [500000.0 + 5, 5320000.0 - 5],
    [500000.0 + 5, 5320000.0 + 5],
    [500000.0 - 5, 5320000.0 + 5],
    [500000.0 - 5, 5320000.0 - 5],  # closed
]


class TestGeofence:
    def test_point_inside_no_violation(self):
        violations: list[Pose] = []
        gf = Geofence(boundary_utm_coords=BOUNDARY_COORDS)
        gf.on_violation = violations.append
        gf.check(_make_pose(500000.0, 5320000.0))
        assert len(violations) == 0

    def test_point_outside_triggers_violation(self):
        violations: list[Pose] = []
        gf = Geofence(boundary_utm_coords=BOUNDARY_COORDS)
        gf.on_violation = violations.append
        gf.check(_make_pose(500100.0, 5320100.0))  # far outside
        assert len(violations) == 1

    def test_point_on_boundary_no_violation(self):
        """shapely covers() treats boundary points as safe."""
        violations: list[Pose] = []
        gf = Geofence(boundary_utm_coords=BOUNDARY_COORDS)
        gf.on_violation = violations.append
        gf.check(_make_pose(500000.0 - 5, 5320000.0))  # exactly on west edge
        assert len(violations) == 0

    def test_from_geojson_feature(self):
        feature = {
            "type": "Feature",
            "properties": {"type": "boundary"},
            "geometry": {
                "type": "Polygon",
                "coordinates": [BOUNDARY_COORDS],
            },
        }
        gf = Geofence.from_geojson_feature(feature)
        violations: list[Pose] = []
        gf.on_violation = violations.append
        gf.check(_make_pose(500100.0, 5320100.0))
        assert len(violations) == 1

    def test_violation_pose_is_the_offending_pose(self):
        violations: list[Pose] = []
        gf = Geofence(boundary_utm_coords=BOUNDARY_COORDS)
        gf.on_violation = violations.append
        bad_pose = _make_pose(600000.0, 6000000.0)
        gf.check(bad_pose)
        assert violations[0] is bad_pose
