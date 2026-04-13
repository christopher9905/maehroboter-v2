import logging
from typing import Callable, Optional

from shapely.geometry import Point, Polygon

from mower.nav.localizer import Pose

logger = logging.getLogger(__name__)


class Geofence:
    """Checks UTM position against a boundary polygon.

    Call check(pose) at 10 Hz. If the position falls outside the boundary,
    on_violation(pose) is called immediately.

    Uses shapely.covers() so points exactly on the boundary edge are safe.
    """

    def __init__(self, boundary_utm_coords: list):
        self._polygon = Polygon([(c[0], c[1]) for c in boundary_utm_coords])
        self.on_violation: Optional[Callable[[Pose], None]] = None

    @classmethod
    def from_geojson_feature(cls, feature: dict) -> "Geofence":
        coords = feature["geometry"]["coordinates"][0]  # outer ring
        return cls(boundary_utm_coords=coords)

    def check(self, pose: Pose):
        point = Point(pose.utm_x, pose.utm_y)
        if not self._polygon.covers(point):
            logger.warning(
                "Geofence violation at UTM (%.2f, %.2f)", pose.utm_x, pose.utm_y
            )
            if self.on_violation:
                self.on_violation(pose)
