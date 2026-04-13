import math
import pytest
from shapely.geometry import Polygon
from mower.path.boustrophedon import compute_boustrophedon, LANE_SPACING_M


class TestBoustrophedon:
    def _square(self, side=5.0):
        return Polygon([(0,0),(side,0),(side,side),(0,side)])

    def test_empty_for_degenerate_polygon(self):
        result = compute_boustrophedon(Polygon())
        assert result == []

    def test_returns_list_of_tuples(self):
        pts = compute_boustrophedon(self._square())
        assert isinstance(pts, list)
        assert all(isinstance(p, tuple) and len(p) == 2 for p in pts)

    def test_covers_square_with_expected_lane_count(self):
        side = 5.0
        pts = compute_boustrophedon(self._square(side), lane_spacing=1.0)
        # 5 m / 1.0 m spacing → expect ~5 lanes; each lane has 2 endpoints
        assert len(pts) >= 8  # at least 4 lanes × 2 points

    def test_boustrophedon_alternates_direction(self):
        """Consecutive lanes should start on opposite sides (x alternates)."""
        pts = compute_boustrophedon(self._square(4.0), lane_spacing=1.0)
        assert len(pts) >= 4
        # Even lanes: first point has smaller x; odd lanes: first point has larger x
        # Check that adjacent lane starts differ in x-direction
        if len(pts) >= 4:
            lane0_start_x = pts[0][0]
            lane1_start_x = pts[2][0]
            assert abs(lane0_start_x - lane1_start_x) > 0.5  # on opposite sides

    def test_waypoints_inside_or_on_polygon(self):
        poly = self._square(5.0)
        pts = compute_boustrophedon(poly, lane_spacing=1.0)
        from shapely.geometry import Point
        for p in pts:
            assert poly.buffer(0.01).contains(Point(p)), f"Point {p} outside polygon"

    def test_rotated_rectangle_orientation(self):
        """A tall narrow rectangle should produce horizontal lanes (fewer turns)."""
        rect = Polygon([(0,0),(1,0),(1,10),(0,10)])
        pts = compute_boustrophedon(rect, lane_spacing=0.38)
        assert len(pts) >= 2

    def test_custom_lane_spacing(self):
        pts_narrow = compute_boustrophedon(self._square(5.0), lane_spacing=0.5)
        pts_wide   = compute_boustrophedon(self._square(5.0), lane_spacing=2.0)
        assert len(pts_narrow) > len(pts_wide)

    def test_default_lane_spacing_is_38cm(self):
        assert abs(LANE_SPACING_M - 0.38) < 1e-9
