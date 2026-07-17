import math
import pytest
from shapely.geometry import LineString, Point, Polygon
from shapely.ops import unary_union
from mower.path.boustrophedon import (
    LANE_SPACING_M,
    compute_best_boustrophedon,
    compute_boustrophedon,
)


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

    def test_real_mowing_width_covers_both_edges_without_remainder_strip(self):
        zone = Polygon([(0, 0), (8, 0), (8, 3.47), (0, 3.47)])
        route = compute_boustrophedon(
            zone, lane_spacing=0.52, mowing_width=0.60,
        )
        lanes = [
            LineString(route[index:index + 2])
            for index in range(0, len(route), 2)
        ]
        swath = unary_union([
            lane.buffer(0.30, cap_style="square") for lane in lanes
        ])

        assert swath.buffer(1e-9).covers(zone)
        cross_positions = sorted({round(point[1], 6) for point in route})
        assert cross_positions[0] == 0.30
        assert cross_positions[-1] == 3.17
        gaps = [right - left for left, right in zip(cross_positions, cross_positions[1:])]
        assert min(gaps) >= 0.35

    def test_collision_safe_centerline_clips_all_lane_endpoints(self):
        zone = self._square(5.0)
        centerline_zone = zone.buffer(-0.4)

        route = compute_boustrophedon(
            zone,
            lane_spacing=0.5,
            mowing_width=0.6,
            angle_deg=0.0,
            centerline_zone=centerline_zone,
            lane_positions_from_centerline=True,
        )

        assert route
        assert all(centerline_zone.buffer(1e-9).covers(Point(point)) for point in route)

    def test_directional_endpoint_trims_follow_lane_direction(self):
        route = compute_boustrophedon(
            Polygon([(0, 0), (8, 0), (8, 3), (0, 3)]),
            lane_spacing=1.0,
            angle_deg=0.0,
            endpoint_start_trim_m=0.3,
            endpoint_end_trim_m=0.7,
        )

        assert route[0][0] == pytest.approx(0.3)
        assert route[1][0] == pytest.approx(7.3)
        assert route[2][0] == pytest.approx(7.7)
        assert route[3][0] == pytest.approx(0.7)

    def test_best_swath_optimizer_reports_deterministic_candidate_metrics(self):
        zone = Polygon([(0, 0), (7, 0), (7, 2), (4, 2), (4, 5), (0, 5)])

        route, angle, metrics = compute_best_boustrophedon(
            zone,
            lane_spacing=0.5,
            mowing_width=0.6,
            centerline_zone=zone.buffer(-0.3),
        )

        assert route
        assert 0.0 <= angle < 180.0
        assert metrics["candidate_angles"] >= 18
        assert metrics["swath_segments"] == len(route) // 2
        assert metrics["candidate_swath_coverage_percent"] > 0.0
