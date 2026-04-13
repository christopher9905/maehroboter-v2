import math
import pytest
from mower.nav.localizer import Pose
from mower.control.stanley import StanleyController, StanleyOutput, STANLEY_K, MAX_STEERING_DEG


def _pose(x: float, y: float, heading_rad: float, speed: float = 1.0) -> Pose:
    return Pose(
        utm_x=x, utm_y=y, heading_rad=heading_rad,
        speed_mps=speed, yaw_rate_rps=0.0,
        timestamp=0.0, fix_quality=4,
    )


class TestStanleyController:
    def test_output_type(self):
        ctrl = StanleyController()
        out = ctrl.compute(_pose(0, 0, 0.0), (0.0, 0.0), (10.0, 0.0))
        assert isinstance(out, StanleyOutput)
        assert hasattr(out, 'steering_deg')
        assert hasattr(out, 'heading_error_deg')
        assert hasattr(out, 'cross_track_error_m')

    def test_perfectly_aligned_on_path_zero_steering(self):
        """Robot at (0,1) heading East (0 rad), path going East, 1m right of path."""
        # Path: (0,0)→(10,0) going East. Robot at (0,1) — 1m to LEFT of path.
        # cross_track_error should be NEGATIVE (robot is left of path).
        ctrl = StanleyController()
        # Place robot ON the path, heading East, path going East
        out = ctrl.compute(_pose(0, 0, 0.0), (0.0, 0.0), (10.0, 0.0))
        assert abs(out.cross_track_error_m) < 1e-9
        assert abs(out.heading_error_deg) < 1e-6
        assert abs(out.steering_deg) < 1e-6

    def test_cross_track_right_of_path_steers_left(self):
        """Robot to RIGHT of eastward path → positive cross-track → positive steering (left)."""
        ctrl = StanleyController()
        # Robot at (5, -1): 1 m to the RIGHT of eastward path (y < 0 = right)
        out = ctrl.compute(_pose(5.0, -1.0, 0.0), (0.0, 0.0), (10.0, 0.0))
        assert out.cross_track_error_m > 0   # robot is right of path
        assert out.steering_deg > 0          # steer left to correct

    def test_cross_track_left_of_path_steers_right(self):
        """Robot to LEFT of eastward path → negative cross-track → negative steering (right)."""
        ctrl = StanleyController()
        out = ctrl.compute(_pose(5.0, 1.0, 0.0), (0.0, 0.0), (10.0, 0.0))
        assert out.cross_track_error_m < 0   # robot is left of path
        assert out.steering_deg < 0          # steer right to correct

    def test_heading_error_contributes(self):
        """Robot facing North on eastward path → -90° heading error → steers right."""
        ctrl = StanleyController()
        # Robot ON path but heading North (π/2 rad), path going East (0 rad)
        # heading_error = 0 - π/2 = -π/2 → steer right (negative)
        out = ctrl.compute(_pose(5.0, 0.0, math.pi / 2), (0.0, 0.0), (10.0, 0.0))
        assert out.steering_deg < 0  # steer right
        assert abs(out.heading_error_deg - (-90.0)) < 0.1

    def test_steering_clamped_to_max(self):
        """Extreme error should be clamped to ±MAX_STEERING_DEG."""
        ctrl = StanleyController()
        # 50m off-path and high gain → without clamp would exceed 45°
        out = ctrl.compute(_pose(5.0, -50.0, 0.0), (0.0, 0.0), (10.0, 0.0))
        assert abs(out.steering_deg) <= MAX_STEERING_DEG + 1e-9

    def test_zero_speed_no_crash(self):
        """At rest, speed_epsilon prevents division by zero."""
        ctrl = StanleyController()
        out = ctrl.compute(_pose(5.0, -1.0, 0.0, speed=0.0), (0.0, 0.0), (10.0, 0.0))
        assert math.isfinite(out.steering_deg)

    def test_degenerate_segment_returns_zero(self):
        """Zero-length path segment → zero steering."""
        ctrl = StanleyController()
        out = ctrl.compute(_pose(0, 0, 0.0), (5.0, 5.0), (5.0, 5.0))
        assert out.steering_deg == 0.0
        assert out.cross_track_error_m == 0.0
        assert out.heading_error_deg == 0.0

    def test_custom_gain(self):
        """Higher k → larger steering for same cross-track error."""
        ctrl_low  = StanleyController(k=1.0)
        ctrl_high = StanleyController(k=5.0)
        pose = _pose(5.0, -1.0, 0.0)
        out_low  = ctrl_low.compute(pose,  (0.0, 0.0), (10.0, 0.0))
        out_high = ctrl_high.compute(pose, (0.0, 0.0), (10.0, 0.0))
        assert abs(out_high.steering_deg) > abs(out_low.steering_deg)

    def test_heading_wrap_near_pi(self):
        """Heading wrap: path heading -π+ε, robot heading +π-ε → small error not ~2π."""
        ctrl = StanleyController()
        # Path going West-ish: path_heading ≈ π (atan2(0,-1))
        # Robot heading ≈ -π + small → difference should be small, not ≈ 2π
        path_heading = math.pi - 0.05
        robot_heading = -(math.pi - 0.05)
        # Place robot on path, only heading error matters
        out = ctrl.compute(_pose(5.0, 0.0, robot_heading), (10.0, 0.0), (0.0, 0.0))
        assert abs(out.heading_error_deg) < 20  # should be ~5.7°, NOT ~360°

    def test_default_constants(self):
        assert STANLEY_K == 2.0
        assert MAX_STEERING_DEG == 45.0
