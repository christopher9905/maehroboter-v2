import math
import time
import pytest
from mower.map.teach_in import TeachIn, TeachInState
from mower.nav.localizer import Pose
from mower.nav.gps_reader import RTK_FIXED


def _make_pose(utm_x: float, utm_y: float, ts: float = None) -> Pose:
    return Pose(
        utm_x=utm_x, utm_y=utm_y,
        heading_rad=0.0, speed_mps=0.5, yaw_rate_rps=0.0,
        timestamp=ts if ts is not None else time.monotonic(),
        fix_quality=RTK_FIXED,
    )


class TestTeachInBasic:
    def test_initial_state_is_idle(self):
        ti = TeachIn()
        assert ti.state == TeachInState.IDLE

    def test_start_transitions_to_recording(self):
        ti = TeachIn()
        ti.start("boundary")
        assert ti.state == TeachInState.RECORDING

    def test_cannot_start_twice(self):
        ti = TeachIn()
        ti.start("boundary")
        with pytest.raises(RuntimeError):
            ti.start("boundary")

    def test_update_before_start_ignored(self):
        ti = TeachIn()
        ti.update(_make_pose(0.0, 0.0))  # must not raise

    def test_first_pose_sets_start_point(self):
        ti = TeachIn()
        ti.start("boundary")
        ti.update(_make_pose(500000.0, 5320000.0))
        assert len(ti.points) == 1


class TestTeachInSampling:
    def test_point_recorded_every_half_meter(self):
        ti = TeachIn(distance_threshold_m=0.5, time_threshold_s=999.0)
        ti.start("boundary")
        ti.update(_make_pose(0.0, 0.0, ts=0.0))    # point 1
        ti.update(_make_pose(0.3, 0.0, ts=0.1))    # 0.3 m — skip
        ti.update(_make_pose(0.6, 0.0, ts=0.2))    # 0.6 m — record
        assert len(ti.points) == 2

    def test_trim_last_distance_interpolates_tail_and_updates_length(self):
        ti = TeachIn(distance_threshold_m=0.1, time_threshold_s=999.0)
        ti.start("boundary")
        ti.update(_make_pose(0.0, 0.0, ts=0.0))
        ti.update(_make_pose(2.0, 0.0, ts=1.0))
        ti.update(_make_pose(4.0, 0.0, ts=2.0))

        removed = ti.trim_last_distance(1.5)

        assert removed == pytest.approx(1.5)
        assert ti.points[-1] == pytest.approx((2.5, 0.0))
        assert ti.path_length_m == pytest.approx(2.5)
        assert ti.state == TeachInState.RECORDING

    def test_trim_keeps_first_anchor_when_distance_exceeds_path(self):
        ti = TeachIn(distance_threshold_m=0.1, time_threshold_s=999.0)
        ti.start("boundary")
        ti.update(_make_pose(10.0, 20.0, ts=0.0))
        ti.update(_make_pose(11.0, 20.0, ts=1.0))

        assert ti.trim_last_distance(5.0) == pytest.approx(1.0)
        assert ti.points == [(10.0, 20.0)]

    def test_point_recorded_every_second_regardless_of_distance(self):
        ti = TeachIn(distance_threshold_m=999.0, time_threshold_s=1.0)
        ti.start("boundary")
        ti.update(_make_pose(0.0, 0.0, ts=0.0))    # point 1
        ti.update(_make_pose(0.01, 0.0, ts=0.5))   # 0.5 s — skip
        ti.update(_make_pose(0.02, 0.0, ts=1.1))   # 1.1 s — record
        assert len(ti.points) == 2

    def test_both_thresholds_checked(self):
        ti = TeachIn(distance_threshold_m=0.5, time_threshold_s=1.0)
        ti.start("boundary")
        ti.update(_make_pose(0.0, 0.0, ts=0.0))
        ti.update(_make_pose(0.2, 0.0, ts=0.5))   # dist < 0.5, time < 1.0 → skip
        assert len(ti.points) == 1
        ti.update(_make_pose(0.25, 0.0, ts=1.2))  # time > 1.0 → record
        assert len(ti.points) == 2


class TestTeachInAutoClose:
    def test_does_not_close_before_robot_has_left_start_area(self):
        ti = TeachIn(close_threshold_m=1.0, min_points=4)
        ti.start("boundary")
        ti.update(_make_pose(0.0, 0.0, ts=0.0))
        ti.update(_make_pose(0.2, 0.0, ts=2.0))
        ti.update(_make_pose(0.4, 0.0, ts=4.0))
        ti.update(_make_pose(0.6, 0.0, ts=6.0))
        assert ti.state == TeachInState.RECORDING

    def test_radius_hysteresis_ignores_first_turn_near_start(self):
        ti = TeachIn(close_threshold_m=1.0, min_points=4)
        ti.start("boundary")
        ti.update(_make_pose(0.0, 0.0, ts=0.0))
        ti.update(_make_pose(0.5, 0.0, ts=2.0))
        ti.update(_make_pose(1.1, 0.0, ts=4.0))
        ti.update(_make_pose(0.9, 0.0, ts=6.0))
        assert ti.state == TeachInState.RECORDING

    def test_auto_close_when_near_start(self):
        ti = TeachIn(close_threshold_m=1.0, min_points=4)
        ti.start("boundary")
        ti.update(_make_pose(0.0, 0.0, ts=0.0))
        ti.update(_make_pose(10.0, 0.0, ts=2.0))
        ti.update(_make_pose(10.0, 10.0, ts=4.0))
        ti.update(_make_pose(0.0, 10.0, ts=6.0))
        ti.update(_make_pose(0.5, 0.0, ts=8.0))  # within 1 m of start
        assert ti.state == TeachInState.CLOSED

    def test_not_closed_before_min_points(self):
        ti = TeachIn(close_threshold_m=1.0, min_points=5)
        ti.start("boundary")
        ti.update(_make_pose(0.0, 0.0, ts=0.0))
        ti.update(_make_pose(0.5, 0.0, ts=2.0))
        assert ti.state == TeachInState.RECORDING

    def test_to_geojson_feature_on_close(self):
        ti = TeachIn(close_threshold_m=1.0, min_points=4)
        ti.start("boundary")
        ti.update(_make_pose(0.0, 0.0, ts=0.0))
        ti.update(_make_pose(10.0, 0.0, ts=2.0))
        ti.update(_make_pose(10.0, 10.0, ts=4.0))
        ti.update(_make_pose(0.0, 10.0, ts=6.0))
        ti.update(_make_pose(0.5, 0.0, ts=8.0))
        feature = ti.to_geojson_feature()
        assert feature["type"] == "Feature"
        assert feature["properties"]["type"] == "boundary"
        assert feature["geometry"]["type"] == "Polygon"

    def test_cancel_saves_draft(self):
        ti = TeachIn()
        ti.start("zone")
        ti.update(_make_pose(0.0, 0.0, ts=0.0))
        ti.update(_make_pose(5.0, 0.0, ts=2.0))
        ti.cancel()
        assert ti.state == TeachInState.CANCELLED
        draft = ti.to_geojson_feature()
        assert draft["properties"].get("draft") is True
