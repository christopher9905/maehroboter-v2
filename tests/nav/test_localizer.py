import math
import time
import pytest
from mower.nav.localizer import Localizer, Pose
from mower.nav.gps_reader import GpsFix, RTK_FIXED
from mower.nav.imu_reader import ImuReading
from mower.nav.odometry import OdometryUpdate


def _make_fix(utm_x=500000.0, utm_y=5320000.0, fix_quality=RTK_FIXED,
              ts=1.0) -> GpsFix:
    return GpsFix(
        lat=48.0, lon=11.5,
        alt=500.0,
        utm_x=utm_x, utm_y=utm_y,
        utm_zone_number=32, utm_zone_letter="U",
        fix_quality=fix_quality,
        hdop=0.9,
        timestamp=ts,
    )


def _make_imu(heading_deg=0.0, yaw_rate_dps=0.0, pitch=0.0, roll=0.0,
              ts=1.0) -> ImuReading:
    return ImuReading(
        heading_deg=heading_deg,
        pitch_deg=pitch,
        roll_deg=roll,
        yaw_rate_dps=yaw_rate_dps,
        timestamp=ts,
    )


def _make_odo(speed=0.5, delta=0.5, total=0.5, ts=1.0) -> OdometryUpdate:
    return OdometryUpdate(
        speed_mps=speed,
        delta_distance_m=delta,
        total_distance_m=total,
        timestamp=ts,
    )


class TestLocalizerInit:
    def test_not_initialized_before_first_gps(self):
        loc = Localizer()
        assert not loc.initialized

    def test_initialized_after_first_gps_fix(self):
        loc = Localizer()
        loc.update_gps(_make_fix())
        assert loc.initialized

    def test_first_gps_sets_position(self):
        poses: list[Pose] = []
        loc = Localizer()
        loc.on_pose = poses.append
        loc.update_gps(_make_fix(utm_x=500000.0, utm_y=5320000.0))
        assert len(poses) == 0  # first fix only initializes, no output
        loc.update_gps(_make_fix(utm_x=500001.0, utm_y=5320001.0, ts=2.0))
        assert len(poses) == 1
        assert abs(poses[0].utm_x - 500001.0) < 1.0
        assert abs(poses[0].utm_y - 5320001.0) < 1.0


class TestLocalizerUpdates:
    def test_gps_update_shifts_position(self):
        poses: list[Pose] = []
        loc = Localizer()
        loc.on_pose = poses.append
        loc.update_gps(_make_fix(utm_x=500000.0, utm_y=5320000.0, ts=0.0))
        loc.update_gps(_make_fix(utm_x=500010.0, utm_y=5320000.0, ts=1.0))
        # After GPS correction, x should be close to 500010
        assert abs(poses[-1].utm_x - 500010.0) < 2.0

    def test_imu_update_requires_initialization_first(self):
        poses: list[Pose] = []
        loc = Localizer()
        loc.on_pose = poses.append
        loc.update_imu(_make_imu(ts=1.0))  # before GPS init — must not raise
        assert len(poses) == 0

    def test_imu_update_after_init_publishes_pose(self):
        poses: list[Pose] = []
        loc = Localizer()
        loc.on_pose = poses.append
        loc.update_gps(_make_fix(ts=0.0))
        # IMU reports 90° heading → math angle = 0.0 rad (East)
        loc.update_imu(_make_imu(heading_deg=90.0, ts=1.0))
        assert len(poses) == 1
        # After IMU correction, heading_rad should have moved toward 0.0 (East)
        # Initial state is 0.0, so it should remain close to 0.0
        assert abs(poses[0].heading_rad) < 0.5  # within 0.5 rad of East

    def test_odometry_update_after_init_publishes_pose(self):
        poses: list[Pose] = []
        loc = Localizer()
        loc.on_pose = poses.append
        loc.update_gps(_make_fix(ts=0.0))
        loc.update_odometry(_make_odo(speed=0.5, ts=1.0))
        assert len(poses) == 1
        # After odometry correction, speed should have moved toward 0.5 m/s
        assert poses[0].speed_mps > 0.0  # moved from 0.0 toward 0.5

    def test_fix_quality_propagated_to_pose(self):
        from mower.nav.gps_reader import RTK_FLOAT
        poses: list[Pose] = []
        loc = Localizer()
        loc.on_pose = poses.append
        loc.update_gps(_make_fix(fix_quality=RTK_FLOAT, ts=0.0))
        loc.update_gps(_make_fix(fix_quality=RTK_FLOAT, ts=1.0))
        assert poses[-1].fix_quality == RTK_FLOAT

    def test_pose_timestamp_matches_sensor(self):
        poses: list[Pose] = []
        loc = Localizer()
        loc.on_pose = poses.append
        loc.update_gps(_make_fix(ts=0.0))
        loc.update_odometry(_make_odo(ts=42.5))
        assert abs(poses[-1].timestamp - 42.5) < 0.01
