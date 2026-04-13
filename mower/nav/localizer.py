import logging
import math
import threading
from dataclasses import dataclass
from typing import Callable, Optional

import numpy as np
from filterpy.kalman import UnscentedKalmanFilter
from filterpy.kalman import MerweScaledSigmaPoints

from mower.nav.gps_reader import GpsFix
from mower.nav.imu_reader import ImuReading
from mower.nav.odometry import OdometryUpdate

logger = logging.getLogger(__name__)

# State vector: [utm_x, utm_y, heading_rad, speed_mps, yaw_rate_rps]
# heading: East = 0, counter-clockwise positive (standard math/UTM convention)
_DIM_X = 5


def _fx(x: np.ndarray, dt: float) -> np.ndarray:
    """Process model: constant velocity + yaw rate (kinematic bicycle)."""
    heading = x[2]
    v = x[3]
    yaw_rate = x[4]
    return np.array([
        x[0] + v * math.cos(heading) * dt,
        x[1] + v * math.sin(heading) * dt,
        x[2] + yaw_rate * dt,
        v,
        yaw_rate,
    ])


def _hx_gps(x: np.ndarray) -> np.ndarray:
    return x[:2]  # [utm_x, utm_y]


def _hx_imu(x: np.ndarray) -> np.ndarray:
    return np.array([x[2], x[4]])  # [heading_rad, yaw_rate_rps]


def _hx_odo(x: np.ndarray) -> np.ndarray:
    return np.array([x[3]])  # [speed_mps]


# Measurement noise covariances (tuned empirically in Phase 2)
_R_GPS = np.diag([0.05 ** 2, 0.05 ** 2])          # 5 cm RTK accuracy
_R_IMU = np.diag([math.radians(2.0) ** 2,          # 2° heading noise
                  math.radians(1.0) ** 2])           # 1°/s gyro noise
_R_ODO = np.array([[0.05 ** 2]])                    # 5 cm/s speed noise


def _residual_imu(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Custom residual for IMU: wrap heading component to (-π, π]."""
    y = a - b
    # component 0 is heading_rad — wrap to (-pi, pi]
    y[0] = (y[0] + math.pi) % (2 * math.pi) - math.pi
    return y


def _compass_to_rad(heading_deg: float) -> float:
    """Convert compass heading (North=0, CW) to math angle (East=0, CCW)."""
    return math.radians(90.0 - heading_deg)


@dataclass
class Pose:
    utm_x: float
    utm_y: float
    heading_rad: float      # 0 = East, CCW positive
    speed_mps: float
    yaw_rate_rps: float
    timestamp: float
    fix_quality: int        # from latest GPS frame


class Localizer:
    """UKF sensor fusion: GPS + IMU + Odometry → Pose at sensor rate.

    Call update_gps / update_imu / update_odometry from their respective
    reader callbacks. Each update call triggers a predict step (using elapsed
    dt) followed by a measurement correction, then publishes a Pose via on_pose.
    """

    def __init__(self):
        points = MerweScaledSigmaPoints(n=_DIM_X, alpha=0.1, beta=2.0, kappa=-2.0)
        self._ukf = UnscentedKalmanFilter(
            dim_x=_DIM_X, dim_z=2,     # dim_z=2 for GPS (default hx)
            dt=0.1, fx=_fx, hx=_hx_gps,
            points=points,
        )
        self._ukf.Q = np.diag([0.02, 0.02, 0.002, 0.5, 0.05])
        self._ukf.P = np.eye(_DIM_X) * 500.0   # large initial uncertainty
        self._initialized = False
        self._latest_fix_quality: int = 0
        self._last_predict_time: Optional[float] = None
        self._lock = threading.Lock()
        self.on_pose: Optional[Callable[[Pose], None]] = None

    @property
    def initialized(self) -> bool:
        return self._initialized

    def update_gps(self, fix: GpsFix):
        with self._lock:
            self._latest_fix_quality = fix.fix_quality
            if not self._initialized:
                self._ukf.x = np.array([
                    fix.utm_x, fix.utm_y, 0.0, 0.0, 0.0
                ])
                self._last_predict_time = fix.timestamp
                self._initialized = True
                return  # first fix only seeds state — no pose published
            self._predict(fix.timestamp)
            self._ukf.update(
                z=np.array([fix.utm_x, fix.utm_y]),
                hx=_hx_gps, R=_R_GPS,
            )
            pose = self._publish(fix.timestamp)
        if self.on_pose:
            self.on_pose(pose)

    def update_imu(self, reading: ImuReading):
        with self._lock:
            if not self._initialized:
                return
            self._predict(reading.timestamp)
            heading_rad = _compass_to_rad(reading.heading_deg)
            # yaw_rate_dps is CCW-positive (BNO085 ENU frame), same as UKF state[4]
            yaw_rate_rps = math.radians(reading.yaw_rate_dps)
            _default_residual_z = self._ukf.residual_z
            self._ukf.residual_z = _residual_imu
            try:
                self._ukf.update(
                    z=np.array([heading_rad, yaw_rate_rps]),
                    hx=_hx_imu, R=_R_IMU,
                )
            finally:
                self._ukf.residual_z = _default_residual_z
            pose = self._publish(reading.timestamp)
        if self.on_pose:
            self.on_pose(pose)

    def update_odometry(self, odo: OdometryUpdate):
        with self._lock:
            if not self._initialized:
                return
            self._predict(odo.timestamp)
            self._ukf.update(
                z=np.array([odo.speed_mps]),
                hx=_hx_odo, R=_R_ODO,
            )
            pose = self._publish(odo.timestamp)
        if self.on_pose:
            self.on_pose(pose)

    def _predict(self, timestamp: float):
        dt = timestamp - self._last_predict_time
        if dt > 0:
            self._ukf.predict(dt=dt)
        self._last_predict_time = timestamp

    def _publish(self, timestamp: float) -> Pose:
        x = self._ukf.x
        return Pose(
            utm_x=float(x[0]),
            utm_y=float(x[1]),
            heading_rad=float(x[2]),
            speed_mps=float(x[3]),
            yaw_rate_rps=float(x[4]),
            timestamp=timestamp,
            fix_quality=self._latest_fix_quality,
        )
