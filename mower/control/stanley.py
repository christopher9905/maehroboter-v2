import math
from dataclasses import dataclass

from mower.nav.localizer import Pose

STANLEY_K: float = 2.0
MAX_STEERING_DEG: float = 45.0
SPEED_EPSILON: float = 0.1


@dataclass
class StanleyOutput:
    steering_deg: float          # positive = steer LEFT (CCW/math convention)
    heading_error_deg: float
    cross_track_error_m: float   # positive = robot to RIGHT of path


class StanleyController:
    def __init__(
        self,
        k: float = STANLEY_K,
        max_steering_deg: float = MAX_STEERING_DEG,
        speed_epsilon: float = SPEED_EPSILON,
    ):
        self._k = k
        self._max_steering_rad = math.radians(max_steering_deg)
        self._eps = speed_epsilon

    def compute(
        self,
        pose: Pose,
        path_start: tuple,
        path_end: tuple,
    ) -> StanleyOutput:
        ax, ay = path_start
        bx, by = path_end
        dx, dy = bx - ax, by - ay
        seg_len = math.hypot(dx, dy)

        if seg_len < 1e-9:
            return StanleyOutput(
                steering_deg=0.0,
                heading_error_deg=0.0,
                cross_track_error_m=0.0,
            )

        # Cross-track error: positive = robot is to the RIGHT of path direction
        cross_track = -(dx * (pose.utm_y - ay) - dy * (pose.utm_x - ax)) / seg_len

        # Heading error: path heading − robot heading, wrapped to (−π, π]
        path_heading = math.atan2(dy, dx)
        heading_error = path_heading - pose.heading_rad
        heading_error = (heading_error + math.pi) % (2 * math.pi) - math.pi

        # Stanley formula
        speed = max(pose.speed_mps, 0.0)
        delta = heading_error + math.atan2(self._k * cross_track, speed + self._eps)
        delta = max(-self._max_steering_rad, min(self._max_steering_rad, delta))

        return StanleyOutput(
            steering_deg=math.degrees(delta),
            heading_error_deg=math.degrees(heading_error),
            cross_track_error_m=cross_track,
        )
