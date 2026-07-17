import math
import time

from mower.control.mpc import MPCController
from mower.nav.localizer import Pose
from mower.path.tractor_route import TractorWaypoint
from mower.path.trajectory import ContinuousTrajectory


def _dense_hairpin() -> ContinuousTrajectory:
    points = [TractorWaypoint(index / 200.0, 0.0) for index in range(201)]
    for index in range(1, 101):
        angle = -math.pi / 2.0 + index * math.pi / 100.0
        points.append(TractorWaypoint(
            1.0 + 0.25 * math.cos(angle),
            0.25 + 0.25 * math.sin(angle),
            "turn",
        ))
    points.extend(
        TractorWaypoint(1.0 - index / 200.0, 0.5)
        for index in range(1, 201)
    )
    return ContinuousTrajectory(points)


def test_controller_does_not_cut_into_hairpin_ten_centimetres_early():
    trajectory = _dense_hairpin()
    pose = Pose(0.90, 0.0, 0.0, 0.194, 0.0, time.time(), "rtk_fix")

    output = MPCController().compute(
        pose,
        trajectory,
        0.90,
        speed_mps=0.194,
        wheelbase_m=0.25,
        previous_steering_deg=0.0,
    )

    assert abs(output.steering_deg) < 1.0


def test_controller_corrects_a_twelve_centimetre_straight_line_error():
    trajectory = ContinuousTrajectory([
        TractorWaypoint(0.0, 0.0),
        TractorWaypoint(10.0, 0.0),
    ])
    pose = Pose(1.0, -0.12, 0.0, 0.194, 0.0, time.time(), "rtk_fix")

    output = MPCController().compute(
        pose,
        trajectory,
        1.0,
        speed_mps=0.194,
        wheelbase_m=0.25,
        previous_steering_deg=0.0,
    )

    assert output.steering_deg > 10.0
