import math

from mower.path.tractor_route import TractorWaypoint
from mower.path.trajectory import ContinuousTrajectory


def test_diagonal_trajectory_interpolates_continuously():
    trajectory = ContinuousTrajectory([
        TractorWaypoint(0, 0), TractorWaypoint(1, 1), TractorWaypoint(2, 2),
    ])
    sample = trajectory.sample(math.sqrt(2) / 2)
    assert abs(sample.x - 0.5) < 1e-6
    assert abs(sample.y - 0.5) < 1e-6
    assert abs(sample.heading_rad - math.pi / 4) < 1e-6


def test_local_projection_does_not_jump_to_nearby_lane():
    trajectory = ContinuousTrajectory([
        TractorWaypoint(0, 0), TractorWaypoint(5, 0),
        TractorWaypoint(5, 0.4, "turn"), TractorWaypoint(0, 0.4),
    ])
    first_lane = trajectory.project(2, 0.35, previous_s=1.8, window_m=1.0)
    assert first_lane.sample.segment_index == 0


def test_projection_uses_travel_heading_at_hairpin():
    trajectory = ContinuousTrajectory([
        TractorWaypoint(0, 0), TractorWaypoint(5, 0),
        TractorWaypoint(5, 0.4, "turn"), TractorWaypoint(0, 0.4),
    ])

    projection = trajectory.project(
        4.5,
        0.35,
        previous_s=4.4,
        window_m=2.0,
        heading_rad=0.0,
        max_forward_m=0.15,
    )

    assert projection.sample.segment_index == 0


def test_projection_has_a_hard_forward_progress_limit():
    trajectory = ContinuousTrajectory([
        TractorWaypoint(0, 0), TractorWaypoint(5, 0),
        TractorWaypoint(5, 0.4, "turn"), TractorWaypoint(0, 0.4),
    ])

    projection = trajectory.project(
        2.0,
        0.4,
        previous_s=1.0,
        window_m=10.0,
        heading_rad=math.pi,
        max_forward_m=0.05,
    )

    assert projection.sample.s <= 1.05 + 1e-9


def test_preview_stops_before_gear_change():
    trajectory = ContinuousTrajectory([
        TractorWaypoint(0, 0), TractorWaypoint(1, 0),
        TractorWaypoint(2, 0, "turn", -1),
    ])
    assert trajectory.clamp_to_current_motion(0.2, 1.8) < 1.0 + 1e-3


def test_preview_crosses_operation_change_in_same_gear():
    trajectory = ContinuousTrajectory([
        TractorWaypoint(0, 0, "turn", 1),
        TractorWaypoint(1, 0, "turn", 1),
        TractorWaypoint(2, 0, "mow", 1),
    ])

    assert trajectory.clamp_to_current_motion(0.2, 1.8) == 1.8
