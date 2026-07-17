# tests/executive/test_cusp_following.py
"""Closed-loop regression tests for cusp (gear-change) maneuver execution.

Reproduces the observed simulator failure: at the headland→first-lane
transition the plan contains a short reverse shunt (gear +1 → −1 → +1).
The follower must execute it as a bounded maneuver instead of sailing
past the cusp with a stuck projection (which produced the driven loop
and the reverse runaway out of the zone).
"""
import math
from unittest.mock import MagicMock

import pytest

from mower.api.control_store import ControlStore
from mower.executive.mission_executive import MissionExecutive, MowerState
from mower.executive.mission_runtime import MissionRuntime
from mower.nav.localizer import Pose
from mower.path.tractor_route import TractorWaypoint
from mower.path.trajectory import ContinuousTrajectory

WHEELBASE_M = 0.25
MAX_SPEED_MPS = 0.5
CONTROL_PERIOD_S = 0.1


def _dense_leg(x0, y0, x1, y1, operation, gear, spacing=0.05):
    length = math.hypot(x1 - x0, y1 - y0)
    steps = max(1, int(round(length / spacing)))
    return [
        TractorWaypoint(
            x0 + (x1 - x0) * i / steps,
            y0 + (y1 - y0) * i / steps,
            operation,
            gear,
        )
        for i in range(1, steps + 1)
    ]


def _transition_route():
    """1 m approach → 25 cm reverse shunt → 4.25 m mowing lane (all on y=0).

    Mirrors the real cached plan's headland→lane transition: arrive heading
    east, back up 0.25 m (gear −1), then mow forward east.
    """
    waypoints = [TractorWaypoint(-1.0, 0.0, "headland", 1)]
    waypoints += _dense_leg(-1.0, 0.0, 0.0, 0.0, "headland", 1)
    waypoints += _dense_leg(0.0, 0.0, -0.25, 0.0, "turn", -1)
    waypoints += _dense_leg(-0.25, 0.0, 4.0, 0.0, "mow", 1)
    return waypoints


def _install_route(runtime, waypoints, *, start_s):
    runtime._route_waypoints = waypoints
    runtime._route_utm = [(p.x, p.y) for p in waypoints]
    runtime._trajectory = ContinuousTrajectory(waypoints)
    runtime._trajectory_s = start_s
    runtime._route_index = max(
        1, runtime._trajectory.sample(start_s).segment_index + 1,
    )


GPS_HZ = 5.0  # SimulatedGpsSource rate — wall-clock, independent of time scale


def _drive_closed_loop(runtime, hardware, executive, *, start_pose, ticks, time_scale):
    """Bicycle-model closed loop mirroring the real runtime mechanics.

    The real control loop compensates acceleration by ticking at
    period / time_scale wall seconds, so the world advances one control
    period per tick regardless of the factor.  What degrades under
    acceleration is the pose rate: GPS runs at 5 Hz wall time, so at 5×
    a fresh pose arrives only every 2 × time_scale control ticks — the
    coarse sampling that makes centimetre-scale cusp shunts alias.
    """
    x, y, heading = start_pose
    speed = 0.0
    t = 0.0
    trace = [(x, y)]
    ticks_per_pose = max(1, int(round(time_scale / (GPS_HZ * CONTROL_PERIOD_S))))
    for tick in range(ticks):
        if tick % ticks_per_pose == 0:
            runtime.on_pose(Pose(x, y, heading % (2 * math.pi), speed, 0.0, t, 4))
        runtime._control_step()
        # The test loop runs instantaneous wall-clock ticks; pretend one real
        # control interval (period / time_scale wall seconds) elapsed so the
        # steering-rate limiter behaves as in the live loop instead of being
        # clamped to its 10 ms minimum.
        if runtime._last_control_time is not None:
            runtime._last_control_time -= CONTROL_PERIOD_S / time_scale
        if executive.state != MowerState.MOWING:
            break
        call = hardware.drive.call_args
        norm, steer = call.args if call else (0.0, 0.0)
        v = norm * MAX_SPEED_MPS
        dt = CONTROL_PERIOD_S  # world seconds per control tick, see docstring
        if abs(v) > 1e-9:
            heading += v / WHEELBASE_M * math.tan(math.radians(steer)) * dt
            x += v * math.cos(heading) * dt
            y += v * math.sin(heading) * dt
        speed = abs(v)
        t += dt
        trace.append((x, y))
    return trace


@pytest.mark.parametrize("time_scale", [1.0, 5.0])
def test_reverse_cusp_transition_executes_without_runaway(time_scale):
    hardware = MagicMock()
    executive = MissionExecutive(hardware)
    store = ControlStore()
    store.update_settings({"deck_lift_settle_ms": 0})
    runtime = MissionRuntime(
        executive, hardware, store,
        max_speed_mps=MAX_SPEED_MPS,
        time_scale_provider=lambda: time_scale,
    )
    waypoints = _transition_route()
    _install_route(runtime, waypoints, start_s=0.1)
    executive.start_mission("cusp-zone")

    trace = _drive_closed_loop(
        runtime, hardware, executive,
        start_pose=(-0.8, 0.02, 0.0),
        ticks=1500,
        time_scale=time_scale,
    )

    max_lateral = max(abs(p[1]) for p in trace)
    min_x = min(p[0] for p in trace)
    cusp2_s = runtime._trajectory.arc_length_at_waypoint(
        max(i for i, p in enumerate(waypoints) if p.gear < 0),
    )
    # 1) Never leaves a tight corridor around the lane (the reported runaway
    #    drove metres away and out of the zone).
    assert max_lateral < 0.5, f"lateral runaway: {max_lateral:.2f} m off the lane"
    # 2) Reverses at most the planned 25 cm plus slack — never a runaway in
    #    reverse past the shunt.
    assert min_x > -1.0, f"reverse runaway: reached x={min_x:.2f}"
    # 3) Actually finished the maneuver and made progress on the mowing lane.
    assert runtime._trajectory_s > cusp2_s + 1.0, (
        f"stuck at s={runtime._trajectory_s:.2f}, "
        f"never advanced onto the lane (cusp at {cusp2_s:.2f})"
    )


def _arc_leg(x0, y0, motion_heading, curvature, length, operation, gear, spacing=0.05):
    """Sample an arc along the direction of motion (headings point along s)."""
    points = []
    steps = max(2, int(round(length / spacing)))
    ds = length / steps
    x, y, h = x0, y0, motion_heading
    for _ in range(steps):
        h += curvature * ds
        x += math.cos(h) * ds
        y += math.sin(h) * ds
        points.append(TractorWaypoint(x, y, operation, gear))
    return points, (x, y, h)


@pytest.mark.parametrize("time_scale", [1.0, 5.0])
def test_curved_reverse_arc_maneuver_tracks_without_abort(time_scale):
    """A 67 cm reverse ARC (three-point-turn leg, like the real plan's
    longest shunt) must be executed without tripping the deviation watchdog.

    Guards the steering sign in the scripted maneuver controller: path
    curvature is sign(v)·tan(δ)/L, so reverse requires negated steering —
    the unsigned variant amplified its own heading error and aborted
    deterministically at ~36 cm deviation on exactly this geometry.
    """
    hardware = MagicMock()
    executive = MissionExecutive(hardware)
    store = ControlStore()
    store.update_settings({"deck_lift_settle_ms": 0})
    runtime = MissionRuntime(
        executive, hardware, store,
        max_speed_mps=MAX_SPEED_MPS,
        time_scale_provider=lambda: time_scale,
    )
    waypoints = [TractorWaypoint(-1.0, 0.0, "headland", 1)]
    waypoints += _dense_leg(-1.0, 0.0, 0.0, 0.0, "headland", 1)
    # Reverse arc: motion heads west (π) and curves at R = 0.6 m for 0.67 m.
    arc_points, (ax, ay, ah) = _arc_leg(
        0.0, 0.0, math.pi, 1.0 / 0.6, 0.67, "turn", -1,
    )
    waypoints += arc_points
    # Forward continuation: the nose after reversing points opposite to the
    # arc's motion direction.
    nose = ah - math.pi
    waypoints += _dense_leg(
        ax, ay, ax + 1.5 * math.cos(nose), ay + 1.5 * math.sin(nose), "mow", 1,
    )
    _install_route(runtime, waypoints, start_s=0.1)
    executive.start_mission("arc-zone")

    _drive_closed_loop(
        runtime, hardware, executive,
        start_pose=(-0.8, 0.01, 0.0),
        ticks=1800,
        time_scale=time_scale,
    )

    arc_end_s = runtime._trajectory.arc_length_at_waypoint(
        max(i for i, p in enumerate(waypoints) if p.gear < 0),
    )
    # MOWING = still driving the lane, RETURNING = finished the whole route —
    # both mean the arc was executed.  PAUSED/ERROR = watchdog abort.
    assert executive.state in (MowerState.MOWING, MowerState.RETURNING), (
        f"maneuver aborted: state={executive.state.name}, "
        f"pause='{executive.pause_reason}'"
    )
    assert runtime._trajectory_s > arc_end_s + 0.5, (
        f"stuck at s={runtime._trajectory_s:.2f}, arc ends at {arc_end_s:.2f}"
    )


@pytest.mark.parametrize("time_scale", [1.0, 5.0])
def test_real_lane_end_turn_executes_as_point_turn(time_scale):
    """A genuine planner-generated lane-end turn (48 cm spacing, R = 0.5 m —
    forces a multi-point turn with reverse legs) must be driven as the tight
    scripted maneuver sequence: steer in, stop, reverse, stop, forward onto
    the next lane.  Guards against the observed bulb-shaped forward loops
    (MPC preview collapsing on the approach leg into each cusp)."""
    from shapely.geometry import box

    from mower.path.tractor_route import compute_curvature_bounded_connection

    # Without a boundary the planner happily returns the forward-only bulb
    # loop; the real field bounds the turn space (headland/geofence), which
    # forces the multi-point turn with reverse legs — replicate that.
    turn_space = box(-3.0, -0.7, 0.55, 1.2)
    connection = compute_curvature_bounded_connection(
        (0.0, 0.0, 0.0),                # end of lane A, heading east
        (0.0, 0.48, math.pi),           # start of lane B, heading west
        turn_radius=0.5,
        point_spacing=0.01,
        boundary=turn_space,
        allow_reverse=True,
        operation="turn",
        turn_id=1,
    )
    assert any(p.gear < 0 for p in connection), "test premise: turn must reverse"

    waypoints = [TractorWaypoint(-2.0, 0.0, "mow", 1)]
    waypoints += _dense_leg(-2.0, 0.0, 0.0, 0.0, "mow", 1)
    waypoints += list(connection[1:])
    waypoints += _dense_leg(0.0, 0.48, -2.0, 0.48, "mow", 1)

    hardware = MagicMock()
    executive = MissionExecutive(hardware)
    store = ControlStore()
    store.update_settings({"deck_lift_settle_ms": 0})
    runtime = MissionRuntime(
        executive, hardware, store,
        max_speed_mps=MAX_SPEED_MPS,
        time_scale_provider=lambda: time_scale,
    )
    _install_route(runtime, waypoints, start_s=0.1)
    executive.start_mission("kturn-zone")

    trace = _drive_closed_loop(
        runtime, hardware, executive,
        start_pose=(-1.8, 0.01, 0.0),
        ticks=3000,
        time_scale=time_scale,
    )

    lane_b_start_s = runtime._trajectory.arc_length_at_waypoint(
        max(i for i, p in enumerate(waypoints) if p.operation == "turn"),
    )
    assert executive.state in (MowerState.MOWING, MowerState.RETURNING), (
        f"turn aborted: state={executive.state.name}, "
        f"pause='{executive.pause_reason}'"
    )
    assert runtime._trajectory_s > lane_b_start_s + 0.5, (
        f"stuck at s={runtime._trajectory_s:.2f}, "
        f"lane B starts at {lane_b_start_s:.2f}"
    )
    # The turn must stay a TIGHT maneuver near the lane ends — no bulb loop
    # (the planned turn space ends at x = 0.55; allow modest tracking slack).
    max_x = max(p[0] for p in trace)
    assert max_x < 0.9, f"bulb turn: reached x={max_x:.2f} beyond the turn space"
    # And it must genuinely reverse at some point (three-point turn).
    reverse_commands = [
        c.args[0] for c in hardware.drive.call_args_list if c.args and c.args[0] < 0
    ]
    assert reverse_commands, "turn never reversed — not a point turn"


def test_projection_progress_never_crosses_gear_boundary_by_itself():
    """The progress latch must stop at the cusp until the explicit handover —
    skipping across produced a forward attempt at a reverse segment."""
    hardware = MagicMock()
    executive = MissionExecutive(hardware)
    store = ControlStore()
    store.update_settings({"deck_lift_settle_ms": 0})
    runtime = MissionRuntime(executive, hardware, store, max_speed_mps=MAX_SPEED_MPS)
    waypoints = _transition_route()
    _install_route(runtime, waypoints, start_s=0.9)
    executive.start_mission("cusp-zone")
    cusp_s = 1.0  # end of the 1 m approach leg

    # Pose already 30 cm PAST the cusp point (aliasing overshoot).
    runtime.on_pose(Pose(0.30, 0.0, 0.0, 0.3, 0.0, 0.0, 4))
    runtime._control_step()

    assert runtime._trajectory_s <= cusp_s + 1e-3, (
        f"progress {runtime._trajectory_s:.3f} crossed the gear boundary at "
        f"{cusp_s:.3f} without a gear handover"
    )
