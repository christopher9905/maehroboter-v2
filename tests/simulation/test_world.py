import math

import pytest

from mower.simulation.world import SimulationWorld


def test_forward_drive_advances_metric_and_gps_position():
    world = SimulationWorld(origin_lat=48.5, origin_lon=11.0, max_speed_mps=0.5)
    start = world.snapshot()
    world.set_drive(0.5, 0.0)

    moved = world.step(2.0)

    assert moved.x_m == pytest.approx(0.5)
    assert moved.y_m == pytest.approx(0.0)
    assert moved.utm_x == pytest.approx(start.utm_x + 0.5)
    assert moved.lon > start.lon
    assert moved.encoder_ticks > 0


def test_steering_changes_heading_through_vehicle_model():
    world = SimulationWorld(max_speed_mps=0.5, wheelbase_m=0.25)
    world.set_drive(0.4, 45.0)

    state = world.step(1.0)

    assert state.heading_rad == pytest.approx(0.8)
    assert state.yaw_rate_rps == pytest.approx(0.8)
    assert math.hypot(state.x_m, state.y_m) == pytest.approx(
        2.0 * 0.25 * math.sin(0.8 / 2.0),
    )
    assert state.y_m > 0


def test_configured_wheelbase_changes_simulated_curvature():
    world = SimulationWorld(max_speed_mps=0.5, wheelbase_m=0.25)
    world.set_wheelbase(0.5)
    world.set_drive(0.4, 45.0)

    state = world.step(1.0)

    assert state.yaw_rate_rps == pytest.approx(0.4)


def test_simulation_speed_factor_accelerates_motion_and_is_reported():
    world = SimulationWorld(max_speed_mps=0.5)
    world.set_speed_factor(3.0)
    world.set_drive(0.5, 0.0)

    state = world.step(2.0)

    assert state.x_m == pytest.approx(1.5)
    assert state.speed_mps == pytest.approx(0.75)
    assert state.simulation_speed_factor == 3.0
    assert world.time_scale() == 3.0


def test_simulation_speed_factor_is_bounded():
    world = SimulationWorld()

    with pytest.raises(ValueError, match="between 1.0 and 5.0"):
        world.set_speed_factor(5.1)


def test_configured_reset_pose_is_applied_at_startup_and_reset():
    world = SimulationWorld()
    world.set_reset_pose(1.25, -0.5, 0.3)

    configured = world.snapshot()
    assert configured.x_m == pytest.approx(1.25)
    assert configured.y_m == pytest.approx(-0.5)
    assert configured.heading_rad == pytest.approx(0.3)

    world.set_drive(1.0, 10.0)
    world.step(1.0)
    reset = world.reset()

    assert reset.x_m == pytest.approx(1.25)
    assert reset.y_m == pytest.approx(-0.5)
    assert reset.heading_rad == pytest.approx(0.3)
    assert reset.speed_mps == 0.0


def test_estop_latches_motion_and_blade_until_reset():
    world = SimulationWorld()
    world.set_drive(1.0, 0.0)
    world.set_blade(True)
    world.emergency_stop()
    world.set_drive(1.0, 0.0)

    stopped = world.step(1.0)

    assert stopped.estop_latched is True
    assert stopped.speed_mps == 0.0
    assert stopped.blade_running is False
    assert stopped.x_m == 0.0

    world.reset_estop()
    world.set_drive(1.0, 0.0)
    assert world.step(1.0).x_m > 0.0


def test_only_raw_simulated_sensor_fields_can_be_injected():
    world = SimulationWorld()
    world.set_sensor_state(
        rain_adc=800,
        lifted=True,
        error_flags=0x08,
        gps_fix_quality=5,
        pitch_deg=12.5,
    )

    state = world.snapshot()

    assert state.rain_adc == 800
    assert state.lifted is True
    assert state.error_flags == 0x08
    assert state.gps_fix_quality == 5
    assert state.pitch_deg == 12.5
    with pytest.raises(ValueError, match="Unknown simulated sensor"):
        world.set_sensor_state(mission_state="MOWING")


def test_raising_decks_stops_blade_and_tracks_each_actuator():
    world = SimulationWorld()
    world.set_blade(True)

    world.set_deck_lift(front_raised=True, rear_raised=False)
    state = world.snapshot()

    assert state.front_deck_raised is True
    assert state.rear_deck_raised is False
    assert state.blade_running is False
