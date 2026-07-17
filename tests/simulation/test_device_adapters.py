import pytest

from mower.hal.hardware_interface import HardwareInterface
from mower.simulation.environment import SimulationEnvironment
from mower.simulation.serial_driver import SimulatedSerialDriver
from mower.simulation.world import SimulationWorld


def test_real_hardware_interface_commands_drive_simulated_world():
    world = SimulationWorld(max_speed_mps=0.5)
    driver = SimulatedSerialDriver(world)
    hardware = HardwareInterface(driver=driver)

    hardware.drive(0.6, 10.0)
    hardware.set_blade(True)
    hardware.ping(42)
    state = world.step(1.0)

    assert state.speed_mps == pytest.approx(0.3)
    assert state.steering_deg == pytest.approx(10.0)
    assert state.blade_running is True
    assert state.watchdog_ok is True

    hardware.set_deck_lift(True, False)
    lifted = world.snapshot()
    assert lifted.front_deck_raised is True
    assert lifted.rear_deck_raised is False
    assert lifted.blade_running is False


def test_simulated_telemetry_uses_real_hardware_callbacks():
    environment = SimulationEnvironment()
    received = {"sensors": None, "soc": None, "status": None}
    environment.hardware.on_sensors = lambda data: received.update(sensors=data)
    environment.hardware.on_soc = lambda data: received.update(soc=data)
    environment.hardware.on_status = lambda data: received.update(status=data)
    environment.world.ping(1)
    environment.world.set_sensor_state(rain_adc=750, error_flags=0x02, charging=True)

    environment.driver.emit_telemetry()

    assert received["sensors"]["rain_adc"] == 750
    assert received["soc"]["soc_percent"] == 80
    assert received["status"]["error_flags"] == 0x02
    assert received["status"]["charging"] is True
    assert received["status"]["watchdog_ok"] is True


def test_accelerated_driver_uses_the_same_distance_per_physics_tick():
    world = SimulationWorld(max_speed_mps=0.5)
    driver = SimulatedSerialDriver(world, telemetry_hz=20.0)
    world.set_drive(1.0, 0.0)

    driver.advance_fixed_step()
    distance_at_one_x = world.snapshot().x_m
    world.reset()
    world.set_drive(1.0, 0.0)
    world.set_speed_factor(5.0)

    driver.advance_fixed_step()

    assert world.snapshot().x_m == pytest.approx(distance_at_one_x)


def test_lift_signal_reaches_existing_hal_estop_path():
    environment = SimulationEnvironment()
    environment.world.set_blade(True)
    environment.world.set_sensor_state(lifted=True)

    environment.driver.emit_telemetry()

    state = environment.world.snapshot()
    assert state.estop_latched is True
    assert state.blade_running is False


def test_operator_reset_releases_simulated_hardware_latch():
    environment = SimulationEnvironment()
    environment.hardware.estop()
    assert environment.world.snapshot().estop_latched is True

    environment.hardware.reset_estop()

    assert environment.world.snapshot().estop_latched is False


def test_gps_and_imu_sources_use_production_data_models():
    environment = SimulationEnvironment()
    environment.world.set_drive(0.5, 30.0)
    environment.world.step(1.0)

    fix = environment.gps.sample(timestamp=123.0)
    imu = environment.imu.sample(timestamp=123.0)

    assert fix.timestamp == 123.0
    assert fix.fix_quality == 4
    assert fix.utm_x == pytest.approx(environment.world.snapshot().utm_x)
    assert imu.timestamp == 123.0
    assert imu.yaw_rate_dps != 0.0


def test_environment_reset_notifies_navigation_consumers():
    environment = SimulationEnvironment()
    reset_called = []
    environment.on_reset = lambda: reset_called.append(True)
    environment.world.set_drive(0.5, 10.0)
    environment.world.step(1.0)

    state = environment.reset()

    assert reset_called == [True]
    assert state.x_m == 0.0
    assert state.speed_mps == 0.0
