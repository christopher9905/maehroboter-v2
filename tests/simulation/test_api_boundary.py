from httpx import ASGITransport, AsyncClient

from mower.api.app import create_app
from mower.api.control_store import ControlStore
from mower.api.runtime_state import RuntimeState
from mower.executive.mission_executive import MissionExecutive
from mower.simulation.environment import SimulationEnvironment
from mower.simulation.api import create_simulation_router, mount_simulation_console


async def test_manual_api_uses_production_hardware_interface_and_simulator():
    environment = SimulationEnvironment()
    runtime = RuntimeState(hardware_connected=True)
    executive = MissionExecutive(environment.hardware)
    app = create_app(
        executive,
        store=ControlStore(),
        runtime=runtime,
        hardware=environment.hardware,
    )

    async with AsyncClient(transport=ASGITransport(app=app), base_url="http://test") as client:
        response = await client.post("/api/manual/drive", json={"speed": 0.9, "steering": 20.0})

    assert response.status_code == 200
    assert response.json()["speed"] == 0.25
    state = environment.world.step(1.0)
    assert state.speed_mps > 0.0
    assert state.steering_deg == 20.0


async def test_simulation_router_injects_raw_sensors_and_resets_world():
    environment = SimulationEnvironment()
    executive = MissionExecutive(environment.hardware)
    app = create_app(
        executive,
        store=ControlStore(),
        runtime=RuntimeState(hardware_connected=True),
        hardware=environment.hardware,
    )
    app.include_router(create_simulation_router(environment, executive))
    mount_simulation_console(app)
    executive.start_mission("test-zone")
    environment.hardware.drive(0.5, 20.0)
    environment.hardware.set_deck_lift(True, True)

    async with AsyncClient(transport=ASGITransport(app=app), base_url="http://test") as client:
        injected = await client.put("/api/simulation/sensors", json={
            "rain_adc": 900,
            "gps_fix_quality": 5,
            "soc_percent": 12,
        })
        accelerated = await client.put("/api/simulation/speed", json={"factor": 3.5})
        reset = await client.post("/api/simulation/reset")
        console = await client.get("/simulation/")

    assert injected.status_code == 200
    assert injected.json()["rain_adc"] == 900
    assert injected.json()["gps_fix_quality"] == 5
    assert injected.json()["soc_percent"] == 12
    assert accelerated.status_code == 200
    assert accelerated.json()["simulation_speed_factor"] == 3.5
    assert reset.json()["simulation_speed_factor"] == 3.5
    assert reset.json()["rain_adc"] == 250
    assert reset.json()["gps_fix_quality"] == 4
    assert reset.json()["soc_percent"] == 80
    assert environment.hardware.output_snapshot()["speed_command"] == 0.0
    assert environment.hardware.output_snapshot()["steering_deg"] == 0.0
    assert environment.hardware.output_snapshot()["front_deck_raised"] is False
    assert environment.hardware.output_snapshot()["rear_deck_raised"] is False
    assert executive.state.name == "IDLE"
    assert console.status_code == 200
    assert "Simulationskonsole" in console.text
