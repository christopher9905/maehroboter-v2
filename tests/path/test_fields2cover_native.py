import pytest
from shapely.geometry import MultiPolygon, Polygon, box

from mower.api.control_store import ControlStore
from mower.path import fields2cover_native
from mower.path.fields2cover_native import (
    Fields2CoverPlan,
    _filter_tiny_mainland_fragments,
    native_status,
    plan_fields2cover,
)
from mower.path.tractor_route import TractorWaypoint


def test_headland_plan_preserves_tiny_disconnected_coverage_fragments():
    geometry = MultiPolygon([box(0, 0, 10, 4), box(12, 0, 12.5, 1)])

    filtered, count, area = _filter_tiny_mainland_fragments(
        geometry,
        machine_width=0.6,
        coverage_width=0.6,
        enabled=True,
    )

    assert filtered.area == 40.5
    assert count == 0
    assert area == 0.0


pytestmark = pytest.mark.skipif(
    not native_status()["available"],
    reason="projektlokale native Fields2Cover-Bibliothek ist nicht gebaut",
)


@pytest.mark.parametrize(
    "route_order",
    ["optimized", "boustrophedon", "snake", "spiral"],
)
def test_native_fields2cover_builds_executable_safe_scale_path(route_order):
    settings = ControlStore().settings()
    settings.update({
        "route_resolution_cm": 0.9,
        "fields2cover_route_order": route_order,
        "fields2cover_optimizer_time_s": 1,
    })

    result = plan_fields2cover(
        Polygon([(0, 0), (8, 0), (8, 6), (4, 4), (0, 6), (0, 0)]),
        machine_width=0.60,
        coverage_width=0.60,
        headland_width=0.70,
        reference_forward_offset=0.0,
        lane_spacing=0.50,
        turn_radius=0.25,
        settings=settings,
    )

    assert len(result.waypoints) > 10
    assert result.metrics["engine"] == "fields2cover_native"
    assert result.metrics["route_order"] == route_order
    assert result.coverage_geometry.is_valid
    assert result.coverage_geometry.area > 0.0
    assert all(point.operation in {"mow", "turn"} for point in result.waypoints)
    assert all(point.gear in {-1, 1} for point in result.waypoints)
    assert any(point.operation == "mow" for point in result.waypoints)
    assert any(point.operation == "turn" for point in result.waypoints)


def test_optimizer_native_abort_falls_back_without_crashing_server(monkeypatch):
    class Receive:
        def poll(self, _timeout):
            return True

        def recv(self):
            raise EOFError

        def close(self):
            pass

    class Send:
        def close(self):
            pass

    class Process:
        exitcode = -6

        def start(self):
            pass

        def terminate(self):
            pass

        def join(self, timeout=None):
            pass

    class Context:
        def Pipe(self, duplex=False):
            assert duplex is False
            return Receive(), Send()

        def Process(self, target, args):
            return Process()

    fallback = Fields2CoverPlan(
        waypoints=[
            TractorWaypoint(1.0, 1.0, "mow", 1, None),
            TractorWaypoint(2.0, 1.0, "mow", 1, None),
        ],
        swath_angle_deg=0.0,
        coverage_geometry=Polygon([(0, 0), (3, 0), (3, 3), (0, 3)]),
        metrics={"route_order": "boustrophedon"},
    )
    monkeypatch.setattr(fields2cover_native.multiprocessing, "get_context", lambda _name: Context())
    monkeypatch.setattr(fields2cover_native, "_plan_fields2cover_in_process", lambda *args, **kwargs: fallback)

    result = plan_fields2cover(
        Polygon([(0, 0), (3, 0), (3, 3), (0, 3)]),
        machine_width=0.6,
        coverage_width=0.6,
        headland_width=0.7,
        reference_forward_offset=0.0,
        lane_spacing=0.5,
        turn_radius=0.25,
        settings={"fields2cover_route_order": "optimized"},
    )

    assert result.metrics["effective_route_order"] == "boustrophedon_fallback"
    assert result.metrics["optimizer_fallback_reason"] == "worker_crash"
