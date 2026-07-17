"""Adapter for the official native Fields2Cover Python bindings."""

from __future__ import annotations

import importlib
import math
import multiprocessing
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Sequence

from shapely.affinity import translate
from shapely.geometry import MultiPolygon, Polygon
from shapely.geometry.base import BaseGeometry
from shapely.wkt import loads as load_wkt

from mower.path.tractor_route import TractorWaypoint


class Fields2CoverUnavailable(RuntimeError):
    """Raised when the selected native planner has not been built."""


@dataclass(frozen=True)
class Fields2CoverPlan:
    waypoints: list[TractorWaypoint]
    swath_angle_deg: float
    coverage_geometry: BaseGeometry
    metrics: dict[str, Any]


def _project_root() -> Path:
    return Path(__file__).resolve().parents[2]


def _native_python_paths() -> list[Path]:
    prefix = _project_root() / ".native" / "fields2cover"
    candidates = [prefix]
    lib = prefix / "lib"
    if lib.exists():
        candidates.extend(sorted(lib.glob("python*/site-packages")))
        candidates.extend(sorted(lib.glob("python*/dist-packages")))
    return [candidate for candidate in candidates if candidate.exists()]


def load_fields2cover() -> Any:
    for candidate in _native_python_paths():
        value = str(candidate)
        if value not in sys.path:
            sys.path.insert(0, value)
    try:
        return importlib.import_module("fields2cover")
    except (ImportError, OSError) as exc:
        raise Fields2CoverUnavailable(
            "Fields2Cover ist nicht nativ gebaut. Bitte "
            "scripts/build_fields2cover_native.sh ausführen."
        ) from exc


def native_status() -> dict[str, Any]:
    try:
        module = load_fields2cover()
    except Fields2CoverUnavailable as exc:
        return {"available": False, "version": None, "detail": str(exc)}
    return {
        "available": True,
        "version": str(getattr(module, "__version__", "2.x native")),
        "detail": "Offizielle native Fields2Cover-Bindings geladen",
    }


def _cell_from_polygon(f2c: Any, polygon: Polygon) -> Any:
    cell = f2c.Cell()
    cell.importFromWkt(polygon.wkt)
    return cell


def _objective(f2c: Any, name: str) -> Any:
    constructors = {
        "n_swath_modified": "OBJ_NSwathModified",
        "n_swath": "OBJ_NSwath",
        "swath_length": "OBJ_SwathLength",
    }
    return getattr(f2c, constructors.get(name, "OBJ_NSwathModified"))()


def _decomposer(f2c: Any, name: str, split_angle_deg: float) -> Any | None:
    constructors = {
        "trapezoidal": "DECOMP_TrapezoidalDecomp",
        "boustrophedon": "DECOMP_Boustrophedon",
    }
    constructor = constructors.get(name)
    if constructor is None:
        return None
    decomposer = getattr(f2c, constructor)()
    decomposer.setSplitAngle(math.radians(split_angle_deg))
    return decomposer


def _turning_planner(f2c: Any, name: str) -> Any:
    constructors = {
        "dubins": "PP_DubinsCurves",
        "dubins_cc": "PP_DubinsCurvesCC",
        "reeds_shepp": "PP_ReedsSheppCurves",
        "reeds_shepp_cc": "PP_ReedsSheppCurvesHC",
    }
    return getattr(f2c, constructors.get(name, "PP_DubinsCurves"))()


def _state_point(state: Any) -> tuple[float, float]:
    point = state.point
    return float(point.getX()), float(point.getY())


def _state_gear(f2c: Any, state: Any) -> int:
    direction = getattr(state, "dir", getattr(state, "direction", 1))
    return -1 if direction == f2c.PathDirection_BACKWARD else 1


def _state_operation(f2c: Any, state: Any) -> str:
    section = getattr(state, "type", getattr(state, "section", ""))
    return "mow" if section in (
        f2c.PathSectionType_SWATH,
        f2c.PathSectionType_HL_SWATH,
    ) else "turn"


def _deduplicate_waypoints(points: Sequence[TractorWaypoint]) -> list[TractorWaypoint]:
    result: list[TractorWaypoint] = []
    turn_id = 0
    previous_operation: str | None = None
    for point in points:
        if result and math.hypot(point.x - result[-1].x, point.y - result[-1].y) < 1e-8:
            continue
        if point.operation == "turn" and previous_operation != "turn":
            turn_id += 1
        result.append(TractorWaypoint(
            point.x,
            point.y,
            point.operation,
            point.gear,
            turn_id if point.operation == "turn" else None,
        ))
        previous_operation = point.operation
    return result


def _filter_tiny_mainland_fragments(
    geometry: BaseGeometry,
    *,
    machine_width: float,
    coverage_width: float,
    enabled: bool,
) -> tuple[BaseGeometry, int, float]:
    """Preserve every reachable coverage fragment.

    Older plans discarded small mainland pieces when an outer pass existed.
    That saved a turn, but it also left visibly unmown islands.  Coverage is
    now maximised independently of perimeter passes, so this compatibility
    hook deliberately keeps the complete geometry.
    """
    del machine_width, coverage_width, enabled
    return geometry, 0, 0.0


def _plan_fields2cover_in_process(
    polygon: Polygon,
    *,
    machine_width: float,
    coverage_width: float,
    headland_width: float,
    reference_forward_offset: float,
    lane_spacing: float,
    turn_radius: float,
    settings: dict[str, Any],
) -> Fields2CoverPlan:
    """Run the official decomposition, swath, route and path modules."""
    f2c = load_fields2cover()
    # Steering-functions and GDAL are markedly more stable near the origin
    # than at seven-digit UTM northings. Plan in local metres and translate
    # the exact native result back afterwards.
    origin_x = float(polygon.centroid.x)
    origin_y = float(polygon.centroid.y)
    local_polygon = translate(polygon, xoff=-origin_x, yoff=-origin_y)
    cell = _cell_from_polygon(f2c, local_polygon)
    cells = f2c.Cells(cell)
    robot = f2c.Robot(float(machine_width), float(coverage_width))
    robot.setMinTurningRadius(float(turn_radius))
    robot.setMaxDiffCurv(float(settings.get("fields2cover_max_diff_curvature", 0.1)))

    headland_width = max(float(machine_width), float(headland_width))
    headland_generator = f2c.HG_Const_gen()
    mid_headland = headland_generator.generateHeadlands(cells, headland_width / 2.0)
    decomposer = _decomposer(
        f2c,
        str(settings.get("fields2cover_decomposition", "boustrophedon")),
        float(settings.get("fields2cover_split_angle_deg", 90.0)),
    )
    decomposed = decomposer.decompose(mid_headland) if decomposer is not None else mid_headland
    mainland = headland_generator.generateHeadlands(decomposed, headland_width / 2.0)
    native_mainland_geometry = load_wkt(mainland.exportToWkt())
    # The native mainland is retained only for its efficient angle/decomposition
    # proposal. MissionRuntime reconstructs complete swaths independently, so
    # the required coverage target remains the entire input polygon.
    coverage_cells = mainland
    coverage_geometry = polygon
    removed_fragment_count = 0
    removed_fragment_area = 0.0

    swath_generator = f2c.SG_BruteForce()
    angle_mode = str(settings.get("fields2cover_angle_mode", "best"))
    if angle_mode == "fixed":
        angle = math.radians(float(settings.get("fields2cover_angle_deg", 0.0)))
        swaths = swath_generator.generateSwaths(angle, lane_spacing, coverage_cells)
        swath_angle_deg = math.degrees(angle)
    else:
        swaths = swath_generator.generateBestSwaths(
            _objective(f2c, str(settings.get("fields2cover_swath_objective", "n_swath_modified"))),
            lane_spacing,
            coverage_cells,
        )
        flat_swaths = swaths.flatten()
        swath_angle_deg = (
            math.degrees(float(flat_swaths.at(0).getInAngle()))
            if flat_swaths.size() else 0.0
        )

    if not swaths.sizeTotal():
        raise ValueError(
            "Fields2Cover hat mit der gewählten Vorgewendebreite keine Mähbahn erzeugt"
        )

    route_order = str(settings.get("fields2cover_route_order", "optimized"))
    effective_route_order = route_order
    if route_order == "optimized":
        route_planner = f2c.RP_RoutePlannerBase()
        route = route_planner.genRoute(
            mid_headland,
            swaths,
            False,
            1e-4,
            True,
            int(settings.get("fields2cover_optimizer_time_s", 2)),
            bool(settings.get("fields2cover_search_optimum", False)),
        )
        route_or_swaths = route
    else:
        sorters = {
            "boustrophedon": lambda: f2c.RP_Boustrophedon(),
            "snake": lambda: f2c.RP_Snake(),
            "spiral": lambda: f2c.RP_Spiral(int(settings.get("fields2cover_spiral_size", 6))),
        }
        sorter = sorters.get(route_order, sorters["boustrophedon"])()
        route_or_swaths = sorter.genSortedSwaths(swaths.flatten())

    path_type = str(settings.get("fields2cover_path_type", "dubins"))
    path = f2c.PP_PathPlanning().planPath(
        robot,
        route_or_swaths,
        _turning_planner(f2c, path_type),
    )
    path.discretize(max(
        0.002,
        min(0.009, float(settings.get("route_resolution_cm", 0.5)) / 100.0),
    ))
    raw: list[TractorWaypoint] = []
    for index in range(path.size()):
        state = path.getState(index)
        ref_x, ref_y = _state_point(state)
        heading = float(state.angle)
        raw.append(TractorWaypoint(
            origin_x + ref_x - reference_forward_offset * math.cos(heading),
            origin_y + ref_y - reference_forward_offset * math.sin(heading),
            _state_operation(f2c, state),
            _state_gear(f2c, state),
            None,
        ))
    if path.size():
        last_state = path.getState(path.size() - 1)
        endpoint = path.atEnd()
        heading = float(last_state.angle)
        raw.append(TractorWaypoint(
            origin_x + float(endpoint.getX()) - reference_forward_offset * math.cos(heading),
            origin_y + float(endpoint.getY()) - reference_forward_offset * math.sin(heading),
            _state_operation(f2c, last_state),
            _state_gear(f2c, last_state),
            None,
        ))
    waypoints = _deduplicate_waypoints(raw)
    if len(waypoints) < 2:
        raise ValueError("Fields2Cover hat keinen ausführbaren Pfad erzeugt")
    return Fields2CoverPlan(
        waypoints=waypoints,
        swath_angle_deg=swath_angle_deg,
        coverage_geometry=coverage_geometry,
        metrics={
            "engine": "fields2cover_native",
            "native_path_states": int(path.size()),
            "decomposition": str(settings.get("fields2cover_decomposition")),
            "swath_objective": str(settings.get("fields2cover_swath_objective")),
            "route_order": route_order,
            "effective_route_order": effective_route_order,
            "path_type": path_type,
            "headland_width_m": round(headland_width, 3),
            "machine_envelope_width_m": round(machine_width, 3),
            "coverage_width_m": round(coverage_width, 3),
            "reference_forward_offset_m": round(reference_forward_offset, 3),
            "native_mainland_area_m2": round(float(native_mainland_geometry.area), 3),
            "removed_mainland_fragments": removed_fragment_count,
            "removed_mainland_fragment_area_m2": round(removed_fragment_area, 3),
        },
    )


def _optimized_worker(connection: Any, arguments: dict[str, Any]) -> None:
    """Run the stateful optimized native pipeline in an isolated process."""
    try:
        polygon = load_wkt(arguments.pop("polygon_wkt"))
        result = _plan_fields2cover_in_process(polygon, **arguments)
        connection.send(("ok", result))
    except BaseException as exc:  # pragma: no cover - exercised via parent
        connection.send(("error", type(exc).__name__, str(exc)))
    finally:
        connection.close()


def plan_fields2cover(
    polygon: Polygon,
    *,
    machine_width: float,
    coverage_width: float,
    headland_width: float,
    reference_forward_offset: float,
    lane_spacing: float,
    turn_radius: float,
    settings: dict[str, Any],
) -> Fields2CoverPlan:
    """Run Fields2Cover, isolating its stateful OR-Tools route pipeline.

    Fields2Cover 2.x can leave native interpolation objects unusable after an
    optimized route raises the known non-monotonic CubicSpline exception.  A
    spawned worker keeps that failure out of the long-running mower process;
    only that case falls back to a fresh, still fully native Boustrophedon
    route.
    """
    if str(settings.get("fields2cover_route_order", "optimized")) != "optimized":
        return _plan_fields2cover_in_process(
            polygon,
            machine_width=machine_width,
            coverage_width=coverage_width,
            headland_width=headland_width,
            reference_forward_offset=reference_forward_offset,
            lane_spacing=lane_spacing,
            turn_radius=turn_radius,
            settings=settings,
        )

    arguments = {
        "polygon_wkt": polygon.wkt,
        "machine_width": machine_width,
        "coverage_width": coverage_width,
        "headland_width": headland_width,
        "reference_forward_offset": reference_forward_offset,
        "lane_spacing": lane_spacing,
        "turn_radius": turn_radius,
        "settings": dict(settings),
    }
    context = multiprocessing.get_context("spawn")
    receive, send = context.Pipe(duplex=False)
    process = context.Process(target=_optimized_worker, args=(send, arguments))
    process.start()
    send.close()
    timeout_s = max(15.0, float(settings.get("fields2cover_optimizer_time_s", 2)) + 10.0)
    payload: tuple[Any, ...]
    try:
        if not receive.poll(timeout_s):
            process.terminate()
            process.join(timeout=2.0)
            payload = ("worker_timeout", "TimeoutError", "Optimierungszeit überschritten")
        else:
            try:
                payload = receive.recv()
            except EOFError:
                # A native CHECK/abort (for example inside OR-Tools'
                # RoutingIndexManager) cannot be translated into a Python
                # exception.  The isolated worker simply closes the pipe.
                # Keep the mower server alive and retry without the optimizer.
                payload = ("worker_crash", "NativeAbort", "Optimierungsprozess wurde beendet")
    finally:
        receive.close()
        process.join(timeout=2.0)
    if payload[0] == "ok":
        return payload[1]

    error_message = str(payload[2])
    recoverable_worker_failure = payload[0] in {"worker_timeout", "worker_crash"}
    if not recoverable_worker_failure and "not sorted" not in error_message:
        raise RuntimeError(error_message)
    fallback_settings = dict(settings)
    fallback_settings["fields2cover_route_order"] = "boustrophedon"
    fallback = _plan_fields2cover_in_process(
        polygon,
        machine_width=machine_width,
        coverage_width=coverage_width,
        headland_width=headland_width,
        reference_forward_offset=reference_forward_offset,
        lane_spacing=lane_spacing,
        turn_radius=turn_radius,
        settings=fallback_settings,
    )
    fallback_metrics = dict(fallback.metrics)
    fallback_metrics["route_order"] = "optimized"
    fallback_metrics["effective_route_order"] = "boustrophedon_fallback"
    fallback_metrics["optimizer_fallback_reason"] = (
        payload[0] if recoverable_worker_failure else "non_monotonic_spline"
    )
    return Fields2CoverPlan(
        waypoints=fallback.waypoints,
        swath_angle_deg=fallback.swath_angle_deg,
        coverage_geometry=fallback.coverage_geometry,
        metrics=fallback_metrics,
    )
