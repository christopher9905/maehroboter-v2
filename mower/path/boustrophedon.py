import math
from shapely.geometry import Polygon, LineString, MultiLineString, GeometryCollection
from shapely.geometry.base import BaseGeometry
from shapely.affinity import rotate
from shapely.ops import unary_union

LANE_SPACING_M: float = 0.38


def _longest_edge_angle_deg(poly: Polygon) -> float:
    """Return the angle (degrees) of the longest edge of poly's MBR."""
    mrr = poly.minimum_rotated_rectangle
    coords = list(mrr.exterior.coords)  # 5 points (closing = first)
    best_angle = 0.0
    best_len = -1.0
    for i in range(len(coords) - 1):
        x1, y1 = coords[i]
        x2, y2 = coords[i + 1]
        length = math.hypot(x2 - x1, y2 - y1)
        if length > best_len:
            best_len = length
            best_angle = math.degrees(math.atan2(y2 - y1, x2 - x1))
    return best_angle


def _rotate_point(x: float, y: float, angle_rad: float,
                  cx: float, cy: float) -> tuple[float, float]:
    cos_a = math.cos(angle_rad)
    sin_a = math.sin(angle_rad)
    dx, dy = x - cx, y - cy
    return cx + cos_a * dx - sin_a * dy, cy + sin_a * dx + cos_a * dy


def compute_boustrophedon(
    zone: Polygon,
    lane_spacing: float = LANE_SPACING_M,
    mowing_width: float | None = None,
    *,
    angle_deg: float | None = None,
    endpoint_trim_m: float = 0.0,
    endpoint_start_trim_m: float | None = None,
    endpoint_end_trim_m: float | None = None,
    centerline_zone: BaseGeometry | None = None,
    lane_position_zone: BaseGeometry | None = None,
    lane_positions_from_centerline: bool = False,
) -> list[tuple[float, float]]:
    if zone.is_empty or not zone.is_valid or zone.area == 0:
        return []

    centroid = zone.centroid
    cx, cy = centroid.x, centroid.y

    if endpoint_trim_m < 0:
        raise ValueError("endpoint_trim_m must not be negative")
    start_trim = (
        endpoint_trim_m if endpoint_start_trim_m is None
        else float(endpoint_start_trim_m)
    )
    end_trim = (
        endpoint_trim_m if endpoint_end_trim_m is None
        else float(endpoint_end_trim_m)
    )
    if start_trim < 0.0 or end_trim < 0.0:
        raise ValueError("endpoint trims must not be negative")
    angle_deg = _longest_edge_angle_deg(zone) if angle_deg is None else float(angle_deg)
    angle_rad = math.radians(angle_deg)

    rotated = rotate(zone, -angle_deg, origin=centroid, use_radians=False)
    rotated_centerline_zone = rotate(
        centerline_zone if centerline_zone is not None else zone,
        -angle_deg,
        origin=centroid,
        use_radians=False,
    )
    if rotated_centerline_zone.is_empty:
        return []
    rotated_lane_position_zone = rotate(
        lane_position_zone if lane_position_zone is not None
        else (centerline_zone if centerline_zone is not None else zone),
        -angle_deg,
        origin=centroid,
        use_radians=False,
    )
    if rotated_lane_position_zone.is_empty:
        return []
    minx, miny, maxx, maxy = rotated.bounds
    mowing_width = lane_spacing if mowing_width is None else max(lane_spacing, mowing_width)

    # Treat the configured lane spacing as a maximum. Both edge lanes remain
    # exact so the complete working width is covered. A very narrow remainder
    # is shared by the last two gaps; this avoids one tiny cleanup lane and the
    # unnecessarily awkward multi-point turn it would otherwise create.
    position_min_y, position_max_y = (
        (rotated_lane_position_zone.bounds[1], rotated_lane_position_zone.bounds[3])
        if lane_positions_from_centerline else (miny, maxy)
    )
    cross_span = position_max_y - position_min_y
    if lane_positions_from_centerline:
        if cross_span <= lane_spacing:
            lane_positions = [(position_min_y + position_max_y) / 2.0]
        else:
            gap_count = max(1, math.ceil(cross_span / lane_spacing))
            actual_gap = cross_span / gap_count
            lane_positions = [
                position_min_y + index * actual_gap
                for index in range(gap_count + 1)
            ]
    elif cross_span <= mowing_width:
        lane_positions = [(position_min_y + position_max_y) / 2.0]
    else:
        usable_span = cross_span - mowing_width
        full_steps = math.floor(usable_span / lane_spacing)
        lane_positions = [
            position_min_y + mowing_width / 2.0 + index * lane_spacing
            for index in range(full_steps + 1)
        ]
        far_edge_position = position_max_y - mowing_width / 2.0
        remainder = far_edge_position - lane_positions[-1]
        if remainder > 1e-9:
            if len(lane_positions) >= 2 and remainder < lane_spacing * 0.55:
                balanced_start = lane_positions[-2]
                balanced_gap = (far_edge_position - balanced_start) / 2.0
                lane_positions[-1] = balanced_start + balanced_gap
            lane_positions.append(far_edge_position)

    waypoints: list[tuple[float, float]] = []
    lane_idx = 0
    for y in lane_positions:
        scan_line = LineString([(minx - 1.0, y), (maxx + 1.0, y)])
        # Lane positions still come from the complete target area so the
        # configured working width covers both edges.  The line itself is
        # clipped against the collision-safe rear-axle area.  This distinction
        # is essential at concave/slanted borders: a valid centre point is not
        # sufficient when a mower deck extends several centimetres sideways.
        inter = rotated_centerline_zone.intersection(scan_line)

        segments: list[LineString] = []
        if isinstance(inter, LineString) and not inter.is_empty:
            segments = [inter]
        elif isinstance(inter, MultiLineString):
            segments = list(inter.geoms)
        elif isinstance(inter, GeometryCollection):
            segments = [g for g in inter.geoms
                        if isinstance(g, LineString) and not g.is_empty]

        if segments:
            segments.sort(key=lambda segment: segment.bounds[0])
            if lane_idx % 2 == 1:
                segments.reverse()
            for seg in segments:
                endpoints = sorted(
                    (
                        (seg.coords[0][0], seg.coords[0][1]),
                        (seg.coords[-1][0], seg.coords[-1][1]),
                    ),
                    key=lambda point: (point[0], point[1]),
                )
                # GEOS does not guarantee the coordinate direction of an
                # intersection line. Enforce the boustrophedon direction
                # explicitly; otherwise adjacent lanes can both point the
                # same way and create a field-wide connector or full circle.
                p1, p2 = endpoints if lane_idx % 2 == 0 else endpoints[::-1]
                length = math.hypot(p2[0] - p1[0], p2[1] - p1[1])
                # A headland is a longitudinal turning area.  Trim the lane
                # along its own direction instead of buffering the complete
                # (possibly concave) field in every direction.  A global
                # negative buffer can pinch off valid inner field sections
                # and creates the apparent lane endings in open grass.
                if start_trim > 0.0 or end_trim > 0.0:
                    if length <= start_trim + end_trim:
                        continue
                    ux = (p2[0] - p1[0]) / length
                    uy = (p2[1] - p1[1]) / length
                    p1 = (
                        p1[0] + ux * start_trim,
                        p1[1] + uy * start_trim,
                    )
                    p2 = (
                        p2[0] - ux * end_trim,
                        p2[1] - uy * end_trim,
                    )
                waypoints.append(_rotate_point(p1[0], p1[1], angle_rad, cx, cy))
                waypoints.append(_rotate_point(p2[0], p2[1], angle_rad, cx, cy))
            lane_idx += 1
    return waypoints


def compute_best_boustrophedon(
    zone: Polygon,
    lane_spacing: float = LANE_SPACING_M,
    mowing_width: float | None = None,
    *,
    endpoint_trim_m: float = 0.0,
    endpoint_start_trim_m: float | None = None,
    endpoint_end_trim_m: float | None = None,
    centerline_zone: BaseGeometry | None = None,
    lane_position_zone: BaseGeometry | None = None,
    angle_step_deg: float = 10.0,
) -> tuple[list[tuple[float, float]], float, dict[str, float | int]]:
    """Choose a low-turn swath angle using a Fields2Cover-style objective.

    Fields2Cover separates swath generation from route/path planning and lets
    an objective choose the angle.  This lightweight implementation evaluates
    deterministic angle candidates, avoids isolated short swaths that cannot
    be connected with a tractor radius, and then minimises the number of
    required lane manoeuvres.  Covered area and non-productive travel break
    the remaining ties.  Exact tractor kinematics and the swept-footprint
    preflight remain later hard constraints.
    """
    if angle_step_deg <= 0.0 or angle_step_deg > 90.0:
        raise ValueError("angle_step_deg must be in (0, 90]")
    preferred = _longest_edge_angle_deg(zone) % 180.0
    candidates = {preferred}
    angle = 0.0
    while angle < 180.0 - 1e-9:
        candidates.add(round(angle, 9))
        angle += angle_step_deg
    coordinates = list(zone.exterior.coords)
    for left, right in zip(coordinates, coordinates[1:]):
        dx, dy = right[0] - left[0], right[1] - left[1]
        if math.hypot(dx, dy) > 1e-6:
            candidates.add(math.degrees(math.atan2(dy, dx)) % 180.0)

    best: tuple[tuple[float, ...], list[tuple[float, float]], float, dict[str, float | int]] | None = None
    for candidate in sorted(candidates):
        waypoints = compute_boustrophedon(
            zone,
            lane_spacing=lane_spacing,
            mowing_width=mowing_width,
            angle_deg=candidate,
            endpoint_trim_m=endpoint_trim_m,
            endpoint_start_trim_m=endpoint_start_trim_m,
            endpoint_end_trim_m=endpoint_end_trim_m,
            centerline_zone=centerline_zone,
            lane_position_zone=lane_position_zone,
            lane_positions_from_centerline=True,
        )
        if not waypoints:
            continue
        lengths = [
            math.dist(waypoints[index], waypoints[index + 1])
            for index in range(0, len(waypoints) - 1, 2)
        ]
        transitions = [
            math.dist(waypoints[index], waypoints[index + 1])
            for index in range(1, len(waypoints) - 1, 2)
        ]
        minimum_useful_lane = max(
            0.75,
            2.0 * lane_spacing,
            1.5 * (mowing_width or lane_spacing),
        )
        fragments = sum(length < minimum_useful_lane for length in lengths)
        segment_count = len(lengths)
        transition_distance = sum(transitions)
        swaths = [
            LineString(waypoints[index:index + 2]).buffer(
                (mowing_width or lane_spacing) / 2.0,
                cap_style="square",
                join_style="round",
            )
            for index in range(0, len(waypoints) - 1, 2)
        ]
        covered_area = unary_union(swaths).intersection(zone).area if swaths else 0.0
        coverage_ratio = covered_area / zone.area if zone.area > 0.0 else 0.0
        # A short isolated swath is a hard drivability warning for a tractor,
        # not merely a distance cost.  An additional lane also requires a full
        # radius-bounded manoeuvre, so maximise coverage only among equally
        # fragmented plans with the same manoeuvre count.
        score = (
            float(fragments),
            float(segment_count),
            -round(coverage_ratio, 6),
            round(transition_distance, 6),
            abs(((candidate - preferred + 90.0) % 180.0) - 90.0),
        )
        metrics: dict[str, float | int] = {
            "candidate_angles": len(candidates),
            "swath_segments": segment_count,
            "fragmented_swaths": fragments,
            "direct_transition_distance_m": round(transition_distance, 3),
            "candidate_swath_coverage_percent": round(coverage_ratio * 100.0, 2),
        }
        if best is None or score < best[0]:
            best = (score, waypoints, candidate, metrics)
    if best is None:
        return [], preferred, {
            "candidate_angles": len(candidates),
            "swath_segments": 0,
            "fragmented_swaths": 0,
            "direct_transition_distance_m": 0.0,
            "candidate_swath_coverage_percent": 0.0,
        }
    return best[1], best[2], best[3]
