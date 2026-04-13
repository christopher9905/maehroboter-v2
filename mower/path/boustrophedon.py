import math
from shapely.geometry import Polygon, LineString, MultiLineString, GeometryCollection
from shapely.affinity import rotate

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
) -> list[tuple[float, float]]:
    if zone.is_empty or not zone.is_valid or zone.area == 0:
        return []

    centroid = zone.centroid
    cx, cy = centroid.x, centroid.y

    angle_deg = _longest_edge_angle_deg(zone)
    angle_rad = math.radians(angle_deg)

    rotated = rotate(zone, -angle_deg, origin=centroid, use_radians=False)
    minx, miny, maxx, maxy = rotated.bounds

    waypoints: list[tuple[float, float]] = []
    y = miny + lane_spacing / 2.0
    lane_idx = 0
    while y <= maxy:
        scan_line = LineString([(minx - 1.0, y), (maxx + 1.0, y)])
        inter = rotated.intersection(scan_line)

        segments: list[LineString] = []
        if isinstance(inter, LineString) and not inter.is_empty:
            segments = [inter]
        elif isinstance(inter, MultiLineString):
            segments = list(inter.geoms)
        elif isinstance(inter, GeometryCollection):
            segments = [g for g in inter.geoms
                        if isinstance(g, LineString) and not g.is_empty]

        for seg in segments:
            p1 = (seg.coords[0][0], seg.coords[0][1])
            p2 = (seg.coords[-1][0], seg.coords[-1][1])
            if lane_idx % 2 == 1:
                p1, p2 = p2, p1
            waypoints.append(_rotate_point(p1[0], p1[1], angle_rad, cx, cy))
            waypoints.append(_rotate_point(p2[0], p2[1], angle_rad, cx, cy))
        lane_idx += 1
        y += lane_spacing

    return waypoints
