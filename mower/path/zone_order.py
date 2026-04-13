import math
from shapely.geometry import Polygon


def _centroid(poly: Polygon) -> tuple[float, float]:
    c = poly.centroid
    return c.x, c.y


def _dist(a: tuple[float, float], b: tuple[float, float]) -> float:
    return math.hypot(b[0] - a[0], b[1] - a[1])


def nearest_neighbor_order(
    zones: list[Polygon],
    start_idx: int = 0,
) -> list[int]:
    """Return zone indices in nearest-neighbour visit order.

    Uses the nearest-neighbour heuristic (greedy TSP approximation):
    start at start_idx, repeatedly pick the closest unvisited zone by
    centroid-to-centroid distance. O(n²) — appropriate for ≤ 50 zones.

    Args:
        zones: List of Shapely Polygons representing lawn zones.
        start_idx: Index of the zone to visit first (default 0).

    Returns:
        List of zone indices in visit order.

    Raises:
        IndexError: If start_idx is out of range for the given zones list.
    """
    if not zones:
        return []
    if start_idx < 0 or start_idx >= len(zones):
        raise IndexError(
            f"start_idx {start_idx} out of range for {len(zones)} zones"
        )

    centroids = [_centroid(z) for z in zones]
    unvisited = list(range(len(zones)))
    order: list[int] = []
    current = start_idx
    unvisited.remove(current)
    order.append(current)

    while unvisited:
        cur_c = centroids[current]
        nearest = min(unvisited, key=lambda i: _dist(cur_c, centroids[i]))
        order.append(nearest)
        unvisited.remove(nearest)
        current = nearest

    return order
