import pytest
from shapely.geometry import Polygon
from mower.path.zone_order import nearest_neighbor_order


def _square(ox: float, oy: float, side: float = 1.0) -> Polygon:
    return Polygon([
        (ox, oy), (ox+side, oy), (ox+side, oy+side), (ox, oy+side)
    ])


class TestNearestNeighborOrder:
    def test_empty_input_returns_empty(self):
        assert nearest_neighbor_order([]) == []

    def test_single_zone_returns_single(self):
        assert nearest_neighbor_order([_square(0, 0)]) == [0]

    def test_two_zones_closer_first(self):
        # Zone 0 at (0,0), Zone 1 at (2,0), Zone 2 at (10,0)
        zones = [_square(0, 0), _square(2, 0), _square(10, 0)]
        order = nearest_neighbor_order(zones, start_idx=0)
        assert order[0] == 0
        assert order[1] == 1   # zone 1 is closer to zone 0 than zone 2
        assert order[2] == 2

    def test_start_idx_respected(self):
        zones = [_square(0, 0), _square(2, 0), _square(10, 0)]
        order = nearest_neighbor_order(zones, start_idx=2)
        assert order[0] == 2

    def test_all_zones_visited_exactly_once(self):
        zones = [_square(i * 3.0, 0) for i in range(5)]
        order = nearest_neighbor_order(zones)
        assert sorted(order) == list(range(5))

    def test_invalid_start_raises(self):
        with pytest.raises(IndexError):
            nearest_neighbor_order([_square(0, 0)], start_idx=5)

    def test_returns_list_of_ints(self):
        zones = [_square(0, 0), _square(5, 0)]
        result = nearest_neighbor_order(zones)
        assert isinstance(result, list)
        assert all(isinstance(i, int) for i in result)
