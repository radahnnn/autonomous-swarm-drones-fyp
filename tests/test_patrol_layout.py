import math

import numpy as np

from sitl import patrol_layout as layout
from sitl.formation_library import slots_for
from sitl.obstacle_sensing import ObstacleSensor
from sitl.route import square_patrol


def test_survey_points_are_the_pair_sector_centres():
    points = np.array(layout.survey_points())
    for style in ("pairs", "pairs_b"):
        slots = slots_for(style, 6)[:, :2]
        for k in range(3):
            centre = (slots[2 * k] + slots[2 * k + 1]) / 2
            assert min(np.linalg.norm(points - centre, axis=1)) < 1e-9


def test_pole_on_route_is_really_on_the_first_edge_and_the_decoy_is_not():
    route = square_patrol((0.0, 0.0), layout.PATROL_SIDE_M, 0.0, 0.3)
    edge = [np.asarray(p) for p in route.waypoints[1:3]]                 # corner 1 -> corner 2
    north, east, radius = layout.POLE_ON_ROUTE
    assert abs(north - edge[0][0]) < 1e-9 and min(edge[0][1], edge[1][1]) < east < max(edge[0][1], edge[1][1])
    n2, e2, _ = layout.POLE_DECOY
    distance = min(math.hypot(n2 - p[0], e2 - p[1]) for p in np.linspace(edge[0], edge[1], 200))
    assert distance > 8.0


def test_poles_do_not_sit_in_the_pairs_way():
    """Avoidance starts at radius + 3 m: no pair slot (either phase) may be inside that, or the pairs would be pushed."""
    for style in ("pairs", "pairs_b"):
        for slot in slots_for(style, 6)[:, :2]:
            for north, east, radius in layout.POLES:
                assert math.hypot(slot[0] - north, slot[1] - east) > radius + 3.0


def test_sensor_finds_the_route_pole_mid_edge_and_the_decoy_from_its_sector():
    sensor = ObstacleSensor(layout.POLES, layout.SENSE_RANGE_M)
    assert sensor.update((0.0, 0.0)) == []                               # nothing known at the start point
    assert sensor.update((5.0, 4.0)) == []                               # first corner: pole 5.5 m from its centre
    assert sensor.update((5.0, 1.0)) == [layout.POLE_ON_ROUTE]           # on the way along the edge
    decoy = ObstacleSensor(layout.POLES, layout.SENSE_RANGE_M)
    assert decoy.update((-9.0, -2.0)) == [layout.POLE_DECOY]             # the partner slot at sector 3


def test_exports_match_the_layout():
    env = layout.shell_env()
    assert 'OBSTACLES="5,-1.5,0.6 -13,-3,0.5"' in env
    assert layout.console_args().startswith("--sensed-obstacle=5,-1.5,0.6 --sensed-obstacle=-13,-3,0.5")
