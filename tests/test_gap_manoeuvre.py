import math

import numpy as np
import pytest

from sitl import gap_manoeuvre as g
from sitl.formation_library import slots_for

NORTH = 0.0
TOWERS = [(20.0, -4.0, 0.6), (20.0, 4.0, 0.6)]          # 8 m apart centre to centre, across a northbound path


def test_finds_the_gap_ahead_and_its_free_width():
    gap = g.find_gap(TOWERS, (0.0, 0.0), NORTH)
    assert gap is not None and gap.centre == (20.0, 0.0) and gap.along_m == pytest.approx(20.0)
    assert gap.free_width_m == pytest.approx(8.0 - 1.2)


def test_ignores_obstacles_behind_beside_or_far_ahead():
    assert g.find_gap(TOWERS, (30.0, 0.0), NORTH) is None                          # already past
    assert g.find_gap([(20.0, 3.0, 0.5), (20.0, 9.0, 0.5)], (0.0, 0.0), NORTH) is None   # both on one side
    assert g.find_gap(TOWERS, (0.0, 0.0), NORTH, lookahead_m=10.0) is None
    assert g.find_gap(TOWERS, (0.0, 0.0), math.pi / 2) is None                     # travelling east: path misses it


def test_v_must_squeeze_and_a_column_fits_through_an_8m_gap():
    gap = g.find_gap(TOWERS, (0.0, 0.0), NORTH)
    v = slots_for("v", 6)[:, :2]
    assert g.formation_width(v, NORTH) == pytest.approx(15.0)       # D6 extends one arm
    assert g.needs_squeeze(v, NORTH, gap)
    column = slots_for("column", 6)[:, :2]                                          # drones along North
    assert g.formation_width(column, NORTH) == pytest.approx(0.0)
    assert not g.needs_squeeze(column, NORTH, gap) and g.passable_single_file(gap)
    assert not g.passable_single_file(gap, sigma_m=1.0)       # 6.8 m free < 8 m needed with 1 m GPS error
    wide = g.find_gap([(20.0, -5.0, 0.6), (20.0, 5.0, 0.6)], (0.0, 0.0), NORTH)
    assert g.passable_single_file(wide, sigma_m=1.0)          # towers 10 m apart: a column fits with GPS error
    assert not g.passable_single_file(g.find_gap([(20, -3, 0.6), (20, 3, 0.6)], (0, 0), NORTH))


def test_all_past_and_stop_point():
    gap = g.find_gap(TOWERS, (0.0, 0.0), NORTH)
    behind = np.array([[30.0, 0.0], [26.0, 0.0], [23.5, 0.0]])
    assert not g.all_past(behind, gap, NORTH)                                      # last one only 3.5 m past
    assert g.all_past(behind + [1.0, 0.0], gap, NORTH)
    stop = g.stop_point(gap, (0.0, 0.0), NORTH, formation_length_m=10.0)
    assert stop == pytest.approx([9.0, 0.0])                                        # 20 - 6 - 5


def test_heading_helpers():
    assert g.heading_to((0, 0), (0, 10)) == pytest.approx(math.pi / 2)
    with pytest.raises(ValueError):
        g.heading_to((1, 1), (1, 1))


def _console(monkeypatch, positions):
    import threading
    from types import SimpleNamespace
    from sitl.independent_swarm_console import IndependentSwarmConsole
    c = object.__new__(IndependentSwarmConsole)
    c.count, c.lock, c.time_scale, c.position_sigma_m = len(positions), threading.Lock(), 1.0, 0.0
    c.latest = {i: {"position": [p[0], p[1], -5.0], "obstacle_sensing": {"items": [list(t) for t in TOWERS]}}
                for i, p in enumerate(positions)}
    c.args = SimpleNamespace(obstacle=[])
    c.goal_ne, c.route, c.formation, c.transition_protocol = np.zeros(2), None, "v", False
    c.transition_pending = None
    c.events, c.calls = [], []
    c.event = lambda *a, **k: c.events.append(a[0])
    c._note_metric = lambda *a, **k: None
    c._publish_intent = lambda: None

    class InlineThread:
        def __init__(self, target, args=(), daemon=None):
            self.target, self.args = target, args
        def start(self):
            c.calls.append(self.args)

    monkeypatch.setattr("sitl.independent_swarm_console.threading.Thread", InlineThread)
    return c


def test_transit_squeezes_into_a_column_passes_and_reforms_the_v(monkeypatch):
    v = slots_for("v", 6)[:, :2]
    c = _console(monkeypatch, v + [8.0, 0.0])                    # V centred near the stop point
    c.transit("40", "0", "0.3")
    assert c._gap["state"] == "cruise" and c.route is not None
    c.route = type("R", (), {"position": lambda self, t: np.array([9.0, 0.0]), "finished": lambda self, t: False})()
    c._tick_gap()
    assert c._gap["state"] == "squeeze" and c.calls[-1][0] == "column" and c.route is None
    assert c.calls[-1][2] == c.GAP_COLUMN_SPACING_M                  # wide single file
    c.formation = "column"
    c._tick_gap()
    assert c._gap["state"] == "through"
    for i in range(6):                                          # everyone past the towers
        c.latest[i]["position"] = [30.0 + i, 0.0, -5.0]
    c._tick_gap()
    assert c._gap["state"] == "reform" and c.calls[-1][0] == "v"
    c.formation = "v"
    c._tick_gap()
    assert c._gap["state"] == "cruise" and "gap_passed" in c.events


def test_a_gap_too_narrow_even_for_a_column_holds_instead(monkeypatch):
    narrow = [(20.0, -3.0, 0.6), (20.0, 3.0, 0.6)]
    v = slots_for("v", 6)[:, :2]
    c = _console(monkeypatch, v + [8.0, 0.0])
    for i in range(6):
        c.latest[i]["obstacle_sensing"]["items"] = [list(t) for t in narrow]
    c.transit("40", "0", "0.3")
    c.route = type("R", (), {"position": lambda self, t: np.array([9.0, 0.0]), "finished": lambda self, t: False})()
    c._tick_gap()
    assert c._gap["state"] == "done" and "gap_too_narrow" in c.events and c.calls == []


def test_min_spacing_spreads_the_column():
    from sitl.formation_transition import plan_formation_transition
    start = slots_for("v", 6)[:, :2]
    plan = plan_formation_transition(start, "column", centroid_ne=np.zeros(2), heading_rad=0.0, min_spacing_m=8.0,
                                     max_transition_s=500.0)
    pts = plan.slots_by_drone[:, :2]
    gaps = min(np.linalg.norm(pts[i] - pts[j]) for i in range(6) for j in range(i + 1, 6))
    assert gaps >= 8.0 - 1e-6 and g.formation_width(pts, 0.0) == pytest.approx(0.0, abs=1e-6)


def test_a_refused_formation_ends_the_manoeuvre_instead_of_waiting_forever(monkeypatch):
    c = _console(monkeypatch, slots_for("v", 6)[:, :2] + [8.0, 0.0])
    c.transit("40", "0", "0.3")
    c._gap["state"] = "squeeze"
    monkeypatch.setattr("sitl.independent_swarm_console.time.sleep", lambda s: None)
    def refuse(style, heading=None, spacing=None):
        c.last_formation_result = {"accepted": False, "message": "transition needs 200 s"}
    c._set_formation = refuse
    c._gap_formation("column", 0.0, 8.0)
    assert c._gap["state"] == "done" and "gap_manoeuvre_abandoned" in c.events


def test_codex_case_route_clipping_a_tower_is_steered_through_the_midpoint():
    """Codex review #6: towers (20,-0.2) and (20,10): the straight path at E=0 hits the first tower; the column must fly
    through the midpoint, which has (9 m free)/2 clearance on each side."""
    gap = g.find_gap([(20.0, -0.2, 0.6), (20.0, 10.0, 0.6)], (0.0, 0.0), NORTH)
    assert gap is not None and gap.centre == pytest.approx((20.0, 4.9))
    route = g.through_route((0.0, 0.0), gap, (60.0, 0.0))
    assert route[1] == pytest.approx((20.0, 4.9))                                  # via the gate's midpoint


def test_staggered_towers_are_not_a_gate():
    assert g.find_gap([(20.0, -4.0, 0.6), (30.0, 4.0, 0.6)], (0.0, 0.0), NORTH) is None


def test_squeeze_then_through_uses_the_midpoint_route(monkeypatch):
    c = _console(monkeypatch, slots_for("v", 6)[:, :2] + [8.0, 0.0])
    c.transit("40", "0", "0.3")
    c.route = type("R", (), {"position": lambda self, t: np.array([9.0, 0.0]), "finished": lambda self, t: False})()
    c._tick_gap()
    c.formation = "column"
    c._tick_gap()
    assert c._gap["state"] == "through" and len(c.route.waypoints) == 3
    assert c.route.waypoints[1] == pytest.approx((20.0, 0.0))
