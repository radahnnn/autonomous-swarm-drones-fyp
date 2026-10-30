"""Software geofence and altitude layers (UNIT + agent/console wiring)."""
import threading
import time
from types import SimpleNamespace

import numpy as np
import pytest

from sitl.geofence import Geofence, altitude_layer_offset
from sitl.independent_agent_node import IndependentAgentNode
from sitl.independent_swarm_console import IndependentSwarmConsole
from sitl.run_metrics import RunMetrics


# ---------------------------------------------------------------- pure geometry
def test_circle_contains_clamps_and_reports_status():
    fence = Geofence("circle", radius_m=20.0, margin_m=2.0, max_alt_m=12.0)
    assert fence.contains((10, 0)) and fence.contains((18, 0), inner=False) and not fence.contains((18.5, 0), inner=True)
    np.testing.assert_allclose(fence.clamp((10, 0)), (10, 0))                 # already inside: unchanged
    clamped = fence.clamp((60, 0))
    np.testing.assert_allclose(clamped, (18.0, 0.0))                          # radius - margin, same direction
    assert fence.contains(clamped, inner=True)
    assert fence.status((0, 0), 5) == "inside" and fence.status((19, 0), 5) == "warn"
    assert fence.status((21, 0), 5) == "breach" and fence.status((0, 0), 13) == "breach"
    assert fence.status((0, 0), 11.7) == "warn"
    assert fence.distance_to_edge((15, 0)) == pytest.approx(5.0) and fence.distance_to_edge((25, 0)) < 0
    assert fence.clamp_altitude(30) == pytest.approx(10.0) and fence.clamp_altitude(5) == 5


def test_box_fence_clamps_each_axis_independently():
    fence = Geofence("box", box=(-10.0, -20.0, 30.0, 20.0), margin_m=2.0)
    np.testing.assert_allclose(fence.clamp((100, -100)), (28.0, -18.0))
    np.testing.assert_allclose(fence.clamp((5, 5)), (5, 5))
    assert fence.status((0, 0)) == "inside" and fence.status((29, 0)) == "warn" and fence.status((31, 0)) == "breach"
    assert fence.contains((0, 0), inner=True) and not fence.contains((-9, 0), inner=True)


def test_from_options_and_validation():
    assert Geofence.from_options() is None
    assert Geofence.from_options(radius=30).kind == "circle"
    box = Geofence.from_options(box="-10,-10,10,10", max_alt=15)
    assert box.kind == "box" and box.max_alt_m == 15 and box.describe()["box"] == [-10, -10, 10, 10]
    assert Geofence.from_options(max_alt=20).radius_m > 1e5                   # altitude-only fence
    for kwargs in (dict(radius=30, box="0,0,9,9"), dict(radius=1.0), dict(box="0,0,1"), dict(box="a,b,c,d"),
                   dict(box="0,0,3,3"), dict(radius=30, max_alt=0.2), dict(radius=30, margin=-1)):
        with pytest.raises(ValueError):
            Geofence.from_options(**kwargs)


def test_altitude_layers_cycle_by_drone_id_and_validate():
    assert [altitude_layer_offset(i, 3, 1.5) for i in range(6)] == [0.0, 1.5, 3.0, 0.0, 1.5, 3.0]
    assert altitude_layer_offset(7, 1, 1.5) == 0.0
    for args in ((0, 0, 1.5), (0, 6, 1.5), (0, 2, -1.0), (0, 2, 9.0), (0, 2, float("nan"))):
        with pytest.raises(ValueError):
            altitude_layer_offset(*args)


# ---------------------------------------------------------------- agent wiring
def make_agent(fence=None, layers=1, spacing=1.5, drone_id=1):
    node = object.__new__(IndependentAgentNode)
    node.drone_id = drone_id
    node.geofence = fence
    node.altitude_offset_m = altitude_layer_offset(drone_id, layers, spacing)
    node.fence_state, node.fence_breach_since, node.fence_breaches = "inside", None, 0
    node.global_origin = np.zeros(3)
    node.position = np.array([0.0, 0.0, -5.0])
    node.landing = False
    node.sent = []
    node.conn = SimpleNamespace(target_system=1, target_component=1, mav=SimpleNamespace(
        set_position_target_local_ned_send=lambda *a: node.sent.append(a)))
    node.landed = []
    node._command_land = lambda reason: node.landed.append(reason)
    return node


def sent_local(node):
    args = node.sent[-1]
    return np.array(args[5:8])        # x (north), y (east), z (down) after time/system/component/frame/mask


def test_every_setpoint_gets_its_altitude_layer_and_is_clamped_into_the_fence():
    node = make_agent(Geofence("circle", radius_m=20.0, margin_m=2.0, max_alt_m=12.0), layers=3, spacing=1.5, drone_id=2)
    node._send_position_target(np.array([50.0, 0.0, -5.0]), 5.0)
    north, east, down = sent_local(node)
    assert (north, east) == pytest.approx((18.0, 0.0))                       # clamped to radius - margin
    assert down == pytest.approx(-(5.0 + 3.0))                                # layer 2 -> +3.0 m
    node._send_position_target(np.array([1.0, 1.0, -5.0]), 20.0)
    assert sent_local(node)[2] == pytest.approx(-(12.0 - 2.0))                # ceiling minus margin
    node._send_position_target(np.array([1.0, 1.0, -5.0]), 5.0, apply_layer=False)   # hold uses current altitude
    assert sent_local(node)[2] == pytest.approx(-5.0)


def test_no_fence_and_one_layer_leave_targets_untouched():
    node = make_agent()
    node._send_position_target(np.array([123.0, -45.0, -5.0]), 5.0)
    np.testing.assert_allclose(sent_local(node), (123.0, -45.0, -5.0))


def test_breach_is_detected_counted_and_lands_only_if_it_persists(monkeypatch, capsys):
    clock = {"t": 100.0}
    monkeypatch.setattr(time, "monotonic", lambda: clock["t"])
    node = make_agent(Geofence("circle", radius_m=20.0, margin_m=2.0))
    node.position = np.array([10.0, 0.0, -5.0]); node._tick_geofence()
    assert node.fence_state == "inside" and node.fence_breaches == 0
    node.position = np.array([19.0, 0.0, -5.0]); node._tick_geofence()
    assert node.fence_state == "warn" and node.fence_breaches == 0
    node.position = np.array([22.0, 0.0, -5.0]); node._tick_geofence()
    assert node.fence_state == "breach" and node.fence_breaches == 1 and node.landed == []
    clock["t"] += IndependentAgentNode.FENCE_BREACH_LAND_S + 1.0
    node._tick_geofence()
    assert node.landed == ["geofence_breach"]
    node.position = np.array([5.0, 0.0, -5.0]); node._tick_geofence()          # recovered
    assert node.fence_state == "inside" and node.fence_breach_since is None


def test_agent_without_a_fence_ignores_the_tick():
    node = make_agent()
    node._tick_geofence()
    assert node.fence_state == "inside"


# ---------------------------------------------------------------- console wiring
def make_console(fence):
    c = object.__new__(IndependentSwarmConsole)
    c.count, c.formation, c.formation_slots = 4, "v", None
    c.goal_ne, c.target_alt_m = np.array([0.0, 0.0]), 5.0
    c.geofence = fence
    c.last_command_result = None
    c.mission_sequence = 0
    c.published = 0
    c._publish_intent = lambda: setattr(c, "published", c.published + 1)
    return c


def test_moves_that_would_leave_the_fence_are_refused_without_changing_the_goal(capsys):
    console = make_console(Geofence("circle", radius_m=14.0, margin_m=2.0))
    assert console._movement_blocked("w", 2.0) is False                     # V slots at +2 N still fit
    assert console._movement_blocked("w", 30.0) is True
    assert "geofence" in capsys.readouterr().out.lower()
    assert console.last_command_result["accepted"] is False
    np.testing.assert_allclose(console.goal_ne, (0.0, 0.0))
    assert console._movement_blocked("u", 100.0) is False                   # no altitude ceiling set
    capped = make_console(Geofence("circle", radius_m=50.0, max_alt_m=10.0))
    assert capped._movement_blocked("u", 2.0) is False and capped._movement_blocked("u", 8.0) is True


def test_a_formation_that_does_not_fit_is_described_per_drone():
    console = make_console(Geofence("circle", radius_m=6.0, margin_m=2.0))
    message = console._fence_problem(console._slot_points((0.0, 0.0), formation="line"))
    assert message and message.startswith("D") and "outside the geofence" in message
    assert make_console(None)._fence_problem(np.array([[999.0, 0.0]])) is None


def test_handle_routes_a_blocked_move_through_the_fence_check():
    console = make_console(Geofence("circle", radius_m=14.0, margin_m=2.0))
    console.mode_switch = None
    console.transition_protocol = False
    console.formation_hold = False
    assert console.handle("w 40") is True
    np.testing.assert_allclose(console.goal_ne, (0.0, 0.0))
    assert console.published == 0


def test_breach_events_reach_the_run_metrics_once_per_breach():
    console = make_console(Geofence("circle", radius_m=14.0))
    console.lock = threading.Lock()
    console.metrics = RunMetrics(2)
    console._fence_seen = {}
    base = {"operator_received_at": time.time(), "airborne": True, "position": [0.0, 0.0, -5.0]}
    for fence_state in ("inside", "breach", "breach", "inside", "breach"):
        console.latest = {0: {**base, "geofence": {"state": fence_state}}, 1: {**base, "geofence": {"state": "inside"}}}
        console._update_metrics()
    assert console.metrics.counters.get("geofence_breach") == 2


def test_dashboard_state_exposes_the_fence_for_the_map():
    from sitl.operator_dashboard import PAGE, dashboard_state
    from tests.test_operator_dashboard import make_session
    session = make_session(airborne=True)
    assert dashboard_state(session)["geofence"] is None
    session.geofence = Geofence("box", box=(-10, -10, 10, 10))
    state = dashboard_state(session)
    assert state["geofence"]["kind"] == "box" and state["geofence"]["box"] == [-10, -10, 10, 10]
    assert "state.geofence" in PAGE


def test_flying_swarm_altitude_command_has_a_floor(capsys):
    console = make_console(None)
    console.mode_switch = None
    console.transition_protocol = False
    console.formation_hold = False
    console.target_alt_m = 5.0
    console.handle("j 40")
    assert console.target_alt_m == IndependentSwarmConsole.MIN_FLIGHT_ALT_M == 2.0
    assert "limited" in capsys.readouterr().out.lower()
    console.handle("u 100")
    assert console.target_alt_m == 25.0                      # existing ceiling still applies
