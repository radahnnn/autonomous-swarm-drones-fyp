"""Mission routes: every drone carries the route, so the swarm keeps flying it without a coordinator."""
import time
from types import SimpleNamespace

import numpy as np
import pytest

from sitl.geofence import Geofence
from sitl.independent_agent_node import IndependentAgentNode
from sitl.independent_swarm_console import IndependentSwarmConsole
from sitl.mission_intent import MissionIntent, MissionLease
from sitl.route import Route, square_patrol


# ------------------------------------------------------------------ geometry
def test_route_follows_the_legs_at_constant_speed_and_holds_at_the_ends():
    route = Route(((0.0, 0.0), (10.0, 0.0), (10.0, 10.0)), start_at=100.0, speed_m_s=0.4)
    np.testing.assert_allclose(route.position(50.0), (0, 0))               # before the start: first waypoint
    np.testing.assert_allclose(route.position(100.0), (0, 0))
    np.testing.assert_allclose(route.position(112.5), (5, 0))
    np.testing.assert_allclose(route.position(137.5), (10, 5))
    np.testing.assert_allclose(route.position(999.0), (10, 10))            # after the end: last waypoint
    assert route.length_m == 20.0 and route.duration_s == pytest.approx(50.0)
    assert not route.finished(140.0) and route.finished(150.0)


def test_goal_is_continuous_in_time_so_no_drone_is_ever_asked_to_jump():
    route = square_patrol((3.0, -2.0), 10.0, start_at=10.0, speed_m_s=0.4)
    previous = route.position(0.0)
    for t in np.arange(0.0, 200.0, 0.05):
        current = route.position(float(t))
        assert np.linalg.norm(current - previous) <= 0.4 * 0.05 + 1e-9    # never faster than the speed
        previous = current


def test_square_patrol_approaches_from_the_centre_then_loops_forever():
    route = square_patrol((0.0, 0.0), 10.0, start_at=0.0, speed_m_s=0.4)
    np.testing.assert_allclose(route.position(0.0), (0, 0))                # starts exactly at the current goal
    corner = np.array([5.0, 5.0])
    t0 = np.linalg.norm(corner) / 0.4                                      # first time at the first corner
    np.testing.assert_allclose(route.position(t0), corner)
    lap = 40.0 / 0.4
    for offset in (0.0, 7.5, 21.0, 33.0):
        np.testing.assert_allclose(route.position(t0 + offset), route.position(t0 + offset + lap), atol=1e-9)
        np.testing.assert_allclose(route.position(t0 + offset), route.position(t0 + offset + 5 * lap), atol=1e-6)
    assert not route.finished(1e6)
    assert max(np.linalg.norm(route.position(t)) for t in np.arange(0, 800, 0.5)) <= np.hypot(5, 5) + 1e-9


@pytest.mark.parametrize("kwargs", [
    dict(waypoints=((0.0, 0.0),), start_at=0.0, speed_m_s=0.5),
    dict(waypoints=((0.0, 0.0), (0.0, 0.0)), start_at=0.0, speed_m_s=0.5),
    dict(waypoints=((0.0, 0.0), (1.0, float("nan"))), start_at=0.0, speed_m_s=0.5),
    dict(waypoints=((0.0, 0.0), (5.0, 0.0)), start_at=0.0, speed_m_s=0.0),
    dict(waypoints=((0.0, 0.0), (5.0, 0.0)), start_at=0.0, speed_m_s=9.0),
    dict(waypoints=((0.0, 0.0), (5.0, 0.0)), start_at=0.0, speed_m_s=0.5),   # above the agents' 0.45 m/s setpoint slew
    dict(waypoints=((0.0, 0.0), (5.0, 0.0)), start_at=float("inf"), speed_m_s=0.5),
    dict(waypoints=((0.0, 0.0), (5.0, 0.0)), start_at=0.0, speed_m_s=0.5, loop_start=1),
    dict(waypoints=tuple((float(i), 0.0) for i in range(40)), start_at=0.0, speed_m_s=0.5),
])
def test_unsafe_routes_are_rejected(kwargs):
    with pytest.raises(ValueError):
        Route(**kwargs)


def test_square_patrol_rejects_a_tiny_square():
    with pytest.raises(ValueError):
        square_patrol((0, 0), 1.0, 0.0, 0.5)


# ------------------------------------------------------------------ transport
def test_route_travels_inside_the_mission_intent_and_old_intents_still_parse():
    route = square_patrol((0.0, 0.0), 10.0, start_at=105.0, speed_m_s=0.4)
    intent = MissionIntent(1, 100.0, 130.0, (0.0, 0.0), **{
        "route_waypoints": route.waypoints, "route_start_at": route.start_at,
        "route_speed_m_s": route.speed_m_s, "route_loop_start": route.loop_start})
    parsed = MissionIntent.from_dict(intent.to_dict(), 100.0)
    assert parsed.route() == route
    plain = MissionIntent.from_dict(MissionIntent(2, 100.0, 130.0, (1.0, 2.0)).to_dict(), 100.0)
    assert plain.route() is None and plain.route_loop_start == -1


def test_a_malformed_route_invalidates_the_whole_intent():
    good = MissionIntent(1, 100.0, 130.0, (0.0, 0.0)).to_dict()
    for bad in ({"route_waypoints": [[0, 0]], "route_speed_m_s": 0.3},
                {"route_waypoints": [[0, 0], [5, 0]], "route_speed_m_s": 99.0},
                {"route_waypoints": [[0, "x"], [5, 0]], "route_speed_m_s": 0.3},
                {"route_waypoints": [[0, 0], [5, 0]], "route_speed_m_s": 0.5, "route_loop_start": True}):
        with pytest.raises(ValueError):
            MissionIntent.from_dict({**good, **bad}, 100.0)


# ------------------------------------------------------------------ agent: same function of the same clock
def node_for(drone_id, route, transition_goal=None, now=100.0):
    node = object.__new__(IndependentAgentNode)
    node.drone_id = drone_id
    node.operator_clock_offset_s = 0.0
    node.agent = SimpleNamespace(cached_mission_intent=None)
    node._transition_goal = lambda t: transition_goal
    intent = MissionIntent(1, now, now + 30.0, (0.0, 0.0), route_waypoints=route.waypoints,
                           route_start_at=route.start_at, route_speed_m_s=route.speed_m_s,
                           route_loop_start=-1 if route.loop_start is None else route.loop_start)
    return node, intent


def test_every_drone_computes_the_same_goal_from_the_shared_clock():
    route = square_patrol((0.0, 0.0), 12.0, start_at=100.0, speed_m_s=0.4)
    goals = []
    for drone in range(10):
        node, intent = node_for(drone, route)
        goals.append(node._goal_override(intent, 137.3))
    assert all(np.allclose(goals[0], g) for g in goals) and goals[0] is not None


def test_a_formation_transition_has_priority_over_the_route_and_no_route_means_no_override():
    route = square_patrol((0.0, 0.0), 12.0, start_at=100.0, speed_m_s=0.4)
    node, intent = node_for(0, route, transition_goal=np.array([7.0, 7.0]))
    np.testing.assert_allclose(node._goal_override(intent, 130.0), (7.0, 7.0))
    plain = MissionIntent(2, 100.0, 130.0, (1.0, 2.0))
    node, _ = node_for(0, route)
    assert node._goal_override(plain, 130.0) is None


def test_decentralized_and_hybrid_keep_following_the_route_after_the_coordinator_is_lost(monkeypatch):
    clock = {"t": 1000.0}
    monkeypatch.setattr(time, "time", lambda: clock["t"])
    monkeypatch.setattr(time, "monotonic", lambda: clock["t"])
    route = square_patrol((0.0, 0.0), 12.0, start_at=1000.0, speed_m_s=0.4)
    for mode, should_continue in (("decentralized", True), ("hybrid", True), ("centralized", False)):
        node, intent = node_for(0, route, now=clock["t"])
        node.control_mode, node.coordinator_connected = mode, False
        node.current_mission = intent.to_dict()
        node.last_coordinator_message_monotonic = clock["t"] - 5.0        # coordinator silent
        node.peers, node.peer_ids = SimpleNamespace(fresh_link_ids=lambda now: [1, 2]), [1, 2]
        node.mission_lease = MissionLease()
        node.mission_lease.accept(intent.to_dict(), clock["t"], clock["t"])
        for _ in range(14):                                               # 70 s, stepped like the real loop
            clock["t"] += 5.0
            node._renew_lease_if_autonomous()
        active = node.mission_lease.active(clock["t"])
        assert (active is not None) is should_continue, mode
        if should_continue:
            before = node._goal_override(active, clock["t"])
            clock["t"] += 5.0
            after = node._goal_override(active, clock["t"])
            assert np.linalg.norm(after - before) > 1.0                    # still moving along the route
        clock["t"] = 1000.0


# ------------------------------------------------------------------ console commands
def make_console(fence=None):
    c = object.__new__(IndependentSwarmConsole)
    c.count, c.formation, c.formation_slots = 4, "v", None
    c.goal_ne, c.target_alt_m = np.array([0.0, 0.0]), 5.0
    c.geofence, c.route = fence, None
    c.mode_switch, c.transition_protocol, c.formation_hold = None, False, False
    c.last_command_result, c.metrics = None, None
    c.args = SimpleNamespace(adaptive_swarm=True, obstacle=[])
    c.events = []
    c.event = lambda kind, **f: c.events.append(kind)
    c.published = 0
    c._publish_intent = lambda: setattr(c, "published", c.published + 1)
    return c


def test_patrol_starts_a_route_and_blocks_conflicting_commands(capsys):
    console = make_console()
    result = console.start_patrol(["square", "10", "0.4"])
    assert result["accepted"] and console.route is not None and console.published == 1
    assert console.route.start_at > time.time() and "route_start" in console.events
    assert console.start_patrol(["square", "10"])["accepted"] is False        # already active
    assert console._movement_blocked("w", 2.0) is True and "route is active" in capsys.readouterr().out
    refused = console._set_formation("line")
    assert refused["accepted"] is False and refused["code"] == "route_active"


def test_patrol_is_refused_when_it_does_not_fit_the_geofence_or_is_malformed():
    tight = make_console(Geofence("circle", radius_m=12.0, margin_m=2.0))
    refused = tight.start_patrol(["square", "20"])
    assert refused["accepted"] is False and refused["code"] == "geofence" and tight.route is None
    assert make_console().start_patrol(["circle", "10"])["accepted"] is False
    assert make_console().start_patrol(["square", "0.5"])["accepted"] is False
    assert make_console().start_patrol(["square", "10", "0.6"])["accepted"] is False     # faster than the setpoint slew


def test_route_stop_holds_exactly_where_the_route_is_with_no_jump():
    console = make_console()
    console.start_patrol(["square", "10", "0.4"])
    console.route = Route(console.route.waypoints, time.time() - 6.0, 0.4, console.route.loop_start)   # 6 s in
    expected = console.route.position(time.time())
    console.route_command(["stop"])
    assert console.route is None and console.published == 2
    np.testing.assert_allclose(console.goal_ne, expected, atol=0.05)
    assert "route_stop" in console.events


def test_dashboard_exposes_the_route_for_the_map():
    from sitl.operator_dashboard import PAGE, dashboard_state
    from tests.test_operator_dashboard import make_session
    session = make_session(airborne=True)
    assert dashboard_state(session)["route"] is None
    session.route = square_patrol((0, 0), 10, 0.0, 0.4)
    state = dashboard_state(session)
    assert len(state["route"]["waypoints"]) == 6 and state["route"]["loop_start"] == 1
    assert "state.route" in PAGE
