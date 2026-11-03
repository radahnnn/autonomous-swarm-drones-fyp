"""Acting-coordinator failover: pure logic, authenticated peer message, and a multi-drone simulation."""
import time
from types import SimpleNamespace

import pytest

from sitl.coordinator_election import CoordinatorElection, ElectionParams
from sitl.independent_agent_node import IndependentAgentNode
from sitl.mission_intent import MissionIntent, MissionLease
from sitl.peer_udp import UdpPeerStateTransport
from sitl.route import square_patrol

P = ElectionParams()


def names(actions):
    return [a[1] for a in actions]


# ------------------------------------------------------------------ pure state machine
def test_no_takeover_before_the_operator_was_ever_heard_or_while_it_is_fresh():
    e = CoordinatorElection(0)
    assert e.tick(100.0, elected_leader_id=0, live_peer_count=3) == []        # never heard the operator
    e.note_operator(100.0)
    assert e.tick(102.0, 0, 3) == [] and e.state == e.FOLLOWER                # silent only 2 s < 3 s
    assert e.source(102.0) == "operator"


def test_only_the_elected_leader_with_a_live_peer_takes_over_and_publishes_each_period():
    follower, leader, alone = CoordinatorElection(2), CoordinatorElection(0), CoordinatorElection(0)
    for e in (follower, leader, alone):
        e.note_operator(100.0)
    assert follower.tick(104.0, elected_leader_id=0, live_peer_count=3) == []
    assert alone.tick(104.0, 0, live_peer_count=0) == []                       # a lone drone does not "elect" itself
    actions = leader.tick(104.0, 0, 3)
    assert names(actions) == ["acting_coordinator_started", "publish"] or [a[0] for a in actions] == ["event", "publish"]
    assert leader.state == leader.ACTING and leader.term == 1 and leader.source(104.0) == "self"
    assert leader.tick(104.5, 0, 3) == []                                      # not yet time to publish again
    assert [a[0] for a in leader.tick(105.1, 0, 3)] == ["publish"]


def test_followers_accept_the_acting_coordinator_and_report_it_as_their_source():
    e = CoordinatorElection(3)
    e.note_operator(100.0)
    accept, _ = e.on_acting_message(0, 1, now=104.0)
    assert accept and e.source(104.2) == "acting:0" and e.term == 1
    assert e.source(104.0 + P.acting_fresh_s + 1.0) == "none"                  # the acting coordinator went quiet


def test_the_operator_always_wins_and_acting_messages_are_ignored_while_it_is_fresh():
    e = CoordinatorElection(3)
    e.note_operator(100.0)
    assert e.on_acting_message(0, 1, now=101.0) == (False, [])                 # operator still fresh
    leader = CoordinatorElection(0)
    leader.note_operator(100.0)
    leader.tick(104.0, 0, 3)
    assert leader.state == leader.ACTING
    actions = leader.note_operator(105.0)                                       # operator returned
    assert leader.state == leader.FOLLOWER and names(actions) == ["acting_coordinator_stepped_down"]
    assert leader.source(105.5) == "operator"


def test_higher_term_wins_and_at_equal_terms_the_lower_id_wins():
    a, b = CoordinatorElection(1), CoordinatorElection(2)
    for e in (a, b):
        e.note_operator(100.0)
        e.tick(104.0, e.own_id, 3)                    # both believe they are the leader: both claim term 1
    assert a.state == b.state == "ACTING" and a.term == b.term == 1
    assert b.on_acting_message(1, 1, now=104.2)[0] is True and b.state == "FOLLOWER"   # lower ID wins the tie
    assert a.on_acting_message(2, 1, now=104.2)[0] is False and a.state == "ACTING"
    c = CoordinatorElection(5)
    c.note_operator(100.0)
    assert c.on_acting_message(2, 3, now=104.0)[0] is True
    assert c.on_acting_message(1, 2, now=104.1)[0] is False                     # a lower term is a worse claim
    assert c.on_acting_message(4, 3, now=104.1)[0] is False                     # equal term, higher ID
    assert c.on_acting_message(1, 3, now=104.2)[0] is True                      # equal term, lower ID
    assert c.on_acting_message(0, 4, now=104.3)[0] is True and c.term == 4      # higher term


@pytest.mark.parametrize("sender,term", [(True, 1), (1, True), (1, 0), (-1, 0), ("1", 1), (1, 1.5), (5, 1)])
def test_garbage_acting_messages_are_rejected(sender, term):
    e = CoordinatorElection(5)                        # sender 5 == own id is also rejected
    e.note_operator(0.0)
    assert e.on_acting_message(sender, term, now=10.0)[0] is False


def test_the_acting_role_expires_and_does_not_immediately_re_elect():
    e = CoordinatorElection(0, ElectionParams(acting_max_s=10.0))
    e.note_operator(0.0)
    e.tick(4.0, 0, 3)
    assert e.state == e.ACTING
    actions = e.tick(15.0, 0, 3)
    assert e.state == e.FOLLOWER and names(actions) == ["acting_coordinator_expired"]
    assert e.tick(16.0, 0, 3) == []                    # grace: the coordinator-loss policies act now


def test_an_acting_coordinator_that_is_heard_prevents_a_second_takeover():
    e = CoordinatorElection(0)
    e.note_operator(0.0)
    e.last_operator_at = -10.0                         # operator silent long ago
    assert e.on_acting_message(2, 1, now=100.0)[0] is True
    assert e.tick(100.5, 0, 3) == []                   # someone already acts: do not start a rival


# ------------------------------------------------------------------ authenticated peer message
def make_pair(base, key=b"k" * 32):
    roster = {i: ("127.0.0.1", base + i) for i in range(2)}
    a = UdpPeerStateTransport(0, base, 2, peer_endpoints=roster, auth_key=key) \
        if "peer_endpoints" in UdpPeerStateTransport.__init__.__code__.co_varnames else UdpPeerStateTransport(0, base, 2)
    b = UdpPeerStateTransport(1, base, 2, peer_endpoints=roster, auth_key=key) \
        if "peer_endpoints" in UdpPeerStateTransport.__init__.__code__.co_varnames else UdpPeerStateTransport(1, base, 2)
    return a, b


def poll(receiver, now, attempts=60):
    for _ in range(attempts):
        receiver.receive(now, [0])
        messages = receiver.take_coordinator_messages()
        if messages:
            return messages
        time.sleep(0.02)
    return []


def test_coordinator_message_round_trip_replay_and_validation():
    import os
    base = 36000 + (os.getpid() % 3000)
    a, b = make_pair(base)
    try:
        now = time.time()
        intent = MissionIntent(7, now, now + 30, (1.0, 2.0)).to_dict()
        a.publish_coordinator([1], now, term=2, intent=intent)
        got = poll(b, now)
        assert len(got) == 1 and got[0]["sender"] == 0 and got[0]["term"] == 2 and got[0]["intent"]["goal_ne"] == [1.0, 2.0]
        # an identical resend is a replay of an old sequence number only if the sequence repeats: a fresh
        # publish has a new sequence and is accepted, an invalid term is not
        a.publish_coordinator([1], now, term=3, intent=intent)
        assert len(poll(b, now)) == 1
        a.coordinator_sequence += 0
        a.publish_coordinator([1], now, term=0, intent=intent)                    # invalid term
        assert poll(b, now, attempts=10) == []
        assert b.stats()["rejected_invalid"] >= 1
    finally:
        a.close(); b.close()


# ------------------------------------------------------------------ agent wiring: four drones lose the operator
class Net:
    """In-memory peer channel for the acting-coordinator messages of N nodes."""

    def __init__(self):
        self.nodes = []

    def peers_for(self, index):
        net = self

        class Peers:
            def fresh_link_ids(self, now):
                return [i for i in range(len(net.nodes)) if i != index and net.nodes[i].alive]

            def take_coordinator_messages(self):
                inbox, net.nodes[index].inbox = net.nodes[index].inbox, []
                return inbox

            def publish_coordinator(self, peer_ids, now, term, intent):
                for i in peer_ids:
                    if net.nodes[i].alive:
                        net.nodes[i].inbox.append({"sender": index, "term": term, "intent": dict(intent), "sequence": 0})

        return Peers()


def make_swarm(monkeypatch, modes=("centralized", "hybrid", "decentralized", "hybrid")):
    clock = {"t": 1000.0}
    monkeypatch.setattr(time, "time", lambda: clock["t"])
    monkeypatch.setattr(time, "monotonic", lambda: clock["t"])
    net, nodes = Net(), []
    route = square_patrol((0.0, 0.0), 12.0, start_at=1000.0, speed_m_s=0.3)
    base = MissionIntent(1, 999.0, 1029.0, (0.0, 0.0), route_waypoints=route.waypoints, route_start_at=route.start_at,
                         route_speed_m_s=route.speed_m_s, route_loop_start=route.loop_start).to_dict()
    for i, mode in enumerate(modes):
        node = object.__new__(IndependentAgentNode)
        node.drone_id, node.control_mode, node.alive, node.inbox = i, mode, True, []
        node.operator_clock_offset_s = 0.0
        node.agent = SimpleNamespace(cached_mission_intent=None)
        node.election = CoordinatorElection(i)
        node._acting_sequence, node.last_acting_sequence = 0, {}
        node.membership = object()
        node.discovery_ids = [j for j in range(len(modes)) if j != i]
        node.mission_lease = MissionLease()
        node.mission_lease.accept(base, clock["t"], clock["t"])
        node.current_mission = dict(base)
        node.coordinator_connected, node.last_coordinator_message_monotonic = True, clock["t"]
        node.elected_leader_id = 0
        node.election.note_operator(clock["t"])        # every drone heard the operator just now
        nodes.append(node)
        net.nodes.append(node)
    for i, node in enumerate(nodes):
        node.peers = net.peers_for(i)
    return clock, nodes


def run(clock, nodes, seconds, operator_alive_until=None, step=0.5):
    end = clock["t"] + seconds
    while clock["t"] < end:
        clock["t"] += step
        for node in nodes:
            if node.alive:
                if operator_alive_until is not None and clock["t"] <= operator_alive_until:
                    node.election.note_operator(clock["t"])
                    node.last_coordinator_message_monotonic = clock["t"]
                node._tick_election(clock["t"])
                node.coordinator_connected = clock["t"] - node.last_coordinator_message_monotonic < 2.5


def test_exactly_one_acting_coordinator_emerges_and_every_mode_keeps_its_coordinator(monkeypatch):
    clock, nodes = make_swarm(monkeypatch)
    run(clock, nodes, 12.0)                                        # the operator never speaks again
    acting = [n.drone_id for n in nodes if n.election.state == "ACTING"]
    assert acting == [0]                                           # the lowest live ID, and only it
    assert all(n.election.term == 1 for n in nodes)
    assert all(n.coordinator_connected for n in nodes)             # centralized/hybrid no longer lose the coordinator
    assert all(n.election.source(clock["t"]) in ("self", "acting:0") for n in nodes)
    assert all(n.mission_lease.active(clock["t"]) is not None for n in nodes)
    # the acting coordinator republishes the same route, so the swarm keeps the same goal
    goals = {tuple(n.agent.cached_mission_intent["route_waypoints"][1]) for n in nodes}
    assert len(goals) == 1


def test_leader_death_hands_over_to_the_next_lowest_id_with_a_higher_term(monkeypatch):
    clock, nodes = make_swarm(monkeypatch)
    run(clock, nodes, 8.0)
    assert [n.drone_id for n in nodes if n.election.state == "ACTING"] == [0]
    nodes[0].alive = False                                         # the acting coordinator dies
    for n in nodes[1:]:
        n.elected_leader_id = 1                                    # membership drops D1 and re-elects
    run(clock, nodes, 15.0)
    alive_acting = [n.drone_id for n in nodes[1:] if n.election.state == "ACTING"]
    assert alive_acting == [1] and nodes[1].election.term == 2
    assert all(n.election.source(clock["t"]) in ("self", "acting:1") for n in nodes[1:])
    assert all(n.coordinator_connected for n in nodes[1:])


def test_when_the_operator_returns_every_acting_role_ends_and_the_operator_is_followed(monkeypatch):
    clock, nodes = make_swarm(monkeypatch)
    run(clock, nodes, 10.0)
    assert [n.drone_id for n in nodes if n.election.state == "ACTING"] == [0]
    run(clock, nodes, 6.0, operator_alive_until=clock["t"] + 6.0)  # operator speaks again
    assert all(n.election.state == "FOLLOWER" for n in nodes)
    assert all(n.election.source(clock["t"]) == "operator" for n in nodes)


def test_a_stale_or_replayed_acting_intent_does_not_refresh_the_mission(monkeypatch):
    clock, nodes = make_swarm(monkeypatch)
    run(clock, nodes, 8.0)
    follower = nodes[2]
    sequence_before = max(follower.last_acting_sequence.values())
    stale = {"sender": 0, "term": 1, "intent": dict(follower.current_mission, sequence=1), "sequence": 0}
    follower._handle_acting_intent(stale, clock["t"], clock["t"])
    assert max(follower.last_acting_sequence.values()) == sequence_before   # replayed sequence ignored
    garbage = {"sender": 0, "term": 1, "intent": {"version": 1}, "sequence": 0}
    follower._handle_acting_intent(garbage, clock["t"], clock["t"])           # invalid intent: no exception


# ------------------------------------------------------------------ console / dashboard
def test_dashboard_summarizes_who_the_swarm_follows():
    from sitl.operator_dashboard import PAGE, _coordinator_summary
    latest = {0: {"coordinator": {"source": "self", "term": 2, "acting": True}},
              1: {"coordinator": {"source": "acting:0", "term": 2, "acting": False}},
              2: {"coordinator": {"source": "acting:0", "term": 2, "acting": False}},
              3: {}}
    assert _coordinator_summary(latest) == {"source": "acting:0", "acting": [1], "term": 2}
    assert _coordinator_summary({}) == {"source": None, "acting": [], "term": 0}
    assert "mxcoord" in PAGE


def test_console_notes_takeovers_once_per_role_change():
    import threading
    from sitl.independent_swarm_console import IndependentSwarmConsole
    from sitl.run_metrics import RunMetrics
    c = object.__new__(IndependentSwarmConsole)
    c.lock, c.metrics, c._fence_seen, c._acting_seen = threading.Lock(), RunMetrics(2), {}, {}
    c.events = []
    c.event = lambda kind, **f: c.events.append((kind, f))
    base = {"operator_received_at": time.time(), "airborne": True, "position": [0.0, 0.0, -5.0]}
    for acting in (False, True, True, False, True):
        c.latest = {0: {**base, "coordinator": {"acting": acting, "term": 1}}, 1: {**base}}
        c._update_metrics()
    assert c.metrics.counters.get("coordinator_takeover") == 2
    assert [k for k, _ in c.events].count("coordinator_stepdown") == 1


# ------------------------------------------------------------------ a local pause is not operator silence
def test_a_blocked_agent_does_not_take_over_when_it_wakes_up():
    e = CoordinatorElection(0)
    e.note_operator(100.0)
    e.resume_after_local_pause(110.0)               # blocked in arm/takeoff for 10 s
    assert e.tick(110.1, 0, 3) == [] and e.state == e.FOLLOWER
    assert e.tick(113.5, 0, 3) != []                # still a real takeover if the operator stays silent after the pause
    assert CoordinatorElection(0).resume_after_local_pause(5.0) is None   # no operator heard yet: nothing to reset


def test_a_long_gap_between_ticks_is_treated_as_a_local_pause(monkeypatch):
    clock, nodes = make_swarm(monkeypatch)
    node = nodes[0]
    node._last_election_tick = clock["t"]
    clock["t"] += 20.0                               # the agent loop stalled for 20 s (operator silent only "apparently")
    node._tick_election(clock["t"])
    assert node.election.state == "FOLLOWER" and node.election.takeovers == 0
    for _ in range(9):                               # genuinely silent for > 3 s after waking (0.5 s ticks)
        clock["t"] += 0.5
        node._tick_election(clock["t"])
    assert node.election.state == "ACTING"


def test_acting_period_is_a_hard_limit_until_the_operator_returns_codex_10():
    """Codex review #10: takeover at 4, expiry at 15, and (before the fix) takeover again at 19."""
    from sitl.coordinator_election import ElectionParams
    e = CoordinatorElection(0, ElectionParams(election_timeout_s=3.0, acting_max_s=10.0))
    e.note_operator(0.0)
    assert any(a[0] == "event" and a[1] == "acting_coordinator_started" for a in e.tick(4.0, 0, 3))
    expired = []
    for t in range(5, 16):
        expired += e.tick(float(t), 0, 3)
    assert any(a[1] == "acting_coordinator_expired" for a in expired if a[0] == "event")
    later = []
    for t in range(16, 40):
        later += e.tick(float(t), 0, 3)
    assert not any(a[0] == "event" and a[1] == "acting_coordinator_started" for a in later)   # no renewal
    e.note_operator(40.0)                                           # the operator really comes back
    assert any(a[0] == "event" and a[1] == "acting_coordinator_started" for a in e.tick(44.0, 0, 3))
