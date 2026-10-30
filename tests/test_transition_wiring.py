"""INTEGRATION: real coordinator + real agent transition logic over an in-memory channel.

No SITL, no UDP, no MAVLink: agents are IndependentAgentNode objects with a fake vehicle/transport,
and time is a controllable clock. Evidence label: INTEGRATION (simulated transport), not SITL.
"""
import time
from types import SimpleNamespace

import numpy as np
import pytest

from sitl.formation_library import slots_for
from sitl.independent_agent_node import IndependentAgentNode
from sitl.independent_swarm_console import IndependentSwarmConsole
from sitl.transition_protocol import AgentTransition, CoordinatorTransition

N = 4
EPOCH = "operator-session-a"
T0 = 5000.0


class Clock:
    def __init__(self):
        self.t = T0

    def __call__(self):
        return self.t


class FakePeers:
    def fresh_link_ids(self, now):
        return list(range(N))


class FakeSocket:
    def __init__(self):
        self.sent = []

    def sendto(self, payload, endpoint):
        self.sent.append(payload)


class Feed:
    def __init__(self):
        self.queue = []

    def poll(self):
        messages, self.queue = self.queue, []
        return messages


class Swarm:
    """Four agents + one coordinator wired through a channel that tests can degrade."""

    def __init__(self, monkeypatch, drop=lambda kind, target: False, start_positions=None):
        self.clock = Clock()
        monkeypatch.setattr(time, "time", self.clock)
        monkeypatch.setattr(time, "monotonic", self.clock)
        monkeypatch.setattr("sitl.independent_agent_node.encode_message", lambda m, k: m)
        self.drop = drop
        self.events = []
        v = np.asarray(slots_for("v", N), dtype=float)
        self.centroid = np.array([20.0, 10.0])
        self.positions = (self.centroid + v[:, :2]) if start_positions is None else start_positions
        self.agents = [self._agent(i) for i in range(N)]
        self.console = self._console()
        self.publish_status(self.clock.t)
        for agent in self.agents:                    # first contact: every peer heard once
            agent._remember_peers([{"id": k, "position": np.array(self.positions[k]),
                                    "velocity": np.zeros(2), "age": 0.1}
                                   for k in range(N) if k != agent.drone_id], self.clock.t)

    def _agent(self, i):
        node = object.__new__(IndependentAgentNode)
        node.drone_id, node.count = i, N
        node.operator = Feed()
        node.status_socket, node.status_endpoint, node.operator_auth_key = FakeSocket(), ("h", 1), b"k"
        node.operator_clock_offset_s, node.clock_sync_epoch = 0.0, EPOCH
        node.transition = AgentTransition(i, N)
        node.peers, node.peer_ids, node._discovered = FakePeers(), [j for j in range(N) if j != i], []
        node._peer_memory = {}
        node.armed = node.airborne = True
        node.landing = False
        node.membership, node.membership_revision = None, 0
        node.formation = "v"
        node.all_slots = np.asarray(slots_for("v", N), dtype=float)
        node.slot = node.all_slots[i].copy()
        node.operator_formation_slots = None
        node.formation_transition_progress = 1.0
        node.last_coordinator_message_monotonic = self.clock.t
        node.position_at = self.clock.t
        node.position = np.array([*self.positions[i], -5.0])
        node.mission_lease = SimpleNamespace()   # unused here
        node.last_command = "ready"
        return node

    def _console(self):
        c = object.__new__(IndependentSwarmConsole)
        c.args = SimpleNamespace(adaptive_swarm=True, obstacle=[])
        c.count = N
        c.transition_protocol = True
        c.transition = CoordinatorTransition(N)
        import threading
        c.transition_lock, c.lock = threading.Lock(), threading.Lock()
        c.latest = {}
        c.clock_sync_message = {"epoch": EPOCH}
        c.goal_ne = self.centroid.copy()
        c.target_alt_m = 5.0
        c.formation = "v"
        c.formation_slots = None
        c.formation_start_slots = None
        c.formation_transition_s = 0.0
        c.formation_transition_deadline = None
        c.formation_resume_goal = None
        c.transition_pending = None
        c.formation_hold = False
        c.last_formation_result = {}
        c.land_requested = False
        c.event = lambda kind, **fields: self.events.append((kind, fields))
        c._send_to_agents = self._to_agents
        return c

    # ---- channel
    def _to_agents(self, message):
        for i, agent in enumerate(self.agents):
            if not self.drop(message["type"], i):
                agent.operator.queue.append(dict(message))
        return N

    def _from_agents(self):
        for i, agent in enumerate(self.agents):
            for message in agent.status_socket.sent:
                if message["type"] in ("transition_ack", "transition_abort"):
                    self.console._on_agent_transition_message(i, message)
            agent.status_socket.sent.clear()

    def publish_status(self, now):
        with self.console.lock:
            for i, agent in enumerate(self.agents):
                self.console.latest[i] = {
                    "operator_received_at": now, "timestamp": now, "position_age_s": 0.1,
                    "position": [*map(float, self.positions[i]), -5.0], "armed": True, "airborne": True,
                    "safety_state": "nominal", "peer_ids": [j for j in range(N) if j != i],
                    "peer_live_ids": [j for j in range(N) if j != i],
                    "transition": agent.transition.status(now),
                }

    # ---- one 0.1 s world step
    def step(self):
        self.clock.t += 0.1
        now = self.clock.t
        for i, agent in enumerate(self.agents):
            tr = agent.transition
            if tr.state in (AgentTransition.RUNNING, AgentTransition.ARRIVED, AgentTransition.HOLD) and tr.plan:
                alpha = {"RUNNING": tr.plan.alpha(now), "ARRIVED": 1.0,
                         "HOLD": tr.frozen_alpha or 0.0}[tr.state]
                self.positions[i] = tr.plan.position_at(i, alpha)    # ideal tracking
            agent.position = np.array([*self.positions[i], -5.0])
            agent.position_at = now
            agent.last_coordinator_message_monotonic = now
            # Production semantics: each tick only the packets that arrived THIS tick are reported
            # (here a rotating subset of one peer), so the agent must remember the others.
            others = [j for j in range(N) if j != i]
            j = others[int(round(now * 10)) % len(others)]
            agent._remember_peers([{"id": j, "position": np.array(self.positions[j]),
                                    "velocity": np.zeros(2), "age": 0.1}], now)
        for agent in self.agents:
            agent._receive_operator_messages_transition_only()
        for agent in self.agents:
            agent._tick_transition(now)
        self._from_agents()
        self.publish_status(now)
        self.console._tick_transition_protocol()
        self._from_agents()

    def run(self, seconds):
        for _ in range(int(seconds / 0.1)):
            self.step()


# Agents are built without a real operator socket; reuse the production dispatcher for the four
# transition message kinds only (everything else needs a live vehicle).
def _receive_transition_only(self):
    from sitl.independent_agent_node import _TRANSITION_KINDS
    for message in self.operator.poll():
        if message.get("type") in _TRANSITION_KINDS:
            self._handle_transition_message(message["type"], message)


IndependentAgentNode._receive_operator_messages_transition_only = _receive_transition_only


def request(swarm, style="line"):
    return swarm.console._set_formation(style)


def test_successful_transition_all_agents_start_together_and_finish_in_the_target_formation(monkeypatch):
    swarm = Swarm(monkeypatch)
    start = swarm.positions.copy()
    result = request(swarm)
    assert result["accepted"], result
    plan = swarm.console.transition.plan
    # nobody moves before the common start, and the lead time is respected
    while swarm.clock.t < plan.start_at - 0.2:
        swarm.step()
        np.testing.assert_allclose(swarm.positions, start)
    assert all(a.transition.state in (AgentTransition.STAGED, AgentTransition.RUNNING) for a in swarm.agents)
    swarm.run(plan.duration_s + 14.0)
    assert swarm.console.transition.state == CoordinatorTransition.IDLE
    assert swarm.console.formation == "line" and not swarm.console.formation_hold
    names = [e[0] for e in swarm.events]
    assert "transition_committed" in names and "transition_started" in names
    assert "transition_complete" in names and "transition_aborted" not in names
    for agent in swarm.agents:
        assert agent.transition.state == AgentTransition.IDLE
        assert agent.formation == "line"
        np.testing.assert_allclose(agent.all_slots, plan.target_slots)
    np.testing.assert_allclose(swarm.console.goal_ne, swarm.centroid)   # centre resumed


def test_missing_prepare_to_one_agent_cancels_before_anyone_moves(monkeypatch):
    swarm = Swarm(monkeypatch, drop=lambda kind, target: kind == "transition_prepare" and target == 3)
    start = swarm.positions.copy()
    assert request(swarm)["accepted"]
    swarm.run(12.0)
    np.testing.assert_allclose(swarm.positions, start)
    assert swarm.console.transition.state == CoordinatorTransition.IDLE
    assert all(a.transition.state == AgentTransition.IDLE for a in swarm.agents)
    aborted = [e for e in swarm.events if e[0] == "transition_aborted"]
    assert aborted and aborted[0][1]["moved"] is False and "ack_timeout" in aborted[0][1]["reason"]
    assert swarm.console.formation == "v" and not swarm.console.formation_hold


def test_lost_go_to_one_agent_holds_the_others_and_never_lands(monkeypatch):
    swarm = Swarm(monkeypatch, drop=lambda kind, target: kind == "transition_go" and target == 2)
    assert request(swarm)["accepted"]
    swarm.run(25.0)
    states = [a.transition.state for a in swarm.agents]
    assert states[2] == AgentTransition.IDLE
    assert [s for i, s in enumerate(states) if i != 2] == [AgentTransition.HOLD] * 3
    assert swarm.console.transition.state == CoordinatorTransition.HOLDING
    assert swarm.console.formation_hold and swarm.console.formation == "v"
    assert not swarm.console.land_requested                       # HOLD, not LAND
    assert all(a.armed and a.airborne for a in swarm.agents)
    moved = [e for e in swarm.events if e[0] == "transition_aborted"]
    assert moved and moved[0][1]["moved"] is True
    # an operator can re-plan from HOLD
    assert request(swarm, "column")["accepted"]


def test_commands_are_rejected_server_side_while_a_transition_is_busy(monkeypatch, capsys):
    swarm = Swarm(monkeypatch)
    assert request(swarm)["accepted"]
    goal = swarm.console.goal_ne.copy()
    second = request(swarm, "column")
    assert not second["accepted"] and second["code"] == "transition_active"
    assert swarm.console.handle("w 5") is True                    # consumed, but ignored
    np.testing.assert_allclose(swarm.console.goal_ne, goal)
    assert "transition active" in capsys.readouterr().out.lower()


def test_land_during_a_running_transition_aborts_it(monkeypatch):
    swarm = Swarm(monkeypatch)
    assert request(swarm)["accepted"]
    swarm.run(8.0)                                                # past start, mid-morph
    assert swarm.console.transition.state == CoordinatorTransition.RUNNING
    with swarm.console.transition_lock:
        actions = swarm.console.transition.abort("operator_land", time.time())
    swarm.console._dispatch_transition(actions)
    swarm.run(1.0)
    assert [a.transition.state for a in swarm.agents] == [AgentTransition.HOLD] * N


def test_peer_memory_keeps_supervision_alive_when_only_one_peer_packet_arrives_per_tick(monkeypatch):
    # Regression for the live-SITL finding: supervision used to see only this tick's packets and
    # aborted with peer_stale:<id> 0.2 s after the commit.
    swarm = Swarm(monkeypatch)
    assert request(swarm)["accepted"]
    swarm.run(26.0)
    assert not [e for e in swarm.events if e[0] == "transition_aborted"]
    assert "transition_complete" in [e[0] for e in swarm.events]


def test_abort_before_the_start_time_returns_everyone_to_idle_with_slots_untouched(monkeypatch):
    swarm = Swarm(monkeypatch)
    before = [a.all_slots.copy() for a in swarm.agents]
    start = swarm.positions.copy()
    assert request(swarm)["accepted"]
    while swarm.console.transition.state != CoordinatorTransition.COMMITTED:
        swarm.step()
    with swarm.console.transition_lock:
        actions = swarm.console.transition.abort("operator", time.time())
    swarm.console._dispatch_transition(actions)
    swarm.run(2.0)
    assert [a.transition.state for a in swarm.agents] == [AgentTransition.IDLE] * N
    for agent, slots in zip(swarm.agents, before):
        np.testing.assert_allclose(agent.all_slots, slots)
    np.testing.assert_allclose(swarm.positions, start)
    aborted = [e for e in swarm.events if e[0] == "transition_aborted"]
    assert aborted and aborted[0][1]["moved"] is False


def test_goal_is_claimed_only_after_the_common_start_so_the_swarm_cannot_drift_during_the_lead_time(monkeypatch):
    # Regression for the second live-SITL finding: moving the goal at prepare time translated the
    # whole swarm ~3 m before any morph started.
    swarm = Swarm(monkeypatch)
    goal_before = swarm.console.goal_ne.copy()
    assert request(swarm)["accepted"]
    plan = swarm.console.transition.plan
    np.testing.assert_allclose(swarm.console.goal_ne, goal_before)       # coordinator never moves it
    while swarm.clock.t < plan.start_at - 0.3:
        swarm.step()
        assert all(a._transition_goal(swarm.clock.t) is None for a in swarm.agents)
    swarm.run(1.0)                                                       # past start_at
    for agent in swarm.agents:
        np.testing.assert_allclose(agent._transition_goal(swarm.clock.t), plan.centroid_ne)
    swarm.run(plan.duration_s + 14.0)                                    # finished: goal released
    assert all(a._transition_goal(swarm.clock.t) is None for a in swarm.agents)
