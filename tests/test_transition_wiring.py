"""INTEGRATION: real coordinator + real agent transition logic over an in-memory channel.

No SITL, no UDP, no MAVLink: agents are IndependentAgentNode objects with a fake vehicle/transport,
and time is a controllable clock. Evidence label: INTEGRATION (simulated transport), not SITL.
"""
import threading
import time
from types import SimpleNamespace

import numpy as np
import pytest

from sitl.formation_library import slots_for
from sitl.independent_agent_node import IndependentAgentNode
from sitl.coordinator_election import CoordinatorElection
from sitl.mode_switch import AgentModeSwitch, CoordinatorModeSwitch
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
        node.mode_switch = AgentModeSwitch(i, "hybrid")
        node.election = CoordinatorElection(i)
        node.control_mode = "hybrid"
        node.agent = SimpleNamespace(mode="hybrid")
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
        c.transition_lock, c.lock = threading.Lock(), threading.Lock()
        c.latest = {}
        c.clock_sync_message = {"epoch": EPOCH}
        c.clock_uncertainty_s = {i: 0.005 for i in range(N)}
        c.time_scale = 1.0
        c.last_clock_sync_measurement = time.monotonic()
        c.goal_ne = self.centroid.copy()
        c.target_alt_m = 5.0
        c.formation = "v"
        c.formation_slots = None
        c.formation_start_slots = None
        c.formation_transition_s = 0.0
        c.formation_transition_deadline = None
        c.formation_resume_goal = None
        c.transition_pending = None
        c.mode_switch = CoordinatorModeSwitch(N, "hybrid")
        c.mode_lock = threading.Lock()
        c.control_mode = "hybrid"
        c.last_command_result = None
        c.mission_sequence = 0
        c.formation_transition_started_at = None
        c.intent_enabled = True
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
                elif message["type"] in ("mode_ack", "mode_abort"):
                    self.console._on_agent_mode_message(i, message)
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
                    "mode_switch": agent.mode_switch.status(now),
                    "control_mode": agent.control_mode,
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
            agent._tick_mode_switch(now)
        self._from_agents()
        self.publish_status(now)
        self.console._tick_transition_protocol()
        self.console._tick_mode_switch_protocol()
        self._from_agents()

    def run(self, seconds):
        for _ in range(int(seconds / 0.1)):
            self.step()


# Agents are built without a real operator socket; reuse the production dispatcher for the four
# transition message kinds only (everything else needs a live vehicle).
def _receive_transition_only(self):
    from sitl.independent_agent_node import _MODE_KINDS, _TRANSITION_KINDS
    for message in self.operator.poll():
        if message.get("type") in _TRANSITION_KINDS:
            self._handle_transition_message(message["type"], message)
        elif message.get("type") in _MODE_KINDS:
            self._handle_mode_message(message["type"], message)


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



# ====================================================================== in-flight mode switching
def modes(swarm):
    return [a.control_mode for a in swarm.agents]


def test_mode_switch_changes_every_agent_together_with_a_smooth_blend(monkeypatch):
    swarm = Swarm(monkeypatch)
    assert swarm.console.set_control_mode("decentralized")["accepted"]
    plan = swarm.console.mode_switch.plan
    weights = []
    while swarm.clock.t < plan.start_at + plan.ramp_s + 3.0:
        swarm.step()
        if swarm.clock.t < plan.start_at - 0.2:
            assert modes(swarm) == ["hybrid"] * N                      # nobody moves early
        weights.append([a.mode_switch.weights(swarm.clock.t)[2] for a in swarm.agents])
    # every agent runs the same ramp at the same time, and it never decreases
    assert all(max(row) - min(row) < 1e-6 for row in weights)
    flat = [row[0] for row in weights if row[0] < 1.0]
    assert flat == sorted(flat) and flat[0] >= 0.0
    swarm.run(8.0)
    assert modes(swarm) == ["decentralized"] * N
    assert swarm.console.control_mode == "decentralized" and not swarm.console.mode_switch.busy
    assert [a.mode_switch.state for a in swarm.agents] == [AgentModeSwitch.IDLE] * N
    names = [e[0] for e in swarm.events]
    assert "mode_committed" in names and "mode_switch_complete" in names and "mode_switch_aborted" not in names


def test_chain_of_switches_through_all_three_modes(monkeypatch):
    swarm = Swarm(monkeypatch)
    for target in ("centralized", "decentralized", "hybrid", "centralized"):
        assert swarm.console.set_control_mode(target)["accepted"], target
        swarm.run(14.0)
        assert modes(swarm) == [target] * N and swarm.console.control_mode == target
    assert [a.agent.mode for a in swarm.agents] == ["centralized"] * N


def test_missing_prepare_to_one_agent_cancels_the_switch_and_nothing_changes(monkeypatch):
    swarm = Swarm(monkeypatch, drop=lambda kind, target: kind == "mode_prepare" and target == 3)
    assert swarm.console.set_control_mode("centralized")["accepted"]
    swarm.run(10.0)
    assert modes(swarm) == ["hybrid"] * N and swarm.console.control_mode == "hybrid"
    aborted = [e for e in swarm.events if e[0] == "mode_switch_aborted"]
    assert aborted and aborted[0][1]["started"] is False and "ack_timeout" in aborted[0][1]["reason"]


def test_lost_go_to_one_agent_rolls_the_whole_swarm_back_to_the_old_mode(monkeypatch):
    swarm = Swarm(monkeypatch, drop=lambda kind, target: kind == "mode_go" and target == 2)
    assert swarm.console.set_control_mode("decentralized")["accepted"]
    swarm.run(20.0)
    assert modes(swarm) == ["hybrid"] * N                      # nobody is left in the new mode
    assert [a.mode_switch.state for a in swarm.agents] == [AgentModeSwitch.IDLE] * N
    assert swarm.console.control_mode == "hybrid" and not swarm.console.mode_switch.busy
    names = [e[0] for e in swarm.events]
    assert "mode_switch_aborted" in names and "mode_switch_rolled_back" in names
    assert "mode_switch_complete" not in names


def test_mode_switch_is_gated_against_conflicting_commands(monkeypatch, capsys):
    swarm = Swarm(monkeypatch)
    assert swarm.console.set_control_mode("decentralized")["accepted"]
    assert swarm.console.set_control_mode("centralized")["code"] == "mode_switch_active"
    refused = swarm.console._set_formation("line")
    assert not refused["accepted"] and refused["code"] == "mode_switch_active"
    swarm.console.handle("formation line")
    assert "switch in progress" in capsys.readouterr().out.lower()
    assert swarm.console.handle("w 1") is True                  # movement is still allowed
    swarm.run(14.0)
    assert modes(swarm) == ["decentralized"] * N


def test_mode_switch_is_refused_during_a_formation_transition(monkeypatch):
    swarm = Swarm(monkeypatch)
    assert request(swarm)["accepted"]                           # formation transition prepared
    refused = swarm.console.set_control_mode("centralized")
    assert not refused["accepted"] and refused["code"] == "transition_active"


def test_land_during_a_mode_switch_rolls_it_back(monkeypatch):
    swarm = Swarm(monkeypatch)
    assert swarm.console.set_control_mode("decentralized")["accepted"]
    plan = swarm.console.mode_switch.plan
    while swarm.clock.t < plan.start_at + 1.0:
        swarm.step()
    swarm.console.mode_switch_abort_for_test = True
    with swarm.console.mode_lock:
        actions = swarm.console.mode_switch.abort("operator_land", time.time())
    swarm.console._dispatch_mode(actions)
    swarm.run(10.0)
    assert modes(swarm) == ["hybrid"] * N and not swarm.console.mode_switch.busy


def test_unknown_or_unchanged_mode_requests_are_rejected_cleanly(monkeypatch):
    swarm = Swarm(monkeypatch)
    assert not swarm.console.set_control_mode("warp")["accepted"]
    unchanged = swarm.console.set_control_mode("hybrid")
    assert unchanged["accepted"] and unchanged.get("unchanged")
    assert not swarm.console.mode_switch.busy


# ------------------------------------------------ per-mode coordinator-loss policy (agent side)
def _policy_node(mode, monkeypatch):
    clock = Clock()
    monkeypatch.setattr(time, "time", clock)
    monkeypatch.setattr(time, "monotonic", clock)
    node = object.__new__(IndependentAgentNode)
    node.drone_id, node.control_mode = 0, mode
    node.operator_clock_offset_s = 0.0
    node.agent = SimpleNamespace(cached_mission_intent=None)
    node.coordinator_connected = False
    node.current_mission = None                                  # set below from the accepted intent
    node.last_coordinator_message_monotonic = clock.t - 10.0
    node.peers, node.peer_ids = FakePeers(), [1, 2]
    node.position = np.array([1.0, 2.0, -5.0])
    node.central_hold_since = None
    from sitl.mission_intent import MissionIntent, MissionLease
    node.mission_lease = MissionLease()
    intent = MissionIntent(1, clock.t - 1, clock.t + 3, (0.0, 0.0)).to_dict()
    node.mission_lease.accept(intent, clock.t, clock.t)
    node.current_mission = intent
    node.sent, node.landed = [], []
    node._send_position_target = lambda target, altitude, apply_layer=True: node.sent.append((target.copy(), altitude))
    node._command_land = lambda reason: node.landed.append(reason)
    return node, clock


@pytest.mark.parametrize("mode", ["decentralized", "hybrid"])
def test_decentralized_and_hybrid_keep_flying_the_last_mission_without_a_coordinator(mode, monkeypatch):
    from sitl.mission_intent import MissionIntent
    node, clock = _policy_node(mode, monkeypatch)
    for _ in range(400):                                        # stepped by 1 s (the real loop runs at ~10 Hz)
        clock.t += 1.0
        node._renew_lease_if_autonomous()
        if clock.t - node.last_coordinator_message_monotonic > IndependentAgentNode.AUTONOMY_MAX_S:
            break
        # the lease AND the agent's cached copy stay valid far beyond the original 3 s / 60 s lifetimes
        assert node.mission_lease.active(clock.t) is not None
        MissionIntent.from_dict(node.agent.cached_mission_intent, clock.t)
    assert clock.t - node.last_coordinator_message_monotonic > IndependentAgentNode.AUTONOMY_MAX_S
    clock.t += 40.0                                             # autonomy window exhausted: lease lapses
    node._renew_lease_if_autonomous()
    assert node.mission_lease.active(clock.t) is None


def test_autonomy_requires_at_least_one_live_peer(monkeypatch):
    node, clock = _policy_node("hybrid", monkeypatch)
    node.peers = SimpleNamespace(fresh_link_ids=lambda now: [])
    clock.t += 30.0
    node._renew_lease_if_autonomous()
    assert node.mission_lease.active(clock.t) is None


def test_centralized_holds_then_lands_when_the_coordinator_is_lost(monkeypatch):
    node, clock = _policy_node("centralized", monkeypatch)
    assert node._centralized_fail_stop(clock.t) is True and node.landed == []
    target, altitude = node.sent[-1]
    np.testing.assert_allclose(target, node.position)           # hovering where it is
    assert altitude == 5.0
    assert node._centralized_fail_stop(clock.t + IndependentAgentNode.CENTRAL_HOLD_LAND_S + 1) is True
    assert node.landed == ["coordinator_lost_centralized"]
    node.coordinator_connected = True                           # coordinator back: normal guidance
    assert node._centralized_fail_stop(clock.t + 30) is False and node.central_hold_since is None


def test_other_modes_never_use_the_centralized_fail_stop(monkeypatch):
    for mode in ("decentralized", "hybrid"):
        node, clock = _policy_node(mode, monkeypatch)
        assert node._centralized_fail_stop(clock.t) is False and node.sent == []


def test_start_lead_shorter_than_clock_budget_is_refused(monkeypatch):
    swarm = Swarm(monkeypatch)
    console = swarm.console
    console.clock_uncertainty_s = {i: 2.0 for i in range(N)}      # 2 s of offset uncertainty: no 3-4 s lead is safe
    assert "clock budget" in console._lead_too_short(console.transition.params.lead_s)
    refused = console.set_control_mode("decentralized")
    assert not refused["accepted"] and "clock budget" in refused["message"]
    console.clock_uncertainty_s = {i: 0.005 for i in range(N)}
    assert console._lead_too_short(console.transition.params.lead_s) is None


def test_time_scale_slows_plans_and_stretches_timeouts():
    from sitl.formation_transition import plan_formation_transition
    from sitl.transition_protocol import TransitionParams, scaled_params
    rng = np.random.default_rng(1)
    positions = np.column_stack([np.linspace(-8, 8, 6), np.zeros(6)]) + rng.normal(0, 0.05, (6, 2))
    normal = plan_formation_transition(positions, "v", max_transition_s=500.0)
    slow = plan_formation_transition(positions, "v", max_transition_s=500.0, time_scale=0.25)
    assert slow.duration_s == pytest.approx(normal.duration_s * 4.0)
    np.testing.assert_allclose(slow.slots_by_drone, normal.slots_by_drone)
    assert scaled_params(1.0) == TransitionParams()
    assert scaled_params(0.25).overrun_s == pytest.approx(TransitionParams().overrun_s * 4.0)
    with pytest.raises(ValueError):
        plan_formation_transition(positions, "v", time_scale=0.0)
    with pytest.raises(ValueError):
        scaled_params(1.5)


def test_mode_switch_is_allowed_while_holding_after_an_abort(monkeypatch):
    swarm = Swarm(monkeypatch)
    swarm.console.formation_hold = True                    # stationary HOLD after an aborted transition
    assert not swarm.console.transition.busy
    assert swarm.console.set_control_mode("decentralized")["accepted"]


def test_transient_pre_start_cancellation_is_retried_but_not_forever(monkeypatch, capsys):
    swarm = Swarm(monkeypatch)
    console = swarm.console
    started = []

    class FakeTimer:
        def __init__(self, delay, fn, args=()):
            self.args, self.daemon = args, False
        def start(self):
            started.append(self.args)

    monkeypatch.setattr("sitl.independent_swarm_console.threading.Timer", FakeTimer)
    abort = ("event", "transition_aborted", {"reason": "agent_abort:1:peer_off_plan:7", "moved": False})
    for _ in range(3):
        console.transition_pending = ("line", None, None)
        console._dispatch_transition([abort])
    assert [a[1] for a in started] == ["line", "line"]           # two retries, then it gives up
    console.transition_pending = ("column", None, None)
    console._dispatch_transition([("event", "transition_aborted", {"reason": "separation:1,2", "moved": False})])
    assert [a[1] for a in started] == ["line", "line"]           # a safety reason is never retried automatically


def test_start_time_is_taken_after_the_settle_wait_codex_4(monkeypatch):
    """Codex review #4: a 5 s settle used to move the clock from 5000 to 5005 while the plan still started at 5004."""
    swarm = Swarm(monkeypatch)
    swarm.run(3.0)

    def slow_settle(*a, **k):
        swarm.clock.t += 5.0                     # waiting costs 5 s of (fake) time
        swarm.publish_status(swarm.clock.t)      # statuses stay fresh after the wait
        return True, 0, 0.0

    swarm.console._wait_settled = slow_settle
    after = swarm.clock.t + 5.0
    result = swarm.console._set_formation("line")
    assert result is None or swarm.console.last_formation_result["accepted"]
    plan = swarm.console.transition.plan
    assert plan is not None and plan.start_at >= after + swarm.console.transition.params.lead_s - 1e-6



def test_retry_keeps_heading_and_spacing_and_is_dropped_after_a_newer_command_codex_2(monkeypatch, capsys):
    swarm = Swarm(monkeypatch)
    console = swarm.console
    timers = []

    class FakeTimer:
        def __init__(self, delay, fn, args=()):
            self.fn, self.args, self.daemon = fn, args, False
        def start(self):
            timers.append(self)

    monkeypatch.setattr("sitl.independent_swarm_console.threading.Timer", FakeTimer)
    console._last_formation_request = ("column", 0.5, 8.0)
    console.transition_pending = ("column", None, None)
    console._dispatch_transition([("event", "transition_aborted", {"reason": "nack:3:stale_peer_links", "moved": False})])
    generation, style, heading, spacing = timers[0].args
    assert (style, heading, spacing) == ("column", 0.5, 8.0)                 # full request kept
    calls = []
    console._set_formation = lambda *a, **k: calls.append(a)
    console._new_mission_generation("land")                                  # a newer command (e.g. LAND)
    timers[0].fn(*timers[0].args)
    assert calls == [] and "dropped" in capsys.readouterr().out


def test_concurrent_publishes_never_reuse_a_sequence_number_codex_2(monkeypatch):
    import threading as th
    swarm = Swarm(monkeypatch)
    console = swarm.console
    sent = []
    console._send_to_agents = lambda message: sent.append(message["intent"]["sequence"])
    console.event = lambda *a, **k: None
    workers = [th.Thread(target=console._publish_intent) for _ in range(20)]
    for w in workers:
        w.start()
    for w in workers:
        w.join()
    assert len(sent) == 20 and len(set(sent)) == 20
