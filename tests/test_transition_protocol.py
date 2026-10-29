"""UNIT/INTEGRATION (pure state machines + simulated lossy channel); no SITL involved."""
import random

import numpy as np
import pytest

from sitl.transition_protocol import (
    AgentContext, AgentTransition, AgentView, CoordinatorTransition, PeerObservation,
    TransitionParams, TransitionPlan,
)

N = 4
P = TransitionParams()
START = [(0.0, 3.0 * i, 0.0) for i in range(N)]
TARGET = [(6.0, 3.0 * i, 0.0) for i in range(N)]  # pure translation keeps 3 m pair clearance


def make_plan(transition_id=1, start_at=10.0, duration=8.0, target=TARGET, start=START):
    return TransitionPlan.build(transition_id, "line", (100.0, 50.0), start, target,
                                duration, start_at)


def ctx_for(plan, drone_id, alpha=0.0, **overrides):
    peers = {j: PeerObservation(tuple(plan.position_at(j, alpha)), 0.1)
             for j in range(N) if j != drone_id}
    base = dict(airborne_armed=True, own_fresh=True, peers_live=True,
                own_position_ne=tuple(plan.position_at(drone_id, alpha)),
                peers=peers, coordinator_age_s=0.2)
    base.update(overrides)
    return AgentContext(**base)


def staged_agent(plan=None, drone_id=1):
    plan = plan or make_plan()
    agent = AgentTransition(drone_id, N)
    reply = agent.on_prepare(plan.to_prepare_message(), 5.0, ctx_for(plan, drone_id))
    assert reply["ok"], reply
    assert agent.state == AgentTransition.STAGED
    return agent, plan


# ------------------------------------------------------------------ plan validation
def test_plan_round_trips_and_hash_detects_tampering():
    plan = make_plan()
    again = TransitionPlan.from_message(plan.to_prepare_message())
    assert again == plan and again.plan_hash == plan.plan_hash
    tampered = plan.to_prepare_message()
    tampered["duration_s"] = 9.0
    with pytest.raises(ValueError):
        TransitionPlan.from_message(tampered)


@pytest.mark.parametrize("bad", [
    dict(duration=0.0), dict(duration=float("nan")), dict(duration=61.0),
    dict(target=[(0, 0, 0), (1, 0, 0), (6, 0, 0), (9, 0, 0)]),  # spacing violation
    dict(target=[(0, 0, 0)] * 4),
])
def test_plan_rejects_unsafe_values(bad):
    with pytest.raises(ValueError):
        make_plan(**bad)


# ------------------------------------------------------------------ agent: prepare
@pytest.mark.parametrize("override,reason", [
    (dict(airborne_armed=False), "not_airborne_armed"),
    (dict(own_fresh=False), "stale_own_telemetry"),
    (dict(peers_live=False), "stale_peer_links"),
])
def test_agent_nacks_unsafe_context_without_changing_state(override, reason):
    plan = make_plan()
    agent = AgentTransition(1, N)
    reply = agent.on_prepare(plan.to_prepare_message(), 5.0, ctx_for(plan, 1, **override))
    assert not reply["ok"] and reply["reason"] == reason
    assert agent.state == AgentTransition.IDLE and agent.plan is None


def test_agent_nacks_start_too_soon_and_position_mismatch():
    plan = make_plan(start_at=10.0)
    agent = AgentTransition(1, N)
    assert agent.on_prepare(plan.to_prepare_message(), 9.5, ctx_for(plan, 1))["reason"] == "start_too_soon"
    far = ctx_for(plan, 1, own_position_ne=(120.0, 50.0))
    reply = agent.on_prepare(plan.to_prepare_message(), 5.0, far)
    assert reply["reason"] == "position_mismatch" and reply["own_position_error_m"] > P.start_tol_m
    assert agent.state == AgentTransition.IDLE


def test_agent_prepare_is_idempotent_and_rejects_a_different_plan_while_staged():
    agent, plan = staged_agent()
    again = agent.on_prepare(plan.to_prepare_message(), 5.1, ctx_for(plan, 1))
    assert again["ok"] and agent.state == AgentTransition.STAGED
    other = make_plan(transition_id=2)
    reply = agent.on_prepare(other.to_prepare_message(), 5.2, ctx_for(other, 1))
    assert not reply["ok"] and reply["reason"] == "already_in_transition"
    assert agent.plan == plan


def test_agent_rejects_garbage_and_wrong_size_prepare():
    agent = AgentTransition(1, N)
    for message in ({}, {"version": 1}, None, "x"):
        assert not agent.on_prepare(message, 5.0, ctx_for(make_plan(), 1))["ok"]
    small = TransitionPlan.build(1, "line", (0, 0), START[:3], TARGET[:3], 8.0, 10.0)
    assert agent.on_prepare(small.to_prepare_message(), 5.0, ctx_for(make_plan(), 1))["reason"] == "plan_invalid"
    assert agent.state == AgentTransition.IDLE


def test_replayed_closed_transition_id_is_rejected():
    agent, plan = staged_agent()
    agent.on_abort({"transition_id": 1, "reason": "x", "freeze_at": 5.0}, 5.0)
    assert agent.state == AgentTransition.IDLE and agent.last_closed_id == 1
    reply = agent.on_prepare(plan.to_prepare_message(), 5.5, ctx_for(plan, 1))
    assert not reply["ok"] and reply["reason"] == "stale_transition"


# ------------------------------------------------------------------ agent: go / run
def test_staged_agent_does_not_move_until_go_and_starts_at_common_time():
    agent, plan = staged_agent()
    np.testing.assert_allclose(agent.current_slots(9.0), plan.start_slots)
    assert agent.on_go({"transition_id": 1, "plan_hash": plan.plan_hash, "start_at": 10.0}, 9.5) is None
    assert agent.state == AgentTransition.RUNNING
    np.testing.assert_allclose(agent.current_slots(10.0), plan.start_slots)
    np.testing.assert_allclose(agent.current_slots(18.0), plan.target_slots)
    mid = agent.current_slots(14.0)
    np.testing.assert_allclose(mid, (np.asarray(plan.start_slots) + np.asarray(plan.target_slots)) / 2)


def test_go_with_wrong_hash_id_or_start_is_ignored():
    agent, plan = staged_agent()
    for message in ({"transition_id": 1, "plan_hash": "0" * 64, "start_at": 10.0},
                    {"transition_id": 2, "plan_hash": plan.plan_hash, "start_at": 10.0},
                    {"transition_id": 1, "plan_hash": plan.plan_hash, "start_at": 11.0}, {}, None):
        assert agent.on_go(message, 9.5) is None
        assert agent.state == AgentTransition.STAGED


def test_late_go_discards_the_plan_and_reports_nack():
    agent, plan = staged_agent()
    reply = agent.on_go({"transition_id": 1, "plan_hash": plan.plan_hash, "start_at": 10.0}, 10.5)
    assert reply["reason"] == "go_late" and not reply["ok"]
    assert agent.state == AgentTransition.IDLE


def test_staged_agent_self_discards_when_no_go_arrives():
    agent, plan = staged_agent()
    assert agent.tick(10.5, ctx_for(plan, 1)) == []
    events = agent.tick(10.0 + P.no_go_grace_s + 0.1, ctx_for(plan, 1))
    assert events and events[0][1] == "no_go" and agent.state == AgentTransition.IDLE


def test_duplicate_go_is_harmless():
    agent, plan = staged_agent()
    go = {"transition_id": 1, "plan_hash": plan.plan_hash, "start_at": 10.0}
    agent.on_go(go, 9.5)
    assert agent.on_go(go, 11.0) is None and agent.state == AgentTransition.RUNNING


# ------------------------------------------------------------------ agent: abort / hold
def running_agent():
    agent, plan = staged_agent()
    agent.on_go({"transition_id": 1, "plan_hash": plan.plan_hash, "start_at": 10.0}, 9.5)
    return agent, plan


def test_abort_while_running_freezes_slots_at_the_common_freeze_time_and_never_lands():
    agent, plan = running_agent()
    agent.on_abort({"transition_id": 1, "reason": "sep", "freeze_at": 14.0}, 14.6)
    assert agent.state == AgentTransition.HOLD
    frozen = agent.current_slots(20.0)
    np.testing.assert_allclose(frozen, plan.slots_at(plan.alpha(14.0)))
    np.testing.assert_allclose(agent.current_slots(25.0), frozen)  # does not continue


def test_abort_for_other_transition_or_garbage_is_ignored():
    agent, plan = running_agent()
    for message in ({"transition_id": 9, "reason": "x", "freeze_at": 12.0}, {}, None):
        agent.on_abort(message, 12.0)
        assert agent.state == AgentTransition.RUNNING


@pytest.mark.parametrize("name,kwargs,expected", [
    ("own", dict(airborne_armed=False), "own_fault"),
    ("coordinator", dict(coordinator_age_s=3.5), "coordinator_lost"),
])
def test_local_supervision_holds_and_broadcasts_abort(name, kwargs, expected):
    agent, plan = running_agent()
    actions = agent.tick(13.0, ctx_for(plan, 1, alpha=plan.alpha(13.0), **kwargs))
    assert agent.state == AgentTransition.HOLD and agent.hold_reason == expected
    sends = [a[1] for a in actions if a[0] == "send"]
    assert sends and sends[0]["type"] == "transition_abort" and sends[0]["reason"] == expected


def test_local_supervision_peer_stale_off_plan_and_close_approach():
    plan = make_plan()
    for mutate, prefix in (
        (lambda c: c.peers.__setitem__(2, PeerObservation(c.peers[2].position_ne, 2.0)), "peer_stale:2"),
        (lambda c: c.peers.pop(0), "peer_stale:0"),
        (lambda c: c.peers.__setitem__(3, PeerObservation((0.0, 0.0), 0.1)), "peer_off_plan:3"),
    ):
        agent, _ = running_agent()
        ctx = ctx_for(plan, 1, alpha=plan.alpha(13.0))
        mutate(ctx)
        agent.tick(13.0, ctx)
        assert agent.hold_reason == prefix, agent.hold_reason
    agent, _ = running_agent()
    ctx = ctx_for(plan, 1, alpha=plan.alpha(13.0))
    own = ctx.own_position_ne
    # a peer that is on-plan-tolerant but physically too close triggers separation hold
    near = PeerObservation((own[0] + 1.0, own[1]), 0.1)
    expected = plan.position_at(2, plan.alpha(13.0))
    loose = TransitionParams(peer_track_tol_m=1000.0)
    agent.params = loose
    ctx.peers[2] = near
    agent.tick(13.0, ctx)
    assert agent.hold_reason == "separation:2" and np.linalg.norm(near.position_ne - expected) > 0


def test_agent_in_hold_accepts_only_a_newer_replan():
    agent, plan = running_agent()
    agent.on_abort({"transition_id": 1, "reason": "x", "freeze_at": 13.0}, 13.0)
    assert agent.on_prepare(plan.to_prepare_message(), 14.0, ctx_for(plan, 1))["reason"] == "stale_transition"
    newer = make_plan(transition_id=2, start_at=20.0)
    here = tuple(plan.position_at(1, 0.0))
    assert agent.on_prepare(newer.to_prepare_message(), 14.0, ctx_for(newer, 1, own_position_ne=here))["ok"]
    assert agent.state == AgentTransition.STAGED


def test_arrived_agent_closes_on_done_or_after_timeout():
    agent, plan = running_agent()
    agent.tick(18.5, ctx_for(plan, 1, alpha=1.0))
    assert agent.state == AgentTransition.ARRIVED
    agent.on_done({"transition_id": 1, "plan_hash": plan.plan_hash})
    assert agent.state == AgentTransition.IDLE
    agent, plan = running_agent()
    agent.tick(18.5, ctx_for(plan, 1, alpha=1.0))
    agent.tick(10.0 + 8.0 + P.arrived_close_s + 0.1, ctx_for(plan, 1, alpha=1.0))
    assert agent.state == AgentTransition.IDLE


# ------------------------------------------------------------------ coordinator
def good_views(plan, now, state="IDLE"):
    alpha = plan.alpha(now)
    return {i: AgentView(0.3, True, True, tuple(plan.position_at(i, alpha)), state, plan.transition_id)
            for i in range(N)}


def ack(plan, i, ok=True, reason=None):
    return {"transition_id": plan.transition_id, "plan_hash": plan.plan_hash, "drone_id": i,
            "ok": ok, "reason": reason}


def types(actions):
    return [a[1]["type"] for a in actions if a[0] == "send"]


def test_coordinator_happy_path_prepare_ack_go_run_settle_done():
    plan, c = make_plan(), CoordinatorTransition(N)
    assert types(c.begin(plan, 5.0)) == ["transition_prepare"] and c.busy
    for i in range(N):
        c.on_ack(ack(plan, i), 5.2)
    assert types(c.tick(5.3, {})) == ["transition_go"] and c.state == c.COMMITTED
    c.tick(9.9, {})  # may legitimately resend go
    assert c.state == c.COMMITTED
    c.tick(10.0, {})
    assert c.state == c.RUNNING
    assert c.tick(14.0, good_views(plan, 14.0, "RUNNING")) == []
    c.tick(18.0, good_views(plan, 18.0, "ARRIVED"))
    assert c.state == c.SETTLING
    assert c.tick(19.0, good_views(plan, 19.0, "ARRIVED")) == []
    done = c.tick(19.0 + P.dwell_s, good_views(plan, 21.0, "ARRIVED"))
    assert types(done) == ["transition_done"] and c.state == c.IDLE and not c.busy


def test_coordinator_aborts_before_start_when_an_ack_is_missing_and_nothing_moves():
    plan, c = make_plan(), CoordinatorTransition(N)
    c.begin(plan, 5.0)
    for i in range(N - 1):
        c.on_ack(ack(plan, i), 5.2)
    assert types(c.tick(5.6, {})) == ["transition_prepare"]  # retransmit
    actions = c.tick(5.0 + P.ack_timeout_s + 0.1, {})
    assert types(actions) == ["transition_abort"] and c.state == c.IDLE
    assert actions[1][2]["moved"] is False and "ack_timeout:[3]" in actions[0][1]["reason"]


def test_coordinator_aborts_on_nack_and_on_late_acks():
    plan, c = make_plan(), CoordinatorTransition(N)
    c.begin(plan, 5.0)
    out = c.on_ack(ack(plan, 2, ok=False, reason="stale_peer_links"), 5.1)
    assert types(out) == ["transition_abort"] and c.state == c.IDLE
    plan2, c2 = make_plan(2, start_at=7.0), CoordinatorTransition(N)
    c2.last_id = 1
    c2.begin(plan2, 5.0)
    for i in range(N):
        c2.on_ack(ack(plan2, i), 6.5)
    assert c2.tick(6.5, {})[0][1]["reason"] == "late_acks"


def test_coordinator_ignores_acks_for_other_plans_and_bad_ids():
    plan, c = make_plan(), CoordinatorTransition(N)
    c.begin(plan, 5.0)
    other = make_plan(transition_id=1, duration=9.0)
    c.on_ack(ack(other, 0), 5.1)
    c.on_ack({**ack(plan, 0), "drone_id": 99}, 5.1)
    c.on_ack({**ack(plan, 0), "ok": "yes"}, 5.1)
    c.on_ack(None, 5.1)
    assert c.acks == {}


@pytest.mark.parametrize("mutate,expected", [
    (lambda v: v.__setitem__(2, AgentView(5.0, position_ne=v[2].position_ne, state="RUNNING", transition_id=1)), "stale_status:2"),
    (lambda v: setattr(v[1], "armed_airborne", False), "not_airborne:1"),
    (lambda v: setattr(v[0], "safety_nominal", False), "unsafe:0"),
    (lambda v: setattr(v[3], "state", "STAGED"), "agent_not_running:3"),
    (lambda v: setattr(v[1], "position_ne", v[0].position_ne), "separation:0,1"),
])
def test_running_supervision_aborts_to_hold(mutate, expected):
    plan, c = make_plan(), CoordinatorTransition(N)
    c.begin(plan, 5.0)
    for i in range(N):
        c.on_ack(ack(plan, i), 5.2)
    c.tick(5.3, {}); c.tick(10.0, {})
    views = good_views(plan, 14.0, "RUNNING")
    mutate(views)
    actions = c.tick(14.0, views)
    assert c.state == c.HOLDING and not c.busy
    assert actions[0][1]["reason"].startswith(expected) and actions[1][2]["moved"] is True


def test_overrun_aborts_and_hold_timeout_asks_for_operator_without_landing():
    plan, c = make_plan(), CoordinatorTransition(N)
    c.begin(plan, 5.0)
    for i in range(N):
        c.on_ack(ack(plan, i), 5.2)
    c.tick(5.3, {}); c.tick(10.0, {})
    late = 10.0 + 8.0 + P.overrun_s + 0.5
    views = good_views(plan, late - 5.0, "RUNNING")  # agents never arrived
    assert c.tick(late, views)[0][1]["reason"] == "overrun"
    events = c.tick(late + P.hold_timeout_s + 1.0, {})
    assert events[0][1] == "needs_operator" and "transition_abort" not in types(events)
    assert "land" not in str(events).lower()


def test_conflicting_begin_is_refused_and_replan_allowed_after_hold():
    plan, c = make_plan(), CoordinatorTransition(N)
    c.begin(plan, 5.0)
    with pytest.raises(RuntimeError, match="transition_active"):
        c.begin(make_plan(2), 5.1)
    c.abort("operator", 5.2)
    c.begin(make_plan(2, start_at=12.0), 6.0)
    assert c.next_id() == 3


# ------------------------------------------------------------------ lossy-channel simulation
def run_sim(seed, loss, kill_go_for=None, dt=0.1, horizon=40.0):
    rng = random.Random(seed)
    plan = make_plan(start_at=9.0)
    coord = CoordinatorTransition(N)
    agents = [AgentTransition(i, N) for i in range(N)]
    to_agents, to_coord, ever_running, committed = [], [], set(), False
    completed = False
    now = 5.0

    def lossy(box, msg):
        if rng.random() >= loss:
            box.append((now + dt, msg))

    def send_all(actions):
        for a in actions:
            if a[0] == "send":
                for i in range(N):
                    if not (kill_go_for == i and a[1]["type"] == "transition_go"):
                        lossy(to_agents, (i, a[1]))
            elif a[1] == "transition_complete":
                nonlocal completed
                completed = True

    send_all(coord.begin(plan, now))
    while now < 5.0 + horizon:
        due = [m for m in to_agents if m[0] <= now]
        to_agents[:] = [m for m in to_agents if m[0] > now]
        for _, (i, msg) in due:
            agent, t = agents[i], msg["type"]
            alpha = plan.alpha(now) if agent.state in ("RUNNING", "ARRIVED") else 0.0
            ctx = ctx_for(plan, i, alpha=alpha)
            if t == "transition_prepare":
                lossy(to_coord, agent.on_prepare(msg, now, ctx))
            elif t == "transition_go":
                reply = agent.on_go(msg, now)
                if reply:
                    lossy(to_coord, reply)
            elif t == "transition_abort":
                agent.on_abort(msg, now)
            elif t == "transition_done":
                agent.on_done(msg)
        ready = [m for m in to_coord if m[0] <= now]
        to_coord[:] = [m for m in to_coord if m[0] > now]
        for _, msg in ready:
            if msg["type"] == "transition_ack":
                send_all(coord.on_ack(msg, now))
            elif msg["type"] == "transition_abort":
                send_all(coord.on_agent_abort(msg, now))
        for i, agent in enumerate(agents):
            alpha = {"RUNNING": plan.alpha(now), "ARRIVED": 1.0,
                     "HOLD": agent.frozen_alpha or 0.0}.get(agent.state, 0.0)
            acts = agent.tick(now, ctx_for(plan, i, alpha=alpha))
            for a in acts:
                if a[0] == "send":
                    lossy(to_coord, a[1])
            if agent.state in ("RUNNING", "ARRIVED"):
                ever_running.add(i)
        views = {}
        for i, agent in enumerate(agents):
            alpha = {"RUNNING": plan.alpha(now), "ARRIVED": 1.0,
                     "HOLD": agent.frozen_alpha or 0.0}.get(agent.state, 0.0)
            views[i] = AgentView(0.3, True, True, tuple(plan.position_at(i, alpha)),
                                 agent.state, plan.transition_id)
        actions = coord.tick(now, views)
        if coord.state in (coord.COMMITTED, coord.RUNNING, coord.SETTLING):
            committed = True
        send_all(actions)
        now += dt
    return coord, agents, ever_running, committed, completed


@pytest.mark.parametrize("loss", [0.0, 0.1, 0.3, 0.6])
def test_lossy_channel_never_starts_unless_all_acked_and_never_leaves_agents_mid_transition(loss):
    outcomes = {"completed": 0, "aborted": 0}
    for seed in range(60):
        coord, agents, ever_running, committed, completed = run_sim(seed, loss)
        if ever_running:
            assert committed, f"seed {seed}: an agent moved without a full set of acks"
        if completed:
            outcomes["completed"] += 1
            assert coord.state == coord.IDLE
            assert all(a.state == AgentTransition.IDLE for a in agents), f"seed {seed}"
        else:
            outcomes["aborted"] += 1
            assert not coord.busy, f"seed {seed}: coordinator stuck in {coord.state}"
            assert all(a.state in (AgentTransition.IDLE, AgentTransition.HOLD) for a in agents), \
                f"seed {seed}: {[a.state for a in agents]}"
    if loss == 0.0:
        assert outcomes["completed"] == 60


def test_lost_go_to_one_agent_aborts_everyone_into_hold_not_land():
    coord, agents, ever_running, committed, completed = run_sim(1, 0.0, kill_go_for=2)
    assert committed and not completed
    assert agents[2].state == AgentTransition.IDLE  # never started
    assert [a.state for i, a in enumerate(agents) if i != 2] == [AgentTransition.HOLD] * 3
    assert coord.state == coord.HOLDING
