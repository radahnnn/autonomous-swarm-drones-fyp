"""UNIT + simulated-channel tests for synchronized in-flight mode switching."""
import random

import pytest

from sitl.mode_switch import (
    AgentModeSwitch, CoordinatorModeSwitch, ModeContext, ModeParams, ModePlan, ModeView, smooth_weight,
)

N = 5
P = ModeParams()
OK = ModeContext()


def plan(switch_id=1, a="hybrid", b="decentralized", start=10.0, ramp=2.0):
    return ModePlan.build(switch_id, a, b, start, ramp)


def staged(mode="hybrid", drone=1):
    agent = AgentModeSwitch(drone, mode)
    reply = agent.on_prepare(plan(a=mode).to_prepare_message(), 5.0, OK)
    assert reply["ok"], reply
    return agent, plan(a=mode)


def scheduled():
    agent, pl = staged()
    assert agent.on_go({"switch_id": 1, "plan_hash": pl.plan_hash, "start_at": 10.0}, 9.0) is None
    assert agent.state == AgentModeSwitch.SCHEDULED
    return agent, pl


def switching():
    agent, pl = scheduled()
    agent.tick(10.5, OK)
    assert agent.state == AgentModeSwitch.SWITCHING
    return agent, pl


# ---------------------------------------------------------------- plan
def test_plan_round_trip_and_validation():
    pl = plan()
    assert ModePlan.from_message(pl.to_prepare_message()) == pl
    bad = pl.to_prepare_message(); bad["ramp_s"] = 3.0
    with pytest.raises(ValueError):
        ModePlan.from_message(bad)
    for args in ((0, "hybrid", "decentralized", 1.0, 2.0), (1, "hybrid", "hybrid", 1.0, 2.0),
                 (1, "hybrid", "warp", 1.0, 2.0), (1, "hybrid", "decentralized", float("nan"), 2.0),
                 (1, "hybrid", "decentralized", 1.0, 0.0), (1, "hybrid", "decentralized", 1.0, 99.0)):
        with pytest.raises(ValueError):
            ModePlan.build(*args)


def test_smooth_weight_is_monotone_and_flat_at_the_ends():
    values = [smooth_weight(i / 20) for i in range(21)]
    assert values[0] == 0.0 and values[-1] == 1.0
    assert all(b >= a for a, b in zip(values, values[1:]))
    assert values[1] < 0.01 and values[-2] > 0.99


# ---------------------------------------------------------------- agent: prepare
@pytest.mark.parametrize("ctx,reason", [
    (ModeContext(healthy=False), "unhealthy"),
    (ModeContext(formation_idle=False), "formation_transition_active"),
])
def test_agent_nacks_unsafe_context_without_changing_state(ctx, reason):
    agent = AgentModeSwitch(1, "hybrid")
    reply = agent.on_prepare(plan().to_prepare_message(), 5.0, ctx)
    assert not reply["ok"] and reply["reason"] == reason and agent.state == AgentModeSwitch.IDLE


def test_agent_nacks_wrong_source_mode_short_lead_and_garbage():
    agent = AgentModeSwitch(1, "centralized")
    assert agent.on_prepare(plan(a="hybrid").to_prepare_message(), 5.0, OK)["reason"] == "mode_mismatch"
    agent = AgentModeSwitch(1, "hybrid")
    assert agent.on_prepare(plan().to_prepare_message(), 9.5, OK)["reason"] == "start_too_soon"
    for message in ({}, None, "x", {"version": 1}):
        assert not agent.on_prepare(message, 5.0, OK)["ok"]
    assert agent.state == AgentModeSwitch.IDLE


def test_prepare_is_idempotent_and_closed_ids_are_rejected():
    agent, pl = staged()
    assert agent.on_prepare(pl.to_prepare_message(), 5.1, OK)["ok"]
    other = plan(switch_id=2, b="centralized")
    assert agent.on_prepare(other.to_prepare_message(), 5.2, OK)["reason"] == "already_switching"
    agent.on_abort({"switch_id": 1}, 5.3)
    assert agent.on_prepare(pl.to_prepare_message(), 5.4, OK)["reason"] == "stale_switch"


# ---------------------------------------------------------------- agent: running
def test_mode_does_not_change_until_the_common_start_and_then_blends_smoothly():
    agent, pl = scheduled()
    assert agent.weights(9.9) == ("hybrid", "hybrid", 1.0)
    prev = -1.0
    for t in [10.0 + 0.1 * k for k in range(0, 21)]:
        agent.tick(t, OK)
        a, b, w = agent.weights(t)
        assert (a, b) == ("hybrid", "decentralized") or agent.state == AgentModeSwitch.VERIFY
        assert w >= prev - 1e-9
        prev = w
    assert agent.state == AgentModeSwitch.VERIFY
    assert agent.weights(12.5) == ("decentralized", "decentralized", 1.0)
    agent.on_done({"switch_id": 1, "plan_hash": pl.plan_hash})
    assert agent.state == AgentModeSwitch.IDLE and agent.mode == "decentralized"


def test_lost_done_is_covered_by_the_self_close_timeout():
    agent, pl = switching()
    agent.tick(12.5, OK)
    assert agent.state == AgentModeSwitch.VERIFY
    events = agent.tick(12.0 + P.verify_close_s + 0.1, OK)
    assert agent.state == AgentModeSwitch.IDLE and agent.mode == "decentralized"
    assert events and events[0][1] == "mode_verify_timeout_commit"


def test_late_go_and_missing_go_discard_the_plan():
    agent, pl = staged()
    reply = agent.on_go({"switch_id": 1, "plan_hash": pl.plan_hash, "start_at": 10.0}, 10.5)
    assert reply["reason"] == "go_late" and agent.state == AgentModeSwitch.IDLE and agent.mode == "hybrid"
    agent, pl = staged()
    assert agent.tick(10.0 + P.no_go_grace_s + 0.1, OK)[0][1] == "mode_no_go"
    assert agent.state == AgentModeSwitch.IDLE


def test_go_for_another_plan_is_ignored():
    agent, pl = staged()
    for message in ({"switch_id": 2, "plan_hash": pl.plan_hash, "start_at": 10.0},
                    {"switch_id": 1, "plan_hash": "0" * 64, "start_at": 10.0},
                    {"switch_id": 1, "plan_hash": pl.plan_hash, "start_at": 11.0}, {}, None):
        assert agent.on_go(message, 9.0) is None and agent.state == AgentModeSwitch.STAGED


# ---------------------------------------------------------------- agent: rollback
def test_abort_before_start_discards_and_keeps_the_old_mode():
    agent, _ = scheduled()
    agent.on_abort({"switch_id": 1, "reason": "x"}, 9.5)
    assert agent.state == AgentModeSwitch.IDLE and agent.mode == "hybrid"
    assert agent.weights(9.6) == ("hybrid", "hybrid", 1.0)


def test_abort_during_the_ramp_blends_back_to_the_old_mode():
    agent, pl = switching()
    _, _, w_at_abort = agent.weights(11.0)
    agent.on_abort({"switch_id": 1, "reason": "separation"}, 11.0)
    assert agent.state == AgentModeSwitch.ROLLBACK
    prev = w_at_abort + 1e-9
    for t in [11.0 + 0.1 * k for k in range(0, 25)]:
        agent.tick(t, OK)
        a, b, w = agent.weights(t)
        if agent.state == AgentModeSwitch.ROLLBACK:
            assert w <= prev + 1e-9        # monotone decrease: no jump back to the old mode
            prev = w
        else:
            assert a == b == "hybrid"      # finished: the same controller on both sides
    assert agent.state == AgentModeSwitch.IDLE and agent.mode == "hybrid"


def test_abort_in_the_verify_phase_still_rolls_back():
    agent, pl = switching()
    agent.tick(12.5, OK)
    assert agent.state == AgentModeSwitch.VERIFY
    agent.on_abort({"switch_id": 1, "reason": "verify"}, 12.6)
    assert agent.state == AgentModeSwitch.ROLLBACK
    agent.tick(12.6 + 2.5, OK)
    assert agent.state == AgentModeSwitch.IDLE and agent.mode == "hybrid"


def test_local_fault_during_the_ramp_aborts_and_reports():
    agent, _ = switching()
    actions = agent.tick(11.0, ModeContext(healthy=False))
    assert agent.state == AgentModeSwitch.ROLLBACK
    sends = [a[1] for a in actions if a[0] == "send"]
    assert sends and sends[0]["type"] == "mode_abort" and sends[0]["reason"] == "own_fault"
    assert agent.tick(11.1, ModeContext(formation_idle=False)) == []   # already rolling back


# ---------------------------------------------------------------- coordinator
def good_views(mode, state="IDLE", switch_id=None):
    return {i: ModeView(0.3, mode, state, switch_id, True, (3.0 * i, 0.0)) for i in range(N)}


def ack(pl, i, ok=True, reason=None):
    return {"switch_id": pl.switch_id, "plan_hash": pl.plan_hash, "drone_id": i, "ok": ok, "reason": reason}


def types(actions):
    return [a[1]["type"] for a in actions if a[0] == "send"]


def begin(c, to="decentralized", now=5.0):
    actions = c.begin(to, now)
    return c.plan, actions


def test_coordinator_happy_path_to_done():
    c = CoordinatorModeSwitch(N, "hybrid")
    pl, actions = begin(c)
    assert types(actions) == ["mode_prepare"] and c.busy
    for i in range(N):
        c.on_ack(ack(pl, i), 5.2)
    assert types(c.tick(5.3, {})) == ["mode_go"] and c.state == c.COMMITTED
    c.tick(pl.start_at, {})
    assert c.state == c.SWITCHING
    assert c.tick(pl.start_at + 1.0, good_views("hybrid", "SWITCHING", 1)) == []
    c.tick(pl.start_at + pl.ramp_s + 0.1, good_views("decentralized", "VERIFY", 1))
    assert c.state == c.VERIFYING
    t_done = pl.start_at + pl.ramp_s + 0.1 + P.verify_dwell_s
    done = c.tick(t_done, good_views("decentralized", "VERIFY", 1))
    assert types(done) == ["mode_done"] and c.state == c.VERIFYING and c.mode == "hybrid"   # not final yet
    assert types(c.tick(t_done + 0.6, good_views("decentralized", "VERIFY", 1))) == ["mode_done"]  # resent
    final = c.tick(t_done + 1.0, good_views("decentralized", "IDLE"))
    assert c.state == c.IDLE and c.mode == "decentralized" and final[0][1] == "mode_switch_complete"


def test_coordinator_aborts_before_start_on_missing_ack_or_nack_without_changing_mode():
    c = CoordinatorModeSwitch(N, "hybrid")
    pl, _ = begin(c)
    for i in range(N - 1):
        c.on_ack(ack(pl, i), 5.2)
    out = c.tick(5.0 + P.ack_timeout_s + 0.1, {})
    assert types(out) == ["mode_abort"] and c.state == c.IDLE and c.mode == "hybrid"
    assert "ack_timeout:[4]" in out[0][1]["reason"] and out[1][2]["started"] is False
    c2 = CoordinatorModeSwitch(N, "hybrid")
    pl2, _ = begin(c2)
    assert types(c2.on_ack(ack(pl2, 2, False, "unhealthy"), 5.1)) == ["mode_abort"]


def test_coordinator_conflicting_begin_is_refused_and_next_id_advances():
    c = CoordinatorModeSwitch(N, "hybrid")
    begin(c)
    with pytest.raises(RuntimeError, match="mode_switch_active"):
        c.begin("centralized", 5.1)
    c.abort("operator", 5.2)
    assert c.next_id() == 2


@pytest.mark.parametrize("mutate,expected", [
    (lambda v: v.__setitem__(1, ModeView(5.0, "hybrid", "SWITCHING", 1, True, (3.0, 0.0))), "stale_status:1"),
    (lambda v: setattr(v[2], "healthy", False), "unhealthy:2"),
    (lambda v: setattr(v[3], "position_ne", v[2].position_ne), "separation:2,3"),
    (lambda v: v.__setitem__(4, ModeView(0.3, "hybrid", "IDLE", None, True, (12.0, 0.0))), "agent_not_switching:4"),
])
def test_coordinator_supervision_aborts_and_rolls_back(mutate, expected):
    c = CoordinatorModeSwitch(N, "hybrid")
    pl, _ = begin(c)
    for i in range(N):
        c.on_ack(ack(pl, i), 5.2)
    c.tick(5.3, {}); c.tick(pl.start_at, {})
    views = good_views("hybrid", "SWITCHING", 1)
    mutate(views)
    out = c.tick(pl.start_at + 1.5, views)
    assert c.state == c.ROLLING_BACK and out[0][1]["reason"].startswith(expected)
    # rollback completes once every agent reports the old mode and is idle
    resent = c.tick(pl.start_at + 2.5, good_views("hybrid", "ROLLBACK", 1))
    assert types(resent) == ["mode_abort"]                       # abort is retransmitted until confirmed
    done = c.tick(pl.start_at + 6.0, good_views("hybrid"))
    assert c.state == c.IDLE and c.mode == "hybrid" and done[0][1] == "mode_switch_rolled_back"


def test_verification_failure_rolls_back_instead_of_committing():
    c = CoordinatorModeSwitch(N, "hybrid")
    pl, _ = begin(c)
    for i in range(N):
        c.on_ack(ack(pl, i), 5.2)
    c.tick(5.3, {}); c.tick(pl.start_at, {})
    late = pl.start_at + pl.ramp_s + P.verify_timeout_s + 0.5
    views = good_views("decentralized", "VERIFY", 1)
    views[2] = ModeView(0.3, "hybrid", "SWITCHING", 1, True, (6.0, 0.0))     # one drone never finishes
    out = c.tick(late, views)
    assert c.state == c.ROLLING_BACK and out[0][1]["reason"] == "verify_timeout" and c.mode == "hybrid"


# ---------------------------------------------------------------- lossy channel simulation
def run_sim(seed, loss, dt=0.1, horizon=30.0):
    rng = random.Random(seed)
    coord = CoordinatorModeSwitch(N, "hybrid")
    agents = [AgentModeSwitch(i, "hybrid") for i in range(N)]
    to_agents, to_coord, now = [], [], 5.0
    ever_scheduled, committed, events = set(), False, []

    def lossy(box, msg):
        if rng.random() >= loss:
            box.append((now + dt, msg))

    def send_all(actions):
        nonlocal committed
        for a in actions:
            if a[0] == "send":
                for i in range(N):
                    lossy(to_agents, (i, a[1]))
            else:
                events.append(a[1])
                committed = committed or a[1] == "mode_committed"

    send_all(coord.begin("decentralized", now))
    while now < 5.0 + horizon:
        due = [m for m in to_agents if m[0] <= now]
        to_agents[:] = [m for m in to_agents if m[0] > now]
        for _, (i, msg) in due:
            agent, t = agents[i], msg["type"]
            if t == "mode_prepare":
                lossy(to_coord, agent.on_prepare(msg, now, OK))
            elif t == "mode_go":
                reply = agent.on_go(msg, now)
                if reply:
                    lossy(to_coord, reply)
            elif t == "mode_abort":
                agent.on_abort(msg, now)
            elif t == "mode_done":
                agent.on_done(msg)
        ready = [m for m in to_coord if m[0] <= now]
        to_coord[:] = [m for m in to_coord if m[0] > now]
        for _, msg in ready:
            if msg["type"] == "mode_ack":
                send_all(coord.on_ack(msg, now))
            elif msg["type"] == "mode_abort":
                send_all(coord.on_agent_abort(msg, now))
        for i, agent in enumerate(agents):
            for a in agent.tick(now, OK):
                if a[0] == "send":
                    lossy(to_coord, a[1])
            if agent.state in (AgentModeSwitch.SWITCHING, AgentModeSwitch.VERIFY, AgentModeSwitch.ROLLBACK):
                ever_scheduled.add(i)
        views = {i: ModeView(0.3, a.dominant_mode(now), a.state, None if a.plan is None else a.plan.switch_id,
                             True, (3.0 * i, 0.0)) for i, a in enumerate(agents)}
        send_all(coord.tick(now, views))
        now += dt
    return coord, agents, ever_scheduled, committed, events


@pytest.mark.parametrize("loss", [0.0, 0.1, 0.3, 0.6])
def test_lossy_channel_never_switches_without_full_acks_and_always_ends_consistent(loss):
    results = {"switched": 0, "kept": 0}
    for seed in range(50):
        coord, agents, ever, committed, events = run_sim(seed, loss)
        if ever:
            assert committed, f"seed {seed}: an agent switched without a full set of acks"
        modes = {a.mode for a in agents}
        assert all(a.state == AgentModeSwitch.IDLE for a in agents) or not coord.busy is False, f"seed {seed}"
        assert not coord.busy, f"seed {seed}: coordinator stuck in {coord.state}"
        assert all(a.state == AgentModeSwitch.IDLE for a in agents), f"seed {seed}: {[a.state for a in agents]}"
        assert len(modes) == 1, f"seed {seed}: mixed modes {modes}"      # never a split swarm at rest
        assert modes == {coord.mode}, f"seed {seed}: swarm {modes} vs coordinator {coord.mode}"
        results["switched" if coord.mode == "decentralized" else "kept"] += 1
    if loss == 0.0:
        assert results["switched"] == 50
