import time
import threading

import numpy as np
import pytest

from sitl.formation_library import slots_for
from sitl.formation_transition import plan_formation_transition
from sitl.mission_intent import MissionIntent
from sitl.independent_swarm_console import IndependentSwarmConsole
from sitl.decentralized_guidance import LocalDroneAgent


def test_transition_plan_assigns_slots_and_respects_spacing():
    current = slots_for("v", 5)[:, :2]
    plan = plan_formation_transition(current, "ring")
    assert plan.slots_by_drone.shape == (5, 3)
    assert plan.duration_s >= 8.0
    assert plan.minimum_path_separation_m >= 2.85 - 1e-9
    for i in range(5):
        for j in range(i + 1, 5):
            assert np.linalg.norm(plan.slots_by_drone[i, :2] - plan.slots_by_drone[j, :2]) >= 3.2 - 1e-9


def test_transition_plan_rejects_colliding_current_positions():
    with pytest.raises(ValueError, match="planned paths come within"):
        plan_formation_transition(np.zeros((3, 2)), "v")


def test_ten_drone_ring_plan_uses_safe_orientation_and_tracking_margin():
    current = slots_for("v", 10)[:, :2]
    plan = plan_formation_transition(current, "ring")
    assert plan.minimum_path_separation_m >= 2.85
    minimum_target_gap = min(
        np.linalg.norm(plan.slots_by_drone[i, :2] - plan.slots_by_drone[j, :2])
        for i in range(10) for j in range(i + 1, 10)
    )
    assert minimum_target_gap >= 4.0 - 1e-9
    assert plan.duration_s <= 45.0


def test_transition_plan_rejects_keep_out_zone_intersection():
    current = slots_for("v", 3)[:, :2]
    with pytest.raises(ValueError, match="keep-out zone"):
        plan_formation_transition(current, "line", obstacles=[(0.0, 0.0, 1.0)])


def test_live_transition_mission_serializes_and_validates():
    current = slots_for("v", 5)[:, :2]
    plan = plan_formation_transition(current, "ring")
    now = time.time()
    intent = MissionIntent(
        sequence=1, issued_at=now, expires_at=now + 30.0, goal_ne=(0.0, 0.0),
        formation="ring", formation_slots=tuple(map(tuple, plan.slots_by_drone)),
        formation_start_slots=tuple(map(tuple, np.column_stack((current, np.zeros(5))))),
        formation_transition_s=plan.duration_s,
    )
    parsed = MissionIntent.from_dict(intent.to_dict(), now + 0.1)
    assert parsed.formation == "ring"
    assert parsed.formation_transition_s == pytest.approx(plan.duration_s)
    assert parsed.formation_start_slots == intent.formation_start_slots


def test_live_transition_rejects_missing_start_slots():
    now = time.time()
    slots = slots_for("v", 3).tolist()
    intent = MissionIntent(
        sequence=1, issued_at=now, expires_at=now + 30.0, goal_ne=(0.0, 0.0),
        formation="v", formation_slots=tuple(map(tuple, slots)),
        formation_transition_s=8.0,
    )
    with pytest.raises(ValueError, match="requires measured start slots"):
        MissionIntent.from_dict(intent.to_dict(), now + 0.1)


def test_live_clearance_monitor_detects_gap_before_hard_minimum():
    now = time.time()
    console = object.__new__(IndependentSwarmConsole)
    console.count = 2
    console.lock = threading.Lock()
    console.formation_transition_deadline = now + 10.0
    console.latest = {
        0: {"timestamp": now, "position_age_s": 0.0, "position": [0.0, 0.0, -5.0],
            "airborne": True, "armed": True, "safety_state": "nominal"},
        1: {"timestamp": now, "position_age_s": 0.0, "position": [0.0, 2.7, -5.0],
            "airborne": True, "armed": True, "safety_state": "nominal"},
    }
    assert "2.70 m" in console._formation_transition_abort_reason(now)


def test_transition_guard_tolerates_normal_status_interval_but_rejects_real_staleness():
    now = time.time()
    console = object.__new__(IndependentSwarmConsole)
    console.count = 2
    console.lock = threading.Lock()
    console.formation_transition_deadline = now + 10.0
    console.latest = {
        index: {
            "timestamp": now - 1.05,
            "operator_received_at": now - 1.05,
            "position_age_s": 0.01,
            "position": [0.0, index * 4.0, -5.0],
            "airborne": True,
            "armed": True,
            "safety_state": "nominal",
        }
        for index in range(2)
    }
    assert console._formation_transition_abort_reason(now) is None

    console.latest[1]["operator_received_at"] = now - 2.1
    assert "D2 transition telemetry is stale" == console._formation_transition_abort_reason(now)


def test_morph_tracks_the_planned_position_instead_of_integrating_flocking_velocity():
    now = time.time()
    agent = LocalDroneAgent(0)
    intent = MissionIntent(1, now, now + 30.0, (2.0, 3.0)).to_dict()
    target = agent.command(
        [0.0, 0.0], now, None, [], {}, 0.1, now=now,
        mission_intent=intent, slot_offset=[1.0, -1.0], track_formation=True,
    )
    assert np.allclose(target, [3.0, 2.0])


def test_independent_follower_stays_anchored_to_slot_after_morph():
    now = time.time()
    agent = LocalDroneAgent(0)
    agent.velocity = np.array([1.5, 0.0])  # still drifting away from the slot
    agent.last_position = np.array([3.0, 0.0])
    agent.last_sample_time = now
    intent = MissionIntent(1, now, now + 30.0, (0.0, 0.0)).to_dict()
    peers = [{"id": 1, "position": np.array([3.0, 5.0]),
              "velocity": np.zeros(2)}]
    first = agent.command(
        [3.0, 0.0], now, None, peers, {1: np.array([0.0, -5.0])},
        0.1, now=now, mission_intent=intent, slot_offset=[0.0, 0.0],
        position_setpoint_tracking=True,
    )
    assert np.linalg.norm(first - np.array([3.0, 0.0])) <= 0.045 + 1e-9
    previous = first
    for tick in range(1, 80):
        target = agent.command(
            [3.0, 0.0], now + tick * 0.1, None, peers,
            {1: np.array([0.0, -5.0])}, 0.1, now=now + tick * 0.1,
            slot_offset=[0.0, 0.0], position_setpoint_tracking=True,
        )
        assert np.linalg.norm(target - previous) <= 0.045 + 1e-9
        previous = target
    assert np.linalg.norm(previous) <= 0.25 + 1e-9


def test_transition_does_not_complete_while_agents_are_far_from_slots():
    now = time.time()
    console = object.__new__(IndependentSwarmConsole)
    console.count = 2
    console.lock = threading.Lock()
    console.goal_ne = np.array([10.0, 0.0])
    console.formation = "line"
    console.formation_slots = np.array([[0.0, 0.0, 0.0], [0.0, 4.0, 0.0]])
    console.latest = {
        index: {
            "operator_received_at": now,
            "position_age_s": 0.1,
            "position": [10.0, index * 4.0, -5.0],
            "formation": "line",
            "formation_transition_active": False,
            "formation_transition_progress": 1.0,
        }
        for index in range(2)
    }
    assert console._formation_transition_arrived(now)
    console.latest[1]["position"][0] = 11.0
    assert not console._formation_transition_arrived(now)


def test_requesting_current_formation_is_an_idempotent_success():
    console = object.__new__(IndependentSwarmConsole)
    console.args = type("Args", (), {"adaptive_swarm": True})()
    console.formation = "v"
    console.formation_transition_deadline = None
    result = console._set_formation("v")
    assert result["accepted"] is True
    assert result["unchanged"] is True
    assert "already active" in result["message"]
