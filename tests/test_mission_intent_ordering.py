"""Stale or replayed MissionIntent messages must not alter agent state (UNIT evidence)."""
import time

import numpy as np
import pytest

from sitl.formation_library import slots_for
from sitl.independent_agent_node import IndependentAgentNode
from sitl.mission_intent import MissionIntent, MissionLease

EPOCH = "operator-session-a"
COUNT = 5
NOW = 1000.0


class _Feed:
    def __init__(self):
        self.queue = []

    def poll(self):
        messages, self.queue = self.queue, []
        return messages


def _intent(sequence, formation="configured", slots=None, start_slots=None,
            transition_s=0.0):
    return MissionIntent(
        sequence, NOW - 1.0, NOW + 20.0, (3.0, -2.0), formation,
        formation_slots=None if slots is None else tuple(map(tuple, slots)),
        formation_start_slots=None if start_slots is None else tuple(map(tuple, start_slots)),
        formation_transition_s=transition_s,
    )


def _node(monkeypatch, *, airborne):
    monkeypatch.setattr("sitl.independent_agent_node.time.time", lambda: NOW)
    monkeypatch.setattr("sitl.independent_agent_node.time.monotonic", lambda: 50.0)
    node = object.__new__(IndependentAgentNode)
    node.operator = _Feed()
    node.operator_clock_offset_s = 0.0
    node.clock_sync_epoch = EPOCH
    node.last_mission_sequence = 0
    node.mission_lease = MissionLease()
    node.current_mission = None
    node.pending_intent = None
    node.landing = False
    node.airborne = airborne
    node.armed = airborne
    node.adaptive_swarm = True
    node.last_command = "ready"
    node.count = COUNT
    node.drone_id = 1
    node.formation = "v"
    node.membership = None
    node.membership_revision = 0
    node.all_slots = np.asarray(slots_for("v", COUNT), dtype=float)
    node.slot = node.all_slots[node.drone_id].copy()
    node.operator_formation_slots = None
    node.formation_transition_start_slots = None
    node.formation_transition_target_slots = None
    node.formation_transition_started_at = None
    node.formation_transition_duration_s = 0.0
    node.formation_transition_progress = 1.0
    node.last_coordinator_message_monotonic = None
    node.coordinator_connected = False
    return node


def _send(node, intent):
    node.operator.queue.append(
        {"version": 1, "type": "mission", "epoch": EPOCH, "intent": intent.to_dict()})
    node._receive_operator_messages()


def _state(node):
    return {
        "formation": node.formation,
        "all_slots": node.all_slots.copy(),
        "slot": node.slot.copy(),
        "operator_slots": (None if node.operator_formation_slots is None
                           else np.asarray(node.operator_formation_slots).copy()),
        "start": (None if node.formation_transition_start_slots is None
                  else node.formation_transition_start_slots.copy()),
        "target": (None if node.formation_transition_target_slots is None
                   else node.formation_transition_target_slots.copy()),
        "started_at": node.formation_transition_started_at,
        "duration": node.formation_transition_duration_s,
        "progress": node.formation_transition_progress,
        "revision": node.membership_revision,
        "sequence": node.last_mission_sequence,
        "mission": node.current_mission,
    }


def _assert_unchanged(before, after):
    assert before.keys() == after.keys()
    for key, value in before.items():
        other = after[key]
        if isinstance(value, np.ndarray):
            np.testing.assert_array_equal(value, other, err_msg=key)
        else:
            assert value == other, key


@pytest.mark.parametrize("stale_sequence", [1, 2], ids=["out_of_order", "replayed_same_sequence"])
def test_stale_mission_cannot_change_grounded_formation(monkeypatch, stale_sequence):
    node = _node(monkeypatch, airborne=False)
    _send(node, _intent(2, "ring"))
    assert node.last_mission_sequence == 2 and node.formation == "ring"

    before = _state(node)
    _send(node, _intent(stale_sequence, "line", slots_for("line", COUNT)))

    _assert_unchanged(before, _state(node))


def test_newer_mission_still_changes_grounded_formation(monkeypatch):
    node = _node(monkeypatch, airborne=False)
    _send(node, _intent(1, "ring"))
    _send(node, _intent(2, "line", slots_for("line", COUNT)))

    assert node.last_mission_sequence == 2
    assert node.formation == "line"
    np.testing.assert_allclose(node.all_slots, slots_for("line", COUNT))


@pytest.mark.parametrize("stale_sequence", [4, 5], ids=["out_of_order", "replayed_same_sequence"])
def test_stale_mission_cannot_restart_airborne_transition(monkeypatch, stale_sequence):
    node = _node(monkeypatch, airborne=True)
    start = np.asarray(slots_for("v", COUNT), dtype=float)
    target = np.asarray(slots_for("line", COUNT), dtype=float)
    _send(node, _intent(5, "line", target, start, transition_s=8.0))
    assert node.last_mission_sequence == 5
    assert node.formation == "line" and node.formation_transition_duration_s == 8.0
    revision = node.membership_revision

    before = _state(node)
    other_target = np.asarray(slots_for("column", COUNT), dtype=float)
    _send(node, _intent(stale_sequence, "column", other_target, target, transition_s=3.0))

    _assert_unchanged(before, _state(node))
    assert node.membership_revision == revision


def test_stale_slot_refresh_cannot_overwrite_operator_slots(monkeypatch):
    """Same-formation intents take a different branch; it must be guarded too."""
    node = _node(monkeypatch, airborne=False)
    _send(node, _intent(3, "configured", slots_for("v", COUNT)))
    before = _state(node)

    shifted = np.asarray(slots_for("v", COUNT), dtype=float) + np.array([10.0, 0.0, 0.0])
    _send(node, _intent(2, "configured", shifted))

    _assert_unchanged(before, _state(node))
