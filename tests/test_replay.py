"""Recording playback through the dashboard (UNIT)."""
import json
from datetime import datetime, timedelta, timezone

import pytest

from sitl.operator_dashboard import dashboard_state
from sitl.replay import Recording, ReplaySession

T0 = datetime(2026, 10, 30, 12, 0, 0, tzinfo=timezone.utc)


def iso(seconds):
    return (T0 + timedelta(seconds=seconds)).isoformat()


def status(drone, seconds, x, mode="hybrid", airborne=True):
    return {"utc": iso(seconds), "kind": "agent_status", "drone_id": drone, "airborne": airborne,
            "armed": airborne, "safety_state": "nominal", "position": [x, 3.0 * drone, -5.0 if airborne else 0.0],
            "operator_received_at": (T0 + timedelta(seconds=seconds)).timestamp(), "formation": "v",
            "control_mode": mode, "mode_switch": {"mode": mode, "state": "IDLE"}, "peer_ids": [], "peer_live_ids": [],
            "active_member_ids": [0, 1], "elected_leader_id": 0, "position_age_s": 0.1,
            "position_target_ne": [x, 3.0 * drone]}


def write_run(path, seconds=20):
    events = []
    for t in range(0, seconds + 1):
        for drone in (0, 1):
            events.append(status(drone, t, float(t), mode="decentralized" if t >= 10 else "hybrid"))
    events += [
        {"utc": iso(1), "kind": "operator_command", "action": "launch"},
        {"utc": iso(8), "kind": "mode_prepare", "from_mode": "hybrid", "to_mode": "decentralized"},
        {"utc": iso(10), "kind": "mode_switch_complete", "mode": "decentralized"},
        {"utc": iso(0.5), "kind": "mission_intent", "sequence": 1, "goal_ne": [1.0, 2.0], "target_alt_m": 5.0},
        {"utc": iso(12), "kind": "mission_intent", "sequence": 2, "goal_ne": [9.0, 2.0], "target_alt_m": 7.0},
        {"utc": iso(5), "kind": "clock_probe_sample"},
    ]
    with path.open("w") as handle:
        for event in sorted(events, key=lambda e: e["utc"]):
            handle.write(json.dumps(event) + "\n")
        handle.write("{broken\n")            # corrupt trailing line must be tolerated
    return path


@pytest.fixture
def recording(tmp_path):
    return Recording(write_run(tmp_path / "events.jsonl"))


def test_recording_indexes_statuses_markers_and_duration(recording):
    assert recording.count == 2 and recording.t1 - recording.t0 == pytest.approx(20.0)
    assert [m["kind"] for m in recording.markers] == ["operator_command", "mode_prepare", "mode_switch_complete"]
    assert recording.markers[1]["label"] == "mode switch hybrid -> decentralized requested"
    assert recording.status_at(0, recording.t0 + 5.4)["position"][0] == 5.0      # latest at or before t
    assert recording.status_at(0, recording.t0 - 1) is None
    assert recording.status_at(0, recording.t0 + 100) is None                     # too old to be believed
    assert recording.intent_at(recording.t0 + 3)["goal_ne"] == [1.0, 2.0]
    assert recording.intent_at(recording.t0 + 13)["goal_ne"] == [9.0, 2.0]


def test_folder_path_resolves_to_its_events_file_and_empty_recordings_are_rejected(tmp_path):
    run = tmp_path / "run1"
    run.mkdir()
    write_run(run / "events.jsonl")
    assert Recording(run).path.name == "events.jsonl"
    empty = tmp_path / "empty.jsonl"
    empty.write_text('{"utc": "2026-10-30T12:00:00+00:00", "kind": "x"}\n')
    with pytest.raises(ValueError):
        Recording(empty)


def test_replay_session_reconstructs_dashboard_state_at_the_cursor(recording):
    session = ReplaySession(recording, speed=1.0, playing=False)
    session.replay_post({"action": "seek", "t": 15.0})
    state = dashboard_state(session)
    assert state["control_mode"] == "decentralized" and state["airborne"] == 2 and state["healthy"] == 2
    assert state["goal"] == [9.0, 2.0]
    assert state["replay"]["t"] == 15.0 and state["replay"]["playing"] is False
    assert state["agents"][0]["position"][0] == 15.0
    json.dumps(state, allow_nan=False)
    session.replay_post({"action": "seek", "t": 2.0})
    assert dashboard_state(session)["control_mode"] == "hybrid"


def test_playback_controls_clamp_and_validate(recording):
    session = ReplaySession(recording, speed=4.0, playing=False)
    assert session.replay_post({"action": "play"})["playing"] is True
    assert session.replay_post({"action": "pause"})["playing"] is False
    assert session.replay_post({"action": "speed", "speed": 999})["speed"] == 32.0
    assert session.replay_post({"action": "speed", "speed": 0.0})["speed"] == 0.25
    assert session.replay_post({"action": "seek", "t": -5})["t"] == 0.0
    assert session.replay_post({"action": "seek", "t": 1e6})["t"] == pytest.approx(20.0)
    with pytest.raises(ValueError):
        session.replay_post({"action": "warp"})


def test_metrics_follow_the_cursor_and_rebuild_when_seeking_backward(recording):
    session = ReplaySession(recording, speed=1.0, playing=False)
    session.replay_post({"action": "seek", "t": 20.0})
    late = session.metrics.snapshot()
    assert late["samples"] > 10 and late["gap_m"] == 3.0 and late["counters"].get("mode_switch_complete") == 1
    session.replay_post({"action": "seek", "t": 4.0})
    early = session.metrics.snapshot()
    assert early["samples"] < late["samples"] and early["counters"] == {}
    session.replay_post({"action": "seek", "t": 20.0})
    assert session.metrics.snapshot()["samples"] == late["samples"]             # deterministic


def test_playing_advances_the_cursor_and_stops_at_the_end(recording, monkeypatch):
    import sitl.replay as module
    clock = {"t": 1000.0}
    monkeypatch.setattr(module.time, "time", lambda: clock["t"])
    session = ReplaySession(recording, speed=4.0, playing=True)
    clock["t"] += 2.0
    session._advance()
    assert session.cursor == pytest.approx(8.0) and session.playing
    clock["t"] += 10.0
    session._advance()
    assert session.cursor == pytest.approx(20.0) and session.playing is False


def test_replay_is_read_only(recording, capsys):
    session = ReplaySession(recording, playing=False)
    assert session.handle("land") is True and "read-only" in capsys.readouterr().out
    assert session._set_formation("ring")["accepted"] is False
    assert session.set_control_mode("hybrid")["accepted"] is False
