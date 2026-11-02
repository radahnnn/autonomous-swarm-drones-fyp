"""Console commands used by the Gazebo survey patrol: goto, capture, time-scaled patrol speed (UNIT)."""
import json
import threading
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from sitl.independent_swarm_console import IndependentSwarmConsole


def _console(tmp_path):
    c = object.__new__(IndependentSwarmConsole)
    c.route, c.mode_switch, c.transition_protocol, c.geofence = None, None, False, None
    c.goal_ne, c.target_alt_m, c.count = np.zeros(2), 5.0, 3
    c.published, c.events, c.metrics_notes = 0, [], []
    c._publish_intent = lambda: setattr(c, "published", c.published + 1)
    c.event = lambda name, **data: c.events.append((name, data))
    c._note_metric = lambda name, **data: c.metrics_notes.append((name, data))
    c.run_dir, c.lock, c.latest = Path(tmp_path), threading.Lock(), {}
    c.last_command_result = {}
    c.formation, c.operator_formation_slots = "v", None
    return c


def test_goto_moves_the_goal_and_publishes(tmp_path, capsys):
    c = _console(tmp_path)
    c.goto("3", "-4.5")
    np.testing.assert_allclose(c.goal_ne, [3.0, -4.5])
    assert c.published == 1 and c.events[0][0] == "goto"


@pytest.mark.parametrize("north, east", [("x", "1"), ("nan", "0"), ("1", "inf")])
def test_goto_rejects_bad_numbers(tmp_path, north, east):
    c = _console(tmp_path)
    c.goto(north, east)
    np.testing.assert_allclose(c.goal_ne, [0.0, 0.0])
    assert c.published == 0


def test_goto_is_refused_while_a_route_or_switch_is_active(tmp_path):
    c = _console(tmp_path)
    c.route = object()
    c.goto("1", "1")
    assert c.published == 0
    c.route = None
    c.mode_switch = SimpleNamespace(busy=True)
    c.goto("1", "1")
    assert c.published == 0


def test_capture_writes_an_index_with_positions(tmp_path, monkeypatch):
    c = _console(tmp_path)
    c.latest = {0: {"position": [1.0, 2.0, -5.0]}, 1: {"position": [3.0, 4.0, -5.0]}}

    def fake_capture(topics, directory, label, timeout_s):
        directory.mkdir(parents=True, exist_ok=True)
        out = {}
        for number in topics:
            if number == 3:
                out[number] = None                              # one camera did not answer
            else:
                out[number] = directory / f"{label}_D{number}.png"
                out[number].write_bytes(b"x")
        return out

    monkeypatch.setattr("sitl.gz_camera.capture_all", fake_capture)
    result = c.capture("corner 1/A")
    assert result["accepted"] is False and "2/3" in result["message"]
    entry = json.loads((tmp_path / "captures" / "index.jsonl").read_text().splitlines()[0])
    assert entry["label"] == "corner_1_A" and [f["file"] is not None for f in entry["frames"]] == [True, True, False]
    assert entry["frames"][0]["position"] == [1.0, 2.0, -5.0]
    assert c.metrics_notes[0][0] == "capture" and c.metrics_notes[0][1]["frames"] == 2


def test_capture_reports_a_missing_simulator_instead_of_crashing(tmp_path, monkeypatch, capsys):
    c = _console(tmp_path)

    def boom(*a, **k):
        raise ImportError("no gz bindings")

    monkeypatch.setattr("sitl.gz_camera.capture_all", boom)
    assert c.capture("x")["accepted"] is False
    assert "Capture failed" in capsys.readouterr().out and c.events[0][0] == "capture_failed"


def test_route_accepts_wall_clock_speeds_of_slow_simulators():
    from sitl.route import square_patrol
    route = square_patrol((0.0, 0.0), 12.0, 0.0, 0.3 * 0.12)       # 0.3 m/s simulated at 0.12x real time
    assert route.speed_m_s == pytest.approx(0.036)
    with pytest.raises(ValueError):
        square_patrol((0.0, 0.0), 12.0, 0.0, 0.01)


def test_wait_settled_waits_for_drifting_drones_and_reports_the_worst(tmp_path):
    c = _console(tmp_path)
    c.position_sigma_m, c.time_scale = 0.0, 1.0
    c.latest = {0: {"position": [0.0, 0.0, -5], "position_target_ne": [0.2, 0.0]},
                1: {"position": [0.0, 0.0, -5], "position_target_ne": [4.0, 0.0]}}
    ok, worst, err = c._wait_settled(timeout_s=0.2, poll_s=0.05)
    assert not ok and worst == 1 and err == pytest.approx(4.0)
    c.latest[1]["position_target_ne"] = [0.5, 0.0]
    assert c._wait_settled(timeout_s=0.2, poll_s=0.05)[0]
    c.position_sigma_m = 3.5                                    # noise widens the tolerance
    c.latest[1]["position_target_ne"] = [4.0, 0.0]
    assert c._wait_settled(timeout_s=0.1, poll_s=0.05)[0]
