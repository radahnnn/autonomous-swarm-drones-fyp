import json

import pytest

from experiments.summarize_three_mode_sitl_comparison import build_comparison


def _write(path, value):
    path.write_text(json.dumps(value), encoding="utf-8")
    return path


def _independent(mode):
    return {
        "control_mode": mode,
        "drones": 10,
        "seed": 1024,
        "peer_loss": 0.02,
        "peer_latency_ms": 25.0,
        "peer_jitter_ms": 5.0,
        "peer_burst_loss": False,
        "passed": True,
        "all_drones_landed_disarmed": True,
        "mission_convergence": {
            "converged": True,
            "goal_ne_m": [2.0, 0.0],
            "formation": "v",
            "target_altitude_m": 5.0,
            "aligned_samples": 3,
            "latest_timestamp": 100.0,
            "latest_per_agent_errors": {
                str(i): {"horizontal_error_m": 0.1, "vertical_error_m": 0.0}
                for i in range(10)
            },
        },
        "separation_metrics": {
            "minimum_horizontal_separation_m": 4.0,
            "safety_gate_passed": True,
        },
        "reproducibility": {"seed": 1024},
    }


def _central_events(path):
    events = [
        {"kind": "run_start", "drones": 10},
        {"kind": "control_mode", "mode": "centralized"},
        {"kind": "centroid_command", "north_m": 2.0, "east_m": 0.0},
        {
            "kind": "sample",
            "settled": True,
            "active": [True] * 10,
            "telemetry_fresh": [True] * 10,
            "positions_ned": [[float(i), 0.0, -5.0] for i in range(10)],
            "targets_ned": [[float(i) + 0.1, 0.0, -5.0] for i in range(10)],
            "utc": "test-time",
        },
    ]
    path.write_text("\n".join(json.dumps(row) for row in events) + "\n", encoding="utf-8")
    return path


def test_builds_matched_three_mode_sitl_summary(tmp_path):
    central = _write(tmp_path / "central.json", {
        "drones": 10,
        "outcome": "landed",
        "pass": True,
        "minimum_separation_m": 4.1,
        "checks": {"landing_confirmed": True},
    })
    dec = _write(tmp_path / "dec.json", _independent("decentralized"))
    hybrid = _write(tmp_path / "hybrid.json", _independent("hybrid"))
    output_path = tmp_path / "comparison.json"

    result = build_comparison(central, _central_events(tmp_path / "central.jsonl"),
                              dec, hybrid, output_path)

    assert result["pass"] is True
    assert result["acceptance"] == {
        "all_modes_converged": True,
        "all_modes_landed_disarmed": True,
        "all_modes_sampled_separation_at_least_2_5m": True,
    }
    assert result["results"]["centralized"]["terminal_snapshot_p95_3d_slot_error_m"] == pytest.approx(0.1)
    assert result["results"]["decentralized"]["terminal_snapshot_p95_3d_slot_error_m"] == pytest.approx(0.1)
    assert json.loads(output_path.read_text(encoding="utf-8"))["pass"] is True


def test_rejects_unmatched_peer_profile(tmp_path):
    central = _write(tmp_path / "central.json", {
        "drones": 10, "outcome": "landed", "pass": True,
        "minimum_separation_m": 4.1, "checks": {"landing_confirmed": True},
    })
    dec = _write(tmp_path / "dec.json", _independent("decentralized"))
    data = _independent("hybrid")
    data["peer_burst_loss"] = True
    hybrid = _write(tmp_path / "hybrid.json", data)

    with pytest.raises(ValueError, match="different peer-channel profile"):
        build_comparison(central, _central_events(tmp_path / "central.jsonl"),
                         dec, hybrid, tmp_path / "comparison.json")
