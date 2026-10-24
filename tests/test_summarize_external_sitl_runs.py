from pathlib import Path

import pytest

from experiments.summarize_external_sitl_runs import summarize, wilson_interval


def run(seed=1, *, passed=True, loss=0.02, operator_loss=False,
        drones=5, control_mode="decentralized"):
    summary = {
        "kind": "external_agent_sitl_validation",
        "drones": drones,
        "control_mode": control_mode,
        "seed": seed,
        "topology": "split-loopback",
        "peer_loss": loss,
        "peer_latency_ms": 25.0,
        "peer_jitter_ms": 5.0,
        "peer_burst_loss": False,
        "peer_burst_start_rate": 0.04,
        "peer_burst_continue_rate": 0.65,
        "peer_duplicate_rate": 0.01,
        "peer_reorder_rate": 0.05,
        "peer_reorder_delay_ms": 150.0,
        "passed": passed,
        "completed_utc": "2026-10-22T00:00:00+00:00",
        "mission_convergence": {"converged": passed},
        "all_five_landed_disarmed": passed,
        "separation_metrics": {"minimum_horizontal_separation_m": 4.1},
        "peer_metrics": {
            "1": {
                "duplicated_packets": 2,
                "reorder_delayed_packets": 3,
                "rejected_replay": 4,
                "dropped_loss": 5,
                "injected_delay_samples": 95,
            }
        },
    }
    if operator_loss:
        summary["operator_loss"] = {
            "operator_process_killed": passed,
            "all_agents_were_airborne": passed,
            "all_agents_requested_mission_expiry_land": passed,
            "all_agents_landed_disarmed": passed,
            "seconds_after_operator_loss": 42.5,
        }
    return summary


def test_summarizer_aggregates_only_comparable_completed_runs():
    result = summarize([
        (Path("run-1.json"), run(seed=159)),
        (Path("run-2.json"), run(seed=161)),
    ])

    assert result["run_count"] == 2
    assert result["passed_count"] == 2
    assert result["observed_pass_fraction"] == 1.0
    assert result["all_runs_converged"] is True
    assert result["all_runs_landed_disarmed"] is True
    assert result["minimum_horizontal_separation_across_runs_m"] == 4.1
    assert result["total_duplicate_packets"] == 4
    assert result["total_reordered_packets"] == 6
    assert result["total_replay_rejections"] == 8
    assert result["total_dropped_loss_packets"] == 10
    assert result["total_accepted_for_impairment_queue"] == 190
    assert result["empirical_loss_fraction_across_runs"] == 0.05
    assert result["pass_fraction_wilson_95"] == pytest.approx(wilson_interval(2, 2))


@pytest.mark.parametrize(
    "second,match",
    [
        (run(seed=2, loss=0.05), "profile differs"),
        (run(seed=2, operator_loss=True), "profile differs"),
        ({"kind": "external_agent_sitl_validation"}, "incomplete run summary"),
    ],
)
def test_summarizer_rejects_mismatched_or_incomplete_runs(second, match):
    with pytest.raises(ValueError, match=match):
        summarize([(Path("first.json"), run()), (Path("second.json"), second)])


def test_summarizer_verifies_operator_loss_specific_checks():
    result = summarize([
        (Path("run-1.json"), run(seed=160, operator_loss=True)),
        (Path("run-2.json"), run(seed=165, operator_loss=True)),
    ])

    assert result["profile"]["operator_loss"] is True
    assert result["operator_loss_checks_all_passed"] is True
    assert result["maximum_seconds_to_landed_disarmed_after_operator_loss"] == 42.5
    assert result["runs"][0]["operator_loss"]["operator_process_killed"] is True


def test_summarizer_keeps_ten_drone_hybrid_runs_comparable_and_uses_dynamic_landing_key():
    ten_drone_run = run(drones=10, control_mode="hybrid")
    ten_drone_run.pop("all_five_landed_disarmed")
    ten_drone_run["all_drones_landed_disarmed"] = True

    result = summarize([
        (Path("hybrid-10-a.json"), ten_drone_run),
        (Path("hybrid-10-b.json"), {**ten_drone_run, "seed": 2}),
    ])

    assert result["profile"]["drones"] == 10
    assert result["profile"]["control_mode"] == "hybrid"
    assert result["all_runs_landed_disarmed"] is True
    assert result["runs"][0]["drones"] == 10
