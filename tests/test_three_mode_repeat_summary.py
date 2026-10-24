from experiments.summarize_three_mode_repeats import summarize, wilson_interval


def test_repeat_summary_identifies_the_failed_gate_and_small_margin():
    report = {
        "scope": "synthetic", "reproducibility": {"revision": "test"},
        "criteria": {"minimum_separation_m": 2.5, "steady_p95_error_m": 1.0},
        "results": [
            {"scenario": "burst", "mode": "hybrid", "seed": 1,
             "minimum_separation_m": 2.502, "steady_p95_error_m": 0.2,
             "pass_criteria": True},
            {"scenario": "burst", "mode": "hybrid", "seed": 2,
             "minimum_separation_m": 2.4, "steady_p95_error_m": 1.2,
             "pass_criteria": False},
        ],
    }
    row = summarize(report)["groups"]["burst"]["hybrid"]
    assert row["passes"] == 1
    assert row["advisory_buffer_shortfalls"] == 2
    assert row["failures"][0]["failed_gates"] == ["separation", "tracking"]
    assert row["worst_clearance_margin_m"] < 0
    low, high = wilson_interval(1, 2)
    assert 0 < low < 0.5 < high < 1
