from experiments.compare_three_modes import MODES, trial


def test_ten_drone_mode_comparison_uses_same_requested_fleet_size():
    rows = [trial(mode, 20261024, "burst_repeated_outage", 10) for mode in MODES]

    assert [row["mode"] for row in rows] == ["centralized", "decentralized", "hybrid"]
    assert all(row["drones"] == 10 for row in rows)
    assert all(row["scenario"] == "burst_repeated_outage" for row in rows)
