"""Offline SIM .BIN 3-D truth alignment tests (UNIT)."""
import math

import numpy as np
import pytest

from tools.bin_truth_audit import audit_trajectories


def test_audit_trajectories_recovers_3d_and_ellipsoid_separation():
    sim_by_drone = {
        0: np.array([[0.0, 0.0, 5.0], [2.0, 0.0, 5.0], [5.0, 0.0, 5.0]]),
        1: np.array([[0.0, 3.0, 7.0], [2.05, 0.4, 7.1], [5.0, 4.0, 7.0]]),
    }
    events = [
        {"kind": "agent_status", "drone_id": 0, "timestamp": 1.0, "airborne": True, "true_position_ne": [0.0, 0.0]},
        {"kind": "agent_status", "drone_id": 1, "timestamp": 1.1, "airborne": True, "true_position_ne": [0.0, 3.0]},
        {"kind": "agent_status", "drone_id": 0, "timestamp": 2.0, "airborne": True, "true_position_ne": [2.0, 0.0]},
        {"kind": "agent_status", "drone_id": 1, "timestamp": 2.1, "airborne": True, "true_position_ne": [2.05, 0.4]},
        # Stale sample (> 1.5 s apart) should not pair with earlier sample
        {"kind": "agent_status", "drone_id": 0, "timestamp": 10.0, "airborne": True, "true_position_ne": [2.0, 0.0]},
    ]
    res = audit_trajectories(events, sim_by_drone, gate_m=2.5, vertical_radius_m=2.0, max_dt_s=1.5)
    assert res["pair_samples"] == 3
    assert res["max_match_error_m"] == 0.0
    expected_h = math.hypot(0.05, 0.4)
    expected_3d = math.hypot(expected_h, 2.1)
    assert res["true_min_horizontal_m"] == pytest.approx(expected_h, abs=1e-3)
    assert res["true_min_3d_m"] == pytest.approx(expected_3d, abs=1e-3)
    assert res["true_min_ellipsoid_m"] > 2.5
    assert res["at_min_horizontal"]["pair"] == [1, 2]
