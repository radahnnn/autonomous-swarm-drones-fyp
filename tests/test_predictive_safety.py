"""Behavior checks for the opt-in Phase 2 spacing experiment."""

import numpy as np

from experiments.test_network_sweep import run_single_trial
from swarm_core.config import (
    build_controllers_from_profile, get_active_profile, get_profile,
    set_active_profile,
)
from swarm_core.predictive_safety import project_acceleration


def test_projection_limits_closing_acceleration_before_vehicles_are_too_close():
    command, feasible = project_acceleration(
        nominal=np.array([-2.5, 0.0]),
        own_position=np.array([1.5, 0.0]),
        own_velocity=np.zeros(2),
        neighbors=[{"position": np.array([-1.5, 0.0]), "velocity": np.zeros(2)}],
    )
    assert feasible
    assert command[0] >= -0.2501


def test_projection_reports_when_available_acceleration_cannot_satisfy_constraint():
    _, feasible = project_acceleration(
        nominal=np.zeros(2),
        own_position=np.array([1.0, 0.0]),
        own_velocity=np.array([-1.5, 0.0]),
        neighbors=[{"position": np.array([-1.0, 0.0]),
                    "velocity": np.array([1.5, 0.0])}],
    )
    assert not feasible


def test_decentralized_controller_receives_declared_goal_gain():
    profile = get_profile("sitl_corrected_s1")
    _, decentralized, _ = build_controllers_from_profile(profile)
    assert decentralized.k_goal == profile.get("k_goal")


def test_low_noise_centralized_morph_uses_common_spacing_and_p95_metrics():
    previous_profile = get_active_profile().name
    try:
        set_active_profile("sitl_corrected_s1")
        result = run_single_trial(
            "centralized", 0.0, num_drones=5, seed=100,
            initial_layout="formation", gps_noise_std=0.04,
            predictive_safety=True,
        )
    finally:
        set_active_profile(previous_profile)
    assert result["min_recorded_distance_m"] >= 2.5
    assert result["steady_p95_slot_error_m"] <= 1.0
