"""
Unit tests for Configuration Profiles, Parameter Provenance, and Safety Distances (Phase 1E, 1F, 1H).
Tests:
1. Top-level package and module imports.
2. Configuration profile lookup, validation, and parameter retrieval.
3. Profile parameter propagation into Drone and SwarmSimulation.
4. Physical dynamics divergence between assumed_baseline and sitl_fitted profiles.
5. Strict safety distance hierarchy: 2*r_drone == d_collision < d_APF < d_spacing < R_comm.
6. Duplicate message rejection in hybrid controller sequence tracking.
7. Total coordinator link outage survivability.
"""

import pytest
import numpy as np

import swarm_core
import simulator
from swarm_core.config import (
    PROFILES,
    SwarmConfigProfile,
    get_profile,
    get_active_profile,
    set_active_profile,
)
from swarm_core.drone import Drone
from swarm_core.controllers.hybrid import HybridController, HybridMode
from swarm_core.formations import FormationType
from simulator.engine import SwarmSimulation


def test_package_imports():
    """Verify core packages and submodules can be imported cleanly without errors."""
    import swarm_core.drone
    import swarm_core.network
    import swarm_core.graph
    import swarm_core.formations
    import swarm_core.metrics
    import swarm_core.controllers.centralized
    import swarm_core.controllers.decentralized
    import swarm_core.controllers.hybrid
    import simulator.engine
    assert swarm_core is not None
    assert simulator is not None


def test_configuration_profile_lookup():
    """Verify available profiles, getters, and error handling for unknown profiles."""
    assert "assumed_baseline" in PROFILES
    assert "sitl_fitted" in PROFILES

    baseline = get_profile("assumed_baseline")
    assert isinstance(baseline, SwarmConfigProfile)
    assert baseline.name == "assumed_baseline"
    assert baseline.get("attitude_tau") == 0.18
    assert baseline.get("drag_coeff") == 0.20

    sitl = get_profile("sitl_fitted")
    assert sitl.name == "sitl_fitted"
    assert sitl.get("attitude_tau") == 0.992
    assert sitl.get("drag_coeff") == 0.637

    # Verify provenance metadata
    prov = sitl.get_provenance("attitude_tau")
    assert prov.provenance == "fitted from SITL"
    assert "experiments/validate_step_response_sitl.py" in prov.reference

    # Error handling for invalid profile
    with pytest.raises(ValueError, match="Unknown configuration profile"):
        get_profile("non_existent_profile")


def test_drone_profile_parameter_propagation():
    """Verify Drone instances inherit physics and sensor parameters from specified profiles."""
    d_base = Drone(drone_id=0, initial_position=[0.0, 0.0], profile="assumed_baseline")
    assert d_base.profile_name == "assumed_baseline"
    assert d_base.attitude_tau == 0.18
    assert d_base.drag_coeff == 0.20
    assert d_base.radius == 0.35
    assert d_base.measurement_noise_std == 0.04

    d_sitl = Drone(drone_id=1, initial_position=[0.0, 0.0], profile="sitl_fitted")
    assert d_sitl.profile_name == "sitl_fitted"
    assert d_sitl.attitude_tau == 0.992
    assert d_sitl.drag_coeff == 0.637
    assert d_sitl.radius == 0.35
    assert d_sitl.measurement_noise_std == 1.50


def test_dynamics_divergence_between_profiles():
    """
    Prove that assumed_baseline and sitl_fitted produce distinctly different
    kinematic and dynamic responses to the exact same acceleration step command.
    """
    dt = 0.05
    steps = 40  # 2.0 seconds

    d_base = Drone(drone_id=0, initial_position=[0.0, 0.0], profile="assumed_baseline")
    d_sitl = Drone(drone_id=1, initial_position=[0.0, 0.0], profile="sitl_fitted")

    # Command a step acceleration of 2.0 m/s^2 along X
    step_cmd = np.array([2.0, 0.0])
    d_base.set_control_input(step_cmd)
    d_sitl.set_control_input(step_cmd)

    for _ in range(steps):
        d_base.step(dt)
        d_sitl.step(dt)

    # 1. Faster lag response in assumed_baseline (tau=0.18s) vs sitl_fitted (tau=0.992s)
    # causes assumed_baseline to accelerate much faster initially
    # 2. Higher drag in sitl_fitted (cd=0.637) vs assumed_baseline (cd=0.20) limits terminal speed
    assert d_base.velocity[0] > d_sitl.velocity[0] + 0.5, (
        f"Expected baseline velocity ({d_base.velocity[0]:.2f}) to significantly exceed "
        f"SITL-fitted velocity ({d_sitl.velocity[0]:.2f})"
    )
    assert d_base.position[0] > d_sitl.position[0] + 1.0, (
        f"Expected baseline position ({d_base.position[0]:.2f}) to lead "
        f"SITL-fitted position ({d_sitl.position[0]:.2f})"
    )


def test_simulation_profile_propagation():
    """Verify SwarmSimulation correctly configures subsystems and records active profile."""
    drones = [Drone(i, [float(i * 2), 0.0], profile="sitl_fitted") for i in range(3)]
    sim = SwarmSimulation(drones, control_mode="centralized", profile="sitl_fitted")

    assert sim.profile_name == "sitl_fitted"
    assert sim.comm_range == 12.0
    assert sim.metrics.collision_threshold == 0.70
    assert sim.formation_spacing == 2.50
    assert sim.central_ctrl.collision_dist == 1.50  # apf_activation_dist for sitl_fitted
    assert sim.decentral_ctrl.safe_radius == 1.50


def test_safety_distance_hierarchy():
    """
    Formally verify the safety spatial hierarchy across all configuration profiles:
    2 * r_drone == d_collision < d_APF < d_spacing < R_comm
    """
    for prof_name in ["assumed_baseline", "sitl_fitted"]:
        prof = get_profile(prof_name)
        r_drone = prof.get("drone_radius")
        d_collision = prof.get("collision_threshold")
        d_apf = prof.get("apf_activation_dist")
        d_spacing = prof.get("nominal_spacing")
        r_comm = prof.get("comm_radius")

        # 1. Hard physical boundary: contact when inter-drone distance <= 2 * radius
        assert np.isclose(d_collision, 2.0 * r_drone), (
            f"Profile '{prof_name}': collision threshold ({d_collision}m) must equal "
            f"2 * drone_radius ({2.0 * r_drone}m)"
        )

        # 2. Emergency repulsive buffer must activate before physical collision occurs
        assert d_collision < d_apf, (
            f"Profile '{prof_name}': collision threshold ({d_collision}m) must be strictly "
            f"less than APF activation distance ({d_apf}m)"
        )

        # 3. APF activation distance must be comfortably inside nominal formation spacing
        assert d_apf < d_spacing, (
            f"Profile '{prof_name}': APF activation distance ({d_apf}m) must be strictly "
            f"less than nominal formation spacing ({d_spacing}m)"
        )

        # 4. Nominal spacing must be well within RF communication radius
        assert d_spacing < r_comm, (
            f"Profile '{prof_name}': nominal spacing ({d_spacing}m) must be strictly "
            f"less than RF communication radius ({r_comm}m)"
        )


def test_duplicate_message_rejection():
    """
    Verify that duplicate coordinator heartbeat packets with already-seen sequence numbers
    are rejected and do not count as valid heartbeat arrivals.
    """
    hybrid = HybridController()
    drone_id = 0
    t = 1.0

    # Initial valid heartbeat seq=100
    valid_1 = hybrid.process_coordinator_heartbeat(
        drone_id=drone_id,
        current_time=t,
        send_timestamp=t - 0.02,
        sequence_num=100,
        target_pos=np.array([1.0, 2.0]),
    )
    assert valid_1 is True
    assert hybrid.last_sequence_num[drone_id] == 100

    # Duplicate heartbeat arriving again with seq=100 (e.g. retransmission or multi-path)
    valid_dup = hybrid.process_coordinator_heartbeat(
        drone_id=drone_id,
        current_time=t + 0.01,
        send_timestamp=t - 0.01,
        sequence_num=100,
        target_pos=np.array([1.0, 2.0]),
    )
    assert valid_dup is False, "Duplicate sequence number must be rejected by HybridController"
    assert hybrid.last_sequence_num[drone_id] == 100

    # Stale/out-of-order seq=99
    valid_old = hybrid.process_coordinator_heartbeat(
        drone_id=drone_id,
        current_time=t + 0.02,
        send_timestamp=t - 0.01,
        sequence_num=99,
        target_pos=np.array([1.0, 2.0]),
    )
    assert valid_old is False, "Stale/out-of-order sequence number must be rejected"

    # Strictly newer seq=101
    valid_new = hybrid.process_coordinator_heartbeat(
        drone_id=drone_id,
        current_time=t + 0.05,
        send_timestamp=t + 0.03,
        sequence_num=101,
        target_pos=np.array([1.0, 2.0]),
    )
    assert valid_new is True, "Higher sequence number must be accepted"
    assert hybrid.last_sequence_num[drone_id] == 101


def test_total_coordinator_outage_transition():
    """
    Verify that 100% coordinator link outage safely transitions all drones
    to decentralized fallback without unhandled exceptions or inter-drone collision.
    """
    drones = [
        Drone(0, initial_position=[0.0, 0.0]),
        Drone(1, initial_position=[2.5, 0.0]),
        Drone(2, initial_position=[-2.5, 0.0]),
    ]
    sim = SwarmSimulation(drones, control_mode="hybrid", comm_range=15.0)
    sim.set_formation(FormationType.LINE)

    # 1. Run 10 steps (0.5s) normal operation
    for _ in range(10):
        sim.step()

    # Verify initially in CENTRALIZED mode
    for d in drones:
        assert sim.hybrid_ctrl.get_drone_mode(d.id) == HybridMode.CENTRALIZED

    # 2. Sever coordinator link completely (100% loss)
    sim.set_coordinator_link(False)

    # Step for 30 steps (1.5s): exceeds degrade_timeout (0.5s)
    for _ in range(30):
        sim.step()

    # Verify all drones transitioned to DECENTRALIZED_FALLBACK
    for d in drones:
        assert sim.hybrid_ctrl.get_drone_mode(d.id) == HybridMode.DECENTRALIZED_FALLBACK
        assert sim.hybrid_ctrl.get_alpha(d.id) == 0.0

    # Check metrics
    summary = sim.metrics.get_summary()
    assert summary["any_collision"] == 0.0
    assert summary["min_recorded_distance_m"] >= 0.70


def test_controller_parameters_and_thresholds_from_profile():
    """
    Verify that Centralized, Decentralized, and Hybrid controller gains
    and thresholds are populated directly from the selected profile.
    """
    drones = [Drone(i, [float(i * 2), 0.0], profile="assumed_baseline") for i in range(3)]
    sim_base = SwarmSimulation(drones, profile="assumed_baseline")

    # Centralized controller gains
    assert sim_base.central_ctrl.kp == 1.8
    assert sim_base.central_ctrl.kd == 2.2
    assert sim_base.central_ctrl.k_repulse == 4.0
    assert sim_base.central_ctrl.collision_dist == 1.20

    # Decentralized controller gains
    assert sim_base.decentral_ctrl.k_sep == 4.0
    assert sim_base.decentral_ctrl.k_align == 1.6
    assert sim_base.decentral_ctrl.k_form == 1.4
    assert sim_base.decentral_ctrl.safe_radius == 1.20

    # Hybrid controller thresholds
    assert sim_base.hybrid_ctrl.degrade_timeout == 0.50
    assert sim_base.hybrid_ctrl.recovery_ratio_threshold == 0.70
    assert sim_base.hybrid_ctrl.min_dwell_time == 2.00
    assert sim_base.hybrid_ctrl.ramp_duration == 0.80
    assert sim_base.hybrid_ctrl.window_size == 20

    # Switch to SITL fitted profile
    drones_sitl = [Drone(i, [float(i * 2), 0.0], profile="sitl_fitted") for i in range(3)]
    sim_sitl = SwarmSimulation(drones_sitl, profile="sitl_fitted")

    # SITL fitted adjustments
    assert sim_sitl.central_ctrl.collision_dist == 1.50
    assert sim_sitl.decentral_ctrl.safe_radius == 1.50


def test_hybrid_feedforward_receives_profile_drag_coeff():
    """
    Verify that compute_hybrid_control receives the active profile drag coefficient
    and applies drag compensation feedforward matching the profile.
    """
    v_target = 0.5
    d_base = Drone(0, initial_position=[0.0, 0.0], initial_velocity=[v_target, 0.0], profile="assumed_baseline")
    d_sitl = Drone(0, initial_position=[0.0, 0.0], initial_velocity=[v_target, 0.0], profile="sitl_fitted")

    sim_base = SwarmSimulation([d_base], control_mode="hybrid", profile="assumed_baseline", latency_mean=0.0)
    sim_sitl = SwarmSimulation([d_sitl], control_mode="hybrid", profile="sitl_fitted", latency_mean=0.0)

    # Provide matched centroid velocity target (error = 0, so command is purely feedforward u_ff = c_d * v)
    sim_base.centroid_velocity = np.array([v_target, 0.0])
    sim_sitl.centroid_velocity = np.array([v_target, 0.0])

    # Execute one step to compute control inputs
    sim_base.step()
    sim_sitl.step()

    # Drag feedforward is u_ff = drag_coeff * target_velocity
    # In assumed_baseline, drag_coeff = 0.20 -> u_ff_x = 0.20 * 0.5 = 0.10 m/s^2
    # In sitl_fitted, drag_coeff = 0.637 -> u_ff_x = 0.637 * 0.5 = 0.3185 m/s^2
    # The difference in feedforward term must equal (0.637 - 0.20) * 0.5 = 0.2185 m/s^2
    diff_cmd = sim_sitl.drones[0].commanded_accel[0] - sim_base.drones[0].commanded_accel[0]
    expected_diff = (0.637 - 0.20) * v_target
    assert np.isclose(diff_cmd, expected_diff, atol=1e-4), (
        f"Hybrid feedforward did not receive profile drag coefficient. "
        f"Diff: {diff_cmd:.4f}, Expected: {expected_diff:.4f}"
    )


def test_set_formation_preserves_profile_spacing():
    """
    Verify that set_formation() preserves the active profile formation spacing
    when spacing=None is passed, rather than silently overwriting it with 2.5.
    """
    drones = [Drone(i, [float(i * 2), 0.0], profile="assumed_baseline") for i in range(3)]
    sim = SwarmSimulation(drones, profile="assumed_baseline")
    initial_spacing = sim.formation_spacing

    # Call set_formation without specifying spacing
    sim.set_formation(FormationType.CIRCLE)
    assert sim.formation_spacing == initial_spacing

    # Call set_formation with explicit spacing
    sim.set_formation(FormationType.GRID, spacing=4.0)
    assert sim.formation_spacing == 4.0

    # Call again without spacing: should preserve 4.0
    sim.set_formation(FormationType.LINE)
    assert sim.formation_spacing == 4.0


def test_optional_pymavlink_behavior(monkeypatch):
    """
    Verify that:
    1. Core imports and SITL adapter classes function without pymavlink installed.
    2. SITL connection fails clearly and safely with ImportError when pymavlink is absent.
    """
    import sitl.mavlink_swarm_adapter as adapter_mod
    from sitl.common_frame import CommonCoordinateFrame

    # Simulate pymavlink absence by setting mavutil = None in adapter module
    monkeypatch.setattr(adapter_mod, "mavutil", None)

    frame = CommonCoordinateFrame()
    iface = adapter_mod.MAVLinkDroneInterface(
        sysid=1, port=14552, label="Drone 1", frame=frame
    )

    # Calling connect() must raise ImportError with clear user instructions
    with pytest.raises(ImportError, match="pymavlink is required to connect to Drone 1"):
        iface.connect()

