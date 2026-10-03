"""
Deterministic Parity Tests: 3-Drone Simulation vs. SITL Pipeline.
Phase 4 Verification Suite.

Tests specifically prove for a three-drone fleet:
1. Same formation type and spacing produce the same formation offsets.
2. Same current positions and target slots produce the same Hungarian assignment.
3. Drone IDs and array ordering are consistent.
4. The selected configuration profile is the same in both paths.
5. The three-drone SITL adapter produces one target per configured drone.
6. Each target is associated with the correct system ID.
7. Common-coordinate-frame transformations are deterministic.
8. A fixed input scenario gives deterministic target generation.
9. Any intentional difference between simulator commands and SITL setpoints is explicitly identified.
"""

from unittest.mock import MagicMock
import numpy as np
import pytest

from swarm_core.config import get_profile, build_controllers_from_profile
from swarm_core.drone import Drone
from swarm_core.formations import (
    FormationGenerator,
    FormationType,
    assign_optimal_slots,
    compute_formation_slots,
    compute_desired_neighbor_offsets,
)
from simulator.engine import SwarmSimulation
from sitl.common_frame import CommonCoordinateFrame
from sitl.mavlink_swarm_adapter import (
    MAVLinkSwarmAdapter,
    acceleration_to_position_setpoint,
    DRONE_SPECS,
)


@pytest.mark.parametrize("formation", [
    FormationType.LINE,
    FormationType.V_SHAPE,
    FormationType.CIRCLE,
    FormationType.GRID,
])
@pytest.mark.parametrize("spacing", [2.0, 3.5])
def test_three_drone_same_formation_offsets(formation, spacing):
    """
    1. Verifies that the same formation type and spacing produce identical
    formation offsets for exactly three drones.
    """
    n_drones = 3
    direct_offsets = FormationGenerator.get_formation_offsets(formation, n_drones, spacing=spacing)
    shared_offsets, world_slots, _ = compute_formation_slots(
        formation_type=formation,
        num_drones=n_drones,
        centroid=np.array([5.0, -2.5]),
        spacing=spacing,
    )

    assert direct_offsets.shape == (3, 2)
    assert shared_offsets.shape == (3, 2)
    assert np.allclose(direct_offsets, shared_offsets, atol=1e-12)
    assert np.allclose(world_slots, shared_offsets + np.array([5.0, -2.5]), atol=1e-12)


def test_three_drone_same_hungarian_assignment():
    """
    2. Verifies that the same current positions and target slots produce the
    exact same Hungarian optimal assignment across simulator and SITL paths.
    """
    n = 3
    spacing = 3.0
    centroid = np.array([0.0, 0.0])

    # Drones initialized out of order (permuted)
    permuted_positions = np.array([
        [-2.5, 3.0],   # Drone 0 closer to slot 2 (right wing)
        [0.0, 0.2],    # Drone 1 closer to slot 0 (apex)
        [-2.6, -3.1],  # Drone 2 closer to slot 1 (left wing)
    ])

    # 1. Simulator path
    drones_sim = [Drone(i, initial_position=permuted_positions[i]) for i in range(n)]
    sim = SwarmSimulation(drones_sim, control_mode="centralized", profile="sitl_fitted")
    sim.set_formation(FormationType.V_SHAPE, centroid=centroid, spacing=spacing)

    offsets_sim, world_sim, assigned_sim = compute_formation_slots(
        formation_type=sim.current_formation,
        num_drones=n,
        centroid=sim.centroid_target,
        spacing=sim.formation_spacing,
        current_positions=np.array([d.position for d in sim.drones]),
    )

    # 2. SITL adapter path
    adapter = MAVLinkSwarmAdapter(control_mode="centralized")
    for i, p in enumerate(permuted_positions):
        adapter.core_drones[i].position = p.copy()
    adapter.set_formation(FormationType.V_SHAPE)
    adapter.formation_spacing = spacing
    adapter.centroid_target[:2] = centroid

    current_pos_sitl = np.array([d.position for d in adapter.core_drones])
    offsets_sitl, world_sitl, assigned_sitl = compute_formation_slots(
        formation_type=adapter.current_formation,
        num_drones=n,
        centroid=adapter.centroid_target[:2],
        spacing=adapter.formation_spacing,
        current_positions=current_pos_sitl,
    )

    # Both paths must match bitwise
    assert np.allclose(offsets_sim, offsets_sitl, atol=1e-12)
    assert np.allclose(world_sim, world_sitl, atol=1e-12)
    assert np.allclose(assigned_sim, assigned_sitl, atol=1e-12)


def test_three_drone_id_and_array_ordering_consistency():
    """
    3. Verifies that drone IDs and array ordering are consistent:
    - 3 drones in fleet
    - swarm_core drone IDs strictly 0, 1, 2
    - SITL MAVLink system IDs strictly 1, 2, 3
    - 1-to-1 correspondence preserved across all arrays.
    """
    adapter = MAVLinkSwarmAdapter()
    assert len(adapter.interfaces) == 3
    assert len(adapter.core_drones) == 3
    assert len(DRONE_SPECS) == 3

    for i in range(3):
        assert adapter.core_drones[i].id == i
        assert adapter.interfaces[i].sysid == DRONE_SPECS[i]["sysid"]
        assert adapter.interfaces[i].sysid == i + 1


def test_three_drone_profile_selection_parity():
    """
    4. Verifies that the selected configuration profile (sitl_fitted)
    provisions identical gains, thresholds, and parameters in both paths.
    """
    profile = get_profile("sitl_fitted")

    # Simulator initialization
    drones_sim = [Drone(i, initial_position=[0.0, 0.0], profile="sitl_fitted") for i in range(3)]
    sim = SwarmSimulation(drones_sim, control_mode="hybrid", profile="sitl_fitted")

    # SITL adapter initialization
    adapter = MAVLinkSwarmAdapter(control_mode="hybrid")

    # Controller parameters must be identical
    assert sim.central_ctrl.kp == adapter.central_ctrl.kp == profile.get("centralized_kp")
    assert sim.central_ctrl.kd == adapter.central_ctrl.kd == profile.get("centralized_kd")
    assert sim.central_ctrl.k_repulse == adapter.central_ctrl.k_repulse == profile.get("k_repulse")
    assert sim.central_ctrl.collision_dist == adapter.central_ctrl.collision_dist == profile.get("apf_activation_dist")

    assert sim.decentral_ctrl.k_sep == adapter.decentral_ctrl.k_sep == profile.get("k_repulse")
    assert sim.decentral_ctrl.k_align == adapter.decentral_ctrl.k_align == profile.get("decentralized_kv")
    assert sim.decentral_ctrl.k_form == adapter.decentral_ctrl.k_form == profile.get("decentralized_k_form")
    assert sim.decentral_ctrl.safe_radius == adapter.decentral_ctrl.safe_radius == profile.get("apf_activation_dist")

    assert sim.hybrid_ctrl.degrade_timeout == adapter.hybrid_ctrl.degrade_timeout == profile.get("hybrid_degrade_timeout")
    assert sim.hybrid_ctrl.min_dwell_time == adapter.hybrid_ctrl.min_dwell_time == profile.get("hybrid_dwell_time")
    assert sim.hybrid_ctrl.recovery_ratio_threshold == adapter.hybrid_ctrl.recovery_ratio_threshold == profile.get("hybrid_recovery_ratio")


def test_three_drone_sitl_adapter_produces_one_target_per_drone():
    """
    5. Verifies that the three-drone SITL adapter produces exactly one target
    per configured drone in its control cycle.
    """
    adapter = MAVLinkSwarmAdapter(control_mode="hybrid", packet_loss=0.0)

    # Attach mock connections
    for iface in adapter.interfaces:
        iface.conn = MagicMock()
        iface.connected = True
        iface.home_global_ned = np.zeros(3)
        iface.global_ned = np.zeros(3)
        iface.conn.recv_match.return_value = None

    target_setpoints = adapter.run_control_cycle(current_time=0.1, dt=0.1)

    assert len(target_setpoints) == 3
    for sp in target_setpoints:
        assert isinstance(sp, np.ndarray)
        assert sp.shape == (3,)
        assert np.isclose(sp[2], -adapter.cruise_alt)


def test_three_drone_each_target_associated_with_correct_sysid():
    """
    6. Verifies that each target setpoint is associated and dispatched to the
    correct MAVLink system ID (1, 2, 3) without cross-talk or index shifts.
    """
    adapter = MAVLinkSwarmAdapter(control_mode="centralized", packet_loss=0.0)
    mock_conns = []

    for i, iface in enumerate(adapter.interfaces):
        mock_conn = MagicMock()
        mock_conn.target_system = iface.sysid
        iface.conn = mock_conn
        iface.connected = True
        iface.home_global_ned = np.array([float(i * 2), 0.0, -5.0])
        iface.global_ned = iface.home_global_ned.copy()
        iface.conn.recv_match.return_value = None
        mock_conns.append(mock_conn)

    adapter.run_control_cycle(current_time=0.1, dt=0.1)

    for i, iface in enumerate(adapter.interfaces):
        conn = mock_conns[i]
        assert conn.mav.set_position_target_local_ned_send.called
        call_args = conn.mav.set_position_target_local_ned_send.call_args[0]
        # call_args structure: (time_boot_ms, target_system, target_component, frame, ...)
        dispatched_sysid = call_args[1]
        assert dispatched_sysid == iface.sysid
        assert dispatched_sysid == i + 1


def test_three_drone_common_coordinate_frame_deterministic():
    """
    7. Verifies that CommonCoordinateFrame geodetic-to-metric conversions
    are deterministic and reversible within sub-millimeter precision.
    """
    frame = CommonCoordinateFrame(datum_lat=-35.3632621, datum_lon=149.1652374, datum_alt=584.0)

    # 3 distinct drone positions
    drone_pts = [
        np.array([0.0, 0.0, -5.0]),
        np.array([-3.0, -3.5, -5.0]),
        np.array([-3.0, 3.5, -5.0]),
    ]

    for pt in drone_pts:
        lat, lon, alt = frame.global_ned_to_gps(pt[0], pt[1], pt[2])
        reconstructed = frame.gps_to_global_ned(lat, lon, alt)
        error = np.linalg.norm(pt - reconstructed)
        assert error < 1e-4, f"Deterministic round-trip error too large: {error} m"


def test_three_drone_fixed_input_scenario_deterministic_target_generation():
    """
    8. Verifies that a fixed input scenario gives bitwise deterministic target
    slot coordinates across multiple repeated runs.
    """
    positions = np.array([[0.0, 0.0], [-3.0, -3.5], [-3.0, 3.5]])
    centroid = np.array([10.0, 5.0])

    run1 = compute_formation_slots(FormationType.V_SHAPE, 3, centroid, spacing=3.5, current_positions=positions)
    run2 = compute_formation_slots(FormationType.V_SHAPE, 3, centroid, spacing=3.5, current_positions=positions)

    assert np.array_equal(run1[0], run2[0])
    assert np.array_equal(run1[1], run2[1])
    assert np.array_equal(run1[2], run2[2])


def test_three_drone_intentional_differences_explicitly_identified():
    """
    9. Verifies that intentional differences between simulator commands
    (acceleration in m/s^2) and SITL setpoints (local-NED position in meters
    via lead filter) are explicitly identified and exposed in code and docstrings.
    """
    import sitl.mavlink_swarm_adapter as adapter_mod

    # 1. Module documentation must contain academic parity disclaimer
    doc = adapter_mod.__doc__
    assert "Reuses the swarm_core controller implementations and translates guidance outputs into ArduPilot local-NED position setpoints" in doc
    assert "ArduPilot’s onboard position controller and SITL dynamics remain part of the execution path" in doc

    # 2. Must not contain prohibited claims
    assert "EXACT SAME" not in doc

    # 3. Acceleration-to-position helper must be explicit about lead scale
    pos = np.array([0.0, 0.0])
    accel = np.array([1.0, 0.0])
    dt = 0.1
    # Lead compensation in hybrid mode uses gamma = 2.0
    sp_hyb = acceleration_to_position_setpoint(pos, accel, dt=dt, cruise_alt=5.05, lookahead_scale=2.0)
    assert np.isclose(sp_hyb[0], 0.20)  # 1.0 * 0.1 * 2.0
    assert np.isclose(sp_hyb[2], -5.05)


def test_three_drone_desired_neighbor_offsets_consensus():
    """
    Verifies compute_desired_neighbor_offsets returns correct relative
    displacement vectors for a 3-drone formation.
    """
    targets = np.array([[0.0, 0.0], [-3.0, 3.5], [-3.0, -3.5]])
    offsets_d0 = compute_desired_neighbor_offsets(0, targets, neighbor_ids=[1, 2], drone_ids=[0, 1, 2])

    assert np.allclose(offsets_d0[1], np.array([3.0, -3.5]))
    assert np.allclose(offsets_d0[2], np.array([3.0, 3.5]))
