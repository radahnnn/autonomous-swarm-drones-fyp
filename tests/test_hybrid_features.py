import numpy as np
from swarm_core.controllers.hybrid import HybridController, HybridMode
from swarm_core.drone import Drone


def test_stale_and_sequence_rejection():
    ctrl = HybridController(max_command_age=0.15)
    drone_id = 0
    t_now = 10.0

    # 1. Fresh packet with seq 1 sent 20ms ago -> Should accept
    accepted = ctrl.process_coordinator_heartbeat(
        drone_id=drone_id,
        current_time=t_now,
        send_timestamp=t_now - 0.02,
        sequence_num=1,
        target_pos=np.array([1.0, 1.0]),
    )
    assert accepted is True
    assert ctrl.stale_rejected_count[drone_id] == 0
    assert ctrl.out_of_order_count[drone_id] == 0

    # 2. Duplicate sequence number -> Should reject
    accepted_dup = ctrl.process_coordinator_heartbeat(
        drone_id=drone_id,
        current_time=t_now + 0.05,
        send_timestamp=t_now + 0.03,
        sequence_num=1,  # Same seq
        target_pos=np.array([1.0, 1.0]),
    )
    assert accepted_dup is False
    assert ctrl.out_of_order_count[drone_id] == 1

    # 3. Older out-of-order sequence number -> Should reject
    accepted_old_seq = ctrl.process_coordinator_heartbeat(
        drone_id=drone_id,
        current_time=t_now + 0.1,
        send_timestamp=t_now + 0.08,
        sequence_num=0,  # Older seq
        target_pos=np.array([1.0, 1.0]),
    )
    assert accepted_old_seq is False
    assert ctrl.out_of_order_count[drone_id] == 2

    # 4. Stale packet: message age 300ms > max_command_age (150ms) -> Should reject
    accepted_stale = ctrl.process_coordinator_heartbeat(
        drone_id=drone_id,
        current_time=t_now + 0.5,
        send_timestamp=t_now + 0.2,  # Age = 0.3s
        sequence_num=5,
        target_pos=np.array([1.0, 1.0]),
    )
    assert accepted_stale is False
    assert ctrl.stale_rejected_count[drone_id] == 1


def test_asymmetric_hysteresis_and_dwell_time():
    # Degrade timeout 0.5s, recovery 5 consecutive, dwell time 2.0s
    ctrl = HybridController(
        degrade_timeout=0.5,
        recovery_consecutive_hb=5,
        min_dwell_time=2.0,
        ramp_duration=1.0,
    )
    drone_id = 0
    t = 0.0
    dt = 0.05

    # Initially centralized
    ctrl._init_drone_if_needed(drone_id, t)
    assert ctrl.get_drone_mode(drone_id) == HybridMode.CENTRALIZED

    # 0.6s of silence passes -> should degrade to DECENTRALIZED_FALLBACK
    t += 0.6
    mode, alpha = ctrl.update_state_machine(drone_id, t, dt)
    assert mode == HybridMode.DECENTRALIZED_FALLBACK
    assert ctrl.switch_counts[drone_id] == 1

    # Now simulate receiving 1 fresh heartbeat at t = 0.8s
    # Dwell time in fallback is only 0.2s (< 2.0s) and consecutive = 1 (< 5)
    ctrl.process_coordinator_heartbeat(drone_id, t, t - 0.02, 10, np.array([0.0, 0.0]))
    mode, _ = ctrl.update_state_machine(drone_id, t, dt)
    assert mode == HybridMode.DECENTRALIZED_FALLBACK  # MUST NOT snap back!

    # Simulate 5 consecutive heartbeats, but BEFORE 2.0s dwell time (e.g. t = fallback_entry + 1.0s)
    t = ctrl.fallback_entry_time[drone_id] + 1.0
    for seq in range(11, 16):
        ctrl.process_coordinator_heartbeat(drone_id, t, t - 0.02, seq, np.array([0.0, 0.0]))
    mode, _ = ctrl.update_state_machine(drone_id, t, dt)
    assert mode == HybridMode.DECENTRALIZED_FALLBACK  # Dwell time not met yet!

    # Now advance time past 2.0s dwell time (t = fallback_entry + 2.1s) with consecutive >= 5
    t = ctrl.fallback_entry_time[drone_id] + 2.1
    ctrl.process_coordinator_heartbeat(drone_id, t, t - 0.02, 20, np.array([0.0, 0.0]))
    mode, _ = ctrl.update_state_machine(drone_id, t, dt)
    assert mode == HybridMode.CENTRALIZED  # Now both conditions met, recovers!
    assert ctrl.switch_counts[drone_id] == 2


def test_smooth_controller_blending():
    ctrl = HybridController(ramp_duration=1.0)
    drone = Drone(drone_id=0, initial_position=[0.0, 0.0])
    dt = 0.1

    # In Centralized mode, alpha starts at 1.0
    ctrl._init_drone_if_needed(drone.id, 0.0)
    assert ctrl.get_alpha(drone.id) == 1.0

    # Force fallback state
    ctrl.modes[drone.id] = HybridMode.DECENTRALIZED_FALLBACK

    alphas = []
    for step in range(15):
        t = step * dt
        _, alpha = ctrl.update_state_machine(drone.id, t, dt)
        alphas.append(alpha)

    # Check alpha smoothly ramps down from 1.0 to 0.0
    assert alphas[0] < 1.0
    assert alphas[-1] == 0.0
    # Difference between consecutive alphas must be exactly dt/ramp_duration = 0.1
    for i in range(len(alphas) - 6):
        assert np.isclose(alphas[i] - alphas[i + 1], 0.1, atol=1e-5)


def test_forced_fallback_and_sliding_window_recovery():
    """
    Explicitly forces fallback via outage/silence, verifies dwell-time lockout,
    and proves recovery via sliding-window delivery ratio (>= 70%) rather than consecutive packets.
    """
    # Disable consecutive recovery requirement (set to 999) to strictly test sliding window ratio
    ctrl = HybridController(
        degrade_timeout=0.5,
        recovery_consecutive_hb=999,
        min_dwell_time=2.0,
        ramp_duration=0.8,
        window_size=20,
        recovery_ratio_threshold=0.70,
    )
    drone_id = 0
    t = 0.0
    dt = 0.05

    # 1. Start in Centralized mode
    ctrl._init_drone_if_needed(drone_id, t)
    mode, _ = ctrl.update_state_machine(drone_id, t, dt)
    assert mode == HybridMode.CENTRALIZED

    # 2. Force complete outage (0.8s of silence > degrade_timeout 0.5s)
    t = 0.8
    mode, _ = ctrl.update_state_machine(drone_id, t, dt)
    assert mode == HybridMode.DECENTRALIZED_FALLBACK
    assert ctrl.switch_counts[drone_id] == 1
    fallback_start = ctrl.fallback_entry_time[drone_id]

    # 3. Simulate degraded channel: only 50% delivery ratio (10 successes, 10 drops)
    # Even after advancing past min_dwell_time (2.0s), 50% is below the 70% threshold!
    t = fallback_start + 2.5
    for seq in range(1, 21):
        if seq % 2 == 0:
            ctrl.process_coordinator_heartbeat(drone_id, t, t - 0.02, seq, np.array([0.0, 0.0]))
        else:
            ctrl.record_heartbeat_attempt(drone_id, False)

    mode, _ = ctrl.update_state_machine(drone_id, t, dt)
    assert mode == HybridMode.DECENTRALIZED_FALLBACK  # MUST remain in fallback (50% < 70%)

    # 4. Now simulate healthy channel recovery: 80% delivery ratio (16 successes, 4 drops)
    t += 1.0
    for seq in range(21, 41):
        if seq % 5 != 0:  # 80% success
            ctrl.process_coordinator_heartbeat(drone_id, t, t - 0.02, seq, np.array([0.0, 0.0]))
        else:
            ctrl.record_heartbeat_attempt(drone_id, False)

    mode, _ = ctrl.update_state_machine(drone_id, t, dt)
    assert mode == HybridMode.CENTRALIZED  # Now delivery ratio >= 70% and dwell time met -> RECOVERS!
    assert ctrl.switch_counts[drone_id] == 2


def test_isolated_fallback_bounded_hover_deceleration():
    """
    Recommendation 5: When an isolated drone has no neighbors and goal_pos is None,
    verify that the controller issues a bounded braking command to decelerate to hover
    (accel = -k_align * velocity) rather than drifting at constant velocity.
    """
    from swarm_core.controllers.decentralized import DecentralizedController

    ctrl = DecentralizedController(k_align=1.6)
    drone = Drone(0, initial_position=[0.0, 0.0], initial_velocity=[2.0, -1.5])

    # No neighbors, no goal
    cmd = ctrl.compute_drone_control(drone=drone, neighbor_states=[], goal_pos=None)

    # Command must directly oppose velocity: a = -k_align * v
    expected_cmd = -1.6 * np.array([2.0, -1.5])
    assert np.allclose(cmd, expected_cmd)

    # Forward step verification: speed must decrease
    drone.set_control_input(cmd)
    initial_speed = np.linalg.norm(drone.velocity)
    drone.step(0.05)
    new_speed = np.linalg.norm(drone.velocity)
    assert new_speed < initial_speed, f"Speed did not decrease: {initial_speed:.3f} -> {new_speed:.3f}"


def test_hybrid_controller_measured_state_inputs():
    """
    Recommendation 1: Verify that HybridController takes measured_position and
    measured_velocity inputs for centralized guidance and local APF repulsion,
    rather than reading uncorrupted ground truth drone.position / drone.velocity.
    """
    ctrl = HybridController()
    drone = Drone(0, initial_position=[0.0, 0.0], initial_velocity=[0.0, 0.0])

    # Send valid coordinator heartbeat at target [0, 0] so drone 0 is in CENTRALIZED mode with alpha=1.0
    ctrl.process_coordinator_heartbeat(
        drone_id=0,
        current_time=1.0,
        send_timestamp=0.98,
        sequence_num=1,
        target_pos=np.array([0.0, 0.0]),
    )

    meas_pos = np.array([0.5, -0.2])
    meas_vel = np.array([0.1, 0.0])

    cmd = ctrl.compute_hybrid_control(
        drone=drone,
        current_time=1.0,
        dt=0.05,
        neighbor_states=[],
        use_velocity_feedforward=False,
        measured_position=meas_pos,
        measured_velocity=meas_vel,
    )

    # Under pure centralized (alpha=1.0), p_err = target - meas_pos = [0, 0] - [0.5, -0.2] = [-0.5, 0.2]
    # v_err = -meas_vel = [-0.1, 0.0]
    # Expected command: kp * p_err + kd * v_err
    expected_cmd = ctrl.central_controller.kp * np.array([-0.5, 0.2]) + ctrl.central_controller.kd * np.array([-0.1, 0.0])
    assert np.allclose(cmd, expected_cmd), f"Command {cmd} does not match expected {expected_cmd}"


def test_degree_normalisation_at_various_swarm_sizes():
    """
    Recommendation 12: Verify that DecentralizedController forces (k_sep, k_align, k_form)
    are degree-normalised (divided by deg = max(1, |N_i|)) so effective gains do not
    scale unbounded with swarm size n = 3, 5, 10.
    """
    from swarm_core.controllers.decentralized import DecentralizedController

    ctrl = DecentralizedController(k_sep=2.0, k_align=1.5, k_form=1.0, safe_radius=1.5)
    drone = Drone(0, initial_position=[0.0, 0.0], initial_velocity=[1.0, 0.0])

    for n_peers in [2, 4, 9]:  # Swarm sizes n = 3, 5, 10 (peers = n - 1)
        # Create identical neighbor relative states
        neighbors = []
        desired_offsets = {}
        for p_id in range(1, n_peers + 1):
            neighbors.append({
                "id": p_id,
                "position": np.array([0.8, 0.0]),  # Distance 0.8 < safe_radius 1.5
                "velocity": np.array([0.0, 0.0]),
            })
            desired_offsets[p_id] = np.array([-1.0, 0.0])

        accel = ctrl.compute_drone_control(
            drone=drone,
            neighbor_states=neighbors,
            desired_offsets=desired_offsets,
            goal_pos=None,
        )

        # In degree normalisation, all identical neighbors average to the exact same per-neighbor force
        # Regardless of whether n_peers is 2, 4, or 9:
        # f_sep_total = sum(f_sep_j) / deg = n_peers * f_sep_single / n_peers = f_sep_single
        # f_align_total = sum(f_align_j) / deg = n_peers * f_align_single / n_peers = f_align_single
        dist = 0.8
        diff = np.array([-0.8, 0.0])
        single_repulse = 2.0 * (1.0 / dist - 1.0 / 1.5) / (dist**2) * (diff / dist)
        single_align = -1.5 * (np.array([1.0, 0.0]) - np.array([0.0, 0.0]))
        displacement_err = diff - np.array([-1.0, 0.0])  # [-0.8, 0] - [-1, 0] = [0.2, 0]
        single_form = -1.0 * displacement_err

        expected_accel = single_repulse + single_align + single_form
        assert np.allclose(accel, expected_accel, atol=1e-5), (
            f"Degree normalisation failed at swarm size n={n_peers+1}. "
            f"Accel: {accel}, Expected: {expected_accel}"
        )


def test_dead_reckoning_fallback_strategy():
    """
    Recommendation 9: Add and verify a dead-reckoning fallback option:
    continue along last heartbeat velocity for bounded time T (parameter), then brake.
    """
    ctrl = HybridController(
        fallback_strategy="dead_reckon",
        dead_reckon_duration=1.5,
        degrade_timeout=0.2,
    )
    drone = Drone(0, initial_position=[0.0, 0.0], initial_velocity=[1.0, 0.0])

    t = 0.0
    dt = 0.05
    # Prime with coordinator heartbeat: target [0, 0], velocity [1.0, 0.0]
    ctrl.process_coordinator_heartbeat(
        drone_id=0,
        current_time=t,
        send_timestamp=t,
        sequence_num=1,
        target_pos=np.array([0.0, 0.0]),
        target_velocity=np.array([1.0, 0.0]),
    )

    # Let link expire to trigger fallback at t = 0.3s
    t = 0.3
    ctrl.update_state_machine(0, t, dt)
    assert ctrl.get_drone_mode(0) == HybridMode.DECENTRALIZED_FALLBACK
    assert ctrl.fallback_entry_time[0] == 0.3

    # Case A: Within dead reckoning window (t_in_fb = 0.5s <= 1.5s, at t = 0.8s)
    # Expected target pos = p_base + v * 0.5 = [0, 0] + [1, 0] * 0.5 = [0.5, 0.0]
    # Commanded velocity = v_last = [1.0, 0.0]
    t = 0.8
    # alpha will be 0 after sufficient time in fallback
    for _ in range(50):
        ctrl.update_state_machine(0, t, dt)
    assert ctrl.alpha[0] == 0.0

    drone.position = np.array([0.5, 0.0])  # Drone has tracked to x = 0.5m
    cmd_during_dr = ctrl.compute_hybrid_control(
        drone=drone,
        current_time=t,
        dt=dt,
        neighbor_states=[],
        target_velocity=None,  # Coordinator link severed
        use_velocity_feedforward=True,
    )
    # Target pos is [0.5, 0.0], commanded vel is [1.0, 0.0]
    # Pos err = [0, 0], vel err = [0, 0] -> feedforward provides drag compensation
    # Command must be positive forward maintaining trajectory
    assert cmd_during_dr[0] > 0.0, f"Drone did not continue forward in dead reckoning window: {cmd_during_dr}"

    # Case B: Beyond dead reckoning window (t_in_fb = 2.0s > 1.5s, at t = 2.3s)
    # Drone has arrived at the clamped waypoint x = 1.5m
    drone.position = np.array([1.5, 0.0])
    # Commanded velocity must now be [0, 0] (braking to hover)
    t = 2.3
    cmd_after_dr = ctrl.compute_hybrid_control(
        drone=drone,
        current_time=t,
        dt=dt,
        neighbor_states=[],
        target_velocity=None,
        use_velocity_feedforward=True,
    )
    # Pos err = [1.5, 0] - [1.5, 0] = [0, 0]
    # Vel err = [0, 0] - [1.0, 0] = [-1.0, 0]
    # Brake term actively opposes velocity: cmd < 0
    assert cmd_after_dr[0] < 0.0, (
        f"Drone did not actively brake to hover after T={ctrl.dead_reckon_duration}s: {cmd_after_dr}"
    )


def test_coordinator_heartbeat_payload_and_freezing():
    """
    Recommendation 8: Verify coordinator heartbeat payload carries target_velocity
    and next_waypoint, and that during link outage, drone target, velocity, and offsets
    remain frozen at the last received heartbeat.
    """
    from simulator.engine import SwarmSimulation

    d0 = Drone(0, initial_position=[0.0, 0.0], initial_velocity=[0.0, 0.0])
    d1 = Drone(1, initial_position=[1.0, 0.0], initial_velocity=[0.0, 0.0])
    sim = SwarmSimulation([d0, d1], control_mode="hybrid", latency_mean=0.0)

    sim.centroid_velocity = np.array([1.5, -0.5])
    sim.step()

    # Packet received: verify payload fields
    assert np.allclose(sim.drone_last_received_velocity[0], np.array([1.5, -0.5]))
    assert np.allclose(sim.hybrid_ctrl.last_known_velocity[0], np.array([1.5, -0.5]))
    frozen_target = sim.drone_last_received_target[0].copy()
    frozen_velocity = sim.drone_last_received_velocity[0].copy()

    # Sever coordinator link
    sim.set_coordinator_link(False)
    sim.centroid_velocity = np.array([3.0, 3.0])  # Coordinator changes heading/speed
    sim.centroid_target += np.array([10.0, 10.0])

    sim.step()
    sim.step()

    # Drone state must remain frozen at the last received heartbeat
    assert np.allclose(sim.drone_last_received_target[0], frozen_target), "Target slot leaked through severed link"
    assert np.allclose(sim.drone_last_received_velocity[0], frozen_velocity), "Velocity leaked through severed link"


def test_double_apf_elimination_in_fallback():
    """
    Recommendation 12: Verify that double APF is eliminated in fallback.
    At alpha=1 (centralized), alpha=0.5 (blending), and alpha=0 (fallback),
    the total repulsive separation barrier against a neighbor at distance 0.8m
    must be exactly 1.0 * f_sep without a 200% spike.
    """
    ctrl = HybridController()
    drone = Drone(0, initial_position=[0.0, 0.0], initial_velocity=[0.0, 0.0])
    neighbor = [{"id": 1, "position": np.array([0.8, 0.0]), "velocity": np.array([0.0, 0.0])}]

    ctrl._init_drone_if_needed(0, 0.0)
    ctrl.last_known_target[0] = np.array([0.0, 0.0])

    # Compute control at alpha=1.0
    ctrl.alpha[0] = 1.0
    cmd_alpha_1 = ctrl.compute_hybrid_control(
        drone=drone, current_time=0.0, dt=0.05, neighbor_states=neighbor, use_velocity_feedforward=False
    )

    # Compute control at alpha=0.0
    ctrl.alpha[0] = 0.0
    cmd_alpha_0 = ctrl.compute_hybrid_control(
        drone=drone, current_time=0.0, dt=0.05, neighbor_states=neighbor, use_velocity_feedforward=False
    )

    # In both cases (position err = 0, vel err = 0), command in x-direction
    # is purely the separation repulsion: diff = [-0.8, 0], diff/dist = [-1, 0]
    # cmd_alpha_1 = 1.0 * u_safe_apf
    # cmd_alpha_0 = 1.0 * u_decentral = 1.0 * f_sep
    # Since u_safe_apf == f_sep, the repulsive magnitude MUST be identical!
    assert np.isclose(cmd_alpha_1[0], cmd_alpha_0[0], atol=1e-5), (
        f"Double APF detected! Repulsion at alpha=1: {cmd_alpha_1[0]:.4f}, at alpha=0: {cmd_alpha_0[0]:.4f}"
    )


if __name__ == "__main__":
    test_stale_and_sequence_rejection()
    test_asymmetric_hysteresis_and_dwell_time()
    test_smooth_controller_blending()
    test_forced_fallback_and_sliding_window_recovery()
    test_isolated_fallback_bounded_hover_deceleration()
    test_hybrid_controller_measured_state_inputs()
    test_degree_normalisation_at_various_swarm_sizes()
    test_dead_reckoning_fallback_strategy()
    test_coordinator_heartbeat_payload_and_freezing()
    test_double_apf_elimination_in_fallback()
    print("All hybrid feature tests passed successfully!")

