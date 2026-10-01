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


if __name__ == "__main__":
    test_stale_and_sequence_rejection()
    test_asymmetric_hysteresis_and_dwell_time()
    test_smooth_controller_blending()
    test_forced_fallback_and_sliding_window_recovery()
    print("All hybrid feature tests passed successfully!")
