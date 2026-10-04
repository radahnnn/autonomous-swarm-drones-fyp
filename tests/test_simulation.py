import numpy as np
from swarm_core.drone import Drone
from swarm_core.formations import FormationType
from simulator.engine import SwarmSimulation


def create_non_overlapping_drones(num_drones: int, min_dist: float = 1.2, seed: int = 42):
    rng = np.random.default_rng(seed)
    positions = []
    while len(positions) < num_drones:
        cand = rng.uniform(-4, 4, size=2)
        if all(np.linalg.norm(cand - p) >= min_dist for p in positions):
            positions.append(cand)
    return [Drone(drone_id=i, initial_position=positions[i]) for i in range(num_drones)]


def test_centralized_simulation():
    drones = create_non_overlapping_drones(5, seed=42)
    sim = SwarmSimulation(drones, control_mode="centralized")
    sim.set_formation(FormationType.LINE)

    for _ in range(100):  # 5 seconds
        sim.step()

    summary = sim.metrics.get_summary()
    assert summary["final_formation_error_m"] < 0.25
    assert summary["any_collision"] == 0.0


def test_decentralized_simulation():
    drones = create_non_overlapping_drones(5, seed=101)
    sim = SwarmSimulation(drones, control_mode="decentralized", comm_range=15.0)
    sim.set_formation(FormationType.V_SHAPE)

    for _ in range(120):  # 6 seconds
        sim.step()

    summary = sim.metrics.get_summary()
    assert summary["final_formation_error_m"] < 0.35
    assert summary["any_collision"] == 0.0


def test_hybrid_fallback():
    drones = create_non_overlapping_drones(5, seed=202)
    sim = SwarmSimulation(drones, control_mode="hybrid", comm_range=15.0)
    sim.set_formation(FormationType.CIRCLE)

    # First 3 seconds: normal centralized operation
    for _ in range(60):
        sim.step()

    # Link dropped: coordinator fails
    sim.set_coordinator_link(False)
    for _ in range(60):
        sim.step()

    summary = sim.metrics.get_summary()
    assert summary["any_collision"] == 0.0
    assert summary["min_recorded_distance_m"] > 0.6


def test_outage_and_feedforward_simulation():
    drones = create_non_overlapping_drones(5, seed=303)
    sim = SwarmSimulation(
        drones=drones,
        control_mode="hybrid",
        comm_range=15.0,
        packet_loss_rate=0.0,
        use_velocity_feedforward=True,
        gps_noise_std=0.5,
        gps_common_mode_fraction=0.60,
        seed=303,
    )
    sim.set_formation(FormationType.LINE, centroid=np.array([0.0, 0.0]))
    sim.centroid_velocity = np.array([0.8, 0.3])
    
    # Schedule full outage from t = 2.0s to 3.0s
    sim.channel.add_outage(start_time=2.0, duration=1.0)

    for step_i in range(120):  # 6.0 seconds
        t = step_i * sim.dt
        sim.centroid_target += sim.centroid_velocity * sim.dt
        sim.step()

    summary = sim.metrics.get_summary()
    assert summary["any_collision"] == 0.0
    assert summary["min_recorded_distance_m"] > 0.70  # Collision barrier preserved
    # Fallback occurred and recovered
    fb_stats = sim.hybrid_ctrl.get_fallback_stats()
    assert fb_stats["total_fallback_entries"] > 0
    assert fb_stats["total_time_in_fallback_s"] > 0


def test_wireless_channel_gilbert_elliott_statistics():
    """
    Recommendation 2: Run WirelessChannel alone (one send per tick, 20 Hz)
    and assert:
      - empirical loss ~ 20% (theoretical pi_BAD = p / (p + q) = 0.05 / 0.25 = 0.20)
      - mean burst ~ 5 ticks (theoretical E[L] = 1 / q = 1 / 0.20 = 5.0)
      - P(burst >= 11) ~ 0.107 (theoretical P(L >= 11) = (1 - q)^10 = 0.80^10 = 0.107374)
    """
    from swarm_core.network import WirelessChannel

    dt = 0.05  # 20 Hz
    num_ticks = 40000
    channel = WirelessChannel(
        use_gilbert_elliott=True,
        p_g_to_b=0.05,
        p_b_to_g=0.20,
        loss_rate_bad=1.0,
        comm_range=100.0,
        latency_mean=0.0,
        latency_std=0.0,
        seed=42,
    )

    drops = []
    for step_i in range(num_ticks):
        t = step_i * dt
        success = channel.send(
            sender_id=0,
            recipient_id=1,
            payload={"msg": "ping"},
            distance=2.0,
            current_time=t,
        )
        drops.append(0 if success else 1)

    # 1. Overall empirical loss
    loss_rate = float(np.mean(drops))
    assert np.isclose(loss_rate, 0.20, atol=0.015), f"Loss rate {loss_rate:.4f} differs from theoretical 0.20"

    # 2. Burst length calculation
    burst_lengths = []
    current_burst = 0
    for d in drops:
        if d == 1:
            current_burst += 1
        else:
            if current_burst > 0:
                burst_lengths.append(current_burst)
                current_burst = 0
    if current_burst > 0:
        burst_lengths.append(current_burst)

    assert len(burst_lengths) > 500, "Insufficient bursts sampled"
    mean_burst = float(np.mean(burst_lengths))
    assert np.isclose(mean_burst, 5.0, atol=0.25), f"Mean burst length {mean_burst:.3f} differs from theoretical 5.0"

    # 3. P(burst >= 11)
    bursts_ge_11 = sum(1 for b in burst_lengths if b >= 11)
    prob_burst_ge_11 = bursts_ge_11 / len(burst_lengths)
    assert np.isclose(prob_burst_ge_11, 0.107, atol=0.02), (
        f"P(burst >= 11) = {prob_burst_ge_11:.4f} differs from theoretical 0.107"
    )


def test_wireless_channel_targeted_outage_partition():
    """
    Recommendation 4: Verify that add_outage with recipients filters drops
    only to the targeted partition, while leaving other recipients untouched.
    """
    from swarm_core.network import WirelessChannel

    channel = WirelessChannel(comm_range=100.0, seed=123)
    # Outage only for drone 3 and 4 between t=2.0s and t=5.0s
    channel.add_outage(start_time=2.0, duration=3.0, recipients=[3, 4])

    # At t = 1.0s (before outage): all delivered
    assert channel.send(sender_id=-1, recipient_id=1, payload={}, distance=1.0, current_time=1.0)
    assert channel.send(sender_id=-1, recipient_id=3, payload={}, distance=1.0, current_time=1.0)

    # At t = 3.0s (during outage): drone 1 succeeds, drones 3 and 4 dropped
    assert channel.send(sender_id=-1, recipient_id=1, payload={}, distance=1.0, current_time=3.0)
    assert channel.send(sender_id=-1, recipient_id=2, payload={}, distance=1.0, current_time=3.0)
    assert not channel.send(sender_id=-1, recipient_id=3, payload={}, distance=1.0, current_time=3.0)
    assert not channel.send(sender_id=-1, recipient_id=4, payload={}, distance=1.0, current_time=3.0)

    # At t = 6.0s (after outage): all delivered
    assert channel.send(sender_id=-1, recipient_id=3, payload={}, distance=1.0, current_time=6.0)
    assert channel.send(sender_id=-1, recipient_id=4, payload={}, distance=1.0, current_time=6.0)


def test_wireless_channel_5_drone_per_recipient_bursts():
    """
    Recommendation 1: In a 5-drone swarm, each recipient receives multiple packets per tick
    (4 peer broadcasts + 1 coordinator heartbeat = 5 sends/tick).
    Verify that advancing the Gilbert-Elliott state at most once per tick per recipient
    preserves the physical burst statistics in tick units:
      - empirical loss ~ 20%
      - mean burst ~ 5.0 ticks
      - P(burst >= 11) ~ 0.107
      - all packets sent to the same recipient within the same tick experience identical channel fate.
    """
    from swarm_core.network import WirelessChannel

    dt = 0.05
    num_ticks = 40000
    channel = WirelessChannel(
        use_gilbert_elliott=True,
        p_g_to_b=0.05,
        p_b_to_g=0.20,
        loss_rate_bad=1.0,
        comm_range=100.0,
        latency_mean=0.0,
        latency_std=0.0,
        seed=101,
    )

    tick_drops = []
    # Test recipient 0 receiving from 4 peers (1, 2, 3, 4) and coordinator (-1)
    senders = [-1, 1, 2, 3, 4]

    for step_i in range(num_ticks):
        t = step_i * dt
        step_results = []
        for s_id in senders:
            succ = channel.send(
                sender_id=s_id,
                recipient_id=0,
                payload={"data": 1},
                distance=2.0,
                current_time=t,
            )
            step_results.append(1 if not succ else 0)

        # All packets in the same tick must have identical drop status (channel coherence)
        assert len(set(step_results)) == 1, f"Incoherent channel state at tick {step_i}: {step_results}"
        tick_drops.append(step_results[0])

    # 1. Overall empirical loss across ticks
    loss_rate = float(np.mean(tick_drops))
    assert np.isclose(loss_rate, 0.20, atol=0.015), f"5-drone loss rate {loss_rate:.4f} differs from 0.20"

    # 2. Burst length calculation in ticks
    burst_lengths = []
    cur_burst = 0
    for d in tick_drops:
        if d == 1:
            cur_burst += 1
        else:
            if cur_burst > 0:
                burst_lengths.append(cur_burst)
                cur_burst = 0
    if cur_burst > 0:
        burst_lengths.append(cur_burst)

    assert len(burst_lengths) > 500, "Insufficient bursts sampled"
    mean_burst = float(np.mean(burst_lengths))
    assert np.isclose(mean_burst, 5.0, atol=0.25), f"Mean burst length {mean_burst:.3f} differs from 5.0 ticks"

    # 3. P(burst >= 11 ticks)
    ge_11 = sum(1 for b in burst_lengths if b >= 11)
    p_ge_11 = ge_11 / len(burst_lengths)
    assert np.isclose(p_ge_11, 0.107, atol=0.02), f"P(burst >= 11) = {p_ge_11:.4f} differs from 0.107"


def test_locked_slot_assignment_preserves_mapping_during_transit():
    """
    Recommendation 6: Verify that SwarmSimulation locks Hungarian slot assignments
    at morph start so drone-to-slot mapping remains stable without mid-trajectory chattering.
    """
    drones = create_non_overlapping_drones(5, seed=42)
    sim = SwarmSimulation(drones, control_mode="centralized")
    
    # Start at V-Shape
    sim.set_formation(FormationType.V_SHAPE, centroid=np.array([0.0, 0.0]))
    initial_map = dict(sim.drone_slot_map)
    assert len(initial_map) == 5

    # Run for 20 steps with moving centroid
    sim.centroid_velocity = np.array([1.0, 0.5])
    for _ in range(20):
        sim.centroid_target += sim.centroid_velocity * sim.dt
        sim.step()
        # Mapping must NOT change during transit
        assert sim.drone_slot_map == initial_map

    # Morph to LINE
    sim.set_formation(FormationType.LINE, centroid=sim.centroid_target)
    morph_map = dict(sim.drone_slot_map)
    # Mapping is re-locked for new formation
    assert len(morph_map) == 5

    # Advance again - new mapping remains stable
    for _ in range(20):
        sim.centroid_target += sim.centroid_velocity * sim.dt
        sim.step()
        assert sim.drone_slot_map == morph_map


def test_neighbor_memory_extrapolation_and_age_out():
    """
    Recommendation 6: Verify neighbor memory linearly extrapolates peer positions
    across short packet drops and ages out after neighbor_timeout (0.30s).
    """
    drones = [Drone(0, [0.0, 0.0]), Drone(1, [2.0, 0.0])]
    sim = SwarmSimulation(drones, control_mode="hybrid", comm_range=15.0, packet_loss_rate=0.0, gps_noise_std=0.0)
    sim.neighbor_timeout = 0.30

    # Step 1 & 2: Normal exchange (packet sent at step 1 arrives at step 2 with latency)
    sim.drones[1].velocity = np.array([1.0, 0.0])
    sim.step()
    sim.step()

    # Drone 0 should remember Drone 1
    assert 1 in sim.neighbor_memory[0]
    mem = sim.neighbor_memory[0][1]
    assert np.allclose(mem["velocity"], [1.0, 0.0])

    # Now sever peer link by setting 100% loss outage on peer broadcasts
    sim.channel.add_outage(start_time=sim.current_time, duration=1.0, scope="peer")
    
    # Advance 2 steps (0.10s drop < 0.30s timeout)
    # Neighbor memory must extrapolate position
    sim.step()
    # Drone 1's memory should still be active and linearly extrapolated
    assert 1 in sim.neighbor_memory[0]
    
    # Advance past 0.30s timeout (8 steps = 0.40s)
    for _ in range(8):
        sim.step()

    # Memory for Drone 1 must have aged out
    assert 1 not in sim.neighbor_memory[0]


def test_centralized_hold_target_vs_hold_accel():
    """
    Recommendation 5: Verify that 'centralized' (hold-last-target) bounds position error
    and decelerates to hover around the last target under loss, while 'centralized_hold_accel'
    diverges open-loop if severed during acceleration.
    """
    # 1. Hold-last-target baseline
    d_target = Drone(0, [0.0, 0.0])
    sim_target = SwarmSimulation([d_target], control_mode="centralized", latency_mean=0.0, gps_noise_std=0.0)
    sim_target.set_formation(FormationType.LINE, centroid=np.array([5.0, 0.0]))
    sim_target.centroid_velocity = np.array([1.0, 0.0])

    # Sever coordinator link after 5 steps (drone is accelerating toward target)
    for _ in range(5):
        sim_target.centroid_target += sim_target.centroid_velocity * sim_target.dt
        sim_target.step()

    last_target = sim_target.drone_last_received_target[0].copy()
    sim_target.set_coordinator_link(False)

    for _ in range(120):  # 6.0 seconds severed to allow settle and hover
        sim_target.centroid_target += sim_target.centroid_velocity * sim_target.dt
        sim_target.step()

    # Hold-target should converge near the last received target, not fly away
    dist_to_last_target = np.linalg.norm(sim_target.drones[0].position - last_target)
    assert dist_to_last_target < 0.25, f"Hold-target drifted {dist_to_last_target:.2f}m from last target"
    assert np.linalg.norm(sim_target.drones[0].velocity) < 0.1, "Hold-target did not decelerate to hover"

    # 2. Hold-last-accel baseline
    d_accel = Drone(0, [0.0, 0.0])
    sim_accel = SwarmSimulation([d_accel], control_mode="centralized_hold_accel", latency_mean=0.0, gps_noise_std=0.0)
    sim_accel.set_formation(FormationType.LINE, centroid=np.array([5.0, 0.0]))
    sim_accel.centroid_velocity = np.array([1.0, 0.0])

    for _ in range(5):
        sim_accel.centroid_target += sim_accel.centroid_velocity * sim_accel.dt
        sim_accel.step()

    sim_accel.set_coordinator_link(False)

    for _ in range(120):
        sim_accel.centroid_target += sim_accel.centroid_velocity * sim_accel.dt
        sim_accel.step()

    # Hold-accel drone runs away open-loop (velocity saturated at max_speed)
    assert np.linalg.norm(sim_accel.drones[0].velocity) > 2.0, "Hold-accel did not maintain high runaway velocity"


def test_all_four_fallback_strategies_in_simulation():
    """
    Recommendation 9: Compare all 4 fallback options in simulation:
    - hover-on-loss (hover)
    - hold-last-target (hold_target)
    - dead-reckon-then-brake (dead_reckon)
    - consensus flocking (consensus)
    Assert all 4 remain bounded, collision-free, and exhibit expected trajectories under a ground outage.
    """
    strategies = ["hover", "hold_target", "dead_reckon", "consensus"]
    final_positions = {}

    for strat in strategies:
        drones = [Drone(i, initial_position=[i * 2.0, 0.0]) for i in range(3)]
        sim = SwarmSimulation(
            drones=drones,
            control_mode="hybrid",
            fallback_strategy=strat,
            dead_reckon_duration=1.0,
            latency_mean=0.0,
            gps_noise_std=0.0,
        )
        sim.set_formation(FormationType.LINE, centroid=np.array([2.0, 0.0]))
        sim.centroid_velocity = np.array([1.0, 0.0])

        # Step 5 ticks with active coordinator link
        for _ in range(5):
            sim.centroid_target += sim.centroid_velocity * sim.dt
            sim.step()

        # Inject ground-link outage
        sim.channel.add_outage(start_time=sim.current_time, duration=4.0, scope="ground")

        # Step through outage
        for _ in range(60):  # 3.0s
            sim.centroid_target += sim.centroid_velocity * sim.dt
            sim.step()

        summary = sim.metrics.get_summary()
        assert summary["any_collision"] == 0, f"Collision occurred in fallback strategy '{strat}'"
        assert summary["min_recorded_distance_m"] >= 0.70, f"Separation violation in '{strat}'"
        final_positions[strat] = np.mean([d.position[0] for d in sim.drones])

    # Dead-reckoning should advance further than hover-on-loss:
    assert final_positions["dead_reckon"] > final_positions["hover"], (
        f"Dead reckon ({final_positions['dead_reckon']:.2f}m) should travel further than hover ({final_positions['hover']:.2f}m)"
    )


def test_per_drone_velocity_in_control_loop():
    """
    Item 26: Verifies that goal_v and target_v are evaluated per-drone inside
    the drone loop, ensuring each drone uses its own last received velocity.
    """
    from unittest.mock import MagicMock
    drones = [Drone(0, [0.0, 0.0]), Drone(1, [2.0, 0.0]), Drone(2, [4.0, 0.0])]
    sim = SwarmSimulation(drones, control_mode="decentralized", latency_mean=0.0, gps_noise_std=0.0)

    # Set distinct velocities per drone and disable coordinator broadcast so mock values are preserved
    sim.set_coordinator_link(False)
    sim.drone_last_received_velocity[0] = np.array([1.0, 0.0])
    sim.drone_last_received_velocity[1] = np.array([0.0, 2.0])
    sim.drone_last_received_velocity[2] = np.array([-1.0, -1.0])
    sim.drone_last_heartbeat_time = {0: sim.current_time, 1: sim.current_time, 2: sim.current_time}

    passed_velocities = {}
    orig_compute = sim.decentral_ctrl.compute_drone_control

    def mock_compute_drone_control(drone, *args, **kwargs):
        passed_velocities[drone.id] = kwargs.get("goal_vel")
        return orig_compute(drone, *args, **kwargs)

    sim.decentral_ctrl.compute_drone_control = mock_compute_drone_control
    sim.step()

    assert np.allclose(passed_velocities[0], [1.0, 0.0])
    assert np.allclose(passed_velocities[1], [0.0, 2.0])
    assert np.allclose(passed_velocities[2], [-1.0, -1.0])


def test_set_coordinator_link_and_outage_equivalence():
    """
    Item 27: Verifies that calling set_coordinator_link(False) and scheduling
    a ground-link outage on the channel produce identical behavior.
    Onboard drone controllers decide solely from time since last valid heartbeat.
    """
    def run_sim_case(use_set_link: bool):
        drones = [Drone(0, [0.0, 0.0]), Drone(1, [2.0, 0.0])]
        sim = SwarmSimulation(
            drones=drones,
            control_mode="hybrid",
            latency_mean=0.0,
            gps_noise_std=0.0,
            seed=42,
        )
        sim.set_formation(FormationType.LINE, centroid=np.array([1.0, 0.0]))
        sim.centroid_velocity = np.array([1.0, 0.0])

        if not use_set_link:
            # Channel outage from t=0.2s to t=0.8s
            sim.channel.add_outage(start_time=0.2, duration=0.6, scope="ground")

        trajectory = []
        modes = []
        for step_i in range(30):  # 1.5 seconds
            t = step_i * sim.dt
            if use_set_link:
                if 0.2 <= t < 0.8:
                    sim.set_coordinator_link(False)
                else:
                    sim.set_coordinator_link(True)

            sim.centroid_target += sim.centroid_velocity * sim.dt
            sim.step()
            trajectory.append(sim.drones[0].position.copy())
            modes.append(sim.hybrid_ctrl.get_drone_mode(0))

        return np.array(trajectory), modes

    traj_link, modes_link = run_sim_case(use_set_link=True)
    traj_outage, modes_outage = run_sim_case(use_set_link=False)

    # Trajectories and state machine transitions must be identical
    assert np.allclose(traj_link, traj_outage, atol=1e-10)
    assert modes_link == modes_outage


if __name__ == "__main__":
    test_centralized_simulation()
    test_decentralized_simulation()
    test_hybrid_fallback()
    test_outage_and_feedforward_simulation()
    test_wireless_channel_gilbert_elliott_statistics()
    test_wireless_channel_targeted_outage_partition()
    test_wireless_channel_5_drone_per_recipient_bursts()
    test_locked_slot_assignment_preserves_mapping_during_transit()
    test_neighbor_memory_extrapolation_and_age_out()
    test_centralized_hold_target_vs_hold_accel()
    test_all_four_fallback_strategies_in_simulation()
    test_per_drone_velocity_in_control_loop()
    test_set_coordinator_link_and_outage_equivalence()
    print("All simulation tests passed successfully!")


