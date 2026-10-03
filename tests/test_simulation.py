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


if __name__ == "__main__":
    test_centralized_simulation()
    test_decentralized_simulation()
    test_hybrid_fallback()
    test_outage_and_feedforward_simulation()
    test_wireless_channel_gilbert_elliott_statistics()
    test_wireless_channel_targeted_outage_partition()
    print("Simulation tests passed successfully!")

