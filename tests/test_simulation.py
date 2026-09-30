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


if __name__ == "__main__":
    test_centralized_simulation()
    test_decentralized_simulation()
    test_hybrid_fallback()
    print("Simulation tests passed successfully!")
