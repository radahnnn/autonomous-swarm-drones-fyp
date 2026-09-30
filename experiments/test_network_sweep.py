"""
Network Impairment Parameter Sweep.
Evaluates Centralized vs Decentralized vs Hybrid robustness under increasing packet loss (0% to 50%).
Generates comparative scientific plots for the FYP report.
"""

import os
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from swarm_core.drone import Drone
from swarm_core.formations import FormationType
from simulator.engine import SwarmSimulation


def run_single_trial(mode: str, loss_rate: float, num_drones: int = 6, seed: int = 42):
    rng = np.random.default_rng(seed)
    # Scatter drones in a 6x6 meter area
    positions = []
    while len(positions) < num_drones:
        cand = rng.uniform(-3.5, 3.5, size=2)
        if all(np.linalg.norm(cand - p) >= 1.2 for p in positions):
            positions.append(cand)
            
    drones = [Drone(drone_id=i, initial_position=positions[i]) for i in range(num_drones)]

    sim = SwarmSimulation(
        drones=drones,
        control_mode=mode,
        comm_range=15.0,
        packet_loss_rate=loss_rate,
        latency_mean=0.03,  # 30ms latency
        dt=0.05,
    )
    # Target: V-Shape formation
    sim.set_formation(FormationType.V_SHAPE)

    # In Centralized mode, packet loss drops commands from the ground station to the drones
    # In Hybrid mode, when packets drop, the drone gracefully falls back to local flocking
    sim_duration = 8.0
    steps = int(sim_duration / sim.dt)

    for step_i in range(steps):
        # Stochastic coordinator drop matching the wireless loss rate
        if mode in ["centralized", "hybrid"]:
            sim.set_coordinator_link(rng.random() >= loss_rate)
        sim.step()

    summary = sim.metrics.get_summary()
    return summary


def main():
    output_dir = "experiments/results"
    os.makedirs(output_dir, exist_ok=True)

    loss_rates = [0.0, 0.05, 0.10, 0.20, 0.30, 0.40, 0.50]
    modes = ["centralized", "decentralized", "hybrid"]
    num_trials = 5  # Monte Carlo trials per condition

    print("==================================================")
    print("  Swarm Drones FYP: Network Packet Loss Sweep     ")
    print("==================================================")

    results = {m: {"mean_err": [], "min_dist": [], "conv_time": []} for m in modes}

    for mode in modes:
        print(f"\nEvaluating mode: {mode.upper()}...")
        for loss in loss_rates:
            errors, dists, times = [], [], []
            for trial in range(num_trials):
                summary = run_single_trial(mode, loss, seed=42 + trial * 17)
                errors.append(summary["final_formation_error_m"])
                dists.append(summary["min_recorded_distance_m"])
                times.append(summary["convergence_time_s"])

            m_err = float(np.mean(errors))
            m_dist = float(np.mean(dists))
            m_time = float(np.mean(times))

            results[mode]["mean_err"].append(m_err)
            results[mode]["min_dist"].append(m_dist)
            results[mode]["conv_time"].append(m_time)

            print(f"  Loss: {loss*100:4.0f}% | Error: {m_err:5.3f}m | MinDist: {m_dist:5.3f}m | ConvTime: {m_time:4.2f}s")

    # Generate comparative plots
    loss_percent = [l * 100 for l in loss_rates]
    styles = {
        "centralized": {"color": "crimson", "marker": "o", "label": "Centralized"},
        "decentralized": {"color": "navy", "marker": "s", "label": "Decentralized"},
        "hybrid": {"color": "forestgreen", "marker": "^", "label": "Hybrid (Proposed)"},
    }

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(13, 5), dpi=150)

    # 1. Formation Error vs Packet Loss
    for mode in modes:
        ax1.plot(
            loss_percent,
            results[mode]["mean_err"],
            linewidth=2.2,
            **styles[mode],
        )
    ax1.set_xlabel("Packet Loss Rate (%)", fontsize=11)
    ax1.set_ylabel("Final Formation Tracking Error (m)", fontsize=11)
    ax1.set_title("Tracking Robustness under Packet Loss", fontweight="bold", fontsize=12)
    ax1.grid(True, linestyle="--", alpha=0.5)
    ax1.legend(fontsize=10)

    # 2. Min Inter-Drone Distance vs Packet Loss (Safety Verification)
    for mode in modes:
        ax2.plot(
            loss_percent,
            results[mode]["min_dist"],
            linewidth=2.2,
            **styles[mode],
        )
    # Collision boundary at 0.70m
    ax2.axhline(0.70, color="black", linestyle="--", linewidth=1.5, label="Collision Threshold (0.7m)")
    ax2.set_xlabel("Packet Loss Rate (%)", fontsize=11)
    ax2.set_ylabel("Min Inter-Drone Distance (m)", fontsize=11)
    ax2.set_title("Safety Boundary Preservation under Packet Loss", fontweight="bold", fontsize=12)
    ax2.grid(True, linestyle="--", alpha=0.5)
    ax2.legend(fontsize=10)

    plt.tight_layout()
    plot_path = f"{output_dir}/network_loss_comparison.png"
    plt.savefig(plot_path)
    plt.close(fig)

    print(f"\nNetwork sweep complete! Comparative figures saved to: {plot_path}")


if __name__ == "__main__":
    main()
