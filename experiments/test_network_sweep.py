"""
Network Impairment Parameter Sweep with Mode Switch Chattering Analysis.
Evaluates:
1. Formation Tracking Error vs Packet Loss (0% to 50%)
2. Min Inter-Drone Distance vs Packet Loss (Safety Verification)
3. Total Mode Switches per Run vs Packet Loss (Chattering Suppression via Asymmetric Hysteresis)
"""

import os
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from swarm_core.drone import Drone
from swarm_core.formations import FormationType
from simulator.engine import SwarmSimulation


def run_single_trial(
    mode: str,
    loss_rate: float,
    num_drones: int = 6,
    seed: int = 42,
    naive_switching: bool = False,
):
    rng = np.random.default_rng(seed)
    # Scatter drones in a 6x6 meter area with non-overlapping initial positions
    positions = []
    while len(positions) < num_drones:
        cand = rng.uniform(-3.5, 3.5, size=2)
        if all(np.linalg.norm(cand - p) >= 1.2 for p in positions):
            positions.append(cand)

    drones = [Drone(drone_id=i, initial_position=positions[i]) for i in range(num_drones)]

    sim = SwarmSimulation(
        drones=drones,
        control_mode="hybrid" if "hybrid" in mode else mode,
        comm_range=15.0,
        packet_loss_rate=loss_rate,
        latency_mean=0.03,  # 30ms average latency
        dt=0.05,
    )

    if naive_switching:
        # Naive hybrid: instant degrade after 1 dropped packet, instant recovery after 1 packet, 0 dwell time
        sim.hybrid_ctrl.degrade_timeout = 0.06
        sim.hybrid_ctrl.recovery_consecutive_hb = 1
        sim.hybrid_ctrl.min_dwell_time = 0.0
        sim.hybrid_ctrl.ramp_duration = 0.01  # Hard switch

    sim.set_formation(FormationType.V_SHAPE)

    sim_duration = 10.0
    steps = int(sim_duration / sim.dt)

    for _ in range(steps):
        sim.step()

    summary = sim.metrics.get_summary()
    return summary


def main():
    output_dir = "experiments/results"
    os.makedirs(output_dir, exist_ok=True)

    loss_rates = [0.0, 0.05, 0.10, 0.20, 0.30, 0.40, 0.50]
    num_trials = 5

    print("=================================================================")
    print("  Swarm Drones FYP: Network Packet Loss & Chattering Sweep       ")
    print("=================================================================")

    modes_to_test = [
        ("centralized", False, "Centralized"),
        ("decentralized", False, "Decentralized"),
        ("hybrid_proposed", False, "Hybrid (Proposed Hysteresis)"),
        ("hybrid_naive", True, "Hybrid (Naive No-Hysteresis)"),
    ]

    results = {
        m_key: {"mean_err": [], "min_dist": [], "conv_time": [], "switches": []}
        for m_key, _, _ in modes_to_test
    }

    for m_key, is_naive, label in modes_to_test:
        print(f"\nEvaluating: {label}...")
        for loss in loss_rates:
            errors, dists, times, switches = [], [], [], []
            for trial in range(num_trials):
                summary = run_single_trial(
                    mode=m_key,
                    loss_rate=loss,
                    seed=42 + trial * 17,
                    naive_switching=is_naive,
                )
                errors.append(summary["final_formation_error_m"])
                dists.append(summary["min_recorded_distance_m"])
                times.append(summary["convergence_time_s"])
                switches.append(summary["total_mode_switches"])

            m_err = float(np.mean(errors))
            m_dist = float(np.mean(dists))
            m_time = float(np.mean(times))
            m_switch = float(np.mean(switches))

            results[m_key]["mean_err"].append(m_err)
            results[m_key]["min_dist"].append(m_dist)
            results[m_key]["conv_time"].append(m_time)
            results[m_key]["switches"].append(m_switch)

            print(
                f"  Loss: {loss*100:4.0f}% | Error: {m_err:5.3f}m | "
                f"MinDist: {m_dist:5.3f}m | Switches: {m_switch:5.1f}"
            )

    # 3-Panel Scientific Plot
    loss_percent = [l * 100 for l in loss_rates]
    styles = {
        "centralized": {"color": "crimson", "marker": "o", "label": "Centralized", "ls": "--"},
        "decentralized": {"color": "navy", "marker": "s", "label": "Decentralized", "ls": "--"},
        "hybrid_proposed": {"color": "forestgreen", "marker": "^", "label": "Proposed Hybrid (Hysteresis)", "ls": "-"},
        "hybrid_naive": {"color": "darkorange", "marker": "x", "label": "Naive Hybrid (No Hysteresis)", "ls": ":"},
    }

    fig, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=(17, 5), dpi=150)

    # 1. Formation Error vs Packet Loss
    for m_key in ["centralized", "decentralized", "hybrid_proposed"]:
        ax1.plot(
            loss_percent,
            results[m_key]["mean_err"],
            linewidth=2.2,
            **styles[m_key],
        )
    ax1.set_xlabel("Packet Loss Rate (%)", fontsize=11)
    ax1.set_ylabel("Final Formation Error (m)", fontsize=11)
    ax1.set_title("Tracking Robustness under Loss", fontweight="bold", fontsize=12)
    ax1.grid(True, linestyle="--", alpha=0.5)
    ax1.legend(fontsize=9)

    # 2. Min Inter-Drone Distance (Safety Verification)
    for m_key in ["centralized", "decentralized", "hybrid_proposed"]:
        ax2.plot(
            loss_percent,
            results[m_key]["min_dist"],
            linewidth=2.2,
            **styles[m_key],
        )
    ax2.axhline(0.70, color="black", linestyle="--", linewidth=1.5, label="Collision Limit (0.7m)")
    ax2.set_xlabel("Packet Loss Rate (%)", fontsize=11)
    ax2.set_ylabel("Min Inter-Drone Distance (m)", fontsize=11)
    ax2.set_title("Safety Barrier Preservation", fontweight="bold", fontsize=12)
    ax2.grid(True, linestyle="--", alpha=0.5)
    ax2.legend(fontsize=9)

    # 3. Mode Switches vs Packet Loss (Chattering Suppression Analysis)
    ax3.plot(
        loss_percent,
        results["hybrid_naive"]["switches"],
        linewidth=2.2,
        **styles["hybrid_naive"],
    )
    ax3.plot(
        loss_percent,
        results["hybrid_proposed"]["switches"],
        linewidth=2.2,
        **styles["hybrid_proposed"],
    )
    ax3.set_xlabel("Packet Loss Rate (%)", fontsize=11)
    ax3.set_ylabel("Average Mode Switches per Run", fontsize=11)
    ax3.set_title("Chattering Suppression (Hysteresis)", fontweight="bold", fontsize=12)
    ax3.grid(True, linestyle="--", alpha=0.5)
    ax3.legend(fontsize=9)

    plt.tight_layout()
    plot_path = f"{output_dir}/network_loss_comparison.png"
    plt.savefig(plot_path)
    plt.close(fig)

    print(f"\nNetwork & chattering sweep complete! 3-panel figure saved to: {plot_path}")


if __name__ == "__main__":
    main()
