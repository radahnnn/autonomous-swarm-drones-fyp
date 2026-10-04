"""
Stress-Tested Network Impairment Parameter Sweep with Moving Trajectory & Formation Switching.
Evaluates:
1. Formation Tracking Error vs Packet Loss (0% to 50%) with dynamic moving target & mid-flight morphing
2. Min Inter-Drone Distance vs Packet Loss (Physical Safety Verification)
3. Total Mode Switches per Run vs Packet Loss (Chattering Suppression via Asymmetric Hysteresis)
Reports Mean +/- Standard Deviation across multiple random seeds.
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
        latency_mean=0.03,  # 30ms nominal latency
        dt=0.05,
        use_velocity_feedforward=True,
        seed=seed,
    )

    if naive_switching:
        # Naive hybrid: instant degrade after 1 dropped packet, instant recovery, 0 dwell time
        sim.hybrid_ctrl.degrade_timeout = 0.06
        sim.hybrid_ctrl.recovery_consecutive_hb = 1
        sim.hybrid_ctrl.min_dwell_time = 0.0
        sim.hybrid_ctrl.ramp_duration = 0.01  # Hard step switch
        sim.hybrid_ctrl.recovery_ratio_threshold = 0.0

    # Start in V-Shape
    sim.set_formation(FormationType.V_SHAPE, centroid=np.array([0.0, 0.0]))

    sim_duration = 12.0
    steps = int(sim_duration / sim.dt)

    # Dynamic moving reference trajectory (stress test)
    centroid = np.array([0.0, 0.0], dtype=np.float64)
    v_target = np.array([0.8, 0.3], dtype=np.float64)

    for step_i in range(steps):
        t = step_i * sim.dt
        # Continuous moving trajectory
        centroid += v_target * sim.dt

        # Mid-flight formation morph at t = 5.0s: V-Shape -> Line
        if t >= 5.0 and sim.current_formation == FormationType.V_SHAPE:
            sim.set_formation(FormationType.LINE, centroid=centroid)
        else:
            sim.centroid_target = centroid.copy()
            sim.centroid_velocity = v_target.copy()

        sim.step()

    summary = sim.metrics.get_summary()
    return summary


def main():
    output_dir = "experiments/results"
    os.makedirs(output_dir, exist_ok=True)

    loss_rates = [0.0, 0.05, 0.10, 0.20, 0.30, 0.40, 0.50]
    num_trials = 6

    print("=================================================================")
    print("  Swarm Drones FYP: Dynamic Stress-Tested Network Sweep          ")
    print("  - Trajectory: Moving Centroid (0.85 m/s)                       ")
    print("  - Formation Switch: V-Shape -> Line at t = 5.0s                ")
    print("  - Dynamics: 1st-Order Attitude Lag (0.18s) + Rotor Drag        ")
    print("=================================================================")

    modes_to_test = [
        ("centralized", False, "Centralized"),
        ("decentralized", False, "Decentralized"),
        ("hybrid_proposed", False, "Proposed Hybrid (Hysteresis)"),
        ("hybrid_naive", True, "Naive Hybrid (No Hysteresis)"),
    ]

    results = {
        m_key: {
            "mean_err": [], "std_err": [],
            "min_dist": [], "std_dist": [],
            "conv_time": [], "switches": [], "std_switches": [],
        }
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
                    seed=42 + trial * 19,
                    naive_switching=is_naive,
                )
                errors.append(summary["final_formation_error_m"])
                dists.append(summary["min_recorded_distance_m"])
                times.append(summary["convergence_time_s"])
                switches.append(summary["total_mode_switches"])

            m_err = float(np.mean(errors))
            s_err = float(np.std(errors))
            m_dist = float(np.mean(dists))
            s_dist = float(np.std(dists))
            m_time = float(np.mean(times))
            m_switch = float(np.mean(switches))
            s_switch = float(np.std(switches))

            results[m_key]["mean_err"].append(m_err)
            results[m_key]["std_err"].append(s_err)
            results[m_key]["min_dist"].append(m_dist)
            results[m_key]["std_dist"].append(s_dist)
            results[m_key]["conv_time"].append(m_time)
            results[m_key]["switches"].append(m_switch)
            results[m_key]["std_switches"].append(s_switch)

            print(
                f"  Loss: {loss*100:4.0f}% | Error: {m_err:5.3f} +/- {s_err:5.3f}m | "
                f"MinDist: {m_dist:5.3f}m | Switches: {m_switch:5.1f}"
            )

    # 3-Panel Scientific Plot with Error Bars
    loss_percent = [l * 100 for l in loss_rates]
    styles = {
        "centralized": {"color": "#D32F2F", "marker": "o", "label": "Centralized", "ls": "--"},
        "decentralized": {"color": "#1976D2", "marker": "s", "label": "Decentralized", "ls": "--"},
        "hybrid_proposed": {"color": "#2E7D32", "marker": "^", "label": "Proposed Hybrid (Hysteresis)", "ls": "-"},
        "hybrid_naive": {"color": "#F57C00", "marker": "x", "label": "Naive Hybrid (No Hysteresis)", "ls": ":"},
    }

    fig, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=(18, 5.2), dpi=180)

    # 1. Formation Error vs Packet Loss (with shaded std bands)
    for m_key, _, _ in modes_to_test:
        m_vals = np.array(results[m_key]["mean_err"])
        s_vals = np.array(results[m_key]["std_err"])
        ax1.plot(loss_percent, m_vals, linewidth=2.0, **styles[m_key])
        ax1.fill_between(loss_percent, m_vals - s_vals, m_vals + s_vals, color=styles[m_key]["color"], alpha=0.12)
        
    ax1.set_xlabel("Packet Loss Rate (%)", fontsize=11, fontweight="bold")
    ax1.set_ylabel("Final Formation Error (m)", fontsize=11, fontweight="bold")
    ax1.set_title("Tracking Robustness under Loss (Moving Target)", fontweight="bold", fontsize=12)
    ax1.grid(True, linestyle="--", alpha=0.5)
    ax1.legend(fontsize=9, loc="upper left")

    # 2. Min Inter-Drone Distance (Physical Safety Verification)
    for m_key, _, _ in modes_to_test:
        m_dist = np.array(results[m_key]["min_dist"])
        ax2.plot(loss_percent, m_dist, linewidth=2.0, **styles[m_key])
    ax2.axhline(0.70, color="black", linestyle="--", linewidth=1.5, label="Collision Threshold (0.7m)")
    ax2.set_xlabel("Packet Loss Rate (%)", fontsize=11, fontweight="bold")
    ax2.set_ylabel("Min Inter-Drone Distance (m)", fontsize=11, fontweight="bold")
    ax2.set_title("Collision Safety Barrier Preservation", fontweight="bold", fontsize=12)
    ax2.grid(True, linestyle="--", alpha=0.5)
    ax2.legend(fontsize=9, loc="lower left")

    # 3. Mode Switches vs Packet Loss (Chattering Suppression Analysis)
    m_naive = np.array(results["hybrid_naive"]["switches"])
    s_naive = np.array(results["hybrid_naive"]["std_switches"])
    m_prop = np.array(results["hybrid_proposed"]["switches"])
    s_prop = np.array(results["hybrid_proposed"]["std_switches"])

    ax3.errorbar(
        loss_percent, m_naive, yerr=s_naive, fmt="x:", color="#F57C00",
        linewidth=2.0, capsize=4, label="Naive Hybrid (No Hysteresis)"
    )
    ax3.errorbar(
        loss_percent, m_prop, yerr=s_prop, fmt="^-", color="#2E7D32",
        linewidth=2.2, capsize=4, label="Proposed Hybrid (Hysteresis)"
    )
    ax3.set_xlabel("Packet Loss Rate (%)", fontsize=11, fontweight="bold")
    ax3.set_ylabel("Average Mode Switches per Run", fontsize=11, fontweight="bold")
    ax3.set_title("Chattering Suppression Analysis", fontweight="bold", fontsize=12)
    ax3.grid(True, linestyle="--", alpha=0.5)
    ax3.legend(fontsize=9, loc="upper left")

    plt.tight_layout()
    plot_path = f"{output_dir}/network_loss_comparison.png"
    plt.savefig(plot_path)
    plt.close(fig)

    print(f"\n>>> Stress-tested network sweep complete! Plot saved to: {plot_path} <<<")


if __name__ == "__main__":
    main()
