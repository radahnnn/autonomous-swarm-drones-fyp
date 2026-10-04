"""
GPS Noise Sweep Experiment (Task B Item 2 & Recommendation 1).
Evaluates the effect of horizontal GPS positioning noise on:
1. Formation Tracking Error (True Physical Position vs Intended Formation Target)
2. Minimum Inter-Drone Separation vs Collision Threshold (0.70m) and APF Safety Radius (1.20m/1.50m)

Sensor Modeling:
- First-order Gauss-Markov time-correlated GPS error (tau_corr ~ 30s):
  e_GPS,i[k+1] = phi * e_GPS,i[k] + sqrt(1 - phi^2) * w_GPS,i
  w_GPS,i = w_common + w_indep,i
  where w_common is shared across the swarm (60% variance) due to identical satellite geometry / atmosphere,
  and w_indep,i is independent per quad (40% variance) due to multipath / receiver noise.
- Velocity error is modeled separately (sigma_v ~ 0.08 m/s, reflecting fused GNSS Doppler/IMU estimation).
- Separation is evaluated strictly on true ground-truth physical coordinates, not corrupted sensor measurements.

Noise Levels Tested:
  sigma in {0.04, 0.5, 1.5, 2.5} m
  - 0.04m: RTK-GPS baseline (assumed upper bound)
  - 0.50m: High-grade GNSS / DGPS
  - 1.50m: Standard plain U-Blox M8N/M9N GPS (assumed baseline)
  - 2.50m: Degraded plain GPS under poor DOP / canopy

Reports raw numbers (mean +/- std) across multiple seeds.
"""

import os
import shutil
from typing import Dict, List
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from swarm_core.drone import Drone
from swarm_core.formations import FormationType
from simulator.engine import SwarmSimulation


def run_gps_noise_trial(
    sigma: float,
    baseline: str = "hybrid_proposed",
    common_mode_fraction: float = 0.60,
    num_drones: int = 5,
    seed: int = 42,
    sim_duration: float = 12.0,
) -> Dict[str, float]:
    rng = np.random.default_rng(seed)
    positions = []
    while len(positions) < num_drones:
        cand = rng.uniform(-3.0, 3.0, size=2)
        if all(np.linalg.norm(cand - p) >= 1.2 for p in positions):
            positions.append(cand)

    drones = [
        Drone(
            drone_id=i,
            initial_position=positions[i],
            measurement_noise_std=0.0,  # SwarmSimulation handles the dual-component noise
        )
        for i in range(num_drones)
    ]

    sim_mode = "hybrid" if "hybrid" in baseline else baseline

    sim = SwarmSimulation(
        drones=drones,
        control_mode=sim_mode,
        comm_range=15.0,
        packet_loss_rate=0.05,
        latency_mean=0.03,
        dt=0.05,
        use_velocity_feedforward=True,
        gps_noise_std=sigma,
        gps_common_mode_fraction=common_mode_fraction,
        seed=seed,
    )

    # Formation setup: V-Shape
    sim.set_formation(FormationType.V_SHAPE, centroid=np.array([0.0, 0.0]))

    steps = int(sim_duration / sim.dt)
    centroid = np.array([0.0, 0.0], dtype=np.float64)
    v_target = np.array([0.8, 0.3], dtype=np.float64)

    for step_i in range(steps):
        t = step_i * sim.dt
        centroid += v_target * sim.dt

        # Morph at t = 6.0s
        if t >= 6.0 and sim.current_formation == FormationType.V_SHAPE:
            sim.set_formation(FormationType.LINE, centroid=centroid)
        else:
            sim.centroid_target = centroid.copy()
            sim.centroid_velocity = v_target.copy()

        sim.step()

    summary = sim.metrics.get_summary()
    return {
        "final_error": summary.get("final_formation_error_m", 0.0),
        "mean_error": summary.get("mean_formation_error_m", 0.0),
        "transient_error": summary.get("transient_morph_error_m", 0.0),
        "steady_error": summary.get("steady_state_error_m", 0.0),
        "min_dist": summary.get("min_recorded_distance_m", 0.0),
        "any_collision": summary.get("any_collision", 0.0),
    }


def main():
    from pathlib import Path
    output_dir = Path(__file__).resolve().parent / "results"
    output_dir.mkdir(parents=True, exist_ok=True)

    sigmas = [0.04, 0.50, 1.50, 2.50]
    num_seeds = 6
    seeds = [100 + i * 23 for i in range(num_seeds)]
    common_mode_ratio = 0.60

    baselines = [
        ("hybrid_proposed", "Proposed Hybrid"),
        ("centralized", "Pure Centralized"),
        ("decentralized", "Pure Decentralized"),
    ]

    print("=========================================================================================")
    print("  EXPERIMENT 2: GPS NOISE SWEEP WITH COMMON-MODE (60%) + INDEPENDENT (40%) ERROR          ")
    print(f"  Sigmas: {sigmas} m | Seeds: {num_seeds} | Common-Mode Ratio: {common_mode_ratio}        ")
    print("=========================================================================================")

    results = {
        b_key: {
            sig: {"steady_err": [], "trans_err": [], "min_dist": [], "collision": []}
            for sig in sigmas
        }
        for b_key, _ in baselines
    }

    print(f"{'Baseline':<20} | {'GPS Sigma':<10} | {'Steady Err (m)':<18} | {'Transient Err (m)':<18} | {'Min Dist (m)':<16} | {'Collisions'}")
    print("-" * 105)

    for sig in sigmas:
        for b_key, b_label in baselines:
            s_err_list, t_err_list, dist_list, col_list = [], [], [], []
            for s in seeds:
                res = run_gps_noise_trial(
                    sigma=sig,
                    baseline=b_key,
                    common_mode_fraction=common_mode_ratio,
                    seed=s,
                )
                s_err_list.append(res["steady_error"])
                t_err_list.append(res["transient_error"])
                dist_list.append(res["min_dist"])
                col_list.append(res["any_collision"])

            m_s_err = float(np.mean(s_err_list))
            s_s_err = float(np.std(s_err_list))
            m_t_err = float(np.mean(t_err_list))
            s_t_err = float(np.std(t_err_list))
            m_dist = float(np.mean(dist_list))
            s_dist = float(np.std(dist_list))
            total_col = int(np.sum(col_list))

            results[b_key][sig]["steady_err"] = (m_s_err, s_s_err)
            results[b_key][sig]["trans_err"] = (m_t_err, s_t_err)
            results[b_key][sig]["min_dist"] = (m_dist, s_dist)
            results[b_key][sig]["collision"] = total_col

            print(
                f"{b_label:<20} | {sig:<10.2f} | {m_s_err:5.3f} ± {s_s_err:5.3f}   | "
                f"{m_t_err:5.3f} ± {s_t_err:5.3f}   | {m_dist:5.3f} ± {s_dist:5.3f} | {total_col}/{num_seeds}"
            )

    # Plot GPS Noise Sweep
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 5.5), dpi=180)
    sig_arr = np.array(sigmas)

    colors = {
        "hybrid_proposed": "#2E7D32",
        "centralized": "#D32F2F",
        "decentralized": "#1976D2",
    }
    markers = {
        "hybrid_proposed": "^",
        "centralized": "o",
        "decentralized": "s",
    }

    # 1. Steady-State Tracking Error vs GPS Noise Sigma
    for b_key, b_label in baselines:
        means = [results[b_key][s]["steady_err"][0] for s in sig_arr]
        stds = [results[b_key][s]["steady_err"][1] for s in sig_arr]
        ax1.errorbar(
            sig_arr, means, yerr=stds, label=b_label,
            color=colors[b_key], marker=markers[b_key], linewidth=2.2, capsize=4
        )
    ax1.plot(sig_arr, sig_arr, 'k--', alpha=0.5, label="y = sigma (1:1 Noise Floor)")
    ax1.set_xlabel("GPS Noise Std Dev $\\sigma$ (m)", fontweight="bold")
    ax1.set_ylabel("Steady-State Tracking Error (m)", fontweight="bold")
    ax1.set_title("Tracking Error vs GPS Sensor Noise", fontweight="bold")
    ax1.grid(True, linestyle="--", alpha=0.5)
    ax1.legend()

    # 2. Min Inter-Drone Distance vs GPS Noise Sigma
    for b_key, b_label in baselines:
        means = [results[b_key][s]["min_dist"][0] for s in sig_arr]
        stds = [results[b_key][s]["min_dist"][1] for s in sig_arr]
        ax2.errorbar(
            sig_arr, means, yerr=stds, label=b_label,
            color=colors[b_key], marker=markers[b_key], linewidth=2.2, capsize=4
        )
    ax2.axhline(0.70, color="crimson", linestyle="--", linewidth=1.8, label="Collision Threshold (0.70m)")
    ax2.axhline(2.50, color="navy", linestyle=":", linewidth=1.5, label="Nominal Spacing (2.50m)")
    ax2.set_xlabel("GPS Noise Std Dev $\\sigma$ (m)", fontweight="bold")
    ax2.set_ylabel("Min Inter-Drone Distance (m)", fontweight="bold")
    ax2.set_title("Physical Safety Preservation vs Sensor Noise", fontweight="bold")
    ax2.grid(True, linestyle="--", alpha=0.5)
    ax2.legend()

    plt.tight_layout()
    plot_path = output_dir / "gps_noise_sweep_comparison.png"
    plt.savefig(plot_path)
    plt.close(fig)

    print(f"\nSaved GPS noise sweep plot to {plot_path}.")


if __name__ == "__main__":
    main()
