"""
Outage and Burst Loss Experiment (Task B Item 1 & Item 3).
Evaluates 4 baselines on identical random seeds:
1. Pure Centralized (with hold-last-command on link loss)
2. Pure Decentralized (local peer-to-peer consensus & APF)
3. Naive Hybrid (instant switching, 0 dwell time, 0 hysteresis)
4. Proposed Hybrid (asymmetric hysteresis, 20-tick sliding window >= 70%, dwell time 2.0s)

Stress-test conditions:
- Gilbert-Elliott correlated burst loss (Markov transition)
- Deterministic complete RF outages of 1.0s, 2.0s, and 3.0s at t = 3.0s
- Moving reference centroid (0.85 m/s) with mid-flight morphing (V-Shape -> Line at t = 6.0s)

Reports raw numbers (mean +/- std):
- Fallback entries
- Time spent in fallback (s)
- Recovery time after outage ends (s)
- Mode switches
- Tracking error (m)
- Min inter-drone distance (m) vs collision barrier (0.7m)
"""

import os
import shutil
from typing import Dict, List, Tuple
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from swarm_core.drone import Drone
from swarm_core.formations import FormationType
from simulator.engine import SwarmSimulation


def run_outage_trial(
    baseline: str,
    outage_duration: float,
    use_burst_loss: bool = False,
    num_drones: int = 5,
    seed: int = 42,
    sim_duration: float = 12.0,
) -> Dict[str, float]:
    rng = np.random.default_rng(seed)
    # Scatter drones in initial non-overlapping positions
    positions = []
    while len(positions) < num_drones:
        cand = rng.uniform(-3.0, 3.0, size=2)
        if all(np.linalg.norm(cand - p) >= 1.2 for p in positions):
            positions.append(cand)

    drones = [Drone(drone_id=i, initial_position=positions[i]) for i in range(num_drones)]

    # Determine control mode
    if baseline == "centralized_hold":
        sim_mode = "centralized"
    elif baseline == "decentralized":
        sim_mode = "decentralized"
    elif baseline in ["hybrid_naive", "hybrid_proposed"]:
        sim_mode = "hybrid"
    else:
        raise ValueError(f"Unknown baseline: {baseline}")

    sim = SwarmSimulation(
        drones=drones,
        control_mode=sim_mode,
        comm_range=15.0,
        packet_loss_rate=0.05 if not use_burst_loss else 0.0,
        latency_mean=0.03,
        dt=0.05,
        use_velocity_feedforward=True,
        seed=seed,
        use_gilbert_elliott=use_burst_loss,
        p_g_to_b=0.05,
        p_b_to_g=0.20,
    )

    if baseline == "hybrid_naive":
        # Naive hybrid: degrade on 1 missed packet, recover on 1 good packet, 0 dwell time
        sim.hybrid_ctrl.degrade_timeout = 0.06
        sim.hybrid_ctrl.recovery_consecutive_hb = 1
        sim.hybrid_ctrl.min_dwell_time = 0.0
        sim.hybrid_ctrl.ramp_duration = 0.01
        sim.hybrid_ctrl.recovery_ratio_threshold = 0.0

    # Add scheduled deterministic outage at t = 3.0s
    if outage_duration > 0:
        sim.channel.add_outage(start_time=3.0, duration=outage_duration)

    # Initial formation: V-Shape
    sim.set_formation(FormationType.V_SHAPE, centroid=np.array([0.0, 0.0]))

    steps = int(sim_duration / sim.dt)
    centroid = np.array([0.0, 0.0], dtype=np.float64)
    v_target = np.array([0.8, 0.3], dtype=np.float64)  # 0.854 m/s moving centroid

    outage_end_time = 3.0 + outage_duration if outage_duration > 0 else 0.0
    recovery_detected_time = -1.0

    for step_i in range(steps):
        t = step_i * sim.dt
        centroid += v_target * sim.dt

        # Morph formation at t = 6.0s (mid-flight stress test)
        if t >= 6.0 and sim.current_formation == FormationType.V_SHAPE:
            sim.set_formation(FormationType.LINE, centroid=centroid)
        else:
            sim.centroid_target = centroid.copy()
            sim.centroid_velocity = v_target.copy()

        sim.step()

        # Track recovery after outage window ends
        if (
            baseline in ["hybrid_proposed", "hybrid_naive"]
            and outage_duration > 0
            and t > outage_end_time
            and recovery_detected_time < 0
        ):
            # Check if all drones have returned to CENTRALIZED
            modes = [sim.hybrid_ctrl.get_drone_mode(d.id).value for d in sim.drones]
            if all(m == "centralized" for m in modes):
                recovery_detected_time = t - outage_end_time

    summary = sim.metrics.get_summary()

    # Extract hybrid fallback metrics
    if baseline in ["hybrid_proposed", "hybrid_naive"]:
        fb_stats = sim.hybrid_ctrl.get_fallback_stats()
        fallback_entries = fb_stats["total_fallback_entries"] / num_drones
        time_in_fallback = fb_stats["total_time_in_fallback_s"] / num_drones
        avg_rec_time = recovery_detected_time if recovery_detected_time >= 0 else (
            fb_stats["avg_recovery_time_s"] if fb_stats["avg_recovery_time_s"] > 0 else 0.0
        )
    elif baseline == "decentralized":
        fallback_entries = 0.0
        time_in_fallback = sim_duration  # 100% time in decentralized
        avg_rec_time = 0.0
    else:  # centralized_hold
        fallback_entries = 0.0
        time_in_fallback = 0.0  # Never falls back to decentralized
        avg_rec_time = 0.0

    return {
        "final_error": summary.get("final_formation_error_m", 0.0),
        "mean_error": summary.get("mean_formation_error_m", 0.0),
        "transient_error": summary.get("transient_morph_error_m", 0.0),
        "steady_error": summary.get("steady_state_error_m", 0.0),
        "min_dist": summary.get("min_recorded_distance_m", 0.0),
        "switches": summary.get("total_mode_switches", 0.0),
        "fallback_entries": fallback_entries,
        "time_in_fallback": time_in_fallback,
        "recovery_time": avg_rec_time,
    }


def main():
    from pathlib import Path
    output_dir = Path(__file__).resolve().parent / "results"
    output_dir.mkdir(parents=True, exist_ok=True)

    outages = [0.0, 1.0, 2.0, 3.0]
    num_seeds = 6
    seeds = [42 + i * 17 for i in range(num_seeds)]

    baselines = [
        ("centralized_hold", "Centralized (Hold Command)"),
        ("decentralized", "Decentralized (Consensus)"),
        ("hybrid_naive", "Naive Hybrid (No Dwell)"),
        ("hybrid_proposed", "Proposed Hybrid (Hardened)"),
    ]

    print("=========================================================================================")
    print("  EXPERIMENT 1: DETERMINISTIC OUTAGES & BURST LOSS ACROSS 4 BASELINES (IDENTICAL SEEDS)   ")
    print(f"  Seeds tested: {num_seeds} ({seeds}) | Outages: {outages}s | Drones: 5               ")
    print("=========================================================================================")

    results = {
        b_key: {
            dur: {"fallback_entries": [], "time_fallback": [], "rec_time": [], "switches": [], "error": [], "min_dist": []}
            for dur in outages
        }
        for b_key, _ in baselines
    }

    # Header for log table
    print(f"{'Baseline':<28} | {'Outage':<6} | {'FB Entries':<12} | {'Time FB (s)':<12} | {'Recov (s)':<10} | {'Switches':<10} | {'Steady Err':<12} | {'Min Dist'}")
    print("-" * 110)

    for dur in outages:
        for b_key, b_label in baselines:
            fb_entries_list, time_fb_list, rec_time_list, switches_list, err_list, dist_list = [], [], [], [], [], []
            for s in seeds:
                res = run_outage_trial(
                    baseline=b_key,
                    outage_duration=dur,
                    use_burst_loss=False,
                    num_drones=5,
                    seed=s,
                )
                fb_entries_list.append(res["fallback_entries"])
                time_fb_list.append(res["time_in_fallback"])
                rec_time_list.append(res["recovery_time"])
                switches_list.append(res["switches"])
                err_list.append(res["steady_error"])
                dist_list.append(res["min_dist"])

            m_fb = float(np.mean(fb_entries_list))
            s_fb = float(np.std(fb_entries_list))
            m_tfb = float(np.mean(time_fb_list))
            s_tfb = float(np.std(time_fb_list))
            m_rec = float(np.mean(rec_time_list))
            s_rec = float(np.std(rec_time_list))
            m_sw = float(np.mean(switches_list))
            s_sw = float(np.std(switches_list))
            m_err = float(np.mean(err_list))
            s_err = float(np.std(err_list))
            m_dist = float(np.mean(dist_list))
            s_dist = float(np.std(dist_list))

            results[b_key][dur]["fallback_entries"] = (m_fb, s_fb)
            results[b_key][dur]["time_fallback"] = (m_tfb, s_tfb)
            results[b_key][dur]["rec_time"] = (m_rec, s_rec)
            results[b_key][dur]["switches"] = (m_sw, s_sw)
            results[b_key][dur]["error"] = (m_err, s_err)
            results[b_key][dur]["min_dist"] = (m_dist, s_dist)

            print(
                f"{b_label:<28} | {dur:<6.1f} | {m_fb:4.1f}±{s_fb:3.1f}     | {m_tfb:5.2f}±{s_tfb:4.2f}   | "
                f"{m_rec:4.2f}±{s_rec:4.2f}   | {m_sw:4.1f}±{s_sw:3.1f}     | {m_err:5.3f}±{s_err:4.3f}m | {m_dist:5.2f}±{s_dist:4.2f}m"
            )

    # Also run Gilbert-Elliott burst loss comparison
    print("\n-----------------------------------------------------------------------------------------")
    print("  GILBERT-ELLIOTT BURST LOSS (p_g_to_b=0.05, p_b_to_g=0.20, burst loss=100%)              ")
    print("-----------------------------------------------------------------------------------------")
    for b_key, b_label in baselines:
        fb_list, tfb_list, sw_list, err_list, dist_list = [], [], [], [], []
        for s in seeds:
            res = run_outage_trial(
                baseline=b_key,
                outage_duration=0.0,
                use_burst_loss=True,
                num_drones=5,
                seed=s,
            )
            fb_list.append(res["fallback_entries"])
            tfb_list.append(res["time_in_fallback"])
            sw_list.append(res["switches"])
            err_list.append(res["steady_error"])
            dist_list.append(res["min_dist"])
        print(
            f"{b_label:<28} | GE-BURST | {np.mean(fb_list):4.1f}±{np.std(fb_list):3.1f}     | {np.mean(tfb_list):5.2f}±{np.std(tfb_list):4.2f}   | "
            f"{'-':<10} | {np.mean(sw_list):4.1f}±{np.std(sw_list):3.1f}     | {np.mean(err_list):5.3f}±{np.std(err_list):4.3f}m | {np.mean(dist_list):5.2f}±{np.std(dist_list):4.2f}m"
        )

    # Plot Outage Performance comparison
    fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(14, 10), dpi=180)
    durations = np.array(outages)

    colors = {
        "centralized_hold": "#D32F2F",
        "decentralized": "#1976D2",
        "hybrid_naive": "#F57C00",
        "hybrid_proposed": "#2E7D32",
    }
    markers = {
        "centralized_hold": "o",
        "decentralized": "s",
        "hybrid_naive": "x",
        "hybrid_proposed": "^",
    }

    # 1. Fallback Duration vs Outage Duration
    for b_key in ["hybrid_naive", "hybrid_proposed"]:
        means = [results[b_key][d]["time_fallback"][0] for d in durations]
        stds = [results[b_key][d]["time_fallback"][1] for d in durations]
        ax1.errorbar(durations, means, yerr=stds, label=dict(baselines)[b_key], color=colors[b_key], marker=markers[b_key], linewidth=2, capsize=4)
    ax1.plot(durations, durations, 'k--', alpha=0.5, label="Ideal 1:1 Outage Duration")
    ax1.set_xlabel("Outage Duration (s)", fontweight="bold")
    ax1.set_ylabel("Time in Fallback Mode (s)", fontweight="bold")
    ax1.set_title("Fallback Dwell Duration", fontweight="bold")
    ax1.grid(True, linestyle="--", alpha=0.5)
    ax1.legend()

    # 2. Recovery Time vs Outage Duration
    for b_key in ["hybrid_naive", "hybrid_proposed"]:
        means = [results[b_key][d]["rec_time"][0] for d in durations]
        stds = [results[b_key][d]["rec_time"][1] for d in durations]
        ax2.errorbar(durations, means, yerr=stds, label=dict(baselines)[b_key], color=colors[b_key], marker=markers[b_key], linewidth=2, capsize=4)
    ax2.set_xlabel("Outage Duration (s)", fontweight="bold")
    ax2.set_ylabel("Recovery Time after Link Restored (s)", fontweight="bold")
    ax2.set_title("Recovery Responsiveness Post-Outage", fontweight="bold")
    ax2.grid(True, linestyle="--", alpha=0.5)
    ax2.legend()

    # 3. Mode Switches (Chattering)
    for b_key in ["hybrid_naive", "hybrid_proposed"]:
        means = [results[b_key][d]["switches"][0] for d in durations]
        stds = [results[b_key][d]["switches"][1] for d in durations]
        ax3.errorbar(durations, means, yerr=stds, label=dict(baselines)[b_key], color=colors[b_key], marker=markers[b_key], linewidth=2, capsize=4)
    ax3.set_xlabel("Outage Duration (s)", fontweight="bold")
    ax3.set_ylabel("Mode Switches per Run", fontweight="bold")
    ax3.set_title("Chattering Suppression during Outages", fontweight="bold")
    ax3.grid(True, linestyle="--", alpha=0.5)
    ax3.legend()

    # 4. Steady-State Tracking Error vs Outage Duration
    for b_key, b_label in baselines:
        means = [results[b_key][d]["error"][0] for d in durations]
        stds = [results[b_key][d]["error"][1] for d in durations]
        ax4.errorbar(durations, means, yerr=stds, label=b_label, color=colors[b_key], marker=markers[b_key], linewidth=2, capsize=4)
    ax4.set_xlabel("Outage Duration (s)", fontweight="bold")
    ax4.set_ylabel("Steady-State Error (m)", fontweight="bold")
    ax4.set_title("Tracking Error Across 4 Baselines", fontweight="bold")
    ax4.grid(True, linestyle="--", alpha=0.5)
    ax4.legend()

    plt.tight_layout()
    plot_file = output_dir / "outage_burst_comparison.png"
    plt.savefig(plot_file)
    plt.close(fig)

    print(f"\nSaved outage experiment plots to {plot_file}.")


if __name__ == "__main__":
    main()
