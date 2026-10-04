"""
Headline Experiment: Ground-Link Outage Overlapping Mid-Flight Turn & Formation Morph
=====================================================================================
Evaluates swarm resilience when the centralized Command & Control (C2) link experiences
a prolonged outage (1.0 to 10.0 seconds), while local inter-drone peer links remain alive.

Crucial Stress-Test Dynamics:
1. Heading Turn: Trajectory centroid turns heading by ~60 degrees at t = 5.0s.
2. Formation Morph: Mid-flight reconfiguration from V-Shape to Line at t = 6.0s.
3. Ground Outage: Link drops at t = 4.5s (overlapping BOTH the turn and the morph).

Fallback Strategies & Baselines Evaluated:
1. Centralized (Hold-Last-Target): Closed-loop onboard tracking of last received waypoint.
2. Decentralized (Consensus + Drag FF): Local peer consensus with drag feedforward.
3. Hybrid: Hover-on-Loss (hover): Immediately brakes to hover at loss point.
4. Hybrid: Hold-Last-Target (hold_target): Brakes to hold last received target slot.
5. Hybrid: Dead-Reckon-then-Brake (dead_reckon): Continues along last velocity for T=2.0s, then brakes.
6. Hybrid: Proposed Consensus Flocking (consensus): Hardened hysteresis peer consensus flocking.

Reports:
- Fallback behavior mode
- Distance from intended path / tracking error (m)
- Min inter-drone separation (m) vs active safety barrier (0.70m)
- Time to regain formation (s) after link restoration
- Collisions (count)
- Active safety parameter provenance (safe_radius, collision_dist, collision_threshold)
"""

import os
import sys
import shutil
from pathlib import Path
from typing import Dict, List, Optional, Tuple

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from swarm_core.drone import Drone
from swarm_core.formations import FormationType
from simulator.engine import SwarmSimulation


def run_headline_trial(
    strategy_key: str,
    outage_duration: float,
    num_drones: int = 5,
    seed: int = 42,
    sim_duration: float = 18.0,
    outage_start: float = 4.5,
    dead_reckon_duration: float = 2.0,
) -> Dict[str, any]:
    rng = np.random.default_rng(seed)
    # Scatter drones in non-overlapping initial positions around origin
    positions = []
    while len(positions) < num_drones:
        cand = rng.uniform(-2.5, 2.5, size=2)
        if all(np.linalg.norm(cand - p) >= 1.2 for p in positions):
            positions.append(cand)

    drones = [Drone(drone_id=i, initial_position=positions[i]) for i in range(num_drones)]

    # Map strategy key to simulation mode and fallback strategy
    if strategy_key == "centralized_hold_target":
        sim_mode = "centralized"
        fb_strat = "consensus"
    elif strategy_key == "decentralized":
        sim_mode = "decentralized"
        fb_strat = "consensus"
    elif strategy_key.startswith("hybrid_"):
        sim_mode = "hybrid"
        fb_strat = strategy_key.replace("hybrid_", "")
    else:
        raise ValueError(f"Unknown strategy: {strategy_key}")

    sim = SwarmSimulation(
        drones=drones,
        control_mode=sim_mode,
        fallback_strategy=fb_strat,
        dead_reckon_duration=dead_reckon_duration,
        comm_range=18.0,
        packet_loss_rate=0.02,
        latency_mean=0.03,
        dt=0.05,
        use_velocity_feedforward=True,
        seed=seed,
    )

    # Inject ground-link-only outage (peers remain fully functional)
    if outage_duration > 0:
        sim.channel.add_outage(
            start_time=outage_start,
            duration=outage_duration,
            scope="ground",
            recipients=None,
        )

    # Initial formation: V-Shape
    sim.set_formation(FormationType.V_SHAPE, centroid=np.array([0.0, 0.0]))

    steps = int(sim_duration / sim.dt)
    centroid = np.array([0.0, 0.0], dtype=np.float64)
    v_target = np.array([0.8, 0.0], dtype=np.float64)  # Initial straight trajectory along +X

    outage_end_time = outage_start + outage_duration if outage_duration > 0 else 0.0
    history_t = []
    history_err = []
    history_min_dist = []
    history_centroid = []
    history_drone_positions = {d.id: [] for d in sim.drones}

    for step_i in range(steps):
        t = step_i * sim.dt

        # Trajectory turn: at t = 5.0s, ground station commands 60-degree heading change
        if t >= 5.0:
            v_target = np.array([0.4, 0.7], dtype=np.float64)
        else:
            v_target = np.array([0.8, 0.0], dtype=np.float64)

        centroid += v_target * sim.dt

        # Formation morph: at t = 6.0s, ground station commands transition from V-Shape to Line
        if t >= 6.0 and sim.current_formation == FormationType.V_SHAPE:
            sim.set_formation(FormationType.LINE, centroid=centroid)
        else:
            sim.centroid_target = centroid.copy()
            sim.centroid_velocity = v_target.copy()

        sim.step()

        # Record histories
        history_t.append(t)
        last_snap = sim.metrics.history[-1] if len(sim.metrics.history) > 0 else None
        err = last_snap.formation_error if last_snap is not None else 0.0
        min_d = last_snap.min_inter_drone_dist if last_snap is not None else 2.0
        history_err.append(err)
        history_min_dist.append(min_d)
        history_centroid.append(centroid.copy())
        for d in sim.drones:
            history_drone_positions[d.id].append(d.position.copy())

    summary = sim.metrics.get_summary()

    # Calculate formation recovery time after outage ends
    formation_rec_time = (
        sim.metrics.calculate_formation_recovery_time(outage_end_time)
        if outage_duration > 0
        else 0.0
    )

    # Fallback duration and entry count
    if sim_mode == "hybrid":
        fb_stats = sim.hybrid_ctrl.get_fallback_stats()
        time_in_fallback = fb_stats["total_time_in_fallback_s"] / num_drones
        switches = sim.hybrid_ctrl.get_total_mode_switches()
    else:
        time_in_fallback = outage_duration if sim_mode == "centralized" else sim_duration
        switches = 0

    return {
        "final_error": summary.get("final_formation_error_m", 0.0),
        "mean_error": summary.get("mean_formation_error_m", 0.0),
        "max_error": float(np.max(history_err)) if history_err else 0.0,
        "steady_error": summary.get("steady_state_error_m", 0.0),
        "min_dist": summary.get("min_recorded_distance_m", 0.0),
        "any_collision": summary.get("any_collision", 0),
        "total_collisions": summary.get("total_collisions", 0),
        "time_in_fallback": time_in_fallback,
        "formation_recovery_time": formation_rec_time,
        "switches": switches,
        "history_t": np.array(history_t),
        "history_err": np.array(history_err),
        "history_min_dist": np.array(history_min_dist),
        "history_centroid": np.array(history_centroid),
        "history_drone_positions": {k: np.array(v) for k, v in history_drone_positions.items()},
    }


def main():
    output_dir = Path(__file__).resolve().parent / "results"
    output_dir.mkdir(parents=True, exist_ok=True)

    dummy_sim = SwarmSimulation([Drone(0, [0.0, 0.0])], control_mode="hybrid")
    active_safe_radius = dummy_sim.decentral_ctrl.safe_radius
    active_collision_thresh = dummy_sim.metrics.collision_threshold
    active_collision_dist = dummy_sim.central_ctrl.collision_dist

    print("====================================================================================================")
    print("  HEADLINE EXPERIMENT: GROUND-LINK OUTAGE OVERLAPPING TRAJECTORY TURN & FORMATION MORPH             ")
    print("  Scenario: Turn at t=5.0s (60 deg) | Morph at t=6.0s (V->Line) | Outage at t=4.5s (Peers Alive)     ")
    print(f"  Active Safety Provenance: safe_radius={active_safe_radius}m | collision_dist={active_collision_dist}m | collision_thresh={active_collision_thresh}m")
    print("====================================================================================================")

    outages = [1.0, 2.0, 4.0, 6.0, 8.0, 10.0]
    num_seeds = 5
    seeds = [42 + i * 13 for i in range(num_seeds)]

    strategies = [
        ("centralized_hold_target", "Centralized (Hold-Last-Target)", "red"),
        ("decentralized", "Decentralized (Consensus + Drag FF)", "orange"),
        ("hybrid_hover", "Hybrid: Hover-on-Loss", "purple"),
        ("hybrid_hold_target", "Hybrid: Hold-Last-Target", "brown"),
        ("hybrid_dead_reckon", "Hybrid: Dead-Reckon-then-Brake (T=2s)", "blue"),
        ("hybrid_consensus", "Hybrid: Proposed Consensus Flocking", "green"),
    ]

    # Store aggregated results
    agg_results = {s_key: {dur: {} for dur in outages} for s_key, _, _ in strategies}

    print(f"{'Strategy':<35} | {'Outage':<6} | {'Max Err (m)':<11} | {'Steady Err':<10} | {'Min Dist (m)':<12} | {'Recov Time (s)':<14} | {'Collisions'}")
    print("-" * 105)

    single_run_traces = {}

    for dur in outages:
        for s_key, s_label, _ in strategies:
            max_err_list, steady_err_list, min_dist_list, recov_time_list, col_list = [], [], [], [], []
            for s_idx, s in enumerate(seeds):
                res = run_headline_trial(
                    strategy_key=s_key,
                    outage_duration=dur,
                    num_drones=5,
                    seed=s,
                    sim_duration=18.0,
                    outage_start=4.5,
                )
                max_err_list.append(res["max_error"])
                steady_err_list.append(res["steady_error"])
                min_dist_list.append(res["min_dist"])
                recov_time_list.append(res["formation_recovery_time"])
                col_list.append(res["total_collisions"])

                # Capture trace for representative seed at dur=6.0s
                if dur == 6.0 and s_idx == 0:
                    single_run_traces[s_key] = res

            agg_results[s_key][dur] = {
                "max_err_mean": float(np.mean(max_err_list)),
                "max_err_std": float(np.std(max_err_list)),
                "steady_err_mean": float(np.mean(steady_err_list)),
                "steady_err_std": float(np.std(steady_err_list)),
                "min_dist_mean": float(np.mean(min_dist_list)),
                "min_dist_min": float(np.min(min_dist_list)),
                "recov_time_mean": float(np.mean(recov_time_list)),
                "recov_time_std": float(np.std(recov_time_list)),
                "total_collisions": int(np.sum(col_list)),
            }

            print(
                f"{s_label:<35} | {dur:<6.1f} | "
                f"{np.mean(max_err_list):<5.2f} ± {np.std(max_err_list):<4.2f} | "
                f"{np.mean(steady_err_list):<5.3f}    | "
                f"{np.min(min_dist_list):<5.2f} (avg {np.mean(min_dist_list):.2f}) | "
                f"{np.mean(recov_time_list):<5.2f} ± {np.std(recov_time_list):<4.2f}     | "
                f"{int(np.sum(col_list))}"
            )
        print("-" * 105)

    # -------------------------------------------------------------------------
    # Visualization: 4-Panel Publication Figure
    # -------------------------------------------------------------------------
    fig = plt.figure(figsize=(18, 12))
    gs = fig.add_gridspec(2, 2, hspace=0.30, wspace=0.25)

    # Panel 1: 2D Spatial Trajectories under 6.0s Outage (overlapping turn & morph)
    ax1 = fig.add_subplot(gs[0, 0])
    ref_res = single_run_traces.get("hybrid_consensus")
    if ref_res is not None:
        ax1.plot(
            ref_res["history_centroid"][:, 0],
            ref_res["history_centroid"][:, 1],
            "k--",
            linewidth=2.5,
            label="Intended Centroid (Turn at t=5s)",
        )
        # Mark turn and morph points
        t_arr = ref_res["history_t"]
        idx_turn = int(5.0 / 0.05)
        idx_morph = int(6.0 / 0.05)
        ax1.plot(ref_res["history_centroid"][idx_turn, 0], ref_res["history_centroid"][idx_turn, 1], "ro", markersize=8, label="Turn Start (t=5s)")
        ax1.plot(ref_res["history_centroid"][idx_morph, 0], ref_res["history_centroid"][idx_morph, 1], "b^", markersize=8, label="Morph Start (t=6s)")

    for s_key, s_label, col in strategies:
        if s_key in single_run_traces:
            trace = single_run_traces[s_key]
            # Plot swarm center of mass
            drone_coords = np.array([trace["history_drone_positions"][d_id] for d_id in trace["history_drone_positions"]])
            swarm_centroid = np.mean(drone_coords, axis=0)
            ax1.plot(swarm_centroid[:, 0], swarm_centroid[:, 1], color=col, linewidth=1.8, label=s_label)

    ax1.set_title("A: 2D Flight Trajectories During Ground Outage (6.0s Duration)", fontsize=12, fontweight="bold")
    ax1.set_xlabel("X Position (m)", fontsize=11)
    ax1.set_ylabel("Y Position (m)", fontsize=11)
    ax1.legend(loc="upper left", fontsize=8.5)
    ax1.grid(True, alpha=0.3)

    # Panel 2: Formation Tracking Error over Time e(t) for 6.0s Outage
    ax2 = fig.add_subplot(gs[0, 1])
    for s_key, s_label, col in strategies:
        if s_key in single_run_traces:
            trace = single_run_traces[s_key]
            ax2.plot(trace["history_t"], trace["history_err"], color=col, linewidth=1.8, label=s_label)

    # Highlight outage duration window [4.5s, 10.5s]
    ax2.axvspan(4.5, 10.5, color="gray", alpha=0.18, label="Ground Outage Window (4.5s - 10.5s)")
    ax2.axvline(5.0, color="red", linestyle=":", label="Trajectory Turn (t=5s)")
    ax2.axvline(6.0, color="blue", linestyle=":", label="Morph Initiation (t=6s)")
    ax2.set_title("B: Formation Tracking Error Evolution e(t)", fontsize=12, fontweight="bold")
    ax2.set_xlabel("Time (s)", fontsize=11)
    ax2.set_ylabel("RMS Tracking Error (m)", fontsize=11)
    ax2.legend(loc="upper right", fontsize=8)
    ax2.grid(True, alpha=0.3)

    # Panel 3: Minimum Inter-Drone Separation Distance d_min(t)
    ax3 = fig.add_subplot(gs[1, 0])
    for s_key, s_label, col in strategies:
        if s_key in single_run_traces:
            trace = single_run_traces[s_key]
            ax3.plot(trace["history_t"], trace["history_min_dist"], color=col, linewidth=1.8, label=s_label)

    ax3.axhline(active_collision_thresh, color="red", linestyle="--", linewidth=2.0, label=f"Collision Barrier ({active_collision_thresh:.2f}m)")
    ax3.axhline(active_safe_radius, color="green", linestyle=":", linewidth=1.5, label=f"Active Safe Radius ({active_safe_radius:.2f}m)")
    ax3.axvspan(4.5, 10.5, color="gray", alpha=0.15)
    ax3.set_title("C: Minimum Inter-Drone Physical Separation Distance", fontsize=12, fontweight="bold")
    ax3.set_xlabel("Time (s)", fontsize=11)
    ax3.set_ylabel("Minimum Distance (m)", fontsize=11)
    ax3.set_ylim(bottom=0.0)
    ax3.legend(loc="upper right", fontsize=8.5)
    ax3.grid(True, alpha=0.3)

    # Panel 4: Outage Duration Sweep (1.0s to 10.0s) vs Formation Recovery Time
    ax4 = fig.add_subplot(gs[1, 1])
    for s_key, s_label, col in strategies:
        rec_means = [agg_results[s_key][dur]["recov_time_mean"] for dur in outages]
        rec_stds = [agg_results[s_key][dur]["recov_time_std"] for dur in outages]
        ax4.errorbar(
            outages,
            rec_means,
            yerr=rec_stds,
            marker="o",
            color=col,
            linewidth=2.0,
            capsize=4,
            label=s_label,
        )

    ax4.set_title("D: Formation Recovery Time vs Ground Outage Duration", fontsize=12, fontweight="bold")
    ax4.set_xlabel("Ground Link Outage Duration (s)", fontsize=11)
    ax4.set_ylabel("Time to Regain Formation (s)", fontsize=11)
    ax4.legend(loc="upper left", fontsize=8.5)
    ax4.grid(True, alpha=0.3)

    suptitle = fig.suptitle(
        f"Ground-Link Outage Resilience: Overlapping Turn & Formation Morph Across 6 Fallback Strategies\n"
        f"Active Safety Provenance: safe_radius = {active_safe_radius:.2f}m | collision_threshold = {active_collision_thresh:.2f}m | collision_dist = {active_collision_dist:.2f}m",
        fontsize=13,
        fontweight="bold",
        y=0.98,
    )

    plot_path = output_dir / "headline_turn_morph_outage.png"
    fig.savefig(plot_path, dpi=200, bbox_inches="tight")
    plt.close(fig)
    print(f"\n[Saved Headline Figure]: {plot_path}")



if __name__ == "__main__":
    main()
