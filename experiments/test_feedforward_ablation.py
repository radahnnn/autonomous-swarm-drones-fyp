"""
Velocity Feedforward Ablation & Error Decomposition Experiment (Task B Item 4).
Evaluates:
1. Steady-state error vs Target Velocity Feedforward (ON vs OFF).
2. Transient error during formation morphing (V-Shape -> Line at t = 5.0s) vs Settled Steady-State.
3. Analytical derivation validation:
   Why steady-state error without velocity feedforward is exactly ~1.14m at 0% loss:
   e_steady = (k_d + c_d) / k_p * ||v_target||
   e_steady = (2.2 + 0.20) / 1.8 * 0.8544 = (2.4 / 1.8) * 0.8544 = 1.1392 m.
"""

import os
from typing import Dict, List
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from swarm_core.drone import Drone
from swarm_core.formations import FormationType
from simulator.engine import SwarmSimulation


def run_feedforward_trial(
    use_feedforward: bool,
    mode: str = "hybrid",
    num_drones: int = 5,
    seed: int = 42,
    sim_duration: float = 12.0,
) -> Dict[str, any]:
    rng = np.random.default_rng(seed)
    positions = []
    while len(positions) < num_drones:
        cand = rng.uniform(-3.0, 3.0, size=2)
        if all(np.linalg.norm(cand - p) >= 1.2 for p in positions):
            positions.append(cand)

    drones = [Drone(drone_id=i, initial_position=positions[i], measurement_noise_std=0.0) for i in range(num_drones)]

    sim = SwarmSimulation(
        drones=drones,
        control_mode=mode,
        comm_range=15.0,
        packet_loss_rate=0.0,  # 0% loss to isolate kinematic/damping tracking lag
        latency_mean=0.001,
        dt=0.05,
        use_velocity_feedforward=use_feedforward,
        seed=seed,
    )

    sim.set_formation(FormationType.V_SHAPE, centroid=np.array([0.0, 0.0]))

    steps = int(sim_duration / sim.dt)
    centroid = np.array([0.0, 0.0], dtype=np.float64)
    v_target = np.array([0.8, 0.3], dtype=np.float64)  # ||v|| = 0.8544 m/s

    time_history = []
    error_history = []

    for step_i in range(steps):
        t = step_i * sim.dt
        centroid += v_target * sim.dt

        # Morph at t = 5.0s
        if t >= 5.0 and sim.current_formation == FormationType.V_SHAPE:
            sim.set_formation(FormationType.LINE, centroid=centroid)
        else:
            sim.centroid_target = centroid.copy()
            sim.centroid_velocity = v_target.copy()

        sim.step()
        time_history.append(t)
        error_history.append(sim.metrics.history[-1].formation_error)

    summary = sim.metrics.get_summary()
    return {
        "summary": summary,
        "time": np.array(time_history),
        "error": np.array(error_history),
    }


def main():
    output_dir = "experiments/results"
    os.makedirs(output_dir, exist_ok=True)

    seeds = [42, 63, 105, 204, 305, 406]
    modes = ["hybrid", "centralized"]

    print("=========================================================================================")
    print("  EXPERIMENT 3: TARGET VELOCITY FEEDFORWARD ABLATION & ERROR DECOMPOSITION                 ")
    print("  Comparing: use_velocity_feedforward in {True, False} at 0% packet loss                  ")
    print("=========================================================================================")

    # Analytical calculation
    kp = 1.8
    kd = 2.2
    cd = 0.20
    v_target_norm = float(np.linalg.norm([0.8, 0.3]))
    expected_lag_no_ff = ((kd + cd) / kp) * v_target_norm

    print(f"\n[ANALYTICAL DERIVATION]")
    print(f"  Target Velocity: ||v_target|| = sqrt(0.8^2 + 0.3^2) = {v_target_norm:.4f} m/s")
    print(f"  Controller Gains: kp = {kp}, kd = {kd}, Rotor Drag: cd = {cd}")
    print(f"  Theoretical Steady Lag: e_steady = ((kd + cd) / kp) * ||v_target||")
    print(f"                         = (({kd} + {cd}) / {kp}) * {v_target_norm:.4f}")
    print(f"                         = ({kd+cd:.2f} / {kp:.2f}) * {v_target_norm:.4f} = {expected_lag_no_ff:.4f} m (~1.14 m)")
    print("-" * 90)

    results_table = []

    for mode in modes:
        for ff_flag in [False, True]:
            t_errs, s_errs, f_errs = [], [], []
            sample_time = None
            sample_error = None
            for s in seeds:
                res = run_feedforward_trial(use_feedforward=ff_flag, mode=mode, seed=s)
                t_errs.append(res["summary"]["transient_morph_error_m"])
                s_errs.append(res["summary"]["steady_state_error_m"])
                f_errs.append(res["summary"]["final_formation_error_m"])
                if sample_time is None:
                    sample_time = res["time"]
                    sample_error = res["error"]

            m_t, s_t = float(np.mean(t_errs)), float(np.std(t_errs))
            m_s, s_s = float(np.mean(s_errs)), float(np.std(s_errs))
            m_f, s_f = float(np.mean(f_errs)), float(np.std(f_errs))

            ff_label = "WITH Feedforward" if ff_flag else "WITHOUT Feedforward"
            print(
                f"Mode: {mode.upper():<11} | {ff_label:<18} | "
                f"Transient Morph Err: {m_t:5.3f}±{s_t:4.3f}m | "
                f"Steady-State Err: {m_s:5.3f}±{s_s:4.3f}m | "
                f"Final Err: {m_f:5.3f}±{s_f:4.3f}m"
            )
            results_table.append({
                "mode": mode,
                "ff": ff_flag,
                "transient": (m_t, s_t),
                "steady": (m_s, s_s),
                "final": (m_f, s_f),
                "time": sample_time,
                "error_curve": sample_error,
            })

    # Plot time-series error comparison showing transient vs steady state
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 5.2), dpi=180)

    # Left plot: Trajectory tracking error over time for Hybrid
    h_no_ff = [r for r in results_table if r["mode"] == "hybrid" and not r["ff"]][0]
    h_with_ff = [r for r in results_table if r["mode"] == "hybrid" and r["ff"]][0]

    ax1.plot(h_no_ff["time"], h_no_ff["error_curve"], color="#D32F2F", linewidth=2.0, label="Without Feedforward (Damping Lag)")
    ax1.plot(h_with_ff["time"], h_with_ff["error_curve"], color="#2E7D32", linewidth=2.2, label="With Feedforward (Drag & Vel Comp)")
    ax1.axhline(expected_lag_no_ff, color="#B71C1C", linestyle="--", alpha=0.8, label=f"Theoretical Steady Lag ({expected_lag_no_ff:.2f}m)")
    ax1.axvspan(5.0, 7.5, color="orange", alpha=0.15, label="Mid-Flight Morph Window (t = 5-7.5s)")
    ax1.axvspan(8.5, 12.0, color="green", alpha=0.10, label="Steady-State Settling (t = 8.5-12s)")
    ax1.set_xlabel("Simulation Time (s)", fontweight="bold")
    ax1.set_ylabel("Formation Tracking Error (m)", fontweight="bold")
    ax1.set_title("Transient vs Steady-State Error Evolution", fontweight="bold")
    ax1.grid(True, linestyle="--", alpha=0.5)
    ax1.legend(loc="upper right", fontsize=8.5)

    # Right plot: Bar chart comparing Transient Morph Error vs Steady-State Error
    labels = ["Hybrid (No FF)", "Hybrid (With FF)", "Central (No FF)", "Central (With FF)"]
    transients = [r["transient"][0] for r in results_table]
    steadies = [r["steady"][0] for r in results_table]
    x = np.arange(len(labels))
    width = 0.35

    rects1 = ax2.bar(x - width/2, transients, width, label="Transient Morph Err (m)", color="#FF9800", alpha=0.85)
    rects2 = ax2.bar(x + width/2, steadies, width, label="Steady-State Err (m)", color="#4CAF50", alpha=0.85)

    ax2.set_ylabel("Error (m)", fontweight="bold")
    ax2.set_title("Decomposition: Transient vs Steady Error", fontweight="bold")
    ax2.set_xticks(x)
    ax2.set_xticklabels(labels, rotation=15, ha="right", fontsize=9)
    ax2.grid(True, linestyle="--", alpha=0.4, axis="y")
    ax2.legend()

    # Add values on top of bars
    for rect in rects1:
        h = rect.get_height()
        ax2.annotate(f"{h:.2f}", xy=(rect.get_x() + rect.get_width() / 2, h), xytext=(0, 3), textcoords="offset points", ha="center", va="bottom", fontsize=8)
    for rect in rects2:
        h = rect.get_height()
        ax2.annotate(f"{h:.2f}", xy=(rect.get_x() + rect.get_width() / 2, h), xytext=(0, 3), textcoords="offset points", ha="center", va="bottom", fontsize=8)

    plt.tight_layout()
    plot_file = f"{output_dir}/feedforward_ablation_comparison.png"
    plt.savefig(plot_file)
    plt.close(fig)

    print(f"\nSaved feedforward ablation plots to {plot_file}.")


if __name__ == "__main__":
    main()
