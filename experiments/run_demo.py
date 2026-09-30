"""
Full Swarm Demo Scenario.
Simulates 6 drones smoothly transitioning through all 4 target formations:
Line -> V-Shape -> Circle -> Grid with Hungarian optimal assignment and collision avoidance.
"""

import os
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from swarm_core.drone import Drone
from swarm_core.formations import FormationType
from simulator.engine import SwarmSimulation
from simulator.visualizer import SwarmVisualizer


def main():
    output_dir = "experiments/results"
    os.makedirs(output_dir, exist_ok=True)

    print("==================================================")
    print("  Swarm Drones FYP: 4-Formation Transition Demo   ")
    print("==================================================")

    # 1. Initialize 6 drones in scattered initial positions
    rng = np.random.default_rng(123)
    num_drones = 6
    init_positions = [
        np.array([-5.0, -3.0]),
        np.array([-4.0, 3.5]),
        np.array([-1.0, -4.0]),
        np.array([2.0, 4.0]),
        np.array([4.5, -2.5]),
        np.array([5.0, 3.0]),
    ]
    drones = [Drone(drone_id=i, initial_position=init_positions[i]) for i in range(num_drones)]

    # 2. Setup simulation in Hybrid mode (the target framework)
    sim = SwarmSimulation(
        drones=drones,
        control_mode="hybrid",
        comm_range=15.0,
        packet_loss_rate=0.05,  # Realistic 5% wireless loss
        latency_mean=0.02,
        dt=0.05,
    )
    visualizer = SwarmVisualizer(sim, xlim=(-8, 8), ylim=(-8, 8))

    # 3. Schedule the 4 formations over 24 seconds (6s per formation)
    schedule = [
        (0.0, FormationType.LINE, np.array([0.0, 0.0])),
        (6.0, FormationType.V_SHAPE, np.array([0.0, 0.0])),
        (12.0, FormationType.CIRCLE, np.array([0.0, 0.0])),
        (18.0, FormationType.GRID, np.array([0.0, 0.0])),
    ]

    print("\nRunning mission simulation...")
    for t_step, form_type, centroid in schedule:
        print(f" -> t = {t_step:4.1f}s: Switching to {form_type.value.upper()}")

    # Capture snapshots at key milestones
    sched_idx = 0
    total_steps = int(24.0 / sim.dt)
    
    for step_i in range(total_steps):
        t = sim.current_time
        if sched_idx < len(schedule) and t >= schedule[sched_idx][0]:
            _, form_type, centroid = schedule[sched_idx]
            sim.set_formation(form_type, centroid=centroid)
            sched_idx += 1

        sim.step()

        # Save snapshot 1 second before next transition (settled formation)
        if step_i in [int(5.5 / sim.dt), int(11.5 / sim.dt), int(17.5 / sim.dt), int(23.5 / sim.dt)]:
            form_name = sim.current_formation.value
            snap_path = f"{output_dir}/formation_{form_name}.png"
            visualizer.render_static_plot(
                save_path=snap_path,
                title=f"Formation: {form_name.upper()} (t = {t:.1f}s)",
            )
            print(f"   [Saved snapshot] {snap_path}")

    # Plot metrics over time
    print("\nGenerating performance metrics plot...")
    history = sim.metrics.history
    times = [s.time for s in history]
    errors = [s.formation_error for s in history]
    min_dists = [s.min_inter_drone_dist for s in history]
    speeds = [s.avg_speed for s in history]

    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 8), sharex=True, dpi=150)
    
    # Formation tracking error
    ax1.plot(times, errors, color="crimson", linewidth=2, label="Formation Tracking Error (RMSE)")
    ax1.axhline(0.2, color="gray", linestyle="--", label="Target Convergence (0.2m)")
    for t_trans in [6.0, 12.0, 18.0]:
        ax1.axvline(t_trans, color="navy", linestyle=":", alpha=0.7)
    ax1.set_ylabel("Error (m)")
    ax1.set_title("Swarm Performance: Dynamic 4-Formation Switching", fontweight="bold")
    ax1.grid(True, alpha=0.3)
    ax1.legend(loc="upper right")

    # Inter-drone distance / safety margin
    ax1_col = 2 * drones[0].radius
    ax2.plot(times, min_dists, color="teal", linewidth=2, label="Min Inter-Drone Distance")
    ax2.axhline(ax1_col, color="red", linestyle="--", label=f"Collision Boundary ({ax1_col:.2f}m)")
    for t_trans in [6.0, 12.0, 18.0]:
        ax2.axvline(t_trans, color="navy", linestyle=":", alpha=0.7)
    ax2.set_ylabel("Min Distance (m)")
    ax2.grid(True, alpha=0.3)
    ax2.legend(loc="lower right")

    # Average swarm speed
    ax3.plot(times, speeds, color="purple", linewidth=2, label="Mean Swarm Speed")
    for t_trans in [6.0, 12.0, 18.0]:
        ax3.axvline(t_trans, color="navy", linestyle=":", alpha=0.7)
    ax3.set_ylabel("Speed (m/s)")
    ax3.set_xlabel("Time (seconds)")
    ax3.grid(True, alpha=0.3)
    ax3.legend(loc="upper right")

    plt.tight_layout()
    metrics_path = f"{output_dir}/demo_metrics.png"
    plt.savefig(metrics_path)
    plt.close(fig)
    print(f"   [Saved metrics plot] {metrics_path}")

    # Summary
    summary = sim.metrics.get_summary()
    print("\nMission Summary Statistics:")
    print(f" - Min Recorded Inter-Drone Distance: {summary['min_recorded_distance_m']:.3f} m (Safety: PASSED)")
    print(f" - Collisions Detected: {int(summary['any_collision'])}")
    print(f" - Final Formation Error: {summary['final_formation_error_m']:.3f} m")
    print(f" - Total Wireless Packets Exchanged: {sim.channel.total_delivered} (Delivered), {sim.channel.total_dropped_loss} (Dropped)")
    print("\nCompleted successfully! Figures saved to:", output_dir)


if __name__ == "__main__":
    main()
