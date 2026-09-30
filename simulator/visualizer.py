"""
Swarm 2D Visualization and Animation Tool.
Renders real-time drone states, communication topologies, trajectory trails, and telemetry.
"""

from typing import List, Optional
import matplotlib
# Use Agg backend for reliable rendering in all environments
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np

from simulator.engine import SwarmSimulation


class SwarmVisualizer:
    """Renders 2D animated visualizations of multi-drone swarm simulations."""

    def __init__(self, sim: SwarmSimulation, xlim=(-10, 10), ylim=(-10, 10)):
        self.sim = sim
        self.xlim = xlim
        self.ylim = ylim

    def render_static_plot(self, save_path: str, title: str = "Swarm Simulation State") -> None:
        """Render a single high-resolution snapshot with trajectory trails and metrics."""
        fig, ax = plt.subplots(figsize=(9, 9), dpi=150)
        ax.set_xlim(self.xlim)
        ax.set_ylim(self.ylim)
        ax.set_aspect("equal")
        ax.grid(True, linestyle="--", alpha=0.5)

        # 1. Draw target formation slots
        if self.sim.target_slots is not None:
            ax.scatter(
                self.sim.target_slots[:, 0],
                self.sim.target_slots[:, 1],
                color="red",
                marker="x",
                s=100,
                linewidth=2,
                label="Target Slots",
                zorder=3,
            )

        # 2. Draw communication edges
        adj = self.sim.graph.compute_adjacency_matrix(self.sim.drones)
        for i in range(self.sim.num_drones):
            for j in range(i + 1, self.sim.num_drones):
                if adj[i, j] > 0:
                    p1 = self.sim.drones[i].position
                    p2 = self.sim.drones[j].position
                    ax.plot(
                        [p1[0], p2[0]],
                        [p1[1], p2[1]],
                        color="gray",
                        linestyle=":",
                        alpha=0.6,
                        zorder=2,
                    )

        # 3. Draw trajectories and drones
        colors = plt.cm.tab10(np.linspace(0, 1, self.sim.num_drones))
        for i, drone in enumerate(self.sim.drones):
            traj = np.array(drone.trajectory)
            if len(traj) > 1:
                ax.plot(traj[:, 0], traj[:, 1], color=colors[i], alpha=0.6, linewidth=1.5)

            # Drone circle
            circle = plt.Circle(
                (drone.position[0], drone.position[1]),
                drone.radius,
                color=colors[i],
                alpha=0.85,
                zorder=4,
            )
            ax.add_patch(circle)
            ax.text(
                drone.position[0],
                drone.position[1],
                str(drone.id),
                color="white",
                fontweight="bold",
                ha="center",
                va="center",
                zorder=5,
            )

            # Velocity vector
            if np.linalg.norm(drone.velocity) > 0.05:
                ax.arrow(
                    drone.position[0],
                    drone.position[1],
                    drone.velocity[0] * 0.4,
                    drone.velocity[1] * 0.4,
                    head_width=0.2,
                    head_length=0.2,
                    fc="black",
                    ec="black",
                    zorder=6,
                )

        # Telemetry HUD
        last_snap = self.sim.metrics.history[-1] if self.sim.metrics.history else None
        err_str = f"{last_snap.formation_error:.2f} m" if last_snap else "N/A"
        min_d_str = f"{last_snap.min_inter_drone_dist:.2f} m" if last_snap else "N/A"
        
        info_text = (
            f"Time: {self.sim.current_time:.1f} s | Mode: {self.sim.control_mode.upper()}\n"
            f"Formation: {self.sim.current_formation.value.upper()}\n"
            f"Formation Error: {err_str} | Min Distance: {min_d_str}\n"
            f"Drones: {self.sim.num_drones} | Loss Rate: {self.sim.channel.packet_loss_rate*100:.0f}%"
        )
        ax.text(
            0.02,
            0.98,
            info_text,
            transform=ax.transAxes,
            verticalalignment="top",
            bbox=dict(boxstyle="round", facecolor="wheat", alpha=0.8),
            fontsize=10,
        )

        ax.set_title(title, fontsize=14, fontweight="bold")
        ax.set_xlabel("X Position (meters)")
        ax.set_ylabel("Y Position (meters)")
        ax.legend(loc="upper right")

        plt.tight_layout()
        plt.savefig(save_path)
        plt.close(fig)

    def generate_animation(
        self,
        duration_s: float,
        formation_schedule: List[tuple],  # [(time, FormationType, (cx, cy))]
        save_path: str,
        fps: int = 20,
    ) -> None:
        """
        Run the simulation and export a complete video/GIF animation.
        """
        total_frames = int(duration_s / self.sim.dt)
        subsample = max(1, int(1.0 / (fps * self.sim.dt)))
        recorded_frames = []

        # Run simulation and store states
        sched_idx = 0
        for step_i in range(total_frames):
            t = self.sim.current_time
            if sched_idx < len(formation_schedule) and t >= formation_schedule[sched_idx][0]:
                _, form_type, centroid = formation_schedule[sched_idx]
                self.sim.set_formation(form_type, centroid=centroid)
                sched_idx += 1

            self.sim.step()

            if step_i % subsample == 0:
                # Capture frame data
                frame_data = {
                    "time": t,
                    "positions": np.array([d.position.copy() for d in self.sim.drones]),
                    "velocities": np.array([d.velocity.copy() for d in self.sim.drones]),
                    "targets": self.sim.target_slots.copy() if self.sim.target_slots is not None else None,
                    "adj": self.sim.graph.compute_adjacency_matrix(self.sim.drones),
                    "formation": self.sim.current_formation.value,
                    "mode": self.sim.control_mode,
                    "error": self.sim.metrics.history[-1].formation_error,
                    "min_dist": self.sim.metrics.history[-1].min_inter_drone_dist,
                }
                recorded_frames.append(frame_data)

        # Render frames to animation
        fig, ax = plt.subplots(figsize=(8, 8), dpi=120)

        def update(frame_idx):
            ax.clear()
            ax.set_xlim(self.xlim)
            ax.set_ylim(self.ylim)
            ax.set_aspect("equal")
            ax.grid(True, linestyle="--", alpha=0.5)

            data = recorded_frames[frame_idx]
            pos = data["positions"]
            vel = data["velocities"]
            targets = data["targets"]
            adj = data["adj"]

            # Draw target slots
            if targets is not None:
                ax.scatter(targets[:, 0], targets[:, 1], color="red", marker="x", s=80, zorder=3)

            # Draw links
            for i in range(self.sim.num_drones):
                for j in range(i + 1, self.sim.num_drones):
                    if adj[i, j] > 0:
                        ax.plot(
                            [pos[i, 0], pos[j, 0]],
                            [pos[i, 1], pos[j, 1]],
                            color="gray",
                            linestyle=":",
                            alpha=0.5,
                            zorder=2,
                        )

            # Draw drones
            colors = plt.cm.tab10(np.linspace(0, 1, self.sim.num_drones))
            for i in range(self.sim.num_drones):
                circle = plt.Circle(
                    (pos[i, 0], pos[i, 1]),
                    self.sim.drones[i].radius,
                    color=colors[i],
                    alpha=0.9,
                    zorder=4,
                )
                ax.add_patch(circle)
                ax.text(
                    pos[i, 0],
                    pos[i, 1],
                    str(self.sim.drones[i].id),
                    color="white",
                    fontweight="bold",
                    ha="center",
                    va="center",
                    zorder=5,
                )

            # HUD
            info_text = (
                f"Time: {data['time']:.1f} s | Mode: {data['mode'].upper()}\n"
                f"Formation: {data['formation'].upper()}\n"
                f"Tracking Error: {data['error']:.2f} m | Min Dist: {data['min_dist']:.2f} m"
            )
            ax.text(
                0.02,
                0.98,
                info_text,
                transform=ax.transAxes,
                verticalalignment="top",
                bbox=dict(boxstyle="round", facecolor="wheat", alpha=0.8),
                fontsize=9,
            )
            ax.set_title("Autonomous Swarm Formation Control", fontsize=12, fontweight="bold")
            return ax

        ani = animation.FuncAnimation(
            fig, update, frames=len(recorded_frames), interval=1000 // fps, blit=False
        )
        
        # Save as animated GIF or MP4 depending on extension
        if save_path.endswith(".gif"):
            ani.save(save_path, writer="pillow", fps=fps)
        else:
            ani.save(save_path, writer="ffmpeg", fps=fps)
        plt.close(fig)
