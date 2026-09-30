"""
Swarm Evaluation Metrics and Logging.
Tracks formation tracking error, convergence duration, safety margins, and velocity consensus.
"""

from dataclasses import dataclass, field
from typing import Dict, List, Optional
import numpy as np
from swarm_core.drone import Drone


@dataclass
class SwarmMetricsSnapshot:
    time: float
    formation_error: float
    min_inter_drone_dist: float
    avg_speed: float
    velocity_variance: float
    collision_occurred: bool
    active_connections: int
    algebraic_connectivity: float
    total_mode_switches: int = 0
    mean_alpha: float = 1.0


class SwarmMetricsTracker:
    """Collects and aggregates performance data over the course of a simulation run."""

    def __init__(self, collision_threshold: float = 0.7):
        self.collision_threshold = float(collision_threshold)
        self.history: List[SwarmMetricsSnapshot] = []

    def record_step(
        self,
        current_time: float,
        drones: List[Drone],
        target_positions: Optional[np.ndarray] = None,
        adj_matrix: Optional[np.ndarray] = None,
        fiedler_val: float = 0.0,
        total_mode_switches: int = 0,
        mean_alpha: float = 1.0,
    ) -> SwarmMetricsSnapshot:
        """Compute metrics for the current timestep."""
        n = len(drones)
        positions = np.array([d.position for d in drones])
        velocities = np.array([d.velocity for d in drones])

        # 1. Formation tracking error (RMS deviation from assigned targets)
        if target_positions is not None and len(target_positions) == n:
            sq_errors = np.sum((positions - target_positions) ** 2, axis=1)
            formation_error = float(np.sqrt(np.mean(sq_errors)))
        else:
            formation_error = 0.0

        # 2. Inter-drone distances & collision verification
        min_dist = float("inf")
        collision = False
        for i in range(n):
            for j in range(i + 1, n):
                dist = float(np.linalg.norm(positions[i] - positions[j]))
                if dist < min_dist:
                    min_dist = dist
                if dist < self.collision_threshold:
                    collision = True

        if min_dist == float("inf"):
            min_dist = 0.0

        # 3. Velocity consensus metrics
        speeds = np.linalg.norm(velocities, axis=1)
        avg_speed = float(np.mean(speeds))
        mean_vel = np.mean(velocities, axis=0)
        vel_variance = float(np.mean(np.sum((velocities - mean_vel) ** 2, axis=1)))

        # 4. Network topology
        active_conns = int(np.sum(adj_matrix) // 2) if adj_matrix is not None else 0

        snapshot = SwarmMetricsSnapshot(
            time=current_time,
            formation_error=formation_error,
            min_inter_drone_dist=min_dist,
            avg_speed=avg_speed,
            velocity_variance=vel_variance,
            collision_occurred=collision,
            active_connections=active_conns,
            algebraic_connectivity=fiedler_val,
            total_mode_switches=total_mode_switches,
            mean_alpha=mean_alpha,
        )
        self.history.append(snapshot)
        return snapshot

    def get_summary(self, convergence_tol: float = 0.25) -> Dict[str, float]:
        """Compute summary statistics for final reporting."""
        if not self.history:
            return {}

        errors = [s.formation_error for s in self.history]
        min_dists = [s.min_inter_drone_dist for s in self.history]
        collisions = [s.collision_occurred for s in self.history]
        times = [s.time for s in self.history]
        switches = [s.total_mode_switches for s in self.history]

        # Calculate convergence time (first time error enters and stays below tol)
        conv_time = -1.0
        for i, err in enumerate(errors):
            if all(e <= convergence_tol for e in errors[i:]):
                conv_time = times[i]
                break

        return {
            "initial_formation_error_m": float(errors[0]),
            "final_formation_error_m": float(errors[-1]),
            "min_recorded_distance_m": float(min(min_dists)),
            "any_collision": float(any(collisions)),
            "convergence_time_s": float(conv_time if conv_time >= 0 else times[-1]),
            "mean_formation_error_m": float(np.mean(errors)),
            "total_mode_switches": float(switches[-1] if switches else 0.0),
        }
