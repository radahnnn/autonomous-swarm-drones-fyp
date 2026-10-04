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
    shape_error: float = 0.0
    total_mode_switches: int = 0
    mean_alpha: float = 1.0


class SwarmMetricsTracker:
    """
    Collects and aggregates performance data over the course of a simulation run.
    Evaluates physical ground-truth inter-drone distances, formation tracking errors,
    centroid-removed shape errors, and link vs formation recovery dynamics.
    """

    def __init__(self, collision_threshold: Optional[float] = None):
        if collision_threshold is None:
            try:
                from swarm_core.config import get_active_profile
                self.collision_threshold = float(get_active_profile().get("collision_threshold", 0.70))
            except Exception:
                self.collision_threshold = 0.70
        else:
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

        # 1. Formation tracking error (absolute) and shape error (centroid- & common-mode-removed) (Item 29)
        if target_positions is not None and len(target_positions) == n:
            sq_errors = np.sum((positions - target_positions) ** 2, axis=1)
            formation_error = float(np.sqrt(np.mean(sq_errors)))

            # Centroid-removed formation shape error:
            actual_centroid = np.mean(positions, axis=0)
            target_centroid = np.mean(target_positions, axis=0)
            centered_actual = positions - actual_centroid
            centered_target = target_positions - target_centroid
            shape_sq_errors = np.sum((centered_actual - centered_target) ** 2, axis=1)
            shape_error = float(np.sqrt(np.mean(shape_sq_errors)))
        else:
            formation_error = 0.0
            shape_error = 0.0

        # 2. True physical inter-drone distances & collision verification (Item 29)
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
            shape_error=shape_error,
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
        shape_errors = [s.shape_error for s in self.history]
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

        # Transient window during mid-flight morph (t in [5.0, 7.5s])
        morph_errors = [s.formation_error for s in self.history if 5.0 <= s.time <= 7.5]
        transient_morph_error = float(np.mean(morph_errors)) if morph_errors else float(errors[-1])
        morph_shape_errors = [s.shape_error for s in self.history if 5.0 <= s.time <= 7.5]
        transient_morph_shape = float(np.mean(morph_shape_errors)) if morph_shape_errors else float(shape_errors[-1])

        # Settled steady-state window post-morph (t >= 8.5s)
        steady_errors = [s.formation_error for s in self.history if s.time >= 8.5]
        steady_state_error = float(np.mean(steady_errors)) if steady_errors else float(errors[-1])
        steady_shape_errors = [s.shape_error for s in self.history if s.time >= 8.5]
        steady_state_shape = float(np.mean(steady_shape_errors)) if steady_shape_errors else float(shape_errors[-1])

        return {
            "initial_formation_error_m": float(errors[0]),
            "final_formation_error_m": float(errors[-1]),
            "initial_shape_error_m": float(shape_errors[0]),
            "final_shape_error_m": float(shape_errors[-1]),
            "mean_formation_error_m": float(np.mean(errors)),
            "mean_shape_error_m": float(np.mean(shape_errors)),
            "min_recorded_distance_m": float(min(min_dists)),
            "min_separation_m": float(min(min_dists)),
            "any_collision": float(any(collisions)),
            "convergence_time_s": float(conv_time if conv_time >= 0 else times[-1]),
            "transient_morph_error_m": transient_morph_error,
            "transient_morph_shape_error_m": transient_morph_shape,
            "steady_state_error_m": steady_state_error,
            "steady_state_shape_error_m": steady_state_shape,
            "total_mode_switches": float(switches[-1] if switches else 0.0),
        }

    def calculate_formation_recovery_time(
        self,
        disturbance_end_time: float,
        convergence_tol: float = 0.25,
    ) -> float:
        """
        Calculates formation recovery duration: elapsed time from when an RF disturbance
        (outage / burst) ceases until tracking error settles and stays below convergence_tol.
        Returns 0.0 if already converged, or elapsed seconds, or -1.0 if never converged.
        """
        post_snapshots = [(s.time, s.formation_error) for s in self.history if s.time >= disturbance_end_time]
        if not post_snapshots:
            return 0.0

        times = [t for t, _ in post_snapshots]
        errors = [err for _, err in post_snapshots]

        for i, err in enumerate(errors):
            if all(e <= convergence_tol for e in errors[i:]):
                return float(times[i] - disturbance_end_time)
        return -1.0
