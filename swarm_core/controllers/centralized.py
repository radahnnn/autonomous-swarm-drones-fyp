"""
Centralized Swarm Controller.
A global mission planner assigns optimal formation slots and commands trajectories.
"""

from typing import Dict, List, Optional
import numpy as np
from swarm_core.drone import Drone
from swarm_core.formations import FormationGenerator, FormationType, assign_optimal_slots


class CentralizedController:
    """Centralized coordinator computing global trajectories and slot assignments."""

    def __init__(
        self,
        kp: float = 1.8,
        kd: float = 2.2,
        k_repulse: float = 4.0,
        collision_dist: float = 1.0,
    ):
        self.kp = float(kp)
        self.kd = float(kd)
        self.k_repulse = float(k_repulse)
        self.collision_dist = float(collision_dist)

    def compute_control_inputs(
        self,
        drones: List[Drone],
        formation_type: FormationType,
        centroid_target: np.ndarray,
        centroid_velocity: Optional[np.ndarray] = None,
        spacing: float = 2.5,
        use_velocity_feedforward: bool = True,
        drag_coeff: float = 0.20,
        measured_positions: Optional[Dict[int, np.ndarray]] = None,
        measured_velocities: Optional[Dict[int, np.ndarray]] = None,
    ) -> np.ndarray:
        """
        Compute acceleration commands for all drones.
        Returns array of shape (N, 2) representing commanded accelerations.
        """
        num_drones = len(drones)
        if measured_positions is not None:
            current_pos = np.array([measured_positions[d.id] for d in drones])
        else:
            current_pos = np.array([d.position for d in drones])

        if measured_velocities is not None:
            current_vel = np.array([measured_velocities[d.id] for d in drones])
        else:
            current_vel = np.array([d.velocity for d in drones])

        if centroid_velocity is not None and use_velocity_feedforward:
            target_vel = np.array(centroid_velocity, dtype=np.float64)
        else:
            target_vel = np.zeros(2, dtype=np.float64)

        # 1. Compute nominal local offsets and translate to world centroid
        local_offsets = FormationGenerator.get_formation_offsets(
            formation_type, num_drones, spacing=spacing
        )
        world_slots = local_offsets + centroid_target

        # 2. Optimal slot assignment (minimizes path crossings)
        assigned_targets = assign_optimal_slots(current_pos, world_slots)

        # 3. Compute PD tracking forces
        pos_error = assigned_targets - current_pos
        vel_error = target_vel - current_vel
        accel_commands = self.kp * pos_error + self.kd * vel_error

        # Feedforward drag compensation if moving target tracking is enabled
        if use_velocity_feedforward and np.linalg.norm(target_vel) > 1e-6:
            accel_commands += drag_coeff * target_vel

        # 4. Centralized safety barrier (Artificial Potential Field collision avoidance)
        for i in range(num_drones):
            for j in range(i + 1, num_drones):
                diff = current_pos[i] - current_pos[j]
                dist = np.linalg.norm(diff)
                if dist < self.collision_dist and dist > 1e-4:
                    # Inverse distance repulsion force
                    repulse_magnitude = self.k_repulse * (1.0 / dist - 1.0 / self.collision_dist) / (dist**2)
                    direction = diff / dist
                    force = repulse_magnitude * direction
                    accel_commands[i] += force
                    accel_commands[j] -= force

        return accel_commands
