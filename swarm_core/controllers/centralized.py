"""
Centralized Swarm Controller.
A global mission planner assigns optimal formation slots and commands trajectories.
"""

from typing import List, Optional
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
    ) -> np.ndarray:
        """
        Compute acceleration commands for all drones.
        Returns array of shape (N, 2) representing commanded accelerations.
        """
        num_drones = len(drones)
        current_pos = np.array([d.position for d in drones])
        current_vel = np.array([d.velocity for d in drones])

        if centroid_velocity is None:
            target_vel = np.zeros(2)
        else:
            target_vel = np.array(centroid_velocity, dtype=np.float64)

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
