"""
Decentralized Swarm Controller.
Implements local consensus algorithms and Reynolds/Olfati-Saber flocking.
Requires no global coordinator; each drone relies strictly on 1-hop neighbor state broadcasts.
"""

from typing import Dict, List, Optional
import numpy as np
from swarm_core.drone import Drone


class DecentralizedController:
    """Distributed consensus and flocking controller operating on 1-hop neighbor graphs."""

    def __init__(
        self,
        k_sep: float = 4.5,       # Separation / repulsion gain
        k_align: float = 1.6,     # Velocity consensus gain
        k_form: float = 1.4,      # Formation cohesion gain
        k_goal: float = 0.8,      # Navigational goal gain
        safe_radius: float = 1.2, # Collision boundary radius
    ):
        self.k_sep = float(k_sep)
        self.k_align = float(k_align)
        self.k_form = float(k_form)
        self.k_goal = float(k_goal)
        self.safe_radius = float(safe_radius)

    def compute_drone_control(
        self,
        drone: Drone,
        neighbor_states: List[Dict[str, np.ndarray]],
        desired_offsets: Optional[Dict[int, np.ndarray]] = None,
        goal_pos: Optional[np.ndarray] = None,
        goal_vel: Optional[np.ndarray] = None,
    ) -> np.ndarray:
        """
        Compute control acceleration for a single drone given perceived neighbor data.
        
        Parameters:
            drone: The local drone instance.
            neighbor_states: List of dicts containing {'id': int, 'position': ndarray, 'velocity': ndarray}.
            desired_offsets: Dict mapping neighbor_id -> desired relative vector (p_drone - p_neighbor).
            goal_pos: Optional navigational waypoint.
            goal_vel: Optional velocity target.
        """
        accel = np.zeros(drone.dim, dtype=np.float64)

        if not neighbor_states:
            # Isolated drone - track goal directly if available
            if goal_pos is not None:
                accel += self.k_goal * (goal_pos - drone.position)
                if goal_vel is not None:
                    accel += 0.5 * self.k_goal * (goal_vel - drone.velocity)
            return accel

        f_sep = np.zeros(drone.dim, dtype=np.float64)
        f_align = np.zeros(drone.dim, dtype=np.float64)
        f_form = np.zeros(drone.dim, dtype=np.float64)

        for n_state in neighbor_states:
            n_id = n_state["id"]
            p_j = n_state["position"]
            v_j = n_state["velocity"]

            diff = drone.position - p_j
            dist = np.linalg.norm(diff)

            # 1. Separation force (strictly local collision avoidance)
            if 1e-4 < dist < self.safe_radius:
                repulse = self.k_sep * (1.0 / dist - 1.0 / self.safe_radius) / (dist**2)
                f_sep += repulse * (diff / dist)

            # 2. Velocity consensus (Laplacian alignment matching: dot{v}_i = - sum (v_i - v_j))
            f_align -= self.k_align * (drone.velocity - v_j)

            # 3. Formation cohesion / relative displacement consensus
            if desired_offsets is not None and n_id in desired_offsets:
                desired_delta = desired_offsets[n_id]  # p_i - p_j should equal desired_delta
                displacement_error = diff - desired_delta
                f_form -= self.k_form * displacement_error
            else:
                # Default flocking cohesion (gentle attraction to prevent swarm dispersion)
                if dist > self.safe_radius * 1.5:
                    f_form -= (self.k_form * 0.4) * (diff / dist) * (dist - self.safe_radius * 1.5)

        # 4. Optional navigational feedback
        f_goal = np.zeros(drone.dim, dtype=np.float64)
        if goal_pos is not None:
            f_goal += self.k_goal * (goal_pos - drone.position)
            if goal_vel is not None:
                f_goal += 0.5 * self.k_goal * (goal_vel - drone.velocity)

        total_accel = f_sep + f_align + f_form + f_goal
        return total_accel
