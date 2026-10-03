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
        measured_position: Optional[np.ndarray] = None,
        measured_velocity: Optional[np.ndarray] = None,
        use_velocity_feedforward: bool = True,
        drag_coeff: float = 0.20,
    ) -> np.ndarray:
        """
        Compute control acceleration for a single drone given perceived neighbor data.
        Degree-normalised across perceived neighbors (Olfati-Saber flocking consensus).
        
        Parameters:
            drone: The local drone instance.
            neighbor_states: List of dicts containing {'id': int, 'position': ndarray, 'velocity': ndarray}.
            desired_offsets: Dict mapping neighbor_id -> desired relative vector (p_drone - p_neighbor).
            goal_pos: Optional navigational waypoint.
            goal_vel: Optional velocity target.
            measured_position: Optional measured/estimated local position (defaults to drone.position).
            measured_velocity: Optional measured/estimated local velocity (defaults to drone.velocity).
            use_velocity_feedforward: Whether to apply drag feedforward compensation along goal_vel.
            drag_coeff: Active aerodynamic drag coefficient.
        """
        pos = np.array(measured_position, dtype=np.float64) if measured_position is not None else drone.position
        vel = np.array(measured_velocity, dtype=np.float64) if measured_velocity is not None else drone.velocity
        accel = np.zeros(drone.dim, dtype=np.float64)

        if not neighbor_states:
            # Isolated drone - track goal directly if available
            if goal_pos is not None:
                accel += self.k_goal * (goal_pos - pos)
                if goal_vel is not None:
                    accel += 0.5 * self.k_goal * (goal_vel - vel)
                    if use_velocity_feedforward and drag_coeff > 0:
                        accel += drag_coeff * goal_vel
            elif goal_vel is not None:
                accel += 0.5 * self.k_goal * (goal_vel - vel)
                if use_velocity_feedforward and drag_coeff > 0:
                    accel += drag_coeff * goal_vel
            else:
                # Bounded failsafe: actively brake to hover when isolated without an active goal
                accel -= self.k_align * vel
            return accel

        deg = max(1, len(neighbor_states))
        f_sep = np.zeros(drone.dim, dtype=np.float64)
        f_align = np.zeros(drone.dim, dtype=np.float64)
        f_form = np.zeros(drone.dim, dtype=np.float64)

        for n_state in neighbor_states:
            n_id = n_state["id"]
            p_j = n_state["position"]
            v_j = n_state["velocity"]

            diff = pos - p_j
            dist = np.linalg.norm(diff)

            # 1. Separation force (strictly local collision avoidance)
            if 1e-4 < dist < self.safe_radius:
                repulse = self.k_sep * (1.0 / dist - 1.0 / self.safe_radius) / (dist**2)
                f_sep += repulse * (diff / dist)

            # 2. Velocity consensus (Laplacian alignment matching: dot{v}_i = - sum (v_i - v_j))
            f_align -= self.k_align * (vel - v_j)

            # 3. Formation cohesion / relative displacement consensus
            if desired_offsets is not None and n_id in desired_offsets:
                desired_delta = desired_offsets[n_id]  # p_i - p_j should equal desired_delta
                displacement_error = diff - desired_delta
                f_form -= self.k_form * displacement_error
            else:
                # Default flocking cohesion (gentle attraction to prevent swarm dispersion)
                if dist > self.safe_radius * 1.5:
                    f_form -= (self.k_form * 0.4) * (diff / dist) * (dist - self.safe_radius * 1.5)

        # Degree normalisation across all perceived neighbors
        f_sep = f_sep / deg
        f_align = f_align / deg
        f_form = f_form / deg

        # 4. Optional navigational feedback & feedforward drag compensation
        f_goal = np.zeros(drone.dim, dtype=np.float64)
        if goal_pos is not None:
            f_goal += self.k_goal * (goal_pos - pos)
            if goal_vel is not None:
                f_goal += 0.5 * self.k_goal * (goal_vel - vel)
                if use_velocity_feedforward and drag_coeff > 0:
                    f_goal += drag_coeff * goal_vel
        elif goal_vel is not None:
            f_goal += 0.5 * self.k_goal * (goal_vel - vel)
            if use_velocity_feedforward and drag_coeff > 0:
                f_goal += drag_coeff * goal_vel

        total_accel = f_sep + f_align + f_form + f_goal
        return total_accel
