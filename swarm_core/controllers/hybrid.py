"""
Hybrid Swarm Controller.
Combines centralized mission guidance with onboard decentralized safety barriers and
autonomous fallback to local flocking upon communication loss.
"""

from enum import Enum
from typing import Dict, List, Optional
import numpy as np
from swarm_core.controllers.centralized import CentralizedController
from swarm_core.controllers.decentralized import DecentralizedController
from swarm_core.drone import Drone
from swarm_core.formations import FormationType


class HybridMode(Enum):
    CENTRALIZED = "centralized"
    DECENTRALIZED_FALLBACK = "decentralized_fallback"


class HybridController:
    """
    Hierarchical hybrid controller managing centralized trajectory following,
    local safety preservation, and link-loss degradation.
    """

    def __init__(
        self,
        heartbeat_timeout: float = 0.5,  # 500ms timeout for central link
        transition_smooth_time: float = 0.4,
    ):
        self.central_controller = CentralizedController()
        self.decentral_controller = DecentralizedController()
        self.heartbeat_timeout = float(heartbeat_timeout)
        self.transition_smooth_time = float(transition_smooth_time)

        # State tracking per drone: drone_id -> {'last_heartbeat': float, 'mode': HybridMode}
        self.drone_modes: Dict[int, HybridMode] = {}
        self.last_heartbeats: Dict[int, float] = {}

    def update_heartbeat(self, drone_id: int, current_time: float) -> None:
        """Register fresh central coordinator signal."""
        self.last_heartbeats[drone_id] = current_time
        self.drone_modes[drone_id] = HybridMode.CENTRALIZED

    def get_mode(self, drone_id: int, current_time: float) -> HybridMode:
        """Evaluate link health and return active operating regime."""
        last_hb = self.last_heartbeats.get(drone_id, -1.0)
        if last_hb < 0 or (current_time - last_hb) > self.heartbeat_timeout:
            mode = HybridMode.DECENTRALIZED_FALLBACK
        else:
            mode = HybridMode.CENTRALIZED
        self.drone_modes[drone_id] = mode
        return mode

    def compute_hybrid_control(
        self,
        drone: Drone,
        current_time: float,
        central_target_pos: Optional[np.ndarray],
        neighbor_states: List[Dict[str, np.ndarray]],
        desired_neighbor_offsets: Optional[Dict[int, np.ndarray]] = None,
    ) -> np.ndarray:
        """
        Compute control input for a drone taking into account link state and safety barriers.
        """
        mode = self.get_mode(drone.id, current_time)

        if mode == HybridMode.CENTRALIZED and central_target_pos is not None:
            # 1. Centralized tracking vector
            p_err = central_target_pos - drone.position
            v_err = -drone.velocity
            u_track = self.central_controller.kp * p_err + self.central_controller.kd * v_err

            # 2. Local decentralized safety barrier (APF collision avoidance active ALWAYS)
            u_safe = np.zeros(drone.dim, dtype=np.float64)
            for n_state in neighbor_states:
                diff = drone.position - n_state["position"]
                dist = np.linalg.norm(diff)
                if 1e-4 < dist < self.decentral_controller.safe_radius:
                    repulse = (
                        self.decentral_controller.k_sep
                        * (1.0 / dist - 1.0 / self.decentral_controller.safe_radius)
                        / (dist**2)
                    )
                    u_safe += repulse * (diff / dist)

            return u_track + u_safe

        else:
            # Degraded link: fall back to purely decentralized consensus & flocking
            return self.decentral_controller.compute_drone_control(
                drone=drone,
                neighbor_states=neighbor_states,
                desired_offsets=desired_neighbor_offsets,
                goal_pos=None,  # No central goal reachable
            )
