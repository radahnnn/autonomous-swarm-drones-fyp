"""
Swarm Simulation Engine.
Coordinates the physics step, wireless network exchange, control computation, and metrics tracking.
"""

from typing import Dict, List, Optional
import numpy as np

from swarm_core.drone import Drone
from swarm_core.graph import SwarmGraph
from swarm_core.network import WirelessChannel
from swarm_core.formations import FormationGenerator, FormationType, assign_optimal_slots
from swarm_core.controllers.centralized import CentralizedController
from swarm_core.controllers.decentralized import DecentralizedController
from swarm_core.controllers.hybrid import HybridController, HybridMode
from swarm_core.metrics import SwarmMetricsTracker


class SwarmSimulation:
    """Manages multi-drone execution loop with network and physics simulation."""

    def __init__(
        self,
        drones: List[Drone],
        control_mode: str = "centralized",  # "centralized", "decentralized", or "hybrid"
        comm_range: float = 12.0,
        packet_loss_rate: float = 0.0,
        latency_mean: float = 0.02,
        dt: float = 0.05,
    ):
        self.drones = drones
        self.num_drones = len(drones)
        self.control_mode = control_mode.lower()
        self.dt = float(dt)
        self.current_time = 0.0

        # Subsystems
        self.graph = SwarmGraph(comm_radius=comm_range)
        self.channel = WirelessChannel(
            comm_range=comm_range,
            packet_loss_rate=packet_loss_rate,
            latency_mean=latency_mean,
        )
        self.metrics = SwarmMetricsTracker(collision_threshold=drones[0].radius * 2.0)

        # Controllers
        self.central_ctrl = CentralizedController()
        self.decentral_ctrl = DecentralizedController()
        self.hybrid_ctrl = HybridController()

        # Mission state
        self.current_formation = FormationType.LINE
        self.formation_spacing = 2.5
        self.centroid_target = np.array([0.0, 0.0], dtype=np.float64)
        self.centroid_velocity = np.array([0.0, 0.0], dtype=np.float64)
        self.target_slots: Optional[np.ndarray] = None

        # Link control (for hybrid failure testing: if False, central coordinator is severed)
        self.coordinator_link_active = True

    def set_formation(
        self,
        formation: FormationType,
        centroid: Optional[np.ndarray] = None,
        spacing: float = 2.5,
    ) -> None:
        """Update target formation geometry and centroid."""
        self.current_formation = formation
        self.formation_spacing = float(spacing)
        if centroid is not None:
            self.centroid_target = np.array(centroid, dtype=np.float64)

    def set_coordinator_link(self, active: bool) -> None:
        """Simulate ground station link cut or reconnection."""
        self.coordinator_link_active = active

    def step(self) -> None:
        """Advance simulation by one timestep dt."""
        n = self.num_drones

        # 1. Update topology graph
        adj_matrix = self.graph.compute_adjacency_matrix(self.drones)
        laplacian = self.graph.compute_laplacian_matrix(adj_matrix)
        fiedler = self.graph.algebraic_connectivity(laplacian)

        # 2. Inter-drone wireless broadcast (Decentralized state exchange)
        # Each drone broadcasts its position & velocity to nearby peers
        for i in range(n):
            for j in range(n):
                if i != j:
                    dist = self.drones[i].distance_to(self.drones[j])
                    payload = {
                        "position": self.drones[i].position.copy(),
                        "velocity": self.drones[i].velocity.copy(),
                    }
                    self.channel.send(
                        sender_id=self.drones[i].id,
                        recipient_id=self.drones[j].id,
                        payload=payload,
                        distance=dist,
                        current_time=self.current_time,
                    )

        # 3. Retrieve arrived packets per drone
        perceived_neighbors: Dict[int, List[Dict]] = {d.id: [] for d in self.drones}
        for d in self.drones:
            pkts = self.channel.receive(d.id, self.current_time)
            for pkt in pkts:
                perceived_neighbors[d.id].append({
                    "id": pkt.sender_id,
                    "position": pkt.payload["position"],
                    "velocity": pkt.payload["velocity"],
                })

        # 4. Target slot generation and optimal Hungarian matching
        local_offsets = FormationGenerator.get_formation_offsets(
            self.current_formation, n, spacing=self.formation_spacing
        )
        world_slots = local_offsets + self.centroid_target
        current_positions = np.array([d.position for d in self.drones])
        assigned_targets = assign_optimal_slots(current_positions, world_slots)
        self.target_slots = assigned_targets

        # 5. Compute Control Inputs based on selected mode
        if self.control_mode == "centralized":
            accels = self.central_ctrl.compute_control_inputs(
                self.drones,
                self.current_formation,
                self.centroid_target,
                self.centroid_velocity,
                spacing=self.formation_spacing,
            )
            for i, d in enumerate(self.drones):
                d.set_control_input(accels[i])

        elif self.control_mode == "decentralized":
            # Pure local consensus & flocking towards target slots
            for i, d in enumerate(self.drones):
                # Desired local offsets relative to perceived neighbors
                target_i = assigned_targets[i]
                desired_offsets = {}
                for n_info in perceived_neighbors[d.id]:
                    # Find nominal offset relative to neighbor j
                    n_idx = [idx for idx, other in enumerate(self.drones) if other.id == n_info["id"]][0]
                    desired_offsets[n_info["id"]] = target_i - assigned_targets[n_idx]

                accel_i = self.decentral_ctrl.compute_drone_control(
                    drone=d,
                    neighbor_states=perceived_neighbors[d.id],
                    desired_offsets=desired_offsets,
                    goal_pos=target_i,
                    goal_vel=self.centroid_velocity,
                )
                d.set_control_input(accel_i)

        elif self.control_mode == "hybrid":
            for i, d in enumerate(self.drones):
                if self.coordinator_link_active:
                    self.hybrid_ctrl.update_heartbeat(d.id, self.current_time)

                target_i = assigned_targets[i]
                desired_offsets = {}
                for n_info in perceived_neighbors[d.id]:
                    n_idx = [idx for idx, other in enumerate(self.drones) if other.id == n_info["id"]][0]
                    desired_offsets[n_info["id"]] = target_i - assigned_targets[n_idx]

                accel_i = self.hybrid_ctrl.compute_hybrid_control(
                    drone=d,
                    current_time=self.current_time,
                    central_target_pos=target_i if self.coordinator_link_active else None,
                    neighbor_states=perceived_neighbors[d.id],
                    desired_neighbor_offsets=desired_offsets,
                )
                d.set_control_input(accel_i)

        # 6. Physical integration step for each drone
        for d in self.drones:
            d.step(self.dt)

        # 7. Record metrics snapshot
        self.metrics.record_step(
            current_time=self.current_time,
            drones=self.drones,
            target_positions=assigned_targets,
            adj_matrix=adj_matrix,
            fiedler_val=fiedler,
        )

        self.current_time += self.dt
