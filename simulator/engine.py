"""
Swarm Simulation Engine.
Coordinates the physics step, wireless network exchange, control computation, and metrics tracking.
"""

from typing import Any, Dict, List, Optional, Union
import numpy as np
from scipy.optimize import linear_sum_assignment

from swarm_core.config import (
    SwarmConfigProfile,
    build_controllers_from_profile,
    get_active_profile,
    get_profile,
)
from swarm_core.drone import Drone
from swarm_core.graph import SwarmGraph
from swarm_core.network import WirelessChannel
from swarm_core.formations import (
    FormationGenerator,
    FormationType,
    assign_optimal_slots,
    compute_desired_neighbor_offsets,
    compute_formation_slots,
    create_world_slots,
)
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
        comm_range: Optional[float] = None,
        packet_loss_rate: Optional[float] = None,
        latency_mean: Optional[float] = None,
        dt: float = 0.05,
        use_velocity_feedforward: bool = True,
        gps_noise_std: Optional[float] = None,
        gps_common_mode_fraction: Optional[float] = None,
        seed: Optional[int] = None,
        use_gilbert_elliott: bool = False,
        p_g_to_b: float = 0.05,
        p_b_to_g: float = 0.20,
        profile: Optional[Union[str, SwarmConfigProfile]] = None,
        fallback_strategy: str = "consensus",
        dead_reckon_duration: float = 1.5,
    ):
        self.drones = drones
        self.num_drones = len(drones)
        self.control_mode = control_mode.lower()
        self.dt = float(dt)
        self.current_time = 0.0
        self.rng = np.random.default_rng(seed)
        self.use_velocity_feedforward = bool(use_velocity_feedforward)
        self.fallback_strategy = str(fallback_strategy).lower()
        self.dead_reckon_duration = float(dead_reckon_duration)

        # Profile resolution: explicit arg > drone profile > active global profile
        if profile is not None:
            self.profile = get_profile(profile) if isinstance(profile, str) else profile
        elif drones and hasattr(drones[0], "profile_name"):
            self.profile = get_profile(drones[0].profile_name)
        else:
            self.profile = get_active_profile()
        self.profile_name = self.profile.name

        # Resolve parameters from profile if not explicitly passed
        self.comm_range = float(self.profile.get("comm_radius") if comm_range is None else comm_range)
        self.packet_loss_rate = float(self.profile.get("packet_loss_rate") if packet_loss_rate is None else packet_loss_rate)
        self.latency_mean = float(self.profile.get("latency_mean") if latency_mean is None else latency_mean)
        self.gps_noise_std = float(self.profile.get("gps_noise_std") if gps_noise_std is None else gps_noise_std)
        self.gps_common_mode_fraction = float(self.profile.get("gps_common_mode_fraction") if gps_common_mode_fraction is None else gps_common_mode_fraction)
        self.gps_corr_time = float(self.profile.get("gps_corr_time", 30.0))
        self.gps_vel_noise_std = float(self.profile.get("gps_vel_noise_std", 0.08)) if self.gps_noise_std > 0 else 0.0

        dim = self.drones[0].dim if self.drones else 2
        self.gps_noise_common = np.zeros(dim, dtype=np.float64)
        self.gps_noise_indep = {d.id: np.zeros(d.dim, dtype=np.float64) for d in self.drones}

        # Subsystems
        self.graph = SwarmGraph(comm_radius=self.comm_range)
        self.channel = WirelessChannel(
            comm_range=self.comm_range,
            packet_loss_rate=self.packet_loss_rate,
            latency_mean=self.latency_mean,
            seed=seed,
            use_gilbert_elliott=use_gilbert_elliott,
            p_g_to_b=p_g_to_b,
            p_b_to_g=p_b_to_g,
        )
        collision_thresh = float(self.profile.get("collision_threshold"))
        self.metrics = SwarmMetricsTracker(collision_threshold=collision_thresh)

        # Controllers configured with profile gains and thresholds
        self.central_ctrl, self.decentral_ctrl, self.hybrid_ctrl = build_controllers_from_profile(
            self.profile, nominal_latency=self.latency_mean
        )
        self.hybrid_ctrl.fallback_strategy = self.fallback_strategy
        self.hybrid_ctrl.dead_reckon_duration = self.dead_reckon_duration

        # Mission state and locked slot assignment
        self.current_formation = FormationType.LINE
        self.formation_spacing = float(self.profile.get("nominal_spacing"))
        self.centroid_target = np.array([0.0, 0.0], dtype=np.float64)
        self.centroid_velocity = np.array([0.0, 0.0], dtype=np.float64)
        self.drone_slot_map: Dict[int, int] = {}
        self.current_local_offsets: np.ndarray = np.zeros((self.num_drones, 2), dtype=np.float64)
        self.target_slots: np.ndarray = np.zeros((self.num_drones, 2), dtype=np.float64)
        self._lock_formation_slots()

        # Onboard drone state received from coordinator (frozen under link loss)
        self.drone_last_received_target: Dict[int, np.ndarray] = {
            d.id: self.target_slots[i].copy() for i, d in enumerate(self.drones)
        }
        self.drone_last_received_velocity: Dict[int, np.ndarray] = {
            d.id: self.centroid_velocity.copy() for d in self.drones
        }
        self.drone_last_received_next_waypoint: Dict[int, np.ndarray] = {
            d.id: (self.target_slots[i] + self.centroid_velocity * 1.0).copy() for i, d in enumerate(self.drones)
        }
        self.drone_last_received_formation: Dict[int, FormationType] = {
            d.id: self.current_formation for d in self.drones
        }
        self.drone_last_received_offsets: Dict[int, Dict[int, np.ndarray]] = {
            d.id: {} for d in self.drones
        }

        # Initial neighbor offsets
        initial_drone_ids = [d.id for d in self.drones]
        for i, d in enumerate(self.drones):
            self.drone_last_received_offsets[d.id] = compute_desired_neighbor_offsets(
                drone_id=d.id,
                assigned_targets=self.target_slots,
                neighbor_ids=[n.id for n in self.drones if n.id != d.id],
                drone_ids=initial_drone_ids,
            )
            self.hybrid_ctrl.last_known_target[d.id] = self.target_slots[i].copy()
            self.hybrid_ctrl.last_known_velocity[d.id] = self.centroid_velocity.copy()
            self.hybrid_ctrl.last_known_next_waypoint[d.id] = (self.target_slots[i] + self.centroid_velocity * 1.0).copy()
            self.hybrid_ctrl.last_known_offsets[d.id] = self.drone_last_received_offsets[d.id]

        # Hold-last-command state for centralized baseline
        init_accels = self.central_ctrl.compute_control_inputs(
            self.drones,
            self.current_formation,
            self.centroid_target,
            self.centroid_velocity,
            spacing=self.formation_spacing,
            use_velocity_feedforward=self.use_velocity_feedforward,
            drag_coeff=float(self.profile.get("drag_coeff")),
        )
        self.last_central_accel: Dict[int, np.ndarray] = {
            d.id: init_accels[i].copy() for i, d in enumerate(self.drones)
        }

        # Neighbor memory with age-out
        self.neighbor_timeout: float = float(self.profile.get("neighbor_timeout", 0.30))
        self.neighbor_memory: Dict[int, Dict[int, Dict[str, Any]]] = {
            d.id: {} for d in self.drones
        }

        # Link control (for hybrid failure testing: if False, central coordinator is severed)
        self.coordinator_link_active = True
        self.coordinator_seq_num = 0

    def _lock_formation_slots(self) -> None:
        """
        Computes formation local offsets and locks drone-to-slot Hungarian matching
        at morph start to prevent slot-swapping chattering during transitions.
        """
        n = self.num_drones
        self.current_local_offsets = FormationGenerator.get_formation_offsets(
            self.current_formation, n, spacing=self.formation_spacing
        )
        world_slots = create_world_slots(self.current_local_offsets, self.centroid_target[:2])
        curr_pos = np.array([d.position[:2] for d in self.drones], dtype=np.float64)
        cost_matrix = np.zeros((n, n), dtype=np.float64)
        for i in range(n):
            for j in range(n):
                cost_matrix[i, j] = np.sum((curr_pos[i] - world_slots[j]) ** 2)
        row_ind, col_ind = linear_sum_assignment(cost_matrix)
        self.drone_slot_map = {int(r): int(c) for r, c in zip(row_ind, col_ind)}

        # Update target_slots
        self.target_slots = np.zeros((n, 2), dtype=np.float64)
        for i in range(n):
            slot_idx = self.drone_slot_map.get(i, i)
            self.target_slots[i] = self.centroid_target[:2] + self.current_local_offsets[slot_idx]

    def set_formation(
        self,
        formation: FormationType,
        centroid: Optional[np.ndarray] = None,
        spacing: Optional[float] = None,
    ) -> None:
        """Update target formation geometry and centroid. If spacing is None, preserves active profile spacing."""
        self.current_formation = formation
        if spacing is not None:
            self.formation_spacing = float(spacing)
        if centroid is not None:
            self.centroid_target = np.array(centroid, dtype=np.float64)
        self._lock_formation_slots()

    def set_coordinator_link(self, active: bool) -> None:
        """Simulate ground station link cut or reconnection."""
        self.coordinator_link_active = active
        if not active:
            for d in self.drones:
                self.hybrid_ctrl.record_heartbeat_attempt(d.id, False)

    def step(self) -> None:
        """Advance simulation by one timestep dt."""
        n = self.num_drones

        # 1. Update topology graph
        adj_matrix = self.graph.compute_adjacency_matrix(self.drones)
        laplacian = self.graph.compute_laplacian_matrix(adj_matrix)
        fiedler = self.graph.algebraic_connectivity(laplacian)

        # 1b. Sensor measurement with first-order Gauss-Markov time-correlated noise (common-mode + independent)
        # and separate velocity estimation error modeling
        if self.gps_noise_std > 0:
            phi = float(np.exp(-self.dt / max(1e-3, self.gps_corr_time)))
            driver_scale = np.sqrt(max(0.0, 1.0 - phi**2))

            sigma_common = np.sqrt(self.gps_common_mode_fraction) * self.gps_noise_std
            sigma_indep = np.sqrt(max(0.0, 1.0 - self.gps_common_mode_fraction)) * self.gps_noise_std

            w_common_step = self.rng.normal(0, sigma_common, size=self.drones[0].dim)
            self.gps_noise_common = phi * self.gps_noise_common + driver_scale * w_common_step

            measured_positions = {}
            measured_velocities = {}
            for d in self.drones:
                w_indep_step = self.rng.normal(0, sigma_indep, size=d.dim)
                self.gps_noise_indep[d.id] = phi * self.gps_noise_indep[d.id] + driver_scale * w_indep_step
                measured_positions[d.id] = d.position + self.gps_noise_common + self.gps_noise_indep[d.id]

                # Velocity estimation error modeled separately (fused Doppler / IMU)
                v_noise = (
                    self.rng.normal(0, self.gps_vel_noise_std, size=d.dim)
                    if self.gps_vel_noise_std > 0
                    else np.zeros(d.dim, dtype=np.float64)
                )
                measured_velocities[d.id] = d.velocity + v_noise
        else:
            measured_positions = {d.id: d.position.copy() for d in self.drones}
            measured_velocities = {d.id: d.velocity.copy() for d in self.drones}

        # 2. Inter-drone wireless broadcast (Decentralized state exchange)
        # Each drone broadcasts its measured position & measured velocity to nearby peers
        for i in range(n):
            for j in range(n):
                if i != j:
                    dist = self.drones[i].distance_to(self.drones[j])
                    payload = {
                        "position": measured_positions[self.drones[i].id].copy(),
                        "velocity": measured_velocities[self.drones[i].id].copy(),
                    }
                    self.channel.send(
                        sender_id=self.drones[i].id,
                        recipient_id=self.drones[j].id,
                        payload=payload,
                        distance=dist,
                        current_time=self.current_time,
                    )

        # 3. Coordinator Heartbeat Broadcast (if central link is physically active)
        # Target slots rigidly follow moving centroid based on locked slot assignment
        assigned_targets = np.zeros((n, 2), dtype=np.float64)
        for i in range(n):
            slot_idx = self.drone_slot_map.get(i, i)
            assigned_targets[i] = self.centroid_target[:2] + self.current_local_offsets[slot_idx]
        self.target_slots = assigned_targets

        # Compute centralized commands
        central_accels = self.central_ctrl.compute_control_inputs(
            self.drones,
            self.current_formation,
            self.centroid_target,
            self.centroid_velocity,
            spacing=self.formation_spacing,
            use_velocity_feedforward=self.use_velocity_feedforward,
            drag_coeff=float(self.profile.get("drag_coeff")),
            measured_positions=measured_positions,
            measured_velocities=measured_velocities,
        )

        drone_ids = [d.id for d in self.drones]
        if self.coordinator_link_active:
            self.coordinator_seq_num += 1
            for i in range(n):
                d_id = self.drones[i].id
                offsets_for_i = compute_desired_neighbor_offsets(
                    drone_id=d_id,
                    assigned_targets=assigned_targets,
                    neighbor_ids=[o.id for o in self.drones if o.id != d_id],
                    drone_ids=drone_ids,
                )
                payload = {
                    "seq_num": self.coordinator_seq_num,
                    "target_pos": assigned_targets[i].copy(),
                    "target_velocity": self.centroid_velocity.copy(),
                    "next_waypoint": (assigned_targets[i] + self.centroid_velocity * 1.0).copy(),
                    "cmd_accel": central_accels[i].copy(),
                    "formation_type": self.current_formation,
                    "desired_offsets": offsets_for_i,
                }
                dist_to_gs = float(np.linalg.norm(self.drones[i].position - self.centroid_target))
                self.channel.send(
                    sender_id=-1,  # -1 represents Central Coordinator
                    recipient_id=d_id,
                    payload=payload,
                    distance=dist_to_gs,
                    current_time=self.current_time,
                )

        # 4. Retrieve arrived packets per drone & update neighbor memory
        for d in self.drones:
            pkts = self.channel.receive(d.id, self.current_time)
            coord_pkt_received = False
            for pkt in pkts:
                if pkt.sender_id == -1:
                    coord_pkt_received = True
                    if "cmd_accel" in pkt.payload:
                        self.last_central_accel[d.id] = np.array(pkt.payload["cmd_accel"], dtype=np.float64)
                    self.drone_last_received_target[d.id] = np.array(pkt.payload["target_pos"], dtype=np.float64)
                    if "target_velocity" in pkt.payload:
                        self.drone_last_received_velocity[d.id] = np.array(pkt.payload["target_velocity"], dtype=np.float64)
                    if "next_waypoint" in pkt.payload:
                        self.drone_last_received_next_waypoint[d.id] = np.array(pkt.payload["next_waypoint"], dtype=np.float64)
                    self.drone_last_received_formation[d.id] = pkt.payload.get("formation_type", self.current_formation)
                    if "desired_offsets" in pkt.payload:
                        self.drone_last_received_offsets[d.id] = pkt.payload["desired_offsets"]
                    # Heartbeat from coordinator: validates sequence & message age
                    self.hybrid_ctrl.process_coordinator_heartbeat(
                        drone_id=d.id,
                        current_time=self.current_time,
                        send_timestamp=pkt.sent_time,
                        sequence_num=pkt.payload["seq_num"],
                        target_pos=pkt.payload["target_pos"],
                        target_velocity=pkt.payload.get("target_velocity"),
                        next_waypoint=pkt.payload.get("next_waypoint"),
                        desired_offsets=pkt.payload.get("desired_offsets"),
                        formation_type=pkt.payload.get("formation_type"),
                    )
                else:
                    self.neighbor_memory[d.id][pkt.sender_id] = {
                        "position": np.array(pkt.payload["position"], dtype=np.float64),
                        "velocity": np.array(pkt.payload["velocity"], dtype=np.float64),
                        "timestamp": self.current_time,
                    }

            if not coord_pkt_received:
                self.hybrid_ctrl.record_heartbeat_attempt(d.id, False)

        # 4b. Assemble perceived neighbors from memory with age-out and linear extrapolation
        perceived_neighbors: Dict[int, List[Dict]] = {d.id: [] for d in self.drones}
        for d in self.drones:
            valid_mem = {}
            for peer_id, mem in self.neighbor_memory[d.id].items():
                age = self.current_time - mem["timestamp"]
                if age <= self.neighbor_timeout:
                    extrap_pos = mem["position"] + mem["velocity"] * age
                    perceived_neighbors[d.id].append({
                        "id": peer_id,
                        "position": extrap_pos,
                        "velocity": mem["velocity"].copy(),
                        "age": age,
                    })
                    valid_mem[peer_id] = mem
            self.neighbor_memory[d.id] = valid_mem

        # 5. Compute Control Inputs based on selected mode
        if self.control_mode in ["centralized", "centralized_hold_target"]:
            # Centralized with onboard target tracking under loss (Recommendation 5)
            for d in self.drones:
                target_i = self.drone_last_received_target[d.id]
                p_err = target_i - measured_positions[d.id]
                v_target = (self.drone_last_received_velocity[d.id] if (self.use_velocity_feedforward and self.coordinator_link_active) else np.zeros(2))
                v_err = v_target - measured_velocities[d.id]
                u_cmd = self.central_ctrl.kp * p_err + self.central_ctrl.kd * v_err
                if self.use_velocity_feedforward and self.coordinator_link_active:
                    u_cmd += float(self.profile.get("drag_coeff")) * v_target

                # Artificial Potential Field collision avoidance against perceived neighbors (degree-normalised)
                deg = max(1, len(perceived_neighbors[d.id]))
                for n_info in perceived_neighbors[d.id]:
                    diff = measured_positions[d.id] - n_info["position"]
                    dist = float(np.linalg.norm(diff))
                    if 1e-4 < dist < self.central_ctrl.collision_dist:
                        repulse = (self.central_ctrl.k_repulse / deg) * (1.0 / dist - 1.0 / self.central_ctrl.collision_dist) / (dist**2)
                        u_cmd += repulse * (diff / dist)
                d.set_control_input(u_cmd)

        elif self.control_mode == "centralized_hold_accel":
            # Extra baseline: Open-loop hold last acceleration command
            for d in self.drones:
                d.set_control_input(self.last_central_accel[d.id])

        elif self.control_mode == "decentralized":
            # Pure local consensus & flocking towards last received target via channel
            goal_v = self.drone_last_received_velocity[d.id] if (self.use_velocity_feedforward and self.coordinator_link_active) else None
            for d in self.drones:
                target_i = self.drone_last_received_target[d.id]
                desired_offsets = self.drone_last_received_offsets[d.id]

                accel_i = self.decentral_ctrl.compute_drone_control(
                    drone=d,
                    neighbor_states=perceived_neighbors[d.id],
                    desired_offsets=desired_offsets,
                    goal_pos=target_i,
                    goal_vel=goal_v,
                    measured_position=measured_positions[d.id],
                    measured_velocity=measured_velocities[d.id],
                    use_velocity_feedforward=self.use_velocity_feedforward,
                    drag_coeff=float(self.profile.get("drag_coeff")),
                )
                d.set_control_input(accel_i)

        elif self.control_mode == "hybrid":
            target_v = self.drone_last_received_velocity[d.id] if self.coordinator_link_active else None
            for d in self.drones:
                desired_offsets = self.drone_last_received_offsets[d.id]

                accel_i = self.hybrid_ctrl.compute_hybrid_control(
                    drone=d,
                    current_time=self.current_time,
                    dt=self.dt,
                    neighbor_states=perceived_neighbors[d.id],
                    desired_neighbor_offsets=desired_offsets,
                    target_velocity=target_v,
                    use_velocity_feedforward=self.use_velocity_feedforward,
                    drag_coeff=float(self.profile.get("drag_coeff")),
                    measured_position=measured_positions[d.id],
                    measured_velocity=measured_velocities[d.id],
                )
                d.set_control_input(accel_i)

        # 6. Physical integration step for each drone
        for d in self.drones:
            d.step(self.dt)

        # 7. Record metrics snapshot
        mode_switches = (
            self.hybrid_ctrl.get_total_mode_switches()
            if self.control_mode == "hybrid"
            else 0
        )
        mean_alpha = (
            float(np.mean([self.hybrid_ctrl.get_alpha(d.id) for d in self.drones]))
            if self.control_mode == "hybrid"
            else 1.0
        )

        self.metrics.record_step(
            current_time=self.current_time,
            drones=self.drones,
            target_positions=assigned_targets,
            adj_matrix=adj_matrix,
            fiedler_val=fiedler,
            total_mode_switches=mode_switches,
            mean_alpha=mean_alpha,
        )

        self.current_time += self.dt
