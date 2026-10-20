"""Per-agent decentralized guidance adapter for the SITL console.

The console is the SITL transport harness. Each LocalDroneAgent receives its own
position/velocity estimate, high-level shared mission goal, and only the states of
its one-hop neighbors. This models decentralized control logic on one host; it does
not place controllers on separate flight computers or create a drone-to-drone radio.
"""

import time
from typing import Dict, List, Optional

import numpy as np

from swarm_core.controllers.decentralized import DecentralizedController
from swarm_core.controllers.hybrid import HybridController
from swarm_core.drone import Drone
from sitl.peer_state_bus import PeerStateBus


def _limit(vector: np.ndarray, maximum: float) -> np.ndarray:
    norm = float(np.linalg.norm(vector))
    return vector * (maximum / norm) if norm > maximum else vector


class LocalDroneAgent:
    """One simulated drone's local controller and telemetry-derived state."""

    MAX_SPEED = 1.5
    MAX_ACCEL = 2.5
    LOOKAHEAD_S = 0.7

    def __init__(self, drone_id: int, safe_radius: float = 3.0, hybrid: bool = False):
        self.drone_id = drone_id
        self.controller = DecentralizedController(
            k_sep=7.0, k_align=1.6, k_form=1.4, k_goal=0.8,
            safe_radius=safe_radius,
        )
        self.hybrid_controller = (
            HybridController(decentral_controller=self.controller, fallback_strategy="hold_target")
            if hybrid else None
        )
        self.coordinator_sequence = 0
        self.drone = Drone(
            drone_id, np.zeros(2), max_speed=self.MAX_SPEED,
            max_accel=self.MAX_ACCEL, attitude_tau=0.2,
            drag_coeff=0.0, measurement_noise_std=0.0,
        )
        self.last_position: Optional[np.ndarray] = None
        self.last_sample_time: Optional[float] = None
        self.velocity = np.zeros(2, dtype=float)

    def command(self, position, sample_time: float, goal, neighbor_states,
                desired_offsets: Dict[int, np.ndarray], dt: float,
                coordinator_connected: bool = True, now: Optional[float] = None) -> np.ndarray:
        """Return a short-horizon local position target in the shared N/E frame."""
        self.update_local_state(position, sample_time)
        pos = np.asarray(position, dtype=float)[:2]
        goal = np.asarray(goal, dtype=float)[:2]
        now = time.time() if now is None else now
        if self.hybrid_controller is not None:
            if coordinator_connected:
                self.coordinator_sequence += 1
                self.hybrid_controller.process_coordinator_heartbeat(
                    self.drone_id, now, now, self.coordinator_sequence,
                    target_pos=goal, target_velocity=np.zeros(2, dtype=float),
                    desired_offsets=desired_offsets,
                )
            accel = self.hybrid_controller.compute_hybrid_control(
                self.drone, current_time=now, dt=dt,
                neighbor_states=neighbor_states,
                desired_neighbor_offsets=desired_offsets,
                target_velocity=np.zeros(2, dtype=float),
                use_velocity_feedforward=False, drag_coeff=0.0,
                measured_position=pos, measured_velocity=self.velocity,
            )
        else:
            accel = self.controller.compute_drone_control(
                self.drone,
                neighbor_states=neighbor_states,
                desired_offsets=desired_offsets,
                goal_pos=goal,
                goal_vel=np.zeros(2, dtype=float),
                measured_position=pos,
                measured_velocity=self.velocity,
                use_velocity_feedforward=False,
                drag_coeff=0.0,
            )
        accel = _limit(np.asarray(accel, dtype=float), self.MAX_ACCEL)
        self.drone.set_control_input(accel)
        velocity_command = _limit(
            self.velocity + self.drone.commanded_accel * dt, self.MAX_SPEED
        )
        return pos + velocity_command * self.LOOKAHEAD_S

    def update_local_state(self, position, sample_time: float) -> None:
        """Update this agent's velocity estimate using only its own telemetry history."""
        pos = np.asarray(position, dtype=float)[:2]
        if self.last_sample_time is None or sample_time > self.last_sample_time:
            if self.last_position is not None and self.last_sample_time is not None:
                sample_dt = sample_time - self.last_sample_time
                if sample_dt > 1e-4:
                    measured = (pos - self.last_position) / sample_dt
                    self.velocity = _limit(0.7 * self.velocity + 0.3 * measured, 3.0)
            self.last_position = pos.copy()
            self.last_sample_time = sample_time


class DecentralizedGuidance:
    """Host-side SITL harness connecting isolated local-agent controller inputs."""

    NEIGHBOR_RADIUS = 5.0

    def __init__(self, count: int, safe_radius: float = 3.0,
                 peer_loss: float = 0.0, peer_latency_ms: float = 30.0,
                 peer_seed: Optional[int] = None, control_mode: str = "decentralized",
                 peer_burst_loss: bool = False, agent_runtime: str = "local"):
        self.agent_errors = {}
        self.count = count
        self.control_mode = control_mode
        self.agent_runtime = agent_runtime
        agent_class = LocalDroneAgent
        if agent_runtime == "process":
            from sitl.agent_process import ProcessDroneAgent
            agent_class = ProcessDroneAgent
        elif agent_runtime != "local":
            raise ValueError("agent_runtime must be local or process")
        self.cached_goal = None
        self.cached_slots = None
        self.agents = [agent_class(i, safe_radius=safe_radius,
                                       hybrid=control_mode == "hybrid") for i in range(count)]
        self.last_time: Optional[float] = None
        self.peer_bus = PeerStateBus(
            count, packet_loss_rate=peer_loss,
            latency_mean=max(0.0, peer_latency_ms / 1000.0),
            latency_std=0.005 if peer_latency_ms > 0 else 0.0,
            seed=peer_seed, burst_loss=peer_burst_loss,
        )

    @staticmethod
    def _one_hop_graph(offsets: List[np.ndarray], active: List[bool]):
        """Create a connected minimum-spanning-tree neighbor graph over active slots."""
        nodes = [i for i, enabled in enumerate(active) if enabled]
        graph = {i: set() for i in nodes}
        if len(nodes) < 2:
            return graph
        for index, i in enumerate(nodes):
            for j in nodes[index + 1:]:
                if np.linalg.norm(offsets[i][:2] - offsets[j][:2]) <= DecentralizedGuidance.NEIGHBOR_RADIUS:
                    graph[i].add(j)
                    graph[j].add(i)
        connected = {nodes[0]}
        frontier = [nodes[0]]
        while frontier:
            i = frontier.pop()
            for j in graph[i] - connected:
                connected.add(j)
                frontier.append(j)
        while len(connected) < len(nodes):
            candidates = [
                (float(np.linalg.norm(offsets[i][:2] - offsets[j][:2])), i, j)
                for i in connected for j in nodes if j not in connected
            ]
            _, i, j = min(candidates)
            graph[i].add(j)
            graph[j].add(i)
            frontier = [j]
            connected.add(j)
            while frontier:
                i = frontier.pop()
                for peer in graph[i] - connected:
                    connected.add(peer)
                    frontier.append(peer)
        return graph

    def step(self, positions, sample_times, mission_goal, offsets, active, fresh,
             now=None, coordinator_connected=True):
        """Compute independent targets using only each active agent's 1-hop peers.

        Each agent combines the shared high-level goal with its own configured slot.
        Hybrid agents receive supervisory targets only while the coordinator is connected.
        """
        now = time.time() if now is None else now
        dt = 0.1 if self.last_time is None else min(0.25, max(0.02, now - self.last_time))
        self.last_time = now
        pos = [np.asarray(p, dtype=float) for p in positions]
        slots = [np.asarray(o, dtype=float) for o in offsets]
        goal = np.asarray(mission_goal, dtype=float)
        if self.control_mode == "hybrid":
            if coordinator_connected:
                self.cached_goal = goal.copy()
                self.cached_slots = [slot.copy() for slot in slots]
            elif self.cached_goal is None:
                return [None] * self.count
            else:
                goal = self.cached_goal.copy()
                slots = [slot.copy() for slot in self.cached_slots]
        graph = self._one_hop_graph(slots, active)
        self.peer_bus.set_graph(graph)
        outputs = [None] * self.count

        # Update each agent's own estimate before publishing across the emulated peer links.
        for i in range(self.count):
            if active[i] and fresh[i]:
                self.agents[i].update_local_state(pos[i], sample_times[i])
        self.peer_bus.publish(pos, [agent.velocity for agent in self.agents], active, now,
                              source_times=sample_times)

        for i in range(self.count):
            if not active[i] or not fresh[i]:
                continue
            neighbors = self.peer_bus.neighbors(i, now)
            desired_offsets = {
                j: slots[i][:2] - slots[j][:2] for j in sorted(graph.get(i, ()))
            }
            if i in self.agent_errors:
                continue
            try:
                horizontal = self.agents[i].command(
                    pos[i], sample_times[i], goal[:2] + slots[i][:2], neighbors,
                    desired_offsets, dt, coordinator_connected=coordinator_connected, now=now,
                )
            except (RuntimeError, ValueError, OSError) as exc:
                self.agent_errors[i] = str(exc)
                continue
            output = pos[i].copy()
            output[:2] = horizontal
            output[2] = float(goal[2] + slots[i][2])
            outputs[i] = output.tolist()
        return outputs

    def close(self):
        for agent in self.agents:
            if hasattr(agent, "close"):
                agent.close()
