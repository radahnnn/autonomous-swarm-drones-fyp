"""Retired 2026-11-04 (user decision, item 5): host-side harness of the legacy local/process runtimes, moved out of
sitl/decentralized_guidance.py unchanged. Not imported by the live code; kept for reference."""
# flake8: noqa
from sitl.decentralized_guidance import *  # noqa: F401,F403


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
        # Keep the process harness away from the live independent swarm's fixed
        # 20000+ range so regression tests can run while a demo is connected.
        self.peer_port_base = 30000 + (os.getpid() % 1000) * 10
        if agent_runtime == "process":
            self.agents = [agent_class(i, safe_radius=safe_radius,
                                       hybrid=control_mode == "hybrid", peer_count=count,
                                       peer_port_base=self.peer_port_base, peer_loss=peer_loss,
                                       peer_latency_ms=peer_latency_ms, peer_seed=peer_seed,
                                       peer_burst_loss=peer_burst_loss) for i in range(count)]
        else:
            self.agents = [agent_class(i, safe_radius=safe_radius,
                                       hybrid=control_mode == "hybrid") for i in range(count)]
        self.last_time: Optional[float] = None
        self.mission_sequence = 0
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
        self.mission_sequence += 1
        intent = MissionIntent(
            sequence=self.mission_sequence,
            issued_at=now,
            expires_at=now + 1.0,
            goal_ne=(float(goal[0]), float(goal[1])),
        ).to_dict()
        graph = self._one_hop_graph(slots, active)
        self.peer_bus.set_graph(graph)
        outputs = [None] * self.count

        # Update each agent's own estimate before publishing across the emulated peer links.
        for i in range(self.count):
            if active[i] and fresh[i]:
                self.agents[i].update_local_state(pos[i], sample_times[i])
        if self.agent_runtime == "local":
            self.peer_bus.publish(pos, [agent.velocity for agent in self.agents], active, now,
                                  source_times=sample_times)

        for i in range(self.count):
            if not active[i] or not fresh[i]:
                continue
            neighbors = self.peer_bus.neighbors(i, now) if self.agent_runtime == "local" else []
            desired_offsets = {
                j: slots[i][:2] - slots[j][:2] for j in sorted(graph.get(i, ()))
            }
            if i in self.agent_errors:
                continue
            try:
                agent_args = (pos[i], sample_times[i], None, neighbors,
                              desired_offsets, dt)
                horizontal = self.agents[i].command(
                    *agent_args,
                    coordinator_connected=coordinator_connected, now=now,
                    mission_intent=intent, slot_offset=slots[i][:2],
                    **({"peer_ids": sorted(graph.get(i, ()))}
                       if self.agent_runtime == "process" else {}),
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
