"""SITL-only peer-message transport for isolated local-agent inputs.

The harness may observe every vehicle, but local agents receive peer state only
through this one-hop message bus. This is an emulated radio, not a real link.
"""

from typing import Dict, Iterable, List, Optional, Set, Tuple

import numpy as np

from swarm_core.network import WirelessChannel


class PeerStateBus:
    """One-hop peer state exchange with delay, loss, stale-data expiry and link cuts."""

    def __init__(self, count: int, comm_range: float = 5.0,
                 packet_loss_rate: float = 0.0, latency_mean: float = 0.03,
                 latency_std: float = 0.005, timeout_s: float = 0.5,
                 seed: Optional[int] = None, burst_loss: bool = False):
        self.count = int(count)
        self.timeout_s = float(timeout_s)
        if not np.isfinite(self.timeout_s) or self.timeout_s <= 0:
            raise ValueError("timeout_s must be finite and positive")
        self.channel = WirelessChannel(
            comm_range=comm_range, packet_loss_rate=packet_loss_rate,
            latency_mean=latency_mean, latency_std=latency_std, seed=seed,
            use_gilbert_elliott=burst_loss,
        )
        self.graph: Dict[int, Set[int]] = {i: set() for i in range(count)}
        self.blocked_links: Set[Tuple[int, int]] = set()
        self.latest: List[Dict[int, dict]] = [dict() for _ in range(count)]
        self.sequence = [0] * self.count
        self.rejected_out_of_order = 0
        self.rejected_stale_source = 0

    def set_graph(self, graph: Dict[int, Iterable[int]]) -> None:
        self.graph = {i: set(graph.get(i, ())) for i in range(self.count)}

    def block_link(self, sender: int, recipient: int, blocked: bool = True) -> None:
        """Block/unblock one directed peer link; IDs are zero-based SITL indices."""
        if not (0 <= sender < self.count and 0 <= recipient < self.count) or sender == recipient:
            raise ValueError("peer link must connect two different valid agent indices")
        key = (sender, recipient)
        if blocked:
            self.blocked_links.add(key)
        else:
            self.blocked_links.discard(key)

    def publish(self, positions: List[np.ndarray], velocities: List[np.ndarray],
                active: List[bool], now: float,
                source_times: Optional[List[float]] = None) -> None:
        """Broadcast each active peer's latest state to configured one-hop neighbors."""
        source_times = source_times if source_times is not None else [now] * self.count
        for sender in range(self.count):
            if not active[sender]:
                continue
            state_time = float(source_times[sender])
            # Do not stamp old telemetry as new just because the relay is ticking.
            if (not np.isfinite(state_time) or not np.all(np.isfinite(positions[sender]))
                    or not np.all(np.isfinite(velocities[sender]))):
                self.rejected_stale_source += 1
                continue
            if state_time < 0 or state_time > now + 1e-4 or now - state_time > self.timeout_s:
                self.rejected_stale_source += 1
                continue
            self.sequence[sender] += 1
            payload = {"position": np.asarray(positions[sender][:2], dtype=float).tolist(),
                       "velocity": np.asarray(velocities[sender][:2], dtype=float).tolist(),
                       "state_time": state_time, "sequence": self.sequence[sender]}
            for recipient in sorted(self.graph.get(sender, ())):
                if not active[recipient] or (sender, recipient) in self.blocked_links:
                    continue
                distance = float(np.linalg.norm(
                    np.asarray(positions[sender][:2]) - np.asarray(positions[recipient][:2])
                ))
                self.channel.send(sender, recipient, payload, distance, now)

        for recipient in range(self.count):
            for packet in self.channel.receive(recipient, now):
                previous = self.latest[recipient].get(packet.sender_id)
                sequence = int(packet.payload["sequence"])
                if previous is not None and sequence <= previous["sequence"]:
                    self.rejected_out_of_order += 1
                    continue
                self.latest[recipient][packet.sender_id] = {
                    "position": np.asarray(packet.payload["position"], dtype=float),
                    "velocity": np.asarray(packet.payload["velocity"], dtype=float),
                    "timestamp": float(packet.payload["state_time"]),
                    "sequence": sequence,
                }

    def neighbors(self, receiver: int, now: float) -> List[dict]:
        """Return fresh one-hop states, linearly extrapolated to the current tick."""
        result = []
        for sender in sorted(self.graph.get(receiver, ())):
            packet = self.latest[receiver].get(sender)
            if packet is None:
                continue
            age = max(0.0, float(now) - packet["timestamp"])
            if age > self.timeout_s:
                continue
            result.append({
                "id": sender,
                "position": packet["position"] + packet["velocity"] * age,
                "velocity": packet["velocity"].copy(),
                "age": age,
                "confidence": max(0.0, 1.0 - age / self.timeout_s),
            })
        return result
