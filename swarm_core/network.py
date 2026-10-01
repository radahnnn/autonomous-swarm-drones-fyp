"""
Wireless Network Impairment Emulator.
Models packet loss, transmission delays, and range-based signal drops for inter-drone communications.
"""

from dataclasses import dataclass
from typing import Any, Dict, List, Optional
import numpy as np


@dataclass
class Packet:
    sender_id: int
    recipient_id: int
    sent_time: float
    delivery_time: float
    payload: Dict[str, Any]


class WirelessChannel:
    """Emulates a lossy, delayed broadcast/unicast wireless medium."""

    def __init__(
        self,
        comm_range: float = 12.0,
        packet_loss_rate: float = 0.0,
        latency_mean: float = 0.02,  # 20ms average delay
        latency_std: float = 0.005,   # 5ms jitter
        seed: Optional[int] = None,
        use_gilbert_elliott: bool = False,
        p_g_to_b: float = 0.05,  # Probability of Good -> Bad transition
        p_b_to_g: float = 0.20,  # Probability of Bad -> Good transition (avg burst = 5 steps)
        loss_rate_bad: float = 1.0,  # Loss rate in Bad state
    ):
        self.comm_range = float(comm_range)
        self.packet_loss_rate = float(np.clip(packet_loss_rate, 0.0, 1.0))
        self.latency_mean = float(max(0.0, latency_mean))
        self.latency_std = float(max(0.0, latency_std))
        
        self.use_gilbert_elliott = bool(use_gilbert_elliott)
        self.p_g_to_b = float(p_g_to_b)
        self.p_b_to_g = float(p_b_to_g)
        self.loss_rate_bad = float(loss_rate_bad)
        self.channel_state: Dict[int, str] = {}  # Per-recipient state: "GOOD" or "BAD"
        
        # Scheduled full outage intervals: List of (start_time, duration)
        self.outages: List[tuple] = []
        
        self.rng = np.random.default_rng(seed)
        self.in_flight_packets: List[Packet] = []
        
        # Statistics
        self.total_transmitted = 0
        self.total_dropped_loss = 0
        self.total_dropped_range = 0
        self.total_dropped_outage = 0
        self.total_dropped_burst = 0
        self.total_delivered = 0

    def add_outage(self, start_time: float, duration: float) -> None:
        """Schedule a deterministic complete RF outage window [start_time, start_time + duration]."""
        self.outages.append((float(start_time), float(duration)))

    def is_in_outage(self, current_time: float) -> bool:
        """Check if channel is currently experiencing a scheduled full outage."""
        for start, dur in self.outages:
            if start <= current_time <= start + dur:
                return True
        return False

    def _update_ge_state(self, recipient_id: int) -> str:
        """Advance Gilbert-Elliott Markov chain state for the given recipient link."""
        cur = self.channel_state.get(recipient_id, "GOOD")
        if cur == "GOOD":
            if self.rng.random() < self.p_g_to_b:
                cur = "BAD"
        else:
            if self.rng.random() < self.p_b_to_g:
                cur = "GOOD"
        self.channel_state[recipient_id] = cur
        return cur

    def send(
        self,
        sender_id: int,
        recipient_id: int,
        payload: Dict[str, Any],
        distance: float,
        current_time: float,
    ) -> bool:
        """Attempt to transmit a packet across the wireless channel."""
        self.total_transmitted += 1

        # 1. Check scheduled complete outage (100% loss during outage window)
        if self.is_in_outage(current_time):
            self.total_dropped_outage += 1
            self.total_dropped_loss += 1
            return False

        # 2. Check range limit
        if distance > self.comm_range:
            self.total_dropped_range += 1
            return False

        # 3. Check Gilbert-Elliott burst loss (if enabled)
        if self.use_gilbert_elliott:
            state = self._update_ge_state(recipient_id)
            if state == "BAD" and self.rng.random() < self.loss_rate_bad:
                self.total_dropped_burst += 1
                self.total_dropped_loss += 1
                return False

        # 4. Check stochastic Bernoulli packet drop
        if self.packet_loss_rate > 0.0 and self.rng.random() < self.packet_loss_rate:
            self.total_dropped_loss += 1
            return False

        # 5. Sample latency (Gaussian truncated at 0.001s)
        delay = max(0.001, self.rng.normal(self.latency_mean, self.latency_std))
        delivery_time = current_time + delay

        packet = Packet(
            sender_id=sender_id,
            recipient_id=recipient_id,
            sent_time=current_time,
            delivery_time=delivery_time,
            payload=payload,
        )
        self.in_flight_packets.append(packet)
        return True

    def receive(self, recipient_id: int, current_time: float) -> List[Packet]:
        """Retrieve all packets that have arrived at or before current_time for recipient_id."""
        delivered: List[Packet] = []
        remaining: List[Packet] = []

        for pkt in self.in_flight_packets:
            if pkt.recipient_id == recipient_id and pkt.delivery_time <= current_time:
                delivered.append(pkt)
                self.total_delivered += 1
            else:
                remaining.append(pkt)

        self.in_flight_packets = remaining
        return delivered

    def reset_stats(self) -> None:
        """Reset network diagnostic counters."""
        self.total_transmitted = 0
        self.total_dropped_loss = 0
        self.total_dropped_range = 0
        self.total_dropped_outage = 0
        self.total_dropped_burst = 0
        self.total_delivered = 0
        self.in_flight_packets.clear()
        self.channel_state.clear()
