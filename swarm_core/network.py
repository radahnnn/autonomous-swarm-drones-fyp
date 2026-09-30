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
    ):
        self.comm_range = float(comm_range)
        self.packet_loss_rate = float(np.clip(packet_loss_rate, 0.0, 1.0))
        self.latency_mean = float(max(0.0, latency_mean))
        self.latency_std = float(max(0.0, latency_std))
        
        self.rng = np.random.default_rng(seed)
        self.in_flight_packets: List[Packet] = []
        
        # Statistics
        self.total_transmitted = 0
        self.total_dropped_loss = 0
        self.total_dropped_range = 0
        self.total_delivered = 0

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

        # Check range limit
        if distance > self.comm_range:
            self.total_dropped_range += 1
            return False

        # Check stochastic packet drop
        if self.rng.random() < self.packet_loss_rate:
            self.total_dropped_loss += 1
            return False

        # Sample latency (Gaussian truncated at 0.001s)
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
        self.total_delivered = 0
        self.in_flight_packets.clear()
