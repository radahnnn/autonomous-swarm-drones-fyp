"""Direct UDP peer-state exchange used by process-isolated SITL agents."""

import hashlib
import hmac
import json
import heapq
import math
import random
import re
import socket
import uuid
from typing import Dict, Iterable

import numpy as np


VERSION = 3
FRAME = "shared_local_north_east_metres"
MAX_PEER_AGE_S = 0.5
MAX_HEARTBEAT_AGE_S = 2.0
MAX_FUTURE_SKEW_S = 0.05
MAX_PENDING_DATAGRAMS = 512
MAX_PENDING_PER_PEER = 64


class PeerLinkWatchdog:
    """Debounce missing required neighbors before the agent selects its safe action."""

    def __init__(self, peer_ids: Iterable[int], timeout_s: float = 2.0,
                 startup_grace_s: float = 0.0):
        if not math.isfinite(timeout_s) or timeout_s <= 0:
            raise ValueError("peer timeout must be a positive finite duration")
        if not math.isfinite(startup_grace_s) or startup_grace_s < 0:
            raise ValueError("peer startup grace must be a non-negative finite duration")
        self.peer_ids = frozenset(int(peer_id) for peer_id in peer_ids)
        self.timeout_s = float(timeout_s)
        self.startup_grace_s = float(startup_grace_s)
        self.missing_since: Dict[int, float] = {}
        self.ever_seen = set()
        self.monitoring = False
        self.monitoring_since: float | None = None

    def activate(self, now: float, fresh_peer_ids: Iterable[int]) -> None:
        """Start flight-time monitoring, including required peers never seen preflight."""
        now = float(now)
        fresh = {int(peer_id) for peer_id in fresh_peer_ids}
        self.monitoring = True
        self.monitoring_since = now
        for peer_id in self.peer_ids:
            if peer_id in fresh:
                self.ever_seen.add(peer_id)
                self.missing_since.pop(peer_id, None)
            else:
                self.missing_since.setdefault(
                    peer_id, now + self.startup_grace_s,
                )

    def deactivate(self) -> None:
        """Pause timeout accounting while grounded; preserve which peers were seen."""
        self.monitoring = False
        self.monitoring_since = None
        self.missing_since.clear()

    def update(self, now: float, fresh_peer_ids: Iterable[int]) -> list[int]:
        """Return peers continuously missing for the timeout; fresh data resets their timer."""
        now = float(now)
        fresh = {int(peer_id) for peer_id in fresh_peer_ids}
        expired = []
        for peer_id in self.peer_ids:
            if peer_id in fresh:
                self.ever_seen.add(peer_id)
                self.missing_since.pop(peer_id, None)
                continue
            if peer_id not in self.ever_seen and not self.monitoring:
                continue
            if (self.monitoring and self.monitoring_since is not None and
                    now - self.monitoring_since < self.startup_grace_s):
                continue
            first_missing = now
            if self.monitoring and self.monitoring_since is not None:
                first_missing = max(first_missing, self.monitoring_since + self.startup_grace_s)
            missing_since = self.missing_since.setdefault(peer_id, first_missing)
            if now - missing_since >= self.timeout_s:
                expired.append(peer_id)
        return sorted(expired)


class UdpPeerStateTransport:
    """Exchange fresh one-hop state packets directly between local agent processes.

    Loopback UDP keeps the runtime boundary close to a radio datagram transport while remaining
    reproducible on one development machine. It does not emulate radio propagation or loss.
    """

    def __init__(self, drone_id: int, peer_port_base: int, peer_count: int,
                 packet_loss_rate: float = 0.0, latency_mean: float = 0.0,
                 latency_std: float = 0.0, seed=None, burst_loss: bool = False,
                 burst_start_rate: float = 0.04, burst_continue_rate: float = 0.65,
                 blocked_links=(), duplicate_rate: float = 0.0,
                 reorder_rate: float = 0.0, reorder_delay: float = 0.15,
                 bind_address=None, peer_endpoints=None, auth_key: bytes | None = None):
        self.drone_id = int(drone_id)
        self.peer_port_base = int(peer_port_base)
        self.peer_count = int(peer_count)
        if not math.isfinite(packet_loss_rate) or not 0.0 <= packet_loss_rate <= 1.0:
            raise ValueError("packet loss rate must be finite and from 0 to 1")
        if (not math.isfinite(latency_mean) or not math.isfinite(latency_std) or
                latency_mean < 0 or latency_std < 0):
            raise ValueError("latency mean and jitter must be finite and non-negative")
        if (not math.isfinite(duplicate_rate) or not 0.0 <= duplicate_rate <= 1.0 or
                not math.isfinite(reorder_rate) or not 0.0 <= reorder_rate <= 1.0):
            raise ValueError("duplicate and reorder rates must be finite and from 0 to 1")
        if not math.isfinite(reorder_delay) or reorder_delay < 0:
            raise ValueError("reorder delay cannot be negative")
        self.packet_loss_rate = float(packet_loss_rate)
        self.latency_mean = float(latency_mean)
        self.latency_std = float(latency_std)
        self.burst_loss = bool(burst_loss)
        if (not math.isfinite(burst_start_rate) or
                not 0.0 <= burst_start_rate <= 1.0 or
                not math.isfinite(burst_continue_rate) or
                not 0.0 <= burst_continue_rate <= 1.0):
            raise ValueError("burst start and continuation rates must be finite and from 0 to 1")
        self.burst_start_rate = float(burst_start_rate)
        self.burst_continue_rate = float(burst_continue_rate)
        self.duplicate_rate = float(duplicate_rate)
        self.reorder_rate = float(reorder_rate)
        self.reorder_delay = float(reorder_delay)
        if auth_key is not None and len(auth_key) < 32:
            raise ValueError("peer authentication key must contain at least 32 bytes")
        self.auth_key = auth_key
        if peer_endpoints is None:
            peer_endpoints = {
                peer_id: ("127.0.0.1", self.peer_port_base + peer_id)
                for peer_id in range(self.peer_count)
            }
        self.peer_endpoints = {
            int(peer_id): (str(endpoint[0]), int(endpoint[1]))
            for peer_id, endpoint in peer_endpoints.items()
        }
        if set(self.peer_endpoints) != set(range(self.peer_count)):
            raise ValueError("peer endpoint map must contain every configured agent ID")
        if any(not host or not 1 <= port <= 65535
               for host, port in self.peer_endpoints.values()):
            raise ValueError("peer endpoints require a host and UDP port from 1 to 65535")
        self.bind_address = ("127.0.0.1", self.peer_port_base + self.drone_id)
        if bind_address is not None:
            if len(bind_address) != 2 or not str(bind_address[0]):
                raise ValueError("bind address must be a (host, port) pair")
            self.bind_address = (str(bind_address[0]), int(bind_address[1]))
            if not 1 <= self.bind_address[1] <= 65535:
                raise ValueError("bind UDP port must be from 1 to 65535")
        self.blocked_links = {(int(sender), int(recipient)) for sender, recipient in blocked_links}
        self.rng = random.Random(seed)
        self.in_bad_burst = False
        self.sequence = 0
        self.last_sequence: Dict[int, int] = {}
        self.heartbeat_sequence = 0
        self.last_heartbeat_sequence: Dict[int, int] = {}
        self.session_id = uuid.uuid4().hex
        self.peer_session_ids: Dict[int, str] = {}
        self.retired_peer_sessions: Dict[int, list[str]] = {}
        self.local_clock_offset_s = 0.0
        self.peer_clock_offsets_s: Dict[int, float] = {}
        self.last_peer_heartbeat_at: Dict[int, float] = {}
        self.pending = []
        self.pending_sequence = 0
        self.packets_queued = 0
        self.packets_sent = 0
        self.delay_sample_count = 0
        self.delay_sample_sum_s = 0.0
        self.delay_sample_sum_squares = 0.0
        self.delay_sample_min_s = None
        self.delay_sample_max_s = None
        self.dropped_loss = 0
        self.dropped_queue = 0
        self.dropped_blocked = 0
        self.delivered = 0
        self.rejected_stale = 0
        self.rejected_replay = 0
        self.rejected_invalid = 0
        self.rejected_auth = 0
        self.rejected_source = 0
        self.blocked_link_changes = 0
        self.duplicated_packets = 0
        self.reorder_delayed_packets = 0
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind(self.bind_address)
        self.socket.setblocking(False)

    def publish(self, position, velocity, sample_time: float, peer_ids: Iterable[int],
                now: float, landing: bool = False) -> None:
        if not isinstance(landing, bool):
            raise ValueError("peer landing state must be boolean")
        self.sequence += 1
        packet = {
            "version": VERSION,
            "frame": FRAME,
            "drone_id": self.drone_id,
            "session_id": self.session_id,
            "sequence": self.sequence,
            "sample_time": float(sample_time),
            "landing": landing,
            "position": np.asarray(position, dtype=float)[:2].tolist(),
            "velocity": np.asarray(velocity, dtype=float)[:2].tolist(),
        }
        encoded = self._encode(packet)
        for peer_id in set(int(value) for value in peer_ids):
            if peer_id < 0 or peer_id >= self.peer_count or peer_id == self.drone_id:
                continue
            if (self.drone_id, peer_id) in self.blocked_links:
                self.dropped_blocked += 1
                continue
            if self._drop_packet():
                self.dropped_loss += 1
                continue
            self._enqueue(peer_id, encoded, now)
        self._flush(float(now))

    def publish_heartbeat(self, peer_ids: Iterable[int], now: float) -> None:
        """Publish agent/link liveness separately from position samples."""
        self.heartbeat_sequence += 1
        packet = {
            "version": VERSION,
            "frame": FRAME,
            "kind": "heartbeat",
            "drone_id": self.drone_id,
            "session_id": self.session_id,
            "sequence": self.heartbeat_sequence,
            "sample_time": float(now),
        }
        encoded = self._encode(packet)
        for peer_id in set(int(value) for value in peer_ids):
            if peer_id < 0 or peer_id >= self.peer_count or peer_id == self.drone_id:
                continue
            if (self.drone_id, peer_id) in self.blocked_links:
                self.dropped_blocked += 1
                continue
            if self._drop_packet():
                self.dropped_loss += 1
                continue
            self._enqueue(peer_id, encoded, now)
        self._flush(float(now))

    def _enqueue(self, peer_id: int, encoded: bytes, now: float) -> None:
        """Queue one impaired datagram and optional duplicate for one peer."""
        delay = max(0.0, self.rng.gauss(self.latency_mean, self.latency_std))
        self.delay_sample_count += 1
        self.delay_sample_sum_s += delay
        self.delay_sample_sum_squares += delay * delay
        self.delay_sample_min_s = (delay if self.delay_sample_min_s is None
                                   else min(self.delay_sample_min_s, delay))
        self.delay_sample_max_s = (delay if self.delay_sample_max_s is None
                                   else max(self.delay_sample_max_s, delay))
        if self.rng.random() < self.reorder_rate:
            # Variable extra latency lets a later sequence overtake this one.
            delay += self.rng.uniform(self.reorder_delay * 0.5, self.reorder_delay * 1.5)
            self.reorder_delayed_packets += 1
        self._queue_delivery(peer_id, encoded, float(now) + delay)
        if self.rng.random() < self.duplicate_rate:
            self._queue_delivery(peer_id, encoded, float(now) + delay + self.rng.uniform(0.001, 0.020))
            self.duplicated_packets += 1

    def _queue_delivery(self, peer_id: int, encoded: bytes, delivery_time: float) -> None:
        peer_pending = sum(1 for entry in self.pending if entry[2] == peer_id)
        if peer_pending >= MAX_PENDING_PER_PEER:
            self._drop_oldest_pending(peer_id)
        if len(self.pending) >= MAX_PENDING_DATAGRAMS:
            self._drop_oldest_pending()
        self.pending_sequence += 1
        heapq.heappush(self.pending, (
            delivery_time, self.pending_sequence, peer_id, encoded,
        ))
        self.packets_queued += 1

    def _drop_oldest_pending(self, peer_id: int | None = None) -> None:
        """Bound delayed traffic by discarding its oldest queued datagram first."""
        candidates = [index for index, entry in enumerate(self.pending)
                     if peer_id is None or entry[2] == peer_id]
        if not candidates:
            return
        oldest = min(candidates, key=lambda index: self.pending[index][1])
        self.pending.pop(oldest)
        heapq.heapify(self.pending)
        self.dropped_queue += 1

    def _encode(self, packet: dict) -> bytes:
        if self.auth_key is not None:
            packet = dict(packet)
            canonical = json.dumps(packet, allow_nan=False, sort_keys=True,
                                   separators=(",", ":")).encode("utf-8")
            packet["auth"] = "hmac-sha256:" + hmac.new(
                self.auth_key, canonical, hashlib.sha256,
            ).hexdigest()
        return json.dumps(packet, allow_nan=False, sort_keys=True,
                          separators=(",", ":")).encode("utf-8")

    def _authenticated(self, packet: dict) -> bool:
        auth = packet.pop("auth", None)
        if self.auth_key is None:
            return auth is None
        if not isinstance(auth, str) or not auth.startswith("hmac-sha256:"):
            return False
        canonical = json.dumps(packet, allow_nan=False, sort_keys=True,
                               separators=(",", ":")).encode("utf-8")
        expected = "hmac-sha256:" + hmac.new(
            self.auth_key, canonical, hashlib.sha256,
        ).hexdigest()
        return hmac.compare_digest(auth, expected)

    def _drop_packet(self) -> bool:
        if self.burst_loss:
            if self.in_bad_burst:
                self.in_bad_burst = self.rng.random() < self.burst_continue_rate
            else:
                self.in_bad_burst = self.rng.random() < self.burst_start_rate
            return self.in_bad_burst
        return self.rng.random() < self.packet_loss_rate

    def set_link_blocked(self, sender: int, recipient: int, blocked: bool) -> bool:
        """Block or restore one directed edge; returns whether its state changed."""
        sender = int(sender)
        recipient = int(recipient)
        if not (0 <= sender < self.peer_count and 0 <= recipient < self.peer_count) or sender == recipient:
            raise ValueError("peer link must connect two different configured agents")
        if sender != self.drone_id:
            raise ValueError("a peer process can only block or restore links it sends")
        edge = (sender, recipient)
        was_blocked = edge in self.blocked_links
        if blocked:
            self.blocked_links.add(edge)
        else:
            self.blocked_links.discard(edge)
        changed = was_blocked != bool(blocked)
        self.blocked_link_changes += int(changed)
        return changed

    def _flush(self, now: float) -> None:
        while self.pending and self.pending[0][0] <= now:
            _, _, peer_id, payload = heapq.heappop(self.pending)
            self.socket.sendto(payload, self.peer_endpoints[peer_id])
            self.packets_sent += 1

    def receive(self, now: float, peer_ids: Iterable[int]) -> list[dict]:
        self._flush(float(now))
        allowed = set(int(value) for value in peer_ids)
        received = []
        while True:
            try:
                raw, source_address = self.socket.recvfrom(4096)
            except BlockingIOError:
                break
            try:
                packet = json.loads(raw.decode("utf-8"))
                if not isinstance(packet, dict):
                    self.rejected_invalid += 1
                    continue
                if not self._authenticated(packet):
                    self.rejected_auth += 1
                    continue
                peer_id = packet.get("drone_id")
                session_id = packet.get("session_id")
                sequence = packet.get("sequence")
                sample_time = float(packet.get("sample_time"))
                kind = packet.get("kind", "state")
                if packet.get("version") != VERSION or packet.get("frame") != FRAME:
                    self.rejected_invalid += 1
                    continue
                if not isinstance(peer_id, int) or peer_id not in allowed or peer_id == self.drone_id:
                    self.rejected_invalid += 1
                    continue
                expected_source = self.peer_endpoints.get(peer_id)
                if expected_source is None or source_address != expected_source:
                    self.rejected_source += 1
                    continue
                if not isinstance(session_id, str) or re.fullmatch(r"[0-9a-f]{32}", session_id) is None:
                    self.rejected_invalid += 1
                    continue
                if not isinstance(sequence, int) or sequence <= 0 or not math.isfinite(sample_time):
                    self.rejected_invalid += 1
                    continue
                # Offsets are in the shared operator time domain: offset = host_wall - operator_wall.
                # Translate sender timestamps into this node's wall-clock domain before age checks.
                sample_time_local = (sample_time - self.peer_clock_offsets_s.get(peer_id, 0.0)
                                     + self.local_clock_offset_s)
                age = float(now) - sample_time_local
                if kind == "heartbeat":
                    if age < -MAX_FUTURE_SKEW_S or age > MAX_HEARTBEAT_AGE_S:
                        self.rejected_stale += 1
                        continue
                    if not self._accept_peer_session(peer_id, session_id):
                        self.rejected_replay += 1
                        continue
                    if sequence <= self.last_heartbeat_sequence.get(peer_id, 0):
                        self.rejected_stale += 1
                        continue
                    self.last_heartbeat_sequence[peer_id] = sequence
                    self.last_peer_heartbeat_at[peer_id] = float(now)
                    continue
                if kind != "state":
                    self.rejected_invalid += 1
                    continue
                position = np.asarray(packet.get("position"), dtype=float)
                velocity = np.asarray(packet.get("velocity"), dtype=float)
                landing = packet.get("landing")
                if not math.isfinite(sample_time) or not -MAX_FUTURE_SKEW_S <= age <= MAX_PEER_AGE_S:
                    self.rejected_stale += 1
                    continue
                if position.shape != (2,) or velocity.shape != (2,):
                    self.rejected_invalid += 1
                    continue
                if not np.isfinite(position).all() or not np.isfinite(velocity).all():
                    self.rejected_invalid += 1
                    continue
                if not isinstance(landing, bool):
                    self.rejected_invalid += 1
                    continue
                if not self._accept_peer_session(peer_id, session_id):
                    self.rejected_replay += 1
                    continue
                if sequence <= self.last_sequence.get(peer_id, 0):
                    self.rejected_replay += 1
                    continue
                self.last_sequence[peer_id] = sequence
                self.delivered += 1
                received.append({
                    "id": peer_id,
                    "position": position,
                    "velocity": velocity,
                    "age": age,
                    "confidence": 1.0,
                    "landing": landing,
                })
            except (UnicodeDecodeError, json.JSONDecodeError, TypeError, ValueError, OverflowError):
                self.rejected_invalid += 1
                continue
        return received

    def set_clock_offsets(self, local_offset_s: float, peer_offsets_s: dict) -> None:
        """Apply measured host-wall offsets from the shared operator clock domain."""
        local_offset = float(local_offset_s)
        offsets = {int(peer_id): float(offset)
                   for peer_id, offset in peer_offsets_s.items()}
        if (not math.isfinite(local_offset) or
                any(not math.isfinite(offset) for offset in offsets.values())):
            raise ValueError("clock offsets must be finite")
        self.local_clock_offset_s = local_offset
        self.peer_clock_offsets_s = offsets

    def _accept_peer_session(self, peer_id: int, session_id: str) -> bool:
        """Accept a process restart once and reject packets from sessions already replaced."""
        active = self.peer_session_ids.get(peer_id)
        if active == session_id:
            return True
        retired = self.retired_peer_sessions.setdefault(peer_id, [])
        if session_id in retired:
            return False
        if active is not None:
            retired.append(active)
            if len(retired) > 64:
                del retired[0]
        self.peer_session_ids[peer_id] = session_id
        self.last_sequence.pop(peer_id, None)
        self.last_heartbeat_sequence.pop(peer_id, None)
        self.last_peer_heartbeat_at.pop(peer_id, None)
        return True

    def fresh_link_ids(self, now: float, max_age_s: float = 1.5) -> list[int]:
        """Return peers with a recent liveness packet; this says nothing about state freshness."""
        now = float(now)
        return sorted(
            peer_id for peer_id, received_at in self.last_peer_heartbeat_at.items()
            if 0.0 <= now - received_at <= max_age_s
        )

    def stats(self) -> dict:
        mean_delay = (self.delay_sample_sum_s / self.delay_sample_count
                      if self.delay_sample_count else 0.0)
        delay_variance = (max(
            0.0, self.delay_sample_sum_squares / self.delay_sample_count - mean_delay ** 2,
        ) if self.delay_sample_count else 0.0)
        return {
            "queued": self.packets_queued,
            "sent": self.packets_sent,
            "injected_delay_samples": self.delay_sample_count,
            "injected_delay_mean_s": mean_delay,
            "injected_delay_std_s": math.sqrt(delay_variance),
            "injected_delay_min_s": self.delay_sample_min_s,
            "injected_delay_max_s": self.delay_sample_max_s,
            "dropped_loss": self.dropped_loss,
            "dropped_queue": self.dropped_queue,
            "dropped_blocked": self.dropped_blocked,
            "blocked_link_changes": self.blocked_link_changes,
            "received": self.delivered,
            "rejected_stale": self.rejected_stale,
            "rejected_replay": self.rejected_replay,
            "rejected_invalid": self.rejected_invalid,
            "rejected_auth": self.rejected_auth,
            "rejected_source": self.rejected_source,
            "duplicated_packets": self.duplicated_packets,
            "reorder_delayed_packets": self.reorder_delayed_packets,
        }

    def close(self) -> None:
        self.socket.close()
