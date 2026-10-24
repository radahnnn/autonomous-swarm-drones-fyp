"""One independently connected ArduPilot SITL agent with direct peer and mission UDP."""

import argparse
import json
import math
import os
import socket
import time
import uuid

import numpy as np
from pymavlink import mavutil

from sitl.common_frame import CommonCoordinateFrame
from sitl.decentralized_guidance import DecentralizedGuidance, LocalDroneAgent
from sitl.fleet import DATUM_ALT, DATUM_LAT, DATUM_LON, build_fleet
from sitl.flight_prep import arm_with_retry, request_message_interval, takeoff_and_verify, wait_ekf_ready
from sitl.formation_library import slots_for
from sitl.mission_intent import MissionIntent, MissionLease
from sitl.operator_udp import NodeOperatorSocket, encode_message
from sitl.peer_udp import PeerLinkWatchdog, UdpPeerStateTransport
from sitl.peer_roster import load_auth_key, load_peer_roster
from sitl.adaptive_membership import AdaptiveMembership


OPERATOR_VERSION = 1
STATUS_PORT_BASE = 22000
NODE_COMMAND_PORT_BASE = 21000
PEER_PORT_BASE = 20000
_TYPE_MASK_POS_YAW = 0b0000101111111000


class IndependentAgentNode:
    """Owns one MAVLink connection, local guidance loop, and direct peer datagrams."""

    COORDINATOR_TIMEOUT_S = 2.5

    def __init__(self, drone_id: int, count: int, endpoint: str, formation: str = "v",
                 control_mode: str = "decentralized",
                 peer_loss: float = 0.0, peer_latency_ms: float = 30.0,
                 peer_latency_jitter_ms: float | None = None,
                 peer_seed: int | None = None, peer_burst_loss: bool = False,
                 peer_burst_start_rate: float = 0.04,
                 peer_burst_continue_rate: float = 0.65,
                 blocked_links=(), peer_duplicate_rate: float = 0.0,
                 peer_reorder_rate: float = 0.0, peer_reorder_delay_ms: float = 150.0,
                 peer_roster_path=None, peer_auth_key: bytes | None = None,
                 peer_port_base: int = PEER_PORT_BASE,
                 command_port_base: int = NODE_COMMAND_PORT_BASE,
                 status_port_base: int = STATUS_PORT_BASE,
                 baudrate: int = 115200, mavlink_system_id: int | None = None,
                 adaptive_swarm: bool = False, obstacles=()):
        self.drone_id = int(drone_id)
        self.count = int(count)
        self.endpoint = endpoint
        self.baudrate = int(baudrate)
        self.mavlink_system_id = (self.drone_id + 1 if mavlink_system_id is None
                                  else int(mavlink_system_id))
        if not 1 <= self.baudrate <= 4_000_000:
            raise ValueError("MAVLink baud rate must be between 1 and 4000000")
        if not 1 <= self.mavlink_system_id <= 255:
            raise ValueError("MAVLink system ID must be between 1 and 255")
        self.formation = formation
        if control_mode not in ("decentralized", "hybrid"):
            raise ValueError("independent agent control mode must be decentralized or hybrid")
        self.control_mode = control_mode
        if not 0 <= self.drone_id < self.count:
            raise ValueError("agent ID must be within the configured swarm")
        self.all_slots = np.asarray(slots_for(formation, count), dtype=float)
        topology = DecentralizedGuidance._one_hop_graph(
            [np.asarray(slot, dtype=float) for slot in self.all_slots], [True] * count,
        )
        self.slot = self.all_slots[self.drone_id].copy()
        self.peer_ids = sorted(topology.get(self.drone_id, ()))
        self.peer_watchdog = PeerLinkWatchdog(
            self.peer_ids, timeout_s=2.0, startup_grace_s=5.0,
        )
        self.peer_timeout_ids = []
        self.adaptive_swarm = bool(adaptive_swarm)
        self.membership = (AdaptiveMembership(self.drone_id, self.count, formation)
                           if self.adaptive_swarm else None)
        self.discovery_ids = (self.membership.discovery_ids if self.membership is not None
                              else list(self.peer_ids))
        self.elected_leader_id = 0
        self.membership_revision = 0
        self.obstacles = [tuple(float(value) for value in obstacle) for obstacle in obstacles]
        if any(len(obstacle) != 3 or not all(math.isfinite(v) for v in obstacle)
               or obstacle[2] <= 0 for obstacle in self.obstacles):
            raise ValueError("obstacles must be finite north,east,radius triples")
        self.peer_roster = load_peer_roster(peer_roster_path, count, peer_port_base)
        if peer_roster_path is None:
            # Honour every caller-selected range. The generated roster otherwise
            # falls back to production command/status ports and prevents parallel
            # tests or a second local deployment.
            for agent_id, entry in self.peer_roster.items():
                entry["command_port"] = int(command_port_base) + agent_id
                entry["status_port"] = int(status_port_base) + agent_id
        self.peer_roster_path = str(peer_roster_path) if peer_roster_path is not None else None
        if peer_latency_jitter_ms is None:
            peer_latency_jitter_ms = 5.0 if peer_latency_ms > 0 else 0.0
        if (not math.isfinite(peer_latency_ms) or peer_latency_ms < 0 or
                not math.isfinite(peer_latency_jitter_ms) or peer_latency_jitter_ms < 0):
            raise ValueError("peer latency mean and jitter must be finite and non-negative")
        self.peer_latency_jitter_ms = float(peer_latency_jitter_ms)
        self.frame = CommonCoordinateFrame(DATUM_LAT, DATUM_LON, DATUM_ALT)
        self.agent = LocalDroneAgent(
            self.drone_id, safe_radius=3.0, hybrid=control_mode == "hybrid",
        )
        seed = None if peer_seed is None else int(peer_seed) + self.drone_id * 65537
        self.peers = UdpPeerStateTransport(
            self.drone_id, peer_port_base, count,
            packet_loss_rate=peer_loss,
            latency_mean=peer_latency_ms / 1000.0,
            latency_std=self.peer_latency_jitter_ms / 1000.0,
            seed=seed,
            burst_loss=peer_burst_loss,
            burst_start_rate=peer_burst_start_rate,
            burst_continue_rate=peer_burst_continue_rate,
            blocked_links=blocked_links,
            duplicate_rate=peer_duplicate_rate,
            reorder_rate=peer_reorder_rate,
            reorder_delay=peer_reorder_delay_ms / 1000.0,
            bind_address=(self.peer_roster[self.drone_id]["bind_host"],
                          self.peer_roster[self.drone_id]["port"]),
            peer_endpoints={agent_id: (entry["host"], entry["port"])
                            for agent_id, entry in self.peer_roster.items()},
            auth_key=peer_auth_key,
        )
        operator_cfg = self.peer_roster[self.drone_id]
        self.operator = NodeOperatorSocket(
            self.drone_id, command_port_base,
            bind_host=operator_cfg["command_bind_host"],
            port=operator_cfg["command_port"], auth_key=peer_auth_key,
        )
        self.status_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.status_port_base = int(status_port_base)
        self.status_endpoint = (operator_cfg["status_host"], operator_cfg["status_port"])
        self.operator_auth_key = peer_auth_key
        self.conn = None
        self.position = None
        self.position_at = 0.0
        self.local_position = None
        self.global_origin = None
        self.last_heartbeat_at = 0.0
        self.last_gcs_heartbeat_at = 0.0
        self.last_peer_heartbeat_at = 0.0
        self.last_operator_sequence = 0
        self.last_operator_action = None
        self.status_sequence = 0
        self.status_session_id = uuid.uuid4().hex
        self.status_session_started_at = time.time()
        self.last_network_fault_sequence = 0
        self.last_mission_sequence = 0
        self.last_clock_sync_sequence = 0
        self.clock_sync_epoch = None
        self.retired_clock_sync_epochs = []
        self.operator_clock_offset_s = 0.0
        self.peer_clock_offsets_s = {index: 0.0 for index in range(self.count)}
        self.current_mission = None
        self.mission_lease = MissionLease()
        self.pending_intent = None
        self.last_coordinator_message_monotonic = 0.0
        self.coordinator_connected = False
        self.airborne = False
        self.armed = False
        self.landing = False
        self.recovery_state = "normal"
        self.command_in_progress = False
        self.closed = False
        self.last_status_at = 0.0
        self.last_command = "booting"
        self.safety_state = "nominal"
        self.safety_reason = None

    def connect(self, timeout: float = 30.0) -> None:
        if self.endpoint.startswith("tcp:"):
            address, raw_port = self.endpoint[4:].rsplit(":", 1)
            port = int(raw_port)
            deadline = time.monotonic() + timeout
            last_error = None
            while time.monotonic() < deadline:
                try:
                    with socket.create_connection((address, port), timeout=0.5):
                        break
                except OSError as exc:
                    last_error = exc
                    time.sleep(0.25)
            else:
                raise TimeoutError(
                    f"agent {self.drone_id + 1}: MAVLink endpoint {self.endpoint} "
                    f"unavailable: {last_error}"
                )
            self.conn = mavutil.mavlink_connection(
                self.endpoint, retries=1, reconnect_delay=0,
            )
        elif self.endpoint.startswith(("udp:", "udpin:", "udpout:")):
            self.conn = mavutil.mavlink_connection(
                self.endpoint, retries=1, reconnect_delay=0,
            )
        else:
            if not os.path.exists(self.endpoint):
                raise FileNotFoundError(
                    f"agent {self.drone_id + 1}: serial MAVLink device "
                    f"{self.endpoint} does not exist"
                )
            self.conn = mavutil.mavlink_connection(
                self.endpoint, baud=self.baudrate, autoreconnect=True,
            )
        heartbeat = self.conn.wait_heartbeat(timeout=5.0)
        if heartbeat is None:
            raise TimeoutError(f"agent {self.drone_id + 1}: no MAVLink heartbeat at {self.endpoint}")
        if self.conn.target_system != self.mavlink_system_id:
            raise RuntimeError(
                f"agent {self.drone_id + 1}: endpoint reports MAVLink system "
                f"{self.conn.target_system}, expected {self.mavlink_system_id}"
            )
        self.conn.mav.request_data_stream_send(
            self.conn.target_system, self.conn.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL, 20, 1,
        )
        request_message_interval(self.conn, 193, 2.0)
        request_message_interval(self.conn, 24, 2.0)
        self._send_gcs_heartbeat()
        if not wait_ekf_ready(self.conn, f"Node {self.drone_id + 1}", wall_timeout=300.0,
                              heartbeat_callback=self._send_gcs_heartbeat):
            raise RuntimeError(f"agent {self.drone_id + 1}: EKF did not become ready")
        self.last_heartbeat_at = time.time()
        self.last_command = "ready"
        print(f"[NODE {self.drone_id + 1}] independently connected to {self.endpoint}", flush=True)

    def _send_gcs_heartbeat(self) -> None:
        """Identify this per-aircraft agent to ArduPilot for its GCS-loss failsafe."""
        if self.conn is None:
            return
        now = time.monotonic()
        if now - self.last_peer_heartbeat_at >= 0.5:
            self.peers.publish_heartbeat(self.discovery_ids, time.time())
            self.last_peer_heartbeat_at = now
        if now - self.last_gcs_heartbeat_at >= 1.0:
            self.conn.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GCS,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                0, 0, mavutil.mavlink.MAV_STATE_ACTIVE,
            )
            self.last_gcs_heartbeat_at = now

    def _read_vehicle(self) -> None:
        while True:
            msg = self.conn.recv_match(blocking=False)
            if msg is None:
                break
            kind = msg.get_type()
            if kind == "HEARTBEAT":
                self.last_heartbeat_at = time.time()
                self.armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                if not self.armed:
                    self.airborne = False
            elif kind == "LOCAL_POSITION_NED":
                self.local_position = np.array([msg.x, msg.y, msg.z], dtype=float)
            elif kind == "GLOBAL_POSITION_INT":
                self.position = self.frame.gps_to_global_ned(
                    msg.lat / 1e7, msg.lon / 1e7, msg.relative_alt / 1000.0,
                )
                self.position_at = time.time()
            if self.position is not None and self.local_position is not None and self.global_origin is None:
                self.global_origin = self.position - self.local_position
        if (self.landing and not self.armed and self.position is not None
                and self.position[2] > -0.5):
            self.airborne = False
            self.landing = False
            if self.recovery_state == "landing_after_process_restart":
                self.recovery_state = "landed_after_process_restart"
            self.last_command = "landed"
        elif (self.armed and self.position is not None and self.position[2] < -0.5
              and not self.airborne):
            # A restarted process has lost its controller and mission history. Land safely
            # instead of silently taking over the old flight; peers propagate this request.
            self.airborne = True
            self.recovery_state = "landing_after_process_restart"
            self.safety_state = "agent_restart_failsafe"
            self.safety_reason = "agent_process_restarted_while_airborne"
            self._command_land(self.safety_reason)

    def _receive_operator_messages(self) -> None:
        for message in self.operator.poll():
            if message.get("version") != OPERATOR_VERSION:
                continue
            kind = message.get("type")
            if kind in ("command", "network_fault"):
                try:
                    issued_at = float(message["issued_at"])
                    expires_at = float(message["expires_at"])
                    wall_now = time.time() - self.operator_clock_offset_s
                    if (not math.isfinite(issued_at) or not math.isfinite(expires_at) or
                            issued_at > wall_now + 0.25 or expires_at <= wall_now or
                            expires_at <= issued_at or expires_at - issued_at > 10.0):
                        continue
                except (KeyError, TypeError, ValueError):
                    continue
            if kind == "clock_probe":
                probe_id = message.get("probe_id")
                if not isinstance(probe_id, str) or not probe_id or len(probe_id) > 128:
                    continue
                agent_receive_monotonic = time.monotonic()
                agent_received_at = time.time()
                agent_sent_at = time.time()
                agent_send_monotonic = time.monotonic()
                response = {
                    "version": OPERATOR_VERSION,
                    "type": "clock_probe_response",
                    "drone_id": self.drone_id,
                    "probe_id": probe_id,
                    "agent_received_at": agent_received_at,
                    "agent_sent_at": agent_sent_at,
                    "agent_processing_s": max(
                        0.0, agent_send_monotonic - agent_receive_monotonic,
                    ),
                }
                self.status_socket.sendto(
                    encode_message(response, self.operator_auth_key), self.status_endpoint,
                )
            elif kind == "clock_sync":
                sequence = message.get("sequence")
                epoch = message.get("epoch")
                raw_offsets = message.get("offsets_s")
                if (not isinstance(sequence, int) or isinstance(sequence, bool) or
                        sequence < 1 or
                        not isinstance(epoch, str) or not epoch or len(epoch) > 128 or
                        not isinstance(raw_offsets, dict)):
                    continue
                if epoch == self.clock_sync_epoch:
                    if sequence <= self.last_clock_sync_sequence:
                        continue
                elif epoch in self.retired_clock_sync_epochs:
                    continue
                try:
                    offsets = {int(agent_id): float(offset)
                               for agent_id, offset in raw_offsets.items()}
                except (TypeError, ValueError):
                    continue
                if (set(offsets) != set(range(self.count)) or
                        not all(math.isfinite(offset) for offset in offsets.values())):
                    continue
                if epoch != self.clock_sync_epoch:
                    if self.clock_sync_epoch is not None:
                        self.retired_clock_sync_epochs.append(self.clock_sync_epoch)
                        if len(self.retired_clock_sync_epochs) > 64:
                            del self.retired_clock_sync_epochs[0]
                    self.last_clock_sync_sequence = 0
                    # Sequence numbers are scoped to the authenticated operator epoch.
                    # A replacement console starts its counters over; retired-epoch
                    # messages are rejected below, so resetting these counters is safe.
                    self.last_mission_sequence = 0
                    self.last_operator_sequence = 0
                    self.last_operator_action = None
                    self.last_network_fault_sequence = 0
                self.last_clock_sync_sequence = sequence
                self.clock_sync_epoch = epoch
                self.peer_clock_offsets_s = offsets
                self.operator_clock_offset_s = offsets[self.drone_id]
                self.peers.set_clock_offsets(self.operator_clock_offset_s, offsets)
            elif kind == "mission":
                if (not isinstance(message.get("epoch"), str) or
                        message.get("epoch") != self.clock_sync_epoch):
                    continue
                try:
                    wall_now = time.time() - self.operator_clock_offset_s
                    intent = MissionIntent.from_dict(message.get("intent"), wall_now)
                except (TypeError, ValueError):
                    continue
                if intent.sequence <= self.last_mission_sequence:
                    continue
                self.mission_lease.accept(intent.to_dict(), wall_now, time.monotonic())
                self.last_mission_sequence = intent.sequence
                self.current_mission = intent.to_dict()
                self.pending_intent = self.current_mission
                self.last_coordinator_message_monotonic = time.monotonic()
                self.coordinator_connected = True
                # Keep terminal landing status visible after touchdown. The operator refreshes
                # mission intent continuously, but a fresh mission does not imply takeoff.
                if not self.landing and not (not self.airborne and self.last_command == "landed"):
                    self.last_command = f"mission:{intent.sequence}"
            elif kind == "network_fault":
                if (not isinstance(message.get("epoch"), str) or
                        message.get("epoch") != self.clock_sync_epoch):
                    continue
                sequence = message.get("sequence")
                sender = message.get("sender")
                recipient = message.get("recipient")
                blocked = message.get("blocked")
                if (not isinstance(sequence, int) or sequence <= self.last_network_fault_sequence
                        or sender != self.drone_id or not isinstance(blocked, bool)):
                    continue
                try:
                    changed = self.peers.set_link_blocked(sender, recipient, blocked)
                except (TypeError, ValueError):
                    continue
                self.last_network_fault_sequence = sequence
                if changed:
                    self.last_command = f"peer_link_{'blocked' if blocked else 'restored'}:{recipient}"
            elif kind == "command":
                epoch = message.get("epoch")
                if not isinstance(epoch, str) or epoch != self.clock_sync_epoch:
                    continue
                sequence = message.get("sequence")
                action = message.get("action")
                if (not isinstance(sequence, int) or isinstance(sequence, bool) or sequence < 1 or
                        action not in ("launch", "land")):
                    continue
                if sequence == self.last_operator_sequence:
                    if action == self.last_operator_action:
                        self._send_command_ack(sequence, action, epoch)
                    continue
                if sequence < self.last_operator_sequence:
                    continue
                self.last_operator_sequence = sequence
                self.last_operator_action = action
                # Acknowledge receipt before running potentially blocking arm/takeoff helpers.
                self._send_command_ack(sequence, action, epoch)
                if action == "launch" and not self.airborne:
                    preflight_failure = self._launch_preflight_failure()
                    if preflight_failure is not None:
                        self.last_command = preflight_failure
                        continue
                    self.last_command = "launching"
                    self.command_in_progress = True
                    try:
                        self.armed = arm_with_retry(self.conn, f"Node {self.drone_id + 1}", timeout=120.0,
                                                    force_after=121.0,
                                                    heartbeat_callback=self._send_gcs_heartbeat)
                        if self.armed:
                            alt = float(message.get("target_alt_m", 5.0))
                            self.airborne = takeoff_and_verify(
                                self.conn, f"Node {self.drone_id + 1}", alt,
                                heartbeat_callback=self._send_gcs_heartbeat,
                            )
                            self.last_command = "airborne" if self.airborne else "takeoff_failed"
                    finally:
                        # The blocking helper drains MAVLink while doing its own checks.
                        # Refresh the watchdog anchor so its intentional wait is not
                        # mistaken for a lost radio/telemetry link on return.
                        self.last_heartbeat_at = time.time()
                        self.position_at = time.time()
                        if self.membership is not None:
                            self.membership.resume_after_local_pause(time.time())
                        self.command_in_progress = False
                elif action == "land":
                    self._command_land("operator")

    def _send_command_ack(self, sequence: int, action: str, epoch: str) -> None:
        ack = {
            "version": OPERATOR_VERSION,
            "type": "command_ack",
            "drone_id": self.drone_id,
            "sequence": int(sequence),
            "action": action,
            "epoch": epoch,
            "received_at": time.time(),
        }
        self.status_socket.sendto(encode_message(ack, self.operator_auth_key),
                                  self.status_endpoint)

    def _command_land(self, reason: str) -> None:
        if not self.landing:
            self.conn.set_mode(9)
            self.conn.mav.command_long_send(
                self.conn.target_system, self.conn.target_component,
                mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0, 0, 0, 0, 0, 0,
            )
            self.landing = True
            self.last_command = f"landing:{reason}"

    def _launch_preflight_failure(self) -> str | None:
        """Require an accepted mission and live required neighbors before arming."""
        if self.mission_lease.active(time.monotonic()) is None:
            return "launch_rejected_no_valid_mission"
        now = time.time()
        # Drain datagrams already in the socket before deciding preflight connectivity.
        self.peers.receive(now, self.peer_ids)
        fresh_peers = set(self.peers.fresh_link_ids(now))
        missing_peers = sorted(set(self.peer_ids) - fresh_peers)
        if missing_peers:
            missing = ",".join(str(peer_id + 1) for peer_id in missing_peers)
            return f"launch_rejected_peer_links:{missing}"
        return None

    def _check_peer_failsafe(self, now: float, fresh_peer_ids, peer_states=()) -> None:
        """Degrade around one lost edge; land only when this node is isolated."""
        if not self.airborne:
            # Learn which configured peers were actually reachable before takeoff, but do
            # not age a missing link into a preflight emergency.
            self.peer_watchdog.update(now, fresh_peer_ids)
            self.peer_watchdog.deactivate()
            self.peer_timeout_ids = []
            return
        # A peer's local LAND is state information, not a swarm-wide command.  Copying
        # it through the graph made one failed directed link cascade into a fleet-wide
        # landing.  Keep its slot reserved and let each remaining node independently
        # assess whether it still has a live route.
        peers_landing = sorted({int(peer["id"]) for peer in peer_states
                                if peer.get("landing") is True})
        if peers_landing and not self.landing:
            ids = ",".join(str(peer_id + 1) for peer_id in peers_landing)
            self.safety_state = "peer_landing_observed"
            self.safety_reason = f"peer_landing_observed:{ids}"
        if not self.peer_watchdog.monitoring:
            self.peer_watchdog.activate(now, fresh_peer_ids)
        self.peer_timeout_ids = self.peer_watchdog.update(
            now, fresh_peer_ids,
        )
        if self.peer_timeout_ids and not self.landing:
            missing = ",".join(str(peer_id + 1) for peer_id in self.peer_timeout_ids)
            required_ids = getattr(self, "peer_ids", self.peer_watchdog.peer_ids)
            live_required = sorted(set(required_ids) & set(int(value)
                                                             for value in fresh_peer_ids))
            if live_required:
                # Guidance already consumes only fresh received neighbors, so this is
                # the local topology reconfiguration: remove the failed edge and keep
                # controlling through the remaining one-hop links.
                self.safety_state = "peer_link_degraded"
                self.safety_reason = f"peer_link_degraded:{missing}"
                self.last_command = self.safety_reason
            else:
                self.safety_state = "peer_isolation_failsafe"
                self.safety_reason = f"isolated_peer_timeout:{missing}"
                self._command_land(self.safety_reason)
        elif (not self.peer_timeout_ids and not peers_landing and not self.landing and
              self.safety_state in {"peer_link_degraded", "peer_landing_observed"}):
            self.safety_state = "nominal"
            self.safety_reason = None
            self.last_command = "peer_links_restored"

    def _send_position_target(self, target_global: np.ndarray, altitude_m: float) -> None:
        if self.global_origin is None:
            return
        target = np.asarray(target_global, dtype=float).copy()
        target[2] = -float(altitude_m)
        local = target - self.global_origin
        self.conn.mav.set_position_target_local_ned_send(
            int(time.monotonic() * 1000) & 0xFFFFFFFF,
            self.conn.target_system, self.conn.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            _TYPE_MASK_POS_YAW,
            float(local[0]), float(local[1]), float(local[2]),
            0, 0, 0, 0, 0, 0, 0.0, 0,
        )

    def _run_guidance(self, intent: MissionIntent, now: float, tick: float,
                      last_loop: float, neighbors,
                      coordinator_connected: bool = True) -> None:
        """Compute one guidance update, converting invalid intent into a safe landing."""
        # Mission intents are timestamped in the operator clock domain. The lease was
        # accepted using this same offset, so cached-intent validation must not compare
        # an operator timestamp with the agent's raw wall clock.
        mission_now = now - self.operator_clock_offset_s
        try:
            target = self.agent.command(
                self.position[:2], self.position_at, None, neighbors,
                {peer: self.slot[:2] - self.all_slots[peer][:2]
                 for peer in self.peer_ids},
                min(0.25, max(0.02, tick - last_loop)), now=mission_now,
                mission_intent=self.pending_intent,
                slot_offset=self.slot[:2],
                coordinator_connected=coordinator_connected,
            )
        except (TypeError, ValueError) as exc:
            # An airborne agent must never exit merely because mission data became
            # invalid between receive and guidance. Stop using it and command LAND.
            self.mission_lease.clear()
            self.current_mission = None
            self.pending_intent = None
            self.safety_state = "mission_failsafe"
            self.safety_reason = f"invalid_mission_intent:{exc}"
            self._command_land("invalid_mission_intent")
            return
        self.pending_intent = None
        target_global = np.array([target[0], target[1], self.position[2]], dtype=float)
        target_global[:2] = self._avoid_obstacles(target_global[:2])
        self._send_position_target(target_global, intent.target_alt_m + self.slot[2])

    def _avoid_obstacles(self, target_ne: np.ndarray) -> np.ndarray:
        """Apply a bounded local repulsion from configured circular keep-out zones."""
        target = np.asarray(target_ne, dtype=float).copy()
        if self.position is None:
            return target
        position = np.asarray(self.position[:2], dtype=float)
        correction = np.zeros(2, dtype=float)
        for north, east, radius in getattr(self, "obstacles", ()):
            delta = position - np.array([north, east], dtype=float)
            distance = float(np.linalg.norm(delta))
            influence = radius + 3.0
            if distance >= influence:
                continue
            direction = delta / max(distance, 0.10)
            correction += direction * min(2.0, influence - distance)
        return target + correction

    def _update_adaptive_membership(self, now: float, fresh_ids) -> None:
        if self.membership is None:
            return
        if self.membership.update(now, fresh_ids):
            self.membership_revision += 1
        slot_map = self.membership.slot_map()
        self.peer_ids = self.membership.neighbors_for()
        self.slot = slot_map[self.drone_id].copy()
        self.all_slots = np.asarray([
            slot_map.get(agent_id, self.all_slots[agent_id])
            for agent_id in range(self.count)
        ], dtype=float)
        self.elected_leader_id = self.membership.leader_id
        if len(self.membership.active_ids) == 1 and self.airborne and not self.landing:
            self.safety_state = "peer_isolation_failsafe"
            self.safety_reason = "adaptive_membership_isolated"
            self._command_land(self.safety_reason)
        elif self.safety_state in {"peer_link_degraded", "peer_landing_observed"}:
            self.safety_state = "nominal"
            self.safety_reason = None

    def _send_status(self, now: float) -> None:
        if now - self.last_status_at < 1.0:
            return
        self.last_status_at = now
        self.status_sequence += 1
        status = {
            "version": 1,
            "type": "status",
            "drone_id": self.drone_id,
            "timestamp": now,
            "sequence": self.status_sequence,
            "session_id": self.status_session_id,
            "session_started_at": self.status_session_started_at,
            "clock_sync_epoch": self.clock_sync_epoch,
            "operator_clock_offset_s": self.operator_clock_offset_s,
            "position": None if self.position is None else self.position.tolist(),
            "position_age_s": None if not self.position_at else max(0.0, now - self.position_at),
            "airborne": bool(self.position is not None and self.position[2] < -0.5),
            "armed": self.armed,
            "peer_ids": self.peer_ids,
            "discovery_ids": self.discovery_ids,
            "peer_live_ids": self.peers.fresh_link_ids(now),
            "peer_timeout_ids": self.peer_timeout_ids,
            "safety_state": self.safety_state,
            "safety_reason": self.safety_reason,
            "adaptive_swarm": self.adaptive_swarm,
            "active_member_ids": (sorted(self.membership.active_ids)
                                  if self.membership is not None else list(range(self.count))),
            "elected_leader_id": self.elected_leader_id,
            "membership_revision": self.membership_revision,
            "obstacle_count": len(self.obstacles),
            "recovery_state": self.recovery_state,
            "mission_sequence": self.last_mission_sequence,
            "mission_active": self.mission_lease.active(time.monotonic()) is not None,
            "control_mode": self.control_mode,
            "coordinator_connected": (self.coordinator_connected
                                       if self.control_mode == "hybrid" else None),
            "hybrid_mode": (
                self.agent.hybrid_controller.get_drone_mode(self.drone_id).value
                if self.agent.hybrid_controller is not None else None
            ),
            "hybrid_alpha": (
                self.agent.hybrid_controller.get_alpha(self.drone_id)
                if self.agent.hybrid_controller is not None else None
            ),
            # Expose the currently accepted shared intent so a replacement operator can
            # resume the same target instead of silently resetting the swarm to home.
            "mission_intent": self.current_mission,
            "command": self.last_command,
            "peer_metrics": self.peers.stats(),
        }
        self.status_socket.sendto(encode_message(status, self.operator_auth_key),
                                  self.status_endpoint)

    def run(self, duration_s: float | None = None) -> None:
        self.connect()
        started = time.monotonic()
        last_loop = time.monotonic()
        try:
            while not self.closed:
                now = time.time()
                tick = time.monotonic()
                if duration_s is not None and tick - started >= duration_s:
                    break
                self._send_gcs_heartbeat()
                self._read_vehicle()
                self._receive_operator_messages()
                # Launch/land handling may block while ArduPilot arms or takes off.
                # Do not use the timestamp captured before that operation for peer
                # expiry, mission leases, telemetry age, or guidance dt.
                now = time.time()
                tick = time.monotonic()
                if self.control_mode == "hybrid":
                    self.coordinator_connected = (
                        self.last_coordinator_message_monotonic > 0 and
                        time.monotonic() - self.last_coordinator_message_monotonic
                        <= self.COORDINATOR_TIMEOUT_S
                    )
                if self.position is None or self.position_at <= 0:
                    time.sleep(0.02)
                    continue
                if not self.command_in_progress and (
                    now - self.position_at > 1.0 or now - self.last_heartbeat_at > 3.5
                ):
                    self._command_land("telemetry_timeout")
                intent = self.mission_lease.active(time.monotonic())
                if intent is None and self.current_mission is not None:
                    self.current_mission = None
                    self.pending_intent = None
                    self._command_land("mission_expired")
                elif intent is None:
                    self.current_mission = None
                self.agent.update_local_state(self.position[:2], self.position_at)
                self.peers.publish(self.position[:2], self.agent.velocity,
                                   self.position_at, self.discovery_ids, now,
                                   landing=self.landing)
                discovered = self.peers.receive(now, self.discovery_ids)
                fresh_peer_ids = self.peers.fresh_link_ids(now)
                self._update_adaptive_membership(now, fresh_peer_ids)
                neighbors = [peer for peer in discovered if peer["id"] in self.peer_ids]
                if self.membership is None:
                    self._check_peer_failsafe(now, fresh_peer_ids, neighbors)
                if self.airborne and intent is not None and not self.landing:
                    self._run_guidance(
                        intent, now, tick, last_loop, neighbors,
                        coordinator_connected=(self.control_mode != "hybrid" or
                                              self.coordinator_connected),
                    )
                elif self.airborne and self.landing:
                    heartbeat = self.conn.recv_match(type="HEARTBEAT", blocking=False)
                    if heartbeat and not (heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
                        self.airborne = False
                        self.landing = False
                        self.last_command = "landed"
                self._send_status(now)
                last_loop = tick
                time.sleep(max(0.0, 0.1 - (time.monotonic() - tick)))
        finally:
            self.close()

    def close(self) -> None:
        if self.closed:
            return
        self.closed = True
        self.operator.close()
        self.peers.close()
        self.status_socket.close()
        if self.conn is not None:
            self.conn.close()


def main(argv=None):
    parser = argparse.ArgumentParser(description="Independent per-drone ArduPilot SITL agent")
    parser.add_argument("--id", type=int, required=True, help="zero-based drone ID")
    parser.add_argument("--count", type=int, required=True)
    parser.add_argument("--endpoint", required=True,
                        help="TCP SITL endpoint or serial MAVLink device path")
    parser.add_argument("--baudrate", type=int, default=115200,
                        help="MAVLink baud rate for serial endpoints")
    parser.add_argument("--mavlink-system-id", type=int, default=None,
                        help="expected ArduPilot SYSID_THISMAV; defaults to zero-based agent ID + 1")
    parser.add_argument("--adaptive-swarm", action="store_true",
                        help="discover membership, repair a one-hop ring, elect a leader and redistribute slots")
    parser.add_argument("--obstacle", action="append", default=[], metavar="N,E,R",
                        help="circular local keep-out zone in shared north/east metres; repeatable")
    parser.add_argument("--formation", default="v")
    parser.add_argument("--control-mode", choices=("decentralized", "hybrid"),
                        default="decentralized")
    parser.add_argument("--peer-loss", type=float, default=0.0)
    parser.add_argument("--peer-latency-ms", type=float, default=0.0)
    parser.add_argument("--peer-jitter-ms", type=float, default=None,
                        help="standard deviation of per-packet peer delay in milliseconds; default 5 when mean delay is nonzero")
    parser.add_argument("--peer-seed", type=int, default=None)
    parser.add_argument("--peer-burst-loss", action="store_true")
    parser.add_argument("--peer-burst-start-rate", type=float, default=0.04,
                        help="probability of entering a packet-loss burst")
    parser.add_argument("--peer-burst-continue-rate", type=float, default=0.65,
                        help="probability that a packet-loss burst continues")
    parser.add_argument("--peer-duplicate-rate", type=float, default=0.0,
                        help="independent-runtime SITL only: duplicate probability per peer datagram")
    parser.add_argument("--peer-reorder-rate", type=float, default=0.0,
                        help="independent-runtime SITL only: probability of adding variable delay that can reorder packets")
    parser.add_argument("--peer-reorder-delay-ms", type=float, default=150.0,
                        help="extra delay scale used when --peer-reorder-rate triggers")
    parser.add_argument("--peer-roster", default=None,
                        help="shared JSON roster of each agent's peer bind and advertised UDP address")
    parser.add_argument("--peer-auth-key-file", default=None,
                        help="path to shared peer HMAC key (minimum 32 bytes); absent means unauthenticated dev mode")
    parser.add_argument("--peer-block", action="append", default=[],
                        help="drop one directed peer edge using zero-based IDs, e.g. 1:3")
    parser.add_argument("--peer-port-base", type=int, default=PEER_PORT_BASE)
    parser.add_argument("--command-port-base", type=int, default=NODE_COMMAND_PORT_BASE)
    parser.add_argument("--status-port-base", type=int, default=STATUS_PORT_BASE)
    parser.add_argument("--duration", type=float, default=None)
    args = parser.parse_args(argv)
    if (not math.isfinite(args.peer_duplicate_rate) or
            not 0.0 <= args.peer_duplicate_rate <= 1.0):
        parser.error("--peer-duplicate-rate must be finite and from 0 to 1")
    if (not math.isfinite(args.peer_burst_start_rate) or
            not 0.0 <= args.peer_burst_start_rate <= 1.0 or
            not math.isfinite(args.peer_burst_continue_rate) or
            not 0.0 <= args.peer_burst_continue_rate <= 1.0):
        parser.error("--peer-burst-start-rate and --peer-burst-continue-rate must be finite and from 0 to 1")
    if (not math.isfinite(args.peer_reorder_rate) or
            not 0.0 <= args.peer_reorder_rate <= 1.0):
        parser.error("--peer-reorder-rate must be finite and from 0 to 1")
    if (not math.isfinite(args.peer_loss) or not 0.0 <= args.peer_loss <= 1.0 or
            not math.isfinite(args.peer_latency_ms) or args.peer_latency_ms < 0):
        parser.error("--peer-loss must be 0..1 and --peer-latency-ms finite and non-negative")
    if (args.peer_jitter_ms is not None and
            (not math.isfinite(args.peer_jitter_ms) or args.peer_jitter_ms < 0)):
        parser.error("--peer-jitter-ms must be finite and non-negative")
    if not math.isfinite(args.peer_reorder_delay_ms) or args.peer_reorder_delay_ms < 0:
        parser.error("--peer-reorder-delay-ms must be finite and non-negative")
    if not 1 <= args.baudrate <= 4_000_000:
        parser.error("--baudrate must be between 1 and 4000000")
    if args.mavlink_system_id is not None and not 1 <= args.mavlink_system_id <= 255:
        parser.error("--mavlink-system-id must be between 1 and 255")
    try:
        auth_key = load_auth_key(args.peer_auth_key_file)
        load_peer_roster(args.peer_roster, args.count, args.peer_port_base)
    except ValueError as exc:
        parser.error(str(exc))
    blocked_links = []
    for raw_link in args.peer_block:
        try:
            sender, recipient = (int(value) for value in raw_link.split(":"))
        except (TypeError, ValueError):
            parser.error("--peer-block must be a directed zero-based edge like 1:3")
        if (sender < 0 or recipient < 0 or sender >= args.count or recipient >= args.count
                or sender == recipient):
            parser.error("--peer-block IDs must be different valid zero-based drone IDs")
        blocked_links.append((sender, recipient))
    obstacles = []
    for raw_obstacle in args.obstacle:
        try:
            obstacle = tuple(float(value) for value in raw_obstacle.split(","))
        except (TypeError, ValueError):
            parser.error("--obstacle must be north,east,radius, for example 5,0,2")
        if (len(obstacle) != 3 or not all(math.isfinite(value) for value in obstacle)
                or obstacle[2] <= 0):
            parser.error("--obstacle must contain finite north,east and positive radius")
        obstacles.append(obstacle)
    node = IndependentAgentNode(
        args.id, args.count, args.endpoint, args.formation,
        control_mode=args.control_mode,
        peer_loss=args.peer_loss, peer_latency_ms=args.peer_latency_ms,
        peer_latency_jitter_ms=args.peer_jitter_ms,
        peer_seed=args.peer_seed, peer_burst_loss=args.peer_burst_loss,
        peer_burst_start_rate=args.peer_burst_start_rate,
        peer_burst_continue_rate=args.peer_burst_continue_rate,
        peer_duplicate_rate=args.peer_duplicate_rate,
        peer_reorder_rate=args.peer_reorder_rate,
        peer_reorder_delay_ms=args.peer_reorder_delay_ms,
        peer_roster_path=args.peer_roster,
        peer_auth_key=auth_key,
        blocked_links=blocked_links,
        peer_port_base=args.peer_port_base,
        command_port_base=args.command_port_base,
        status_port_base=args.status_port_base,
        baudrate=args.baudrate,
        mavlink_system_id=args.mavlink_system_id,
        adaptive_swarm=args.adaptive_swarm,
        obstacles=obstacles,
    )
    node.run(args.duration)


if __name__ == "__main__":
    main()
