"""Operator shell for agents that own their individual SITL MAVLink connections."""

from datetime import datetime, timezone
import json
import math
from pathlib import Path
import socket
import subprocess
import sys
import threading
import time
import uuid

import numpy as np
from pymavlink import mavutil

from sitl.fleet import build_fleet
from sitl.formation_library import STYLES
from sitl.formation_transition import plan_formation_transition
from sitl.clock_sync import best_clock_sample, estimate_clock_offset
from sitl.mission_intent import MissionIntent
from sitl.operator_udp import decode_message, send_to_nodes
from sitl.peer_roster import load_auth_key, load_peer_roster
from sitl.swarm_console import SwarmConsole


SITL_SENSOR_PROFILE_OVERRIDES = {
    "baseline": {},
    # The project Matek H743 parameter file uses only its primary compass.
    # SITL otherwise exposes three simulated compasses, which were observed
    # switching during flight and preceded an EKF lane switch in the 10-drone
    # live run. Keep this as an explicit profile so baseline semantics stay
    # unchanged while hardware-matched swarm runs use the same compass count.
    "matek-single-compass": {
        "COMPASS_USE2": "0",
        "COMPASS_USE3": "0",
    },
    # Explicit stress assumptions for sensitivity testing; not calibrated to the physical drone.
    "stress": {
        "SIM_GPS1_HNSE": "0.50",       # horizontal GPS wander radius, metres
        "SIM_GPS1_NOISE": "0.30",      # vertical GPS sinusoidal error amplitude, metres
        "SIM_GPS1_LAG_MS": "250",      # simulated GPS measurement latency, milliseconds
        "SIM_GPS1_ACC": "0.60",        # reported GPS position accuracy, metres
        "SIM_WIND_SPD": "2.00",        # steady wind speed, metres/second
        "SIM_WIND_DIR": "180.0",       # wind coming from south
        "SIM_WIND_TURB": "0.50",       # random wind variation, metres/second
    },
}
_CLOCK_RESYNC_INTERVAL_S = 60.0


class IndependentSwarmConsole:
    """Mission entry, display, and logging only; node processes own all MAVLink I/O."""

    def __init__(self, args):
        self.args = args
        self.count = args.drones
        self.control_mode = getattr(args, "control_mode", "decentralized")
        self.fleet = build_fleet(self.count)
        self.peer_roster = load_peer_roster(args.peer_roster, self.count)
        self.operator_auth_key = load_auth_key(args.peer_auth_key_file)
        self.operator_targets = {
            index: (entry["command_host"], entry["command_port"])
            for index, entry in self.peer_roster.items()
        }
        self.root = Path(__file__).resolve().parent.parent
        self.launcher = SwarmConsole(
            n=self.count,
            use_gazebo=True if args.gazebo else (False if args.standalone else None),
            force_restart=args.restart,
            control_mode="centralized",
            agent_runtime="local",
        )
        self.launcher.evidence = None
        self.nodes = []
        self.status_sockets = []
        self.latest = {}
        self.lock = threading.Lock()
        self.clock_offsets_s = {}
        self.clock_uncertainty_s = {}
        self.clock_probe_pending = {}
        self.clock_probe_samples = {index: [] for index in range(self.count)}
        self.clock_sync_sequence = 0
        self.clock_sync_message = None
        self.last_clock_sync_publish = 0.0
        self.last_clock_sync_measurement = 0.0
        self.running = False
        self.closed = False
        self.intent_enabled = True
        self.mission_sequence = 0
        self.operator_sequence = 0
        self.command_acks = {}
        self.network_fault_sequence = 0
        self.land_requested = False
        self.goal_ne = np.zeros(2, dtype=float)
        self.target_alt_m = 5.0
        self.formation = args.formation
        self.formation_slots = None
        self.formation_start_slots = None
        self.formation_transition_s = 0.0
        self.formation_transition_deadline = None
        self.formation_transition_started_at = None
        self.formation_resume_goal = None
        self.formation_abort_issued = False
        self.last_formation_result = {"accepted": False, "message": "No formation request yet."}
        self.last_command_result = None
        self.external_agents = bool(getattr(args, "external_agents", False))
        self.operator_bind_host = getattr(args, "operator_bind_host", "127.0.0.1")
        self.status_thread = None
        self.intent_thread = None
        self.dashboard_server = None
        evidence_parent = (Path(args.evidence_dir) if getattr(args, "evidence_dir", None)
                           else self.root / "runs" / "independent")
        self.run_dir = evidence_parent / datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S.%fZ")
        self.run_dir.mkdir(parents=True, exist_ok=False)
        self.launcher.sitl_run_dir = self.run_dir / "sitl-instances"
        self.log_path = self.run_dir / "events.jsonl"
        self.log = self.log_path.open("a", encoding="utf-8")
        self.log_lock = threading.Lock()
        self.process_manifest = self.run_dir / "processes.json"
        self.sitl_params_path = None
        self.sitl_sensor_profile = getattr(args, "sitl_sensor_profile", "baseline")
        if (getattr(args, "sitl_gcs_failsafe", False) or
                self.sitl_sensor_profile != "baseline"):
            self.sitl_params_path = self._write_sitl_profile(
                enable_gcs_failsafe=getattr(args, "sitl_gcs_failsafe", False),
                sensor_profile=self.sitl_sensor_profile,
            )
            self.launcher.sitl_params_file = self.sitl_params_path

    def _write_sitl_profile(self, enable_gcs_failsafe: bool = False,
                            sensor_profile: str = "baseline") -> Path:
        """Create a run-local SITL-only copy with declared test-profile overrides."""
        source = self.root / "sitl" / "swarm_params.parm"
        if sensor_profile not in SITL_SENSOR_PROFILE_OVERRIDES:
            raise ValueError(f"unknown SITL sensor profile: {sensor_profile}")
        values = dict(SITL_SENSOR_PROFILE_OVERRIDES[sensor_profile])
        if enable_gcs_failsafe:
            values.update({"FS_GCS_ENABLE": "5", "FS_GCS_TIMEOUT": "5.0", "FS_OPTIONS": "0"})
        found = set()
        lines = []
        for line in source.read_text(encoding="utf-8").splitlines():
            fields = line.split()
            if fields and fields[0] in values:
                key = fields[0]
                lines.append(f"{key:<20}{values[key]}    # run-local SITL validation profile")
                found.add(key)
            else:
                lines.append(line)
        for key, value in values.items():
            if key not in found:
                lines.append(f"{key:<20}{value}    # run-local SITL validation profile")
        path = self.run_dir / "independent_profile.parm"
        path.write_text("\n".join(lines) + "\n", encoding="utf-8")
        return path

    def event(self, kind: str, **fields) -> None:
        record = {"utc": datetime.now(timezone.utc).isoformat(), "kind": kind, **fields}
        with self.log_lock:
            if self.log.closed:
                return
            self.log.write(json.dumps(record, allow_nan=False, separators=(",", ":")) + "\n")
            self.log.flush()

    def _agent_command(self, spec) -> list[str]:
        # When this console starts ArduPilot directly, each node owns the
        # vehicle's TCP server.  When an external sim_vehicle/MAVProxy session
        # is already running (for example to mirror telemetry into QGC), the
        # node must consume its dedicated UDP output instead; MAVProxy already
        # owns the ArduPilot TCP connection in that topology.
        use_tcp = getattr(getattr(self, "launcher", None), "use_tcp", True)
        endpoint = (f"tcp:127.0.0.1:{spec.tcp_port}" if use_tcp
                    else f"udpin:127.0.0.1:{spec.port}")
        command = [
            sys.executable, "-u", "-m", "sitl.independent_agent_node",
            "--id", str(spec.index), "--count", str(self.count),
            "--endpoint", endpoint,
            "--control-mode", self.control_mode,
            "--formation", self.formation,
            "--peer-loss", str(self.args.peer_loss),
            "--peer-latency-ms", str(self.args.peer_latency_ms),
            "--peer-duplicate-rate", str(self.args.peer_duplicate_rate),
            "--peer-reorder-rate", str(self.args.peer_reorder_rate),
            "--peer-reorder-delay-ms", str(self.args.peer_reorder_delay_ms),
            "--peer-port-base", "20000", "--command-port-base", "21000",
            "--status-port-base", "22000",
        ]
        peer_jitter_ms = getattr(self.args, "peer_jitter_ms", None)
        if peer_jitter_ms is not None:
            command += ["--peer-jitter-ms", str(peer_jitter_ms)]
        if self.args.peer_block:
            for link in self.args.peer_block.split(","):
                sender, recipient = (int(value) - 1 for value in link.split(":"))
                command += ["--peer-block", f"{sender}:{recipient}"]
        if self.args.peer_seed is not None:
            command += ["--peer-seed", str(self.args.peer_seed)]
        if self.args.peer_burst_loss:
            command.append("--peer-burst-loss")
        if getattr(self.args, "adaptive_swarm", False):
            command.append("--adaptive-swarm")
        for obstacle in getattr(self.args, "obstacle", ()):
            command += ["--obstacle", str(obstacle)]
        if self.args.peer_roster:
            command += ["--peer-roster", str(self.args.peer_roster)]
        if self.args.peer_auth_key_file:
            command += ["--peer-auth-key-file", str(self.args.peer_auth_key_file)]
        return command

    def _spawn_agent(self, index: int):
        process = subprocess.Popen(
            self._agent_command(self.fleet[index]), cwd=self.root, start_new_session=True,
        )
        if index == len(self.nodes):
            self.nodes.append(process)
        else:
            self.nodes[index] = process
        self._write_process_manifest()
        return process

    def start(self) -> None:
        print(f">> Independent-node evidence: {self.run_dir}")
        if getattr(self.args, "sitl_gcs_failsafe", False):
            print(">> SITL GCS failsafe profile: LAND after 5 seconds without an agent heartbeat")
        print(f">> SITL sensor profile: {self.sitl_sensor_profile}")
        if not self.external_agents:
            self.launcher.launch_sitl()
        self._write_process_manifest()
        for index in range(self.count):
            status_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            status_socket.bind((self.operator_bind_host,
                                self.peer_roster[index]["status_port"]))
            status_socket.setblocking(False)
            self.status_sockets.append(status_socket)
        for spec in ([] if self.external_agents else self.fleet):
            self._spawn_agent(spec.index)
        self.running = True
        self.status_thread = threading.Thread(target=self._status_loop, daemon=True)
        self.status_thread.start()
        self._wait_for_ready()
        if self.external_agents:
            self._restore_external_mission()
        self._synchronize_clocks()
        self.event("nodes_ready", count=self.count, control_mode=self.control_mode,
                   formation=self.formation,
                   peer_loss=self.args.peer_loss, peer_latency_ms=self.args.peer_latency_ms,
                   peer_jitter_ms=(getattr(self.args, "peer_jitter_ms", None)
                                   if getattr(self.args, "peer_jitter_ms", None) is not None
                                   else (5.0 if self.args.peer_latency_ms > 0 else 0.0)),
                   peer_duplicate_rate=self.args.peer_duplicate_rate,
                   peer_reorder_rate=self.args.peer_reorder_rate,
                   peer_reorder_delay_ms=self.args.peer_reorder_delay_ms,
                   peer_roster=(str(self.args.peer_roster)
                                if self.args.peer_roster is not None else None),
                   peer_auth_enabled=bool(self.args.peer_auth_key_file),
                   peer_seed=self.args.peer_seed, peer_block=self.args.peer_block,
                   sensor_profile=self.sitl_sensor_profile,
                   gcs_failsafe=bool(getattr(self.args, "sitl_gcs_failsafe", False)),
                   clock_offsets_s=self.clock_offsets_s,
                   clock_uncertainty_s=self.clock_uncertainty_s,
                   operator_takeover=self.external_agents,
                   adaptive_swarm=bool(getattr(self.args, "adaptive_swarm", False)),
                   obstacles=list(getattr(self.args, "obstacle", ())),
                   resumed_mission_sequence=self.mission_sequence if self.external_agents else None)
        self._publish_intent()
        self.intent_thread = threading.Thread(target=self._intent_loop, daemon=True)
        self.intent_thread.start()
        dashboard_port = int(getattr(self.args, "dashboard_port", 0) or 0)
        if dashboard_port:
            from sitl.operator_dashboard import start_dashboard
            self.dashboard_server = start_dashboard(self, dashboard_port)
            print(f">> Visual controller: http://127.0.0.1:{dashboard_port}")
        print(f"\nIndependent {self.control_mode} swarm ready ({self.count} drones). "
              "Commands: preflight, launch, w/s/a/d [m], u/j [m], status, "
              "link-block <sender> <recipient>, link-restore <sender> <recipient>, "
              "fail <id>, restart <id>, rescue <id>, coord off/on, q")

    def _write_process_manifest(self) -> None:
        payload = {
            "sitl_pids": [process.pid for process in self.launcher.procs],
            "agent_pids": [process.pid for process in self.nodes],
        }
        self.process_manifest.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")

    def mode_relaunch_command(self, mode: str) -> str:
        """Build the explicit 2D relaunch command used by the dashboard selector."""
        base = ["python3", "-m", "sitl.swarm_console", "--drones", str(self.count),
                "--standalone", "--control-mode", mode]
        if mode in {"decentralized", "hybrid"}:
            base += ["--agent-runtime", "independent", "--adaptive-swarm"]
            dashboard_port = int(getattr(self.args, "dashboard_port", 0) or 0)
            if dashboard_port:
                base += ["--dashboard-port", str(dashboard_port)]
        return " ".join(base)

    @staticmethod
    def _status_confirms_landed(status: dict) -> bool:
        position = status.get("position")
        if not isinstance(position, list) or len(position) != 3:
            return False
        try:
            down_m = float(position[2])
            timestamp = float(status.get("timestamp"))
        except (TypeError, ValueError):
            return False
        return bool(
            status.get("command") == "landed"
            and not status.get("armed", True)
            and not status.get("airborne", True)
            and np.isfinite(down_m) and down_m > -0.5
            and np.isfinite(timestamp) and 0 <= time.time() - timestamp <= 3.0
        )

    def _wait_for_ready(self, timeout: float = 360.0) -> None:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.nodes and all((node.poll() is not None) for node in self.nodes):
                raise RuntimeError("an independent agent exited before readiness")
            now = time.time()
            with self.lock:
                if self.external_agents:
                    ready = sum(
                        1 for index in range(self.count)
                        if index in self.latest and
                        0 <= now - self.latest[index].get(
                            "operator_received_at",
                            self.latest[index].get("timestamp", 0.0),
                        ) <= 3.0
                    )
                else:
                    ready = sum(1 for value in self.latest.values()
                                if value.get("command") == "ready")
            label = "live agent status" if self.external_agents else "agents EKF-ready"
            print(f"\r>> Independent {label}: {ready}/{self.count}", end="", flush=True)
            if ready == self.count:
                print()
                return
            time.sleep(0.5)
        if self.external_agents:
            raise TimeoutError("not all external agents reported fresh status before timeout")
        raise TimeoutError("not all independent agents became ready before timeout")

    def _restore_external_mission(self, timeout: float = 5.0) -> None:
        """Resume a consistent live mission when attaching a replacement operator."""
        deadline = time.monotonic() + timeout
        while True:
            with self.lock:
                statuses = [self.latest.get(index, {}).copy() for index in range(self.count)]
            active = [status.get("mission_intent") if status.get("mission_active") else None
                      for status in statuses]
            if not any(intent is not None for intent in active):
                self.event("operator_takeover_mission", resumed=False,
                           reason="no_active_agent_mission")
                return
            if all(intent is not None for intent in active):
                try:
                    sequences = [intent["sequence"] for intent in active]
                    goals = [tuple(float(value) for value in intent["goal_ne"])
                             for intent in active]
                    formations = [intent["formation"] for intent in active]
                    altitudes = [float(intent["target_alt_m"]) for intent in active]
                    valid = (
                        all(isinstance(sequence, int) and not isinstance(sequence, bool)
                            and sequence > 0 for sequence in sequences) and
                        all(len(goal) == 2 and all(math.isfinite(value) for value in goal)
                            for goal in goals) and
                        all(isinstance(formation, str) and formation for formation in formations) and
                        all(math.isfinite(altitude) and 0.5 <= altitude <= 25.0
                            for altitude in altitudes)
                    )
                except (KeyError, TypeError, ValueError):
                    valid = False
                if (valid and all(goal == goals[0] for goal in goals) and
                        all(formation == formations[0] for formation in formations) and
                        all(altitude == altitudes[0] for altitude in altitudes)):
                    self.goal_ne = np.asarray(goals[0], dtype=float)
                    self.formation = formations[0]
                    self.target_alt_m = altitudes[0]
                    self.mission_sequence = max(sequences)
                    self.event("operator_takeover_mission", resumed=True,
                               prior_sequences=sequences, goal_ne=list(goals[0]),
                               formation=self.formation, target_alt_m=self.target_alt_m)
                    print(f">> Resuming shared mission {self.mission_sequence}: "
                          f"N={goals[0][0]:.1f}m E={goals[0][1]:.1f}m "
                          f"Alt={self.target_alt_m:.1f}m")
                    return
            if time.monotonic() >= deadline:
                raise RuntimeError(
                    "external agents disagree about their active mission; refusing operator takeover"
                )
            time.sleep(0.1)

    def _status_loop(self) -> None:
        while self.running:
            for index, sock in enumerate(self.status_sockets):
                while True:
                    try:
                        raw, _ = sock.recvfrom(8192)
                    except BlockingIOError:
                        break
                    status = decode_message(raw, self.operator_auth_key)
                    if status is None:
                        continue
                    if status.get("type") == "clock_probe_response":
                        self._record_clock_probe(index, status)
                        continue
                    if status.get("type") == "command_ack":
                        try:
                            ack_id = status.get("drone_id")
                            ack_sequence = status.get("sequence")
                            ack_action = status.get("action")
                            ack_epoch = status.get("epoch")
                            current_epoch = (self.clock_sync_message or {}).get("epoch")
                            if (ack_id != index or not isinstance(ack_sequence, int) or
                                    isinstance(ack_sequence, bool) or ack_sequence < 1 or
                                    ack_action not in ("launch", "land") or
                                    not isinstance(ack_epoch, str) or ack_epoch != current_epoch):
                                continue
                        except (TypeError, ValueError):
                            continue
                        with self.lock:
                            self.command_acks[index] = (ack_sequence, ack_action, ack_epoch)
                        self.event("command_ack", drone_id=index, sequence=ack_sequence,
                                   action=ack_action, epoch=ack_epoch)
                        continue
                    if status.get("type") != "status" or status.get("drone_id") != index:
                        continue
                    try:
                        sequence = int(status["sequence"])
                        timestamp = float(status["timestamp"])
                        session_started_at = float(status["session_started_at"])
                        session_id = str(status["session_id"])
                    except (KeyError, TypeError, ValueError):
                        continue
                    offset = self.clock_offsets_s.get(index)
                    status["operator_received_at"] = time.time()
                    status["agent_timestamp"] = timestamp
                    status["agent_session_started_at"] = session_started_at
                    status["clock_offset_s"] = offset
                    if offset is not None:
                        timestamp -= offset
                        session_started_at -= offset
                    status["timestamp"] = timestamp
                    status["session_started_at"] = session_started_at
                    previous = self.latest.get(index, {})
                    previous_session_id = previous.get("session_id")
                    same_session = session_id == previous_session_id
                    if ((same_session and sequence <= previous.get("sequence", 0)) or
                            (previous_session_id is not None and not same_session and
                             session_started_at <= previous.get("session_started_at", 0.0)) or
                            not session_id or not np.isfinite(session_started_at) or
                            not np.isfinite(timestamp) or
                            (offset is not None and timestamp > time.time() + 2.0) or
                            session_started_at > timestamp + 0.25 or
                            (offset is not None and time.time() - timestamp > 10.0)):
                        continue
                    with self.lock:
                        self.latest[index] = status
                    self.event("agent_status", **status)
            time.sleep(0.05)

    def _record_clock_probe(self, agent_id: int, message: dict) -> None:
        probe_id = message.get("probe_id")
        if not isinstance(probe_id, str):
            return
        with self.lock:
            pending = self.clock_probe_pending.pop(probe_id, None)
        if pending is None or pending[0] != agent_id or message.get("drone_id") != agent_id:
            return
        t0, t0_monotonic = pending[1], pending[2]
        t3 = time.time()
        t3_monotonic = time.monotonic()
        operator_wall_elapsed = t3 - t0
        operator_monotonic_elapsed = t3_monotonic - t0_monotonic
        if abs(operator_wall_elapsed - operator_monotonic_elapsed) > 0.005:
            self.event("clock_probe_rejected", drone_id=agent_id,
                       reason="operator_wall_clock_changed_during_probe",
                       wall_elapsed_s=operator_wall_elapsed,
                       monotonic_elapsed_s=operator_monotonic_elapsed)
            return
        try:
            offset, round_trip = estimate_clock_offset(
                t0, message.get("agent_received_at"), message.get("agent_sent_at"), t3,
                agent_processing_s=message.get("agent_processing_s"),
            )
        except (TypeError, ValueError):
            self.event("clock_probe_rejected", drone_id=agent_id,
                       reason="invalid_timestamps_or_agent_clock_step")
            return
        with self.lock:
            self.clock_probe_samples[agent_id].append((offset, round_trip))
        self.event("clock_probe_sample", drone_id=agent_id, offset_s=offset,
                   round_trip_s=round_trip)

    def _synchronize_clocks(self, rounds: int = 5, max_round_trip_s: float = 0.25,
                            ack_timeout_s: float = 6.0) -> None:
        """Measure every agent clock, then distribute offsets for peer/intent timestamps."""
        with self.lock:
            self.clock_probe_pending.clear()
            self.clock_probe_samples = {index: [] for index in range(self.count)}
        for _round in range(rounds):
            for agent_id, target in self.operator_targets.items():
                probe_id = uuid.uuid4().hex
                t0 = time.time()
                t0_monotonic = time.monotonic()
                with self.lock:
                    self.clock_probe_pending[probe_id] = (agent_id, t0, t0_monotonic)
                send_to_nodes(
                    {"version": 1, "type": "clock_probe", "probe_id": probe_id},
                    endpoints={agent_id: target}, auth_key=self.operator_auth_key,
                )
            deadline = time.monotonic() + 0.5
            while time.monotonic() < deadline:
                with self.lock:
                    enough = all(len(self.clock_probe_samples[index]) >= _round + 1
                                 for index in range(self.count))
                if enough:
                    break
                time.sleep(0.01)

        offsets = {}
        uncertainty = {}
        for agent_id in range(self.count):
            try:
                offsets[agent_id], uncertainty[agent_id] = best_clock_sample(
                    self.clock_probe_samples[agent_id], max_round_trip_s,
                )
            except ValueError as exc:
                raise TimeoutError(
                    f"clock synchronization failed for D{agent_id + 1}: {exc}"
                ) from exc
        with self.lock:
            self.clock_probe_pending.clear()
        self.clock_offsets_s = offsets
        self.clock_uncertainty_s = uncertainty
        self.clock_sync_sequence += 1
        epoch = uuid.uuid4().hex
        sync_message = {
            "version": 1, "type": "clock_sync", "sequence": self.clock_sync_sequence,
            "epoch": epoch,
            "offsets_s": {str(index): value for index, value in offsets.items()},
        }
        self.clock_sync_message = sync_message
        self.last_clock_sync_publish = time.monotonic()
        self._send_to_agents(sync_message)
        deadline = time.monotonic() + ack_timeout_s
        next_retry = time.monotonic() + 0.5
        while time.monotonic() < deadline:
            with self.lock:
                ready = all(self.latest.get(index, {}).get("clock_sync_epoch") == epoch
                            for index in range(self.count))
            if ready:
                self.event("clock_sync_complete", epoch=epoch,
                           offsets_s={str(index): value for index, value in offsets.items()},
                           uncertainty_s={str(index): value
                                          for index, value in uncertainty.items()},
                           resynchronized=self.clock_sync_sequence > 1)
                self.last_clock_sync_measurement = time.monotonic()
                return
            if time.monotonic() >= next_retry:
                self._send_to_agents(sync_message)
                next_retry = time.monotonic() + 0.5
            time.sleep(0.05)
        raise TimeoutError("agents did not acknowledge the measured clock offsets")

    def _publish_intent(self) -> None:
        self.mission_sequence += 1
        now = time.time()
        issued_at = self.formation_transition_started_at or now
        intent = MissionIntent(
            sequence=self.mission_sequence,
            issued_at=issued_at,
            expires_at=issued_at + 60.0,
            goal_ne=(float(self.goal_ne[0]), float(self.goal_ne[1])),
            formation=self.formation,
            target_alt_m=float(self.target_alt_m),
            formation_slots=(None if self.formation_slots is None else
                             tuple(tuple(float(component) for component in slot)
                                   for slot in self.formation_slots)),
            formation_start_slots=(None if self.formation_start_slots is None else
                                   tuple(tuple(float(component) for component in slot)
                                         for slot in self.formation_start_slots)),
            formation_transition_s=float(self.formation_transition_s),
        ).to_dict()
        epoch = (self.clock_sync_message or {}).get("epoch")
        if not epoch:
            raise RuntimeError("cannot publish mission intent before clock synchronization")
        self._send_to_agents({"version": 1, "type": "mission", "epoch": epoch,
                              "intent": intent})
        self.event("mission_intent", **intent)

    def _send_to_agents(self, message: dict) -> int:
        return send_to_nodes(message, endpoints=self.operator_targets,
                             auth_key=self.operator_auth_key)

    def _formation_transition_abort_reason(self, now: float) -> str | None:
        """Abort a morph if live telemetry loses the clearance assumed by its plan."""
        if self.formation_transition_deadline is None:
            return None
        if now > self.formation_transition_deadline + 12.0:
            return "formation did not settle within 12 s of the planned transition"
        with self.lock:
            states = [self.latest.get(index, {}).copy() for index in range(self.count)]
        positions = []
        for index, state in enumerate(states):
            try:
                # Agents publish status once per second. Use the time this
                # operator received it, rather than the agent's clock-adjusted
                # timestamp, and allow one missed publication before aborting.
                status_age = now - float(state.get("operator_received_at", state.get("timestamp")))
                position_age = float(state.get("position_age_s"))
                position = np.asarray(state.get("position"), dtype=float)
            except (TypeError, ValueError):
                return f"D{index + 1} has invalid transition telemetry"
            if (not 0.0 <= status_age <= 2.0 or
                    not 0.0 <= position_age <= 1.0 or
                    status_age + position_age > 2.0):
                return f"D{index + 1} transition telemetry is stale"
            if state.get("airborne") is not True or state.get("armed") is not True:
                return f"D{index + 1} is no longer airborne and armed"
            if state.get("safety_state") != "nominal":
                return f"D{index + 1} reports {state.get('safety_state')} safety"
            if position.shape != (3,) or not np.isfinite(position).all():
                return f"D{index + 1} has invalid transition position"
            positions.append(position[:2])
        minimum = min(
            float(np.linalg.norm(positions[i] - positions[j]))
            for i in range(self.count) for j in range(i + 1, self.count)
        )
        if minimum < 2.8:
            return f"live separation fell to {minimum:.2f} m"
        return None

    def _formation_transition_arrived(self, now: float) -> bool:
        """All agents must actually reach their assigned slots before the morph ends."""
        with self.lock:
            states = [self.latest.get(index, {}).copy() for index in range(self.count)]
        slots = np.asarray(self.formation_slots, dtype=float)
        if slots.shape != (self.count, 3):
            return False
        for index, state in enumerate(states):
            try:
                age = now - float(state.get("operator_received_at", state.get("timestamp")))
                position_age = float(state.get("position_age_s"))
                position = np.asarray(state.get("position"), dtype=float)
                progress = float(state.get("formation_transition_progress", 0.0))
            except (TypeError, ValueError):
                return False
            if (not 0.0 <= age <= 2.0 or not 0.0 <= position_age <= 1.0 or
                    position.shape != (3,) or not np.isfinite(position).all() or
                    state.get("formation") != self.formation or
                    state.get("formation_transition_active", True) or progress < 1.0 or
                    np.linalg.norm(position[:2] - self.goal_ne - slots[index, :2]) > 0.75):
                return False
        return True

    def _intent_loop(self) -> None:
        next_intent_publish = 0.0
        while self.running:
            if self.formation_transition_deadline is not None:
                reason = self._formation_transition_abort_reason(time.time())
                if reason and not getattr(self, "formation_abort_issued", False):
                    self.formation_abort_issued = True
                    self.intent_enabled = False
                    self.event("formation_transition_aborted", formation=self.formation,
                               reason=reason, action="land_all")
                    print(f"!! Formation transition aborted: {reason}; commanding all vehicles to LAND.",
                          flush=True)
                    threading.Thread(target=self._operator_command, args=("land",),
                                     daemon=True).start()
            if self.intent_enabled and time.monotonic() >= next_intent_publish:
                try:
                    if (self.formation_transition_deadline is not None and
                            time.time() >= self.formation_transition_deadline):
                        now = time.time()
                        if self._formation_transition_arrived(now):
                            if getattr(self, "formation_settled_since", None) is None:
                                self.formation_settled_since = now
                        else:
                            self.formation_settled_since = None
                        if (self.formation_settled_since is not None and
                                now - self.formation_settled_since >= 2.0):
                            self.goal_ne = self.formation_resume_goal.copy()
                            self.formation_resume_goal = None
                            self.formation_transition_deadline = None
                            self.formation_transition_started_at = None
                            self.formation_transition_s = 0.0
                            self.formation_start_slots = None
                            self.formation_abort_issued = False
                            self.formation_settled_since = None
                            self.event("formation_transition_complete", formation=self.formation)
                    self._publish_intent()
                    next_intent_publish = time.monotonic() + 1.0
                except OSError as exc:
                    self.event("mission_publish_error", error=str(exc))
            if getattr(self, "formation_abort_issued", False):
                with self.lock:
                    all_landed = all(
                        index in self.latest and self._status_confirms_landed(self.latest[index])
                        for index in range(self.count)
                    )
                if all_landed:
                    self.formation_transition_deadline = None
                    self.formation_transition_started_at = None
                    self.formation_transition_s = 0.0
                    self.formation_start_slots = None
                    self.formation_resume_goal = None
                    self.formation_abort_issued = False
                    self.formation_settled_since = None
            now = time.monotonic()
            if self.clock_sync_message is not None and now - self.last_clock_sync_publish >= 5.0:
                try:
                    self._send_to_agents(self.clock_sync_message)
                    self.last_clock_sync_publish = now
                except OSError as exc:
                    self.event("clock_sync_publish_error", error=str(exc))
            if (self.clock_sync_message is not None and
                    now - self.last_clock_sync_measurement >= _CLOCK_RESYNC_INTERVAL_S):
                try:
                    self._synchronize_clocks()
                except (OSError, TimeoutError) as exc:
                    self.event("clock_resync_error", error=str(exc))
                    self.last_clock_sync_measurement = time.monotonic()
            time.sleep(0.1)

    def _operator_command(self, action: str) -> dict:
        if action == "land":
            self.land_requested = True
        self.operator_sequence += 1
        sequence = self.operator_sequence
        epoch = (self.clock_sync_message or {}).get("epoch")
        if not epoch:
            self.event("operator_command_blocked", action=action,
                       reason="clock_not_synchronized")
            print(">> Command not sent: operator clock synchronization is not ready.")
            return {"accepted": False, "message": "Operator clock synchronization is not ready."}
        message = {"version": 1, "type": "command", "sequence": self.operator_sequence,
                   "epoch": epoch,
                   "action": action, "target_alt_m": self.target_alt_m,
                   "issued_at": time.time(), "expires_at": time.time() + 5.0}
        with self.lock:
            for index in range(self.count):
                self.command_acks.pop(index, None)
        deadline = time.monotonic() + 4.0
        next_retry = 0.0
        retries = 0
        while time.monotonic() < deadline:
            now = time.monotonic()
            if now >= next_retry:
                self._send_to_agents(message)
                retries += 1
                next_retry = now + 0.4
            with self.lock:
                acknowledged = sorted(
                    index for index in range(self.count)
                    if self.command_acks.get(index) == (sequence, action, epoch)
                )
            if len(acknowledged) == self.count:
                break
            time.sleep(0.025)
        missing = sorted(set(range(self.count)) - set(acknowledged))
        self.event("operator_command", **message, acknowledged_ids=acknowledged,
                   missing_ack_ids=missing, transmissions=retries)
        if missing:
            print(">> Command was not acknowledged by: " +
                  ", ".join(f"D{index + 1}" for index in missing) +
                  ". Check their status; the command may still have been received.")
        return {"accepted": not missing,
                "message": (f"{action.title()} acknowledged by all {self.count} agents."
                            if not missing else
                            f"{action.title()} acknowledgement missing from " +
                            ", ".join(f"D{index + 1}" for index in missing) + "."),
                "acknowledged_ids": acknowledged, "missing_ack_ids": missing}

    def network_preflight(self, now: float | None = None) -> dict:
        """Check every agent, mission lease, synchronized clock, and required peer link."""
        now = time.time() if now is None else float(now)
        with self.lock:
            states = {index: value.copy() for index, value in self.latest.items()}
        expected_epoch = (self.clock_sync_message or {}).get("epoch")
        agents = []
        positions = []
        for index in range(self.count):
            state = states.get(index)
            reasons = []
            if state is None:
                state = {}
                reasons.append("no_status")
            try:
                age = now - float(state.get("timestamp"))
            except (TypeError, ValueError):
                age = float("inf")
            if not 0.0 <= age <= 3.0:
                reasons.append("status_stale")
            report_age = age if math.isfinite(age) else None
            if state.get("airborne") is not False or state.get("armed") is not False:
                reasons.append("not_grounded_disarmed")
            if state.get("mission_active") is not True:
                reasons.append("no_active_mission")
            if not expected_epoch or state.get("clock_sync_epoch") != expected_epoch:
                reasons.append("clock_not_synchronized")
            try:
                telemetry_age = float(state.get("position_age_s"))
            except (TypeError, ValueError):
                telemetry_age = float("inf")
            if not 0.0 <= telemetry_age <= 1.0:
                reasons.append("telemetry_stale")
            report_telemetry_age = telemetry_age if math.isfinite(telemetry_age) else None
            try:
                position = np.asarray(state.get("position"), dtype=float)
            except (TypeError, ValueError):
                position = np.asarray([], dtype=float)
            if position.shape != (3,) or not np.isfinite(position).all():
                reasons.append("position_unavailable")
            else:
                positions.append((index, position[:2]))
            if state.get("safety_state") != "nominal":
                reasons.append("safety_not_nominal")
            try:
                expected = {int(peer) for peer in state.get("peer_ids", [])}
                live = {int(peer) for peer in state.get("peer_live_ids", [])}
            except (TypeError, ValueError):
                expected, live = set(), set()
                reasons.append("invalid_peer_status")
            missing = sorted(expected - live)
            if missing:
                reasons.append("peer_links_missing")
            agents.append({
                "drone_id": index,
                "ready": not reasons,
                "reasons": reasons,
                "expected_peer_ids": sorted(expected),
                "live_peer_ids": sorted(live),
                "missing_peer_ids": missing,
                "status_age_s": report_age,
                "telemetry_age_s": report_telemetry_age,
                "mission_active": state.get("mission_active") is True,
                "clock_synchronized": bool(expected_epoch) and
                                      state.get("clock_sync_epoch") == expected_epoch,
            })
        minimum_separation = float("inf")
        if len(positions) == self.count:
            minimum_separation = min(
                float(np.linalg.norm(positions[i][1] - positions[j][1]))
                for i in range(self.count) for j in range(i + 1, self.count)
            )
            if minimum_separation < 2.8:
                for agent in agents:
                    agent["reasons"].append("inter_agent_spacing_below_launch_minimum")
                    agent["ready"] = False
        return {"passed": all(agent["ready"] for agent in agents), "agents": agents,
                "minimum_separation_m": (minimum_separation if math.isfinite(minimum_separation)
                                         else None)}

    def _wait_for_network_preflight(self, timeout_s: float = 5.0) -> dict:
        deadline = time.monotonic() + max(0.0, float(timeout_s))
        result = self.network_preflight()
        while not result["passed"] and time.monotonic() < deadline:
            time.sleep(min(0.1, max(0.0, deadline - time.monotonic())))
            result = self.network_preflight()
        return result

    @staticmethod
    def _print_network_preflight(result: dict) -> None:
        for agent in result["agents"]:
            drone_id = agent["drone_id"] + 1
            live = ",".join(f"D{peer + 1}" for peer in agent["live_peer_ids"]) or "none"
            missing = ",".join(f"D{peer + 1}" for peer in agent["missing_peer_ids"])
            state = "READY" if agent["ready"] else "BLOCKED: " + ", ".join(agent["reasons"])
            print(f"  D{drone_id}: {state}; live peers={live}"
                  f"{'; missing=' + missing if missing else ''}")
        print(f">> Swarm network preflight: {'PASS' if result['passed'] else 'FAIL'}")

    def status(self) -> None:
        with self.lock:
            states = {key: value.copy() for key, value in self.latest.items()}
        print(f"\n--- Independent {self.control_mode} {self.count}-drone swarm ---")
        for index in range(self.count):
            state = states.get(index)
            process = self.nodes[index] if index < len(self.nodes) else None
            if process is not None and process.poll() is not None:
                if state is None:
                    print(f"D{index + 1}: AGENT OFFLINE (exit={process.returncode}); vehicle state unknown")
                else:
                    print(f"D{index + 1}: AGENT OFFLINE (exit={process.returncode}); last state may be stale")
                continue
            if state is None:
                print(f"D{index + 1}: waiting for agent status")
                continue
            status_age = time.time() - float(state.get("timestamp", 0.0))
            if status_age > 3.0:
                print(f"D{index + 1}: STATUS STALE ({status_age:.1f}s); last known state is not current")
                continue
            position = state.get("position")
            if position:
                north, east, down = position
                position_text = f"N={north:.2f} E={east:.2f} Alt={-down:.2f}m"
            else:
                position_text = "position unavailable"
            age = state.get("position_age_s")
            print(f"D{index + 1}: {'airborne' if state.get('airborne') else 'grounded':8s} "
                  f"armed={'yes' if state.get('armed') else 'no'} "
                  f"mode={state.get('hybrid_mode') or state.get('control_mode', self.control_mode)} "
                  f"alpha={state.get('hybrid_alpha') if state.get('hybrid_alpha') is not None else 'n/a'} "
                  f"coordinator={'up' if state.get('coordinator_connected') else 'down'} "
                  f"{position_text} peers={state.get('peer_ids', [])} "
                  f"live_peers={state.get('peer_live_ids', [])} "
                  f"telemetry_age={age if age is not None else 'n/a'} "
                  f"leader=D{int(state.get('elected_leader_id', 0)) + 1} "
                  f"members={[int(i) + 1 for i in state.get('active_member_ids', [])]} "
                  f"safety={state.get('safety_state', 'unknown')}"
                  f"{':' + state['safety_reason'] if state.get('safety_reason') else ''}")
        print(f"Mode: {self.control_mode} | Mission: N={self.goal_ne[0]:.1f}m "
              f"E={self.goal_ne[1]:.1f}m Alt={self.target_alt_m:.1f}m | "
              f"coordinator stream={'ON' if self.intent_enabled else 'OFF'}\n")

    def _set_peer_link(self, sender: int, recipient: int, blocked: bool) -> None:
        if not (1 <= sender <= self.count and 1 <= recipient <= self.count) or sender == recipient:
            print(f"Drone IDs must be different values from 1 to {self.count}.")
            return
        self.network_fault_sequence += 1
        message = {
            "version": 1, "type": "network_fault", "sequence": self.network_fault_sequence,
            "epoch": (self.clock_sync_message or {}).get("epoch"),
            "sender": sender - 1, "recipient": recipient - 1, "blocked": bool(blocked),
            "issued_at": time.time(), "expires_at": time.time() + 5.0,
        }
        self._send_to_agents(message)
        self.event("peer_link_fault", sender=sender - 1, recipient=recipient - 1,
                   blocked=bool(blocked), sequence=self.network_fault_sequence)
        print(f">> Directed peer link D{sender} -> D{recipient} "
              f"{'cut' if blocked else 'restored'}; reverse traffic is unchanged.")

    def _set_formation(self, style: str) -> None:
        def result(accepted: bool, message: str, **details):
            self.last_formation_result = {
                "accepted": accepted, "message": message, **details,
            }
            print((">> " if accepted else ">> Formation change blocked: ") + message)
            return self.last_formation_result

        if style not in STYLES:
            return result(False, f"Unknown formation {style!r}; choose from {', '.join(STYLES)}.")
        if not getattr(self.args, "adaptive_swarm", False):
            return result(False, "Formation changes require adaptive swarm mode.")
        if style == self.formation:
            return result(True, f"Formation {style.upper()} is already active; no transition needed.",
                          formation=style, unchanged=True)
        if getattr(self, "formation_transition_deadline", None) is not None:
            return result(False, "A formation transition is already active; wait for it to finish.")
        with self.lock:
            states = [self.latest.get(index, {}).copy() for index in range(self.count)]
        now = time.time()
        if any(not state.get("operator_received_at") or
               not 0.0 <= now - float(state.get("operator_received_at", 0.0)) <= 2.0
               for state in states):
            return result(False, "Every drone needs a fresh status heartbeat.")
        grounded = all(state.get("armed") is False and state.get("airborne") is False
                       for state in states)
        airborne = all(state.get("armed") is True and state.get("airborne") is True
                       for state in states)
        if not grounded and not airborne:
            return result(False, "All drones must be grounded or all airborne and armed.")

        if grounded:
            if any(not 0.0 <= float(state.get("position_age_s", float("inf"))) <= 1.0 or
                   state.get("safety_state") != "nominal"
                   for state in states):
                return result(False, "Every grounded drone needs fresh position telemetry and nominal safety status.")
            try:
                positions = np.asarray([state.get("position", [])[:2] for state in states], dtype=float)
            except (TypeError, ValueError):
                positions = np.asarray([], dtype=float)
            if positions.shape != (self.count, 2) or not np.isfinite(positions).all():
                return result(False, "Position reports are incomplete or invalid.")
            centroid = positions.mean(axis=0)
            try:
                plan = plan_formation_transition(positions, style, centroid_ne=centroid)
            except (TypeError, ValueError) as exc:
                return result(False, str(exc))
            self.formation_resume_goal = self.goal_ne.copy()
            self.goal_ne = centroid.copy()
            self.formation = style
            self.formation_slots = plan.slots_by_drone
            self.formation_start_slots = None
            self.formation_transition_s = 0.0
            self._publish_intent()
            self.event("formation_command", formation=style, airborne=False,
                       minimum_path_separation_m=plan.minimum_path_separation_m)
            return result(True, f"Formation set to {style.upper()}; launch paths screen at "
                          f"{plan.minimum_path_separation_m:.2f} m clearance.",
                          formation=style, airborne=False,
                          minimum_path_separation_m=plan.minimum_path_separation_m)

        if any(not 0.0 <= float(state.get("position_age_s", float("inf"))) <= 1.0 or
               state.get("safety_state") != "nominal" for state in states):
            return result(False, "Every airborne drone needs fresh position telemetry and nominal safety status.")

        positions = np.asarray([state["position"][:2] for state in states], dtype=float)
        if positions.shape != (self.count, 2) or not np.isfinite(positions).all():
            return result(False, "Position reports are incomplete or invalid.")
        for index, state in enumerate(states):
            expected = {int(peer) for peer in state.get("peer_ids", [])}
            live = {int(peer) for peer in state.get("peer_live_ids", [])}
            if expected - live:
                return result(False, f"D{index + 1} has stale peer links.")
        obstacles = [tuple(float(value) for value in str(item).split(","))
                     for item in getattr(self.args, "obstacle", ())]
        centroid = positions.mean(axis=0)
        try:
            plan = plan_formation_transition(
                positions, style, centroid_ne=centroid, obstacles=obstacles,
            )
        except (TypeError, ValueError) as exc:
            return result(False, str(exc))
        self.formation_resume_goal = self.goal_ne.copy()
        self.goal_ne = centroid.copy()  # hold the swarm center during the morph
        self.formation = style
        self.formation_slots = plan.slots_by_drone
        self.formation_start_slots = np.column_stack((positions - centroid, np.zeros(self.count)))
        self.formation_transition_s = plan.duration_s
        self.formation_transition_deadline = now + plan.duration_s
        self.formation_transition_started_at = now
        self.formation_abort_issued = False
        self.formation_settled_since = None
        self._publish_intent()
        self.event("formation_transition_started", formation=style,
                   duration_s=plan.duration_s,
                   min_planned_separation_m=plan.minimum_path_separation_m,
                   target_slots=plan.slots_by_drone.tolist())
        message = (f"Smooth transition to {style.upper()} started over {plan.duration_s:.1f}s; "
                   f"planned path clearance {plan.minimum_path_separation_m:.2f}m. "
                   "The swarm center will resume its previous target when complete.")
        result(True, message, formation=style, duration_s=plan.duration_s,
               minimum_path_separation_m=plan.minimum_path_separation_m)

    def handle(self, line: str) -> bool:
        parts = line.strip().lower().split()
        if not parts:
            return True
        action = parts[0]
        numeric_actions = {"w", "north", "s", "south", "a", "west", "d", "east", "u", "up", "j", "down"}
        if (getattr(self, "formation_transition_deadline", None) is not None and
                action not in {"status", "land", "q", "quit"}):
            print(">> Formation transition active; wait for it to finish or command LAND.")
            return True
        value = float(parts[1]) if len(parts) > 1 and action in numeric_actions else None
        step = 2.0 if value is None else value
        if action in ("launch", "takeoff"):
            preflight = self._wait_for_network_preflight()
            self.event("launch_preflight", **preflight)
            self._print_network_preflight(preflight)
            if not preflight["passed"]:
                print(">> Launch was not sent; fix the listed status, mission, clock, or peer-link issue and retry.")
                self.last_command_result = {
                    "accepted": False,
                    "message": "Launch blocked by network preflight.",
                    "preflight": preflight,
                }
                return True
            self.last_command_result = self._operator_command("launch")
        elif action in ("q", "quit", "land"):
            self.intent_enabled = False
            self.last_command_result = self._operator_command("land")
            return False
        elif action == "status":
            self.status()
        elif action in ("shape", "formation") and len(parts) == 2:
            self._set_formation(parts[1])
        elif action == "preflight":
            preflight = self.network_preflight()
            self.event("network_preflight", **preflight)
            self._print_network_preflight(preflight)
            self.last_command_result = {
                "accepted": bool(preflight["passed"]),
                "message": ("Network preflight passed." if preflight["passed"] else
                            "Network preflight blocked launch; inspect the listed reasons."),
                "preflight": preflight,
            }
        elif action in ("link-block", "link-restore") and len(parts) == 3:
            try:
                sender, recipient = int(parts[1]), int(parts[2])
            except ValueError:
                print(f"Use {action} <sender> <recipient>, for example: {action} 1 2")
                return True
            self._set_peer_link(sender, recipient, blocked=(action == "link-block"))
        elif action in ("fail", "crash") and len(parts) == 2:
            if self.external_agents:
                print("External agents are not child processes; stop the selected agent on its host for this fault test.")
                return True
            try:
                index = int(parts[1]) - 1
            except ValueError:
                print("Use fail <drone-id>, for example: fail 3")
                return True
            if not 0 <= index < self.count:
                print(f"Drone ID must be between 1 and {self.count}")
                return True
            node = self.nodes[index]
            if node.poll() is None:
                node.kill()
                node.wait(timeout=3.0)
                self.event("agent_process_killed", drone_id=index, exit_code=node.returncode,
                           note="ArduPilot SITL remains active; agent loss is not equivalent to FC loss")
                print(f">> Agent D{index + 1} stopped. Its SITL flight controller remains active; "
                      f"use rescue {index + 1} to land it through an emergency MAVLink connection.")
            else:
                print(f">> Agent D{index + 1} is already stopped.")
        elif action == "restart" and len(parts) == 2:
            if self.external_agents:
                print("External agents must be restarted by their supervisor on the agent host.")
                return True
            try:
                index = int(parts[1]) - 1
            except ValueError:
                print("Use restart <drone-id>, for example: restart 3")
                return True
            if not 0 <= index < self.count:
                print(f"Drone ID must be between 1 and {self.count}.")
                return True
            previous = self.nodes[index]
            if previous.poll() is None:
                previous.kill()
                previous.wait(timeout=3.0)
            replacement = self._spawn_agent(index)
            self.event("agent_process_restarted", drone_id=index,
                       old_pid=previous.pid, new_pid=replacement.pid,
                       policy="LAND if vehicle is armed and airborne at process restart")
            print(f">> Agent D{index + 1} restarted (PID {replacement.pid}). "
                  "An already-airborne vehicle will LAND before any later operator relaunch.")
        elif action == "rescue" and len(parts) == 2:
            if self.external_agents:
                print("Emergency MAVLink rescue is unavailable in external-agent mode; use that aircraft's local failsafe/operator.")
                return True
            try:
                index = int(parts[1]) - 1
            except ValueError:
                print("Use rescue <drone-id>, for example: rescue 3")
                return True
            if not 0 <= index < self.count:
                print(f"Drone ID must be between 1 and {self.count}")
                return True
            if self.nodes[index].poll() is None:
                print(f">> D{index + 1}'s agent is still running; rescue is only for a stopped agent.")
            else:
                landed, final_position = self._rescue_agent(index)
                if landed:
                    with self.lock:
                        previous = self.latest.get(index, {}).copy()
                        previous.update({"timestamp": time.time(), "position": final_position,
                                         "position_age_s": 0.0, "airborne": False, "armed": False,
                                         "command": "landed"})
                        self.latest[index] = previous
                    self.event("agent_rescue_landed", drone_id=index, position=final_position)
                    print(f">> D{index + 1} rescue LAND confirmed, disarmed at ground level.")
                else:
                    self.event("agent_rescue_unconfirmed", drone_id=index,
                               final_position=final_position)
                    print(f"!! D{index + 1} landing was not confirmed; keep SITL running for recovery.")
        elif action == "coord" and len(parts) > 1:
            self.intent_enabled = parts[1] in ("on", "up", "restore")
            self.event("mission_stream", enabled=self.intent_enabled)
            print(f">> Mission intent stream {'resumed' if self.intent_enabled else 'paused'}; agents retain the last accepted goal until expiry.")
            if self.intent_enabled:
                self._publish_intent()
        elif action in ("w", "north"):
            self.goal_ne[0] += step
            self._publish_intent()
        elif action in ("s", "south"):
            self.goal_ne[0] -= step
            self._publish_intent()
        elif action in ("a", "west"):
            self.goal_ne[1] -= step
            self._publish_intent()
        elif action in ("d", "east"):
            self.goal_ne[1] += step
            self._publish_intent()
        elif action in ("u", "up"):
            self.target_alt_m = min(25.0, self.target_alt_m + step)
            self._publish_intent()
        elif action in ("j", "down"):
            self.target_alt_m = max(0.5, self.target_alt_m - step)
            self._publish_intent()
        else:
            print("Commands: preflight | launch | w/s/a/d [m] | u/j [m] | status | link-block/restore <sender> <recipient> | "
                  "fail <id> | rescue <id> | coord off/on | q")
        return True

    def _rescue_agent(self, index: int, timeout: float = 60.0) -> tuple[bool, list[float] | None]:
        """Emergency-only connection for landing a vehicle after its local agent process dies."""
        endpoint = f"tcp:127.0.0.1:{self.fleet[index].tcp_port}"
        connection = mavutil.mavlink_connection(endpoint, retries=1, reconnect_delay=0)
        final_position = None
        armed = None
        try:
            heartbeat = connection.wait_heartbeat(timeout=5.0)
            if heartbeat is None:
                return False, final_position
            connection.mav.request_data_stream_send(
                connection.target_system, connection.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1,
            )
            connection.set_mode(9)
            connection.mav.command_long_send(
                connection.target_system, connection.target_component,
                mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0, 0, 0, 0, 0, 0,
            )
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                message = connection.recv_match(
                    type=["HEARTBEAT", "GLOBAL_POSITION_INT"], blocking=True, timeout=0.5,
                )
                if message is None:
                    continue
                if message.get_type() == "HEARTBEAT":
                    armed = bool(message.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                elif message.get_type() == "GLOBAL_POSITION_INT":
                    final_position = self.launcher.frame.gps_to_global_ned(
                        message.lat / 1e7, message.lon / 1e7, message.relative_alt / 1000.0,
                    ).tolist()
                    if (armed is False and message.relative_alt / 1000.0 < 0.5):
                        return True, final_position
            return False, final_position
        finally:
            connection.close()

    def run(self) -> None:
        try:
            self.start()
            while True:
                self.status()
                if not self.handle(input("Independent Swarm > ")):
                    break
        except KeyboardInterrupt:
            print("\n>> Sending LAND to all independent agents.")
            self.intent_enabled = False
            self._operator_command("land")
        finally:
            self.stop()

    def stop(self) -> None:
        if self.closed:
            return
        self.closed = True
        self.intent_enabled = False
        if ((self.external_agents and self.running) or
                any(node.poll() is None for node in self.nodes)):
            if not self.land_requested:
                try:
                    self._operator_command("land")
                except OSError:
                    pass
            deadline = time.monotonic() + 45.0
            while time.monotonic() < deadline:
                with self.lock:
                    active_indices = ([index for index, node in enumerate(self.nodes)
                                       if node.poll() is None] if not self.external_agents
                                      else list(range(self.count)))
                    landed = all(
                        index in self.latest and self._status_confirms_landed(self.latest[index])
                        for index in active_indices
                    )
                if landed:
                    break
                time.sleep(0.25)
        # A dead agent cannot acknowledge the normal operator LAND intent. Use a
        # separate emergency MAVLink connection before deciding whether it is safe
        # to tear down the simulator.
        for index, node in enumerate(self.nodes):
            if node.poll() is not None:
                landed, final_position = self._rescue_agent(index, timeout=60.0)
                if landed:
                    with self.lock:
                        previous = self.latest.get(index, {}).copy()
                        previous.update({"timestamp": time.time(), "position": final_position,
                                         "position_age_s": 0.0, "airborne": False, "armed": False,
                                         "command": "landed"})
                        self.latest[index] = previous
                    self.event("agent_rescue_landed", drone_id=index, position=final_position,
                               reason="console_shutdown")
                else:
                    self.event("agent_rescue_unconfirmed", drone_id=index,
                               final_position=final_position, reason="console_shutdown")
        self.running = False
        if self.dashboard_server is not None:
            self.dashboard_server.shutdown()
            self.dashboard_server.server_close()
        if self.intent_thread is not None:
            self.intent_thread.join(timeout=1.5)
        if self.status_thread is not None:
            self.status_thread.join(timeout=1.5)
        for sock in self.status_sockets:
            sock.close()
        with self.lock:
            landed = len(self.latest) == self.count and all(
                self._status_confirms_landed(value) for value in self.latest.values()
            )
        if landed and not self.external_agents:
            for node in self.nodes:
                if node.poll() is None:
                    node.terminate()
            for node in self.nodes:
                try:
                    node.wait(timeout=3.0)
                except subprocess.TimeoutExpired:
                    node.kill()
                    node.wait(timeout=2.0)
            self.launcher.cleanup()
        elif not landed:
            print("!! Landing was not confirmed; leaving SITL and agent processes running.")
        self.event("session_end", agent_processes=len(self.nodes), landed=landed,
                   sim_processes_kept_for_safety=not landed)
        with self.log_lock:
            self.log.close()


def run_independent_console(args) -> None:
    session = IndependentSwarmConsole(args)
    try:
        session.run()
    except Exception as exc:
        # run() closes its evidence file in a finally block, so only append the wrapper-level
        # error when startup failed before normal shutdown had closed the log.
        if not session.log.closed:
            session.event("session_error", error=str(exc))
        session.stop()
        raise
