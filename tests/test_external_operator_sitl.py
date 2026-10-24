import json
import math
import os
import socket
import threading
import time
from types import SimpleNamespace
import pytest
import numpy as np

from experiments.validate_independent_operator_loss import (
    currently_confirmed_landed,
    leader_reached_north_goal,
    status_confirms_landed,
)
from experiments.validate_external_agent_sitl import (
    formation_tracking_metrics,
    mission_convergence_state,
    minimum_time_aligned_separation,
    peer_impairment_cli_args,
)
import sitl.independent_agent_node as independent_agent_module
import sitl.independent_swarm_console as independent_console_module
from sitl.independent_agent_node import IndependentAgentNode
from sitl.independent_swarm_console import IndependentSwarmConsole
from sitl.operator_udp import NodeOperatorSocket, encode_message
from sitl.peer_roster import load_auth_key


def test_external_sitl_forwards_reorder_duplicate_and_burst_impairments():
    args = SimpleNamespace(
        peer_loss=0.02,
        peer_latency_ms=25.0,
        peer_jitter_ms=5.0,
        peer_burst_loss=True,
        peer_burst_start_rate=0.04,
        peer_burst_continue_rate=0.65,
        peer_duplicate_rate=0.01,
        peer_reorder_rate=0.05,
        peer_reorder_delay_ms=150.0,
    )

    assert peer_impairment_cli_args(args) == [
        "--peer-loss", "0.02",
        "--peer-latency-ms", "25.0",
        "--peer-jitter-ms", "5.0",
        "--peer-duplicate-rate", "0.01",
        "--peer-reorder-rate", "0.05",
        "--peer-reorder-delay-ms", "150.0",
        "--peer-burst-start-rate", "0.04",
        "--peer-burst-continue-rate", "0.65",
        "--peer-burst-loss",
    ]


def test_binary_auth_key_loader_preserves_whitespace_bytes(tmp_path):
    key = b"\n" + bytes(range(1, 31)) + b"\r"
    path = tmp_path / "binary-auth-key"
    path.write_bytes(key)

    assert load_auth_key(path) == key


def test_swarm_network_preflight_requires_fresh_direct_peer_links():
    console = object.__new__(IndependentSwarmConsole)
    console.count = 2
    console.lock = threading.Lock()
    console.clock_sync_message = {"epoch": "clock-epoch"}
    now = time.time()

    def status(drone_id, peer_id):
        return {
            "timestamp": now,
            "airborne": False,
            "armed": False,
            "mission_active": True,
            "clock_sync_epoch": "clock-epoch",
            "position_age_s": 0.1,
            "safety_state": "nominal",
            "peer_ids": [peer_id],
            "peer_live_ids": [peer_id],
        }

    console.latest = {0: status(0, 1), 1: status(1, 0)}
    assert console.network_preflight(now)["passed"] is True

    console.latest[0]["peer_live_ids"] = []
    failed = console.network_preflight(now)
    assert failed["passed"] is False
    assert "peer_links_missing" in failed["agents"][0]["reasons"]
    assert failed["agents"][0]["missing_peer_ids"] == [1]


def test_external_console_accepts_fresh_status_from_live_mission_agents(monkeypatch):
    console = object.__new__(IndependentSwarmConsole)
    console.external_agents = True
    console.count = 2
    console.nodes = []
    console.latest = {
        index: {"timestamp": time.time(), "command": "mission:14"}
        for index in range(2)
    }
    console.lock = threading.Lock()
    monkeypatch.setattr(independent_console_module.time, "sleep", lambda _duration: None)

    console._wait_for_ready(timeout=0.1)


def test_external_takeover_restores_consistent_active_mission():
    console = object.__new__(IndependentSwarmConsole)
    console.count = 2
    console.lock = threading.Lock()
    console.latest = {
        0: {"mission_active": True, "mission_intent": {
            "sequence": 14, "goal_ne": [8.0, -3.0], "formation": "v", "target_alt_m": 6.0,
        }},
        1: {"mission_active": True, "mission_intent": {
            "sequence": 15, "goal_ne": [8.0, -3.0], "formation": "v", "target_alt_m": 6.0,
        }},
    }
    console.goal_ne = np.zeros(2)
    console.formation = "line"
    console.target_alt_m = 5.0
    console.mission_sequence = 0
    events = []
    console.event = lambda kind, **fields: events.append((kind, fields))

    console._restore_external_mission()

    assert console.goal_ne.tolist() == [8.0, -3.0]
    assert console.formation == "v"
    assert console.target_alt_m == 6.0
    assert console.mission_sequence == 15
    assert events == [("operator_takeover_mission", {
        "resumed": True, "prior_sequences": [14, 15], "goal_ne": [8.0, -3.0],
        "formation": "v", "target_alt_m": 6.0,
    })]


def test_external_takeover_refuses_conflicting_active_missions(monkeypatch):
    console = object.__new__(IndependentSwarmConsole)
    console.count = 2
    console.lock = threading.Lock()
    console.latest = {
        index: {"mission_active": True, "mission_intent": {
            "sequence": index + 1, "goal_ne": [float(index), 0.0],
            "formation": "v", "target_alt_m": 5.0,
        }}
        for index in range(2)
    }
    clock = [0.0]
    monkeypatch.setattr(independent_console_module.time, "monotonic",
                        lambda: clock[0])
    monkeypatch.setattr(independent_console_module.time, "sleep",
                        lambda duration: clock.__setitem__(0, clock[0] + duration))

    with pytest.raises(RuntimeError, match="disagree about their active mission"):
        console._restore_external_mission(timeout=0.2)


def test_external_sitl_separation_uses_time_aligned_airborne_statuses():
    events = [
        {"kind": "agent_status", "drone_id": 0, "timestamp": 10.0,
         "airborne": True, "position": [0.0, 0.0, -5.0]},
        {"kind": "agent_status", "drone_id": 1, "timestamp": 10.1,
         "airborne": True, "position": [3.0, 4.0, -5.0]},
        {"kind": "agent_status", "drone_id": 0, "timestamp": 11.0,
         "airborne": True, "position": [0.0, 0.0, -5.0]},
        {"kind": "agent_status", "drone_id": 1, "timestamp": 11.4,
         "airborne": True, "position": [1.0, 0.0, -5.0]},
    ]

    result = minimum_time_aligned_separation(events, count=2, max_time_delta_s=0.25)

    assert result["minimum_horizontal_separation_m"] == 5.0
    assert result["time_aligned_samples"] == 1
    assert result["max_alignment_error_s"] == pytest.approx(0.1)


def test_external_sitl_tracking_metrics_use_shared_goal_and_each_v_slot():
    events = [
        {
            "kind": "agent_status", "drone_id": 0, "timestamp": 10.0,
            "airborne": True, "mission_active": True,
            "command": "mission:1",
            "mission_intent": {
                "goal_ne": [4.0, 5.0], "formation": "v", "target_alt_m": 5.0,
            },
            "position": [4.2, 2.7, -5.0],
        },
        {
            "kind": "agent_status", "drone_id": 1, "timestamp": 10.02,
            "airborne": True, "mission_active": True,
            "command": "mission:1",
            "mission_intent": {
                "goal_ne": [4.0, 5.0], "formation": "v", "target_alt_m": 5.0,
            },
            "position": [4.2, 8.0, -5.0],
        },
    ]
    events.extend([
        {
            **event,
            "timestamp": 11.0,
            "airborne": True,
            "command": "landing:operator",
            "position": [event["position"][0], event["position"][1], -3.0],
        }
        for event in events.copy()
    ])

    result = formation_tracking_metrics(events, count=2, max_time_delta_s=0.1)

    assert result["coordinate_frame"] == "shared_global_ned_from_common_wgs84_datum"
    assert len(result["missions"]) == 1
    mission = result["missions"][0]
    assert mission["aligned_samples"] == 1
    assert mission["per_agent"]["1"]["horizontal_error"]["mean_m"] == pytest.approx(
        math.sqrt(0.08),
    )
    assert mission["per_agent"]["2"]["horizontal_error"]["mean_m"] == pytest.approx(
        math.sqrt(0.29),
    )
    assert mission["formation_pairwise_offset_error"]["mean_m"] == pytest.approx(0.3)
    steady = mission["within_altitude_band"]
    assert steady["aligned_samples"] == 1
    assert steady["fleet_horizontal_error"]["rmse_m"] == pytest.approx(
        math.sqrt((0.08 + 0.29) / 2),
    )


def test_mission_convergence_requires_all_agents_and_contiguous_dwell_samples():
    from sitl.formation_library import slots_for

    slots = slots_for("v", 2)
    goal = [2.0, 0.0]
    events = []

    def add_sample(timestamp, bad=False, command="mission:4"):
        for drone_id in range(2):
            position = [
                goal[0] + slots[drone_id, 0],
                goal[1] + slots[drone_id, 1],
                -5.0,
            ]
            if bad and drone_id == 1:
                position[0] += 0.6
            events.append({
                "kind": "agent_status", "drone_id": drone_id,
                "timestamp": timestamp + drone_id * 0.02,
                "airborne": True, "mission_active": True, "command": command,
                "mission_intent": {
                    "goal_ne": goal, "formation": "v", "target_alt_m": 5.0,
                },
                "position": position,
            })

    for timestamp in (1.0, 2.0, 3.0):
        add_sample(timestamp)
    state = mission_convergence_state(events, count=2, goal_ne=(2.0, 0.0))
    assert state["converged"] is True
    assert state["current_dwell_samples"] == 3

    add_sample(4.0, bad=True)
    add_sample(5.0)
    add_sample(6.0)
    state = mission_convergence_state(events, count=2, goal_ne=(2.0, 0.0))
    assert state["converged"] is False
    assert state["best_dwell_samples"] == 3
    assert state["current_dwell_samples"] == 2

    add_sample(7.0)
    add_sample(8.0, command="landing:operator")
    state = mission_convergence_state(events, count=2, goal_ne=(2.0, 0.0))
    assert state["converged"] is True
    assert state["latest_timestamp"] == pytest.approx(7.0)

    too_fast = []
    for timestamp in (1.0, 1.1, 1.2):
        for event in events[:2]:
            too_fast.append({**event, "timestamp": timestamp + event["drone_id"] * 0.01})
    state = mission_convergence_state(too_fast, count=2, goal_ne=(2.0, 0.0))
    assert state["converged"] is False
    assert state["aligned_samples"] == 1


def test_operator_clock_step_during_probe_discards_sample(monkeypatch):
    console = object.__new__(IndependentSwarmConsole)
    console.lock = threading.Lock()
    console.clock_probe_pending = {"probe-1": (0, 100.0, 10.0)}
    console.clock_probe_samples = {0: []}
    events = []
    console.event = lambda kind, **fields: events.append((kind, fields))
    monkeypatch.setattr(independent_console_module.time, "time", lambda: 100.6)
    monkeypatch.setattr(independent_console_module.time, "monotonic", lambda: 10.1)

    console._record_clock_probe(0, {
        "type": "clock_probe_response", "drone_id": 0, "probe_id": "probe-1",
        "agent_received_at": 200.02, "agent_sent_at": 200.021,
        "agent_processing_s": 0.001,
    })

    assert console.clock_probe_samples[0] == []
    assert events == [("clock_probe_rejected", {
        "drone_id": 0, "reason": "operator_wall_clock_changed_during_probe",
        "wall_elapsed_s": pytest.approx(0.6), "monotonic_elapsed_s": pytest.approx(0.1),
    })]


def test_agent_clock_probe_reports_monotonic_processing_time(monkeypatch):
    from sitl.operator_udp import decode_message

    sent = []

    class Operator:
        @staticmethod
        def poll():
            return [{"version": 1, "type": "clock_probe", "probe_id": "probe-1"}]

    class StatusSocket:
        @staticmethod
        def sendto(payload, endpoint):
            sent.append((payload, endpoint))

    node = object.__new__(IndependentAgentNode)
    node.operator = Operator()
    node.operator_auth_key = None
    node.drone_id = 2
    node.status_socket = StatusSocket()
    node.status_endpoint = ("127.0.0.1", 22002)
    monotonic_values = iter((10.0, 10.004))
    wall_values = iter((200.01, 200.014))
    monkeypatch.setattr(independent_agent_module.time, "monotonic",
                        lambda: next(monotonic_values))
    monkeypatch.setattr(independent_agent_module.time, "time", lambda: next(wall_values))

    node._receive_operator_messages()

    assert len(sent) == 1
    response = decode_message(sent[0][0])
    assert sent[0][1] == ("127.0.0.1", 22002)
    assert response["agent_processing_s"] == pytest.approx(0.004)


def test_operator_launch_is_blocked_when_swarm_preflight_fails():
    console = object.__new__(IndependentSwarmConsole)
    failed = {"passed": False, "agents": []}
    sent = []
    events = []
    console._wait_for_network_preflight = lambda: failed
    console._operator_command = sent.append
    console.event = lambda kind, **fields: events.append((kind, fields))

    assert console.handle("launch") is True
    assert sent == []
    assert events == [("launch_preflight", failed)]


def test_operator_retries_command_with_same_sequence_until_acknowledged(monkeypatch):
    console = object.__new__(IndependentSwarmConsole)
    console.count = 2
    console.operator_sequence = 0
    console.target_alt_m = 5.0
    console.land_requested = False
    console.clock_sync_message = {"epoch": "operator-test-epoch"}
    console.lock = threading.Lock()
    console.command_acks = {}
    sent = []
    events = []
    clock = [0.0]

    def send(message):
        sent.append(message.copy())
        if len(sent) == 1:
            with console.lock:
                console.command_acks[0] = (1, "launch", "operator-test-epoch")
        elif len(sent) == 2:
            with console.lock:
                console.command_acks[1] = (1, "launch", "operator-test-epoch")
        return 2

    def sleep(duration):
        clock[0] += duration

    console._send_to_agents = send
    console.event = lambda kind, **fields: events.append((kind, fields))
    monkeypatch.setattr(independent_console_module.time, "monotonic", lambda: clock[0])
    monkeypatch.setattr(independent_console_module.time, "sleep", sleep)

    console._operator_command("launch")

    assert len(sent) == 2
    assert sent[0]["sequence"] == sent[1]["sequence"] == 1
    assert sent[0]["action"] == sent[1]["action"] == "launch"
    assert events[-1][1]["acknowledged_ids"] == [0, 1]
    assert events[-1][1]["missing_ack_ids"] == []
    assert events[-1][1]["transmissions"] == 2


def test_agent_acknowledges_duplicate_command_without_repeating_action(monkeypatch):
    from sitl.independent_agent_node import IndependentAgentNode
    from sitl.operator_udp import decode_message

    now = time.time()
    command = {"version": 1, "type": "command", "sequence": 7,
               "epoch": "agent-epoch", "action": "launch",
               "issued_at": now - 0.1, "expires_at": now + 4.0}
    sent = []
    preflight_calls = []

    class Operator:
        def poll(self):
            return [command, command]

    class StatusSocket:
        def sendto(self, payload, endpoint):
            sent.append((payload, endpoint))

    node = object.__new__(IndependentAgentNode)
    node.operator = Operator()
    node.operator_clock_offset_s = 0.0
    node.last_operator_sequence = 0
    node.last_operator_action = None
    node.clock_sync_epoch = "agent-epoch"
    node.status_socket = StatusSocket()
    node.status_endpoint = ("127.0.0.1", 22000)
    node.operator_auth_key = None
    node.drone_id = 0
    node.airborne = False
    node.last_command = "ready"
    node._launch_preflight_failure = lambda: preflight_calls.append(True) or "launch_rejected_test"
    monkeypatch.setattr(independent_agent_module.time, "time", lambda: now)

    node._receive_operator_messages()

    assert len(sent) == 2
    assert all(endpoint == node.status_endpoint for _, endpoint in sent)
    acks = [decode_message(payload) for payload, _ in sent]
    assert all(ack["type"] == "command_ack" and ack["sequence"] == 7
               and ack["action"] == "launch" and ack["epoch"] == "agent-epoch"
               for ack in acks)
    assert len(preflight_calls) == 1
    assert node.last_command == "launch_rejected_test"


def test_independent_agent_uses_configured_peer_latency_jitter(monkeypatch):
    captured = {}

    class FakePeerTransport:
        def __init__(self, *args, **kwargs):
            captured.update(kwargs)

        @staticmethod
        def close():
            pass

    class FakeOperator:
        def __init__(self, *args, **kwargs):
            pass

        @staticmethod
        def close():
            pass

    monkeypatch.setattr(independent_agent_module, "UdpPeerStateTransport", FakePeerTransport)
    monkeypatch.setattr(independent_agent_module, "NodeOperatorSocket", FakeOperator)
    node = IndependentAgentNode(
        0, 2, "tcp:127.0.0.1:5760", peer_latency_ms=40,
        peer_latency_jitter_ms=17,
    )
    try:
        assert captured["latency_mean"] == 0.04
        assert captured["latency_std"] == 0.017
        assert node.peer_latency_jitter_ms == 17.0
    finally:
        node.close()


def test_independent_console_passes_peer_jitter_to_agent_process():
    console = object.__new__(IndependentSwarmConsole)
    console.count = 5
    console.formation = "v"
    console.control_mode = "decentralized"
    console.args = SimpleNamespace(
        peer_loss=0.02, peer_latency_ms=40.0, peer_jitter_ms=17.0,
        peer_duplicate_rate=0.0, peer_reorder_rate=0.0,
        peer_reorder_delay_ms=150.0, peer_block=None, peer_seed=8,
        peer_burst_loss=False, peer_roster=None, peer_auth_key_file=None,
    )
    from sitl.fleet import build_fleet
    command = console._agent_command(build_fleet(5)[0])
    index = command.index("--peer-jitter-ms")
    assert command[index + 1] == "17.0"


def test_independent_console_uses_private_udp_output_for_mavproxy_routed_sitl():
    console = object.__new__(IndependentSwarmConsole)
    console.count = 10
    console.formation = "v"
    console.control_mode = "decentralized"
    console.launcher = SimpleNamespace(use_tcp=False)
    console.args = SimpleNamespace(
        peer_loss=0.0, peer_latency_ms=25.0, peer_jitter_ms=5.0,
        peer_duplicate_rate=0.0, peer_reorder_rate=0.0,
        peer_reorder_delay_ms=150.0, peer_block=None, peer_seed=1024,
        peer_burst_loss=False, peer_roster=None, peer_auth_key_file=None,
    )
    from sitl.fleet import build_fleet
    command = console._agent_command(build_fleet(10)[9])

    assert command[command.index("--endpoint") + 1] == "udpin:127.0.0.1:14642"


def test_independent_console_passes_hybrid_mode_to_agent_process():
    console = object.__new__(IndependentSwarmConsole)
    console.count = 10
    console.formation = "v"
    console.control_mode = "hybrid"
    console.args = SimpleNamespace(
        peer_loss=0.0, peer_latency_ms=30.0, peer_jitter_ms=None,
        peer_duplicate_rate=0.0, peer_reorder_rate=0.0,
        peer_reorder_delay_ms=150.0, peer_block=None, peer_seed=None,
        peer_burst_loss=False, peer_roster=None, peer_auth_key_file=None,
    )
    from sitl.fleet import build_fleet
    command = console._agent_command(build_fleet(10)[0])

    assert command[command.index("--count") + 1] == "10"
    assert command[command.index("--control-mode") + 1] == "hybrid"


def test_operator_loss_harness_accepts_measured_leader_target_within_tolerance():
    assert leader_reached_north_goal("D1: airborne armed=yes N=2.26 E=-0.16 Alt=5.00m", 2.0)
    assert not leader_reached_north_goal("D1: airborne armed=yes N=1.40 E=0.00 Alt=5.00m", 2.0)


def test_operator_loss_harness_requires_disarmed_landed_confirmation():
    status = {"command": "landed", "armed": False, "airborne": False,
              "position": [0.0, 0.0, 0.01]}
    assert status_confirms_landed(status)
    assert not status_confirms_landed({**status, "armed": True})
    assert not status_confirms_landed({**status, "command": "landing:mission_expired"})


def test_operator_loss_landing_confirmation_tracks_fresh_latest_status():
    landed = {"command": "landed", "armed": False, "airborne": False,
              "position": [0.0, 0.0, 0.02]}
    latest = {0: landed.copy(), 1: landed.copy()}
    received = {0: 99.0, 1: 99.0}
    airborne = {0, 1}
    assert currently_confirmed_landed(latest, received, 100.0, airborne, 2) == {0, 1}
    latest[1] = {**landed, "armed": True}
    assert currently_confirmed_landed(latest, received, 100.0, airborne, 2) == {0}
    latest[1] = landed.copy()
    received[1] = 96.0
    assert currently_confirmed_landed(latest, received, 100.0, airborne, 2) == {0}


def test_startup_error_is_not_masked_after_run_closes_evidence_log(monkeypatch):
    class FailedSession:
        def __init__(self, args):
            self.log = SimpleNamespace(closed=False)
            self.stopped = False

        def run(self):
            self.log.closed = True
            raise PermissionError("local socket creation denied")

        def event(self, *_args, **_kwargs):
            raise AssertionError("closed evidence log must not be written")

        def stop(self):
            self.stopped = True

    monkeypatch.setattr(independent_console_module, "IndependentSwarmConsole", FailedSession)
    with pytest.raises(PermissionError, match="local socket creation denied"):
        independent_console_module.run_independent_console(SimpleNamespace())


def test_independent_agent_can_open_serial_mavlink_with_explicit_system_id(
        tmp_path, monkeypatch):
    serial_device = tmp_path / "fake-serial-device"
    serial_device.touch()
    calls = []

    class FakeMavlink:
        target_system = 42
        target_component = 1

        class mav:
            @staticmethod
            def request_data_stream_send(*args):
                pass

        @staticmethod
        def wait_heartbeat(timeout):
            return object()

        @staticmethod
        def close():
            pass

    monkeypatch.setattr(
        independent_agent_module.mavutil, "mavlink_connection",
        lambda *args, **kwargs: (calls.append((args, kwargs)) or FakeMavlink()),
    )
    monkeypatch.setattr(independent_agent_module, "request_message_interval", lambda *args: None)
    monkeypatch.setattr(independent_agent_module, "wait_ekf_ready", lambda *args, **kwargs: True)
    node = IndependentAgentNode(
        0, 2, str(serial_device), peer_port_base=43000 + (os.getpid() % 1000),
        command_port_base=44000 + (os.getpid() % 1000),
        status_port_base=45000 + (os.getpid() % 1000),
        baudrate=921600, mavlink_system_id=42,
    )
    monkeypatch.setattr(node, "_send_gcs_heartbeat", lambda: None)
    try:
        node.connect(timeout=0.1)
        assert calls == [((str(serial_device),), {"baud": 921600, "autoreconnect": True})]
    finally:
        node.close()


def test_external_operator_console_runs_five_agent_command_status_cycle(tmp_path, monkeypatch):
    monkeypatch.setattr(independent_console_module, "_CLOCK_RESYNC_INTERVAL_S", 0.05)
    count = 5
    # Select a genuinely free contiguous range. Desktop applications commonly
    # occupy random high UDP ports, so a PID-derived port is not isolated.
    base = None
    for candidate in range(46000, 62000, 250):
        probes = []
        try:
            for agent_id in range(count):
                for host, port in ((f"127.0.0.{agent_id + 2}", candidate + agent_id),
                                   ("127.0.0.1", candidate + 100 + agent_id)):
                    probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                    probe.bind((host, port))
                    probes.append(probe)
            base = candidate
            break
        except OSError:
            pass
        finally:
            for probe in probes:
                probe.close()
    assert base is not None, "no free UDP block for operator integration test"
    key = b"external-console-five-agent-test-key-32-bytes"
    key_path = tmp_path / "test-key"
    key_path.write_bytes(key)
    operator_host = "127.0.0.1"
    roster_rows = []
    command_receivers = []
    for agent_id in range(count):
        host = f"127.0.0.{agent_id + 2}"
        command_port = base + agent_id
        status_port = base + 100 + agent_id
        roster_rows.append({
            "id": agent_id,
            "bind_host": host,
            "host": host,
            "port": base + 200 + agent_id,
            "command_bind_host": host,
            "command_host": host,
            "command_port": command_port,
            "status_host": operator_host,
            "status_port": status_port,
        })
        command_receivers.append(NodeOperatorSocket(
            agent_id, bind_host=host, port=command_port, auth_key=key,
        ))
    roster_path = tmp_path / "roster.json"
    roster_path.write_text(json.dumps({"version": 1, "agents": roster_rows}), encoding="utf-8")

    stop_agents = threading.Event()
    received = [[] for _ in range(count)]
    state = [{"command": "ready", "sequence": 0} for _ in range(count)]
    clock_skews = [(agent_id - 2) * 3.0 for agent_id in range(count)]

    def fake_agent(agent_id):
        sequence = 0
        clock_skew_s = clock_skews[agent_id]
        session_started = time.time() + clock_skew_s
        session_id = f"test-agent-{agent_id}"
        while not stop_agents.is_set():
            for message in command_receivers[agent_id].poll():
                received[agent_id].append(message)
                if message.get("type") == "clock_probe":
                    now = time.time() + clock_skews[agent_id]
                    response = {
                        "version": 1, "type": "clock_probe_response", "drone_id": agent_id,
                        "probe_id": message["probe_id"], "agent_received_at": now,
                        "agent_sent_at": time.time() + clock_skews[agent_id],
                    }
                    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
                        sender.sendto(encode_message(response, key),
                                      (operator_host, roster_rows[agent_id]["status_port"]))
                elif message.get("type") == "clock_sync":
                    state[agent_id]["clock_sync_epoch"] = message["epoch"]
                elif message.get("type") == "command":
                    ack = {
                        "version": 1, "type": "command_ack", "drone_id": agent_id,
                        "sequence": message["sequence"], "action": message["action"],
                        "epoch": message["epoch"],
                        "received_at": time.time() + clock_skew_s,
                    }
                    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
                        sender.sendto(encode_message(ack, key),
                                      (operator_host, roster_rows[agent_id]["status_port"]))
                    if message.get("action") == "land":
                        state[agent_id]["command"] = "landed"
            sequence += 1
            status = {
                "version": 1,
                "type": "status",
                "drone_id": agent_id,
                "timestamp": time.time() + clock_skews[agent_id],
                "sequence": sequence,
                "session_id": session_id,
                "session_started_at": session_started,
                "clock_sync_epoch": state[agent_id].get("clock_sync_epoch"),
                "position": [0.0, 0.0, 0.0],
                "position_age_s": 0.0,
                "airborne": False,
                "armed": False,
                "peer_ids": [],
                "peer_live_ids": [],
                "peer_timeout_ids": [],
                "safety_state": "nominal",
                "safety_reason": None,
                "mission_sequence": 1,
                "mission_active": True,
                "command": state[agent_id]["command"],
                "peer_metrics": {},
            }
            payload = encode_message(status, key)
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
                sender.sendto(payload, (operator_host, roster_rows[agent_id]["status_port"]))
            time.sleep(0.05)

    agent_threads = [threading.Thread(target=fake_agent, args=(i,), daemon=True)
                     for i in range(count)]
    for thread in agent_threads:
        thread.start()

    args = SimpleNamespace(
        drones=count,
        gazebo=False,
        standalone=True,
        restart=False,
        control_mode="decentralized",
        peer_roster=roster_path,
        peer_auth_key_file=key_path,
        formation="v",
        peer_loss=0.0,
        peer_latency_ms=30.0,
        peer_duplicate_rate=0.0,
        peer_reorder_rate=0.0,
        peer_reorder_delay_ms=150.0,
        peer_seed=None,
        peer_burst_loss=False,
        peer_block=None,
        sitl_gcs_failsafe=False,
        sitl_sensor_profile="baseline",
        external_agents=True,
        operator_bind_host=operator_host,
        evidence_dir=tmp_path / "operator-evidence",
    )
    console = IndependentSwarmConsole(args)
    wait_for_ready = console._wait_for_ready
    console._wait_for_ready = lambda timeout=360.0: wait_for_ready(timeout=5.0)
    try:
        console.start()
        assert len(console.nodes) == 0
        assert len(console.launcher.procs) == 0
        assert set(console.latest) == set(range(count))
        assert set(console.clock_offsets_s) == set(range(count))
        for agent_id, offset in console.clock_offsets_s.items():
            assert abs(offset - (agent_id - 2) * 3.0) < 0.05
        assert all(uncertainty <= 0.125
                   for uncertainty in console.clock_uncertainty_s.values())
        assert all(console.latest[index].get("clock_sync_epoch") for index in range(count))
        assert all(sum(message.get("type") == "clock_probe"
                       for message in received[index]) == 5 for index in range(count))
        assert all(any(message.get("type") == "clock_sync"
                       for message in received[index]) for index in range(count))
        initial_offsets = dict(console.clock_offsets_s)
        # Simulate host clock drift after startup, then require fresh probes to replace the map.
        clock_skews[:] = [offset + 0.4 for offset in clock_skews]
        deadline = time.monotonic() + 5.0
        while console.clock_sync_sequence < 2 and time.monotonic() < deadline:
            time.sleep(0.02)
        assert console.clock_sync_sequence == 2
        for agent_id, offset in console.clock_offsets_s.items():
            assert abs(offset - (initial_offsets[agent_id] + 0.4)) < 0.125
        assert all(sum(message.get("type") == "clock_probe"
                       for message in received[index]) == 10 for index in range(count))
        preflight = console._wait_for_network_preflight(timeout_s=2.0)
        assert preflight["passed"] is True, preflight
        assert console.handle("launch") is True
        assert console.handle("land") is False
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            if all(any(message.get("action") == "land" for message in rows)
                   for rows in received):
                break
            time.sleep(0.01)
        assert all(any(message.get("action") == "launch" for message in rows)
                   for rows in received)
        assert all(any(message.get("action") == "land" for message in rows)
                   for rows in received)
        assert all(console.command_acks[index] == (console.operator_sequence, "land",
                                                   console.clock_sync_message["epoch"])
                   for index in range(count))
        console.stop()
        assert all(console._status_confirms_landed(console.latest[index])
                   for index in range(count))
    finally:
        if not console.closed:
            console.stop()
        stop_agents.set()
        for thread in agent_threads:
            thread.join(timeout=1.0)
        for receiver in command_receivers:
            receiver.close()
