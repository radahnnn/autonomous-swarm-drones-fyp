import os
import math
from unittest.mock import MagicMock

import numpy as np
import pytest

from sitl.peer_state_bus import PeerStateBus
from sitl.run_evidence import SettledMotionDetector


def test_peer_bus_delivers_only_one_hop_state_and_expires_stale_packets():
    bus = PeerStateBus(3, comm_range=10.0, latency_mean=0.0, latency_std=0.0)
    bus.set_graph({0: {1}, 1: {0, 2}, 2: {1}})
    positions = [np.array([0.0, 0.0]), np.array([2.0, 0.0]), np.array([4.0, 0.0])]
    velocities = [np.zeros(2) for _ in range(3)]
    bus.publish(positions, velocities, [True] * 3, now=0.0)

    assert [n["id"] for n in bus.neighbors(0, 0.0)] == [1]
    assert [n["id"] for n in bus.neighbors(1, 0.0)] == [0, 2]
    assert bus.neighbors(0, 0.0)[0]["id"] != 2
    assert bus.neighbors(1, 0.51) == []


def test_peer_bus_can_block_one_directed_link_without_partitioning_other_peer():
    bus = PeerStateBus(3, comm_range=10.0, latency_mean=0.0, latency_std=0.0)
    bus.set_graph({0: {1}, 1: {0, 2}, 2: {1}})
    bus.block_link(0, 1)
    positions = [np.array([0.0, 0.0]), np.array([2.0, 0.0]), np.array([4.0, 0.0])]
    velocities = [np.zeros(2) for _ in range(3)]
    bus.publish(positions, velocities, [True] * 3, now=0.0)

    assert [n["id"] for n in bus.neighbors(1, 0.0)] == [2]
    assert [n["id"] for n in bus.neighbors(0, 0.0)] == [1]
    assert [n["id"] for n in bus.neighbors(2, 0.0)] == [1]


def test_peer_bus_does_not_deliver_before_configured_latency():
    bus = PeerStateBus(2, comm_range=10.0, latency_mean=0.1, latency_std=0.0)
    bus.set_graph({0: {1}, 1: {0}})
    positions = [np.array([0.0, 0.0]), np.array([2.0, 0.0])]
    velocities = [np.zeros(2) for _ in range(2)]
    bus.publish(positions, velocities, [True, True], now=0.0)
    assert bus.neighbors(0, 0.05) == []

    bus.publish(positions, velocities, [True, True], now=0.1)
    assert [n["id"] for n in bus.neighbors(0, 0.1)] == [1]


def test_peer_bus_does_not_refresh_old_telemetry_at_relay_time():
    bus = PeerStateBus(2, comm_range=10.0, latency_mean=0.0, latency_std=0.0)
    bus.set_graph({0: {1}, 1: {0}})
    positions = [np.array([0.0, 0.0]), np.array([2.0, 0.0])]
    velocities = [np.zeros(2) for _ in range(2)]
    bus.publish(positions, velocities, [True, True], now=1.0,
                source_times=[0.4, 1.0])
    assert [n["id"] for n in bus.neighbors(1, 1.0)] == []
    assert bus.rejected_stale_source == 1


def test_peer_bus_rejects_late_arriving_older_sequence():
    from swarm_core.network import Packet

    bus = PeerStateBus(2, comm_range=10.0, latency_mean=0.0, latency_std=0.0)
    bus.set_graph({0: {1}, 1: {0}})
    bus.channel.in_flight_packets.extend([
        Packet(0, 1, 0.0, 0.0, {"position": [2.0, 0.0], "velocity": [0.0, 0.0],
                                "state_time": 0.0, "sequence": 2}),
        Packet(0, 1, 0.0, 0.0, {"position": [1.0, 0.0], "velocity": [0.0, 0.0],
                                "state_time": 0.0, "sequence": 1}),
    ])
    bus.publish([np.zeros(2), np.zeros(2)], [np.zeros(2), np.zeros(2)],
                [False, False], now=0.0)
    neighbors = bus.neighbors(1, 0.0)
    assert neighbors[0]["position"].tolist() == [2.0, 0.0]
    assert bus.rejected_out_of_order == 1


def test_decentralized_local_agent_holds_position_and_bounds_acceleration():
    from sitl.decentralized_guidance import LocalDroneAgent

    agent = LocalDroneAgent(0)
    target = agent.command(
        position=np.zeros(2), sample_time=1.0, goal=np.array([20.0, 0.0]),
        neighbor_states=[], desired_offsets={}, dt=0.1, now=1.0,
    )
    assert np.allclose(target, 0.0)
    assert np.linalg.norm(agent.drone.commanded_accel) <= agent.MAX_ACCEL + 1e-9


def test_independent_agent_gcs_heartbeat_is_rate_limited(monkeypatch):
    from sitl.independent_agent_node import IndependentAgentNode

    now = [10.0]
    monkeypatch.setattr("sitl.independent_agent_node.time.monotonic", lambda: now[0])
    node = object.__new__(IndependentAgentNode)
    node.conn = type("Connection", (), {"mav": MagicMock()})()
    node.last_gcs_heartbeat_at = 0.0
    node.last_peer_heartbeat_at = 0.0
    node.peer_ids = [1]
    node.discovery_ids = [1]
    node.peers = MagicMock()

    node._send_gcs_heartbeat()
    node._send_gcs_heartbeat()
    assert node.conn.mav.heartbeat_send.call_count == 1
    assert node.peers.publish_heartbeat.call_count == 1

    now[0] = 11.0
    node._send_gcs_heartbeat()
    assert node.conn.mav.heartbeat_send.call_count == 2
    assert node.peers.publish_heartbeat.call_count == 2


def test_udp_peer_liveness_is_independent_of_position_freshness():
    from sitl.peer_udp import UdpPeerStateTransport

    sender = UdpPeerStateTransport(0, 30310, 2)
    receiver = UdpPeerStateTransport(1, 30310, 2)
    try:
        sender.publish_heartbeat([1], 100.0)
        assert receiver.receive(100.1, [0]) == []
        assert receiver.fresh_link_ids(100.1) == [0]

        # A live agent can prove link liveness while its last position sample ages out.
        sender.publish_heartbeat([1], 101.0)
        sender.publish(np.array([1.0, 2.0]), np.zeros(2), 99.0, [1], 101.0)
        assert receiver.receive(101.1, [0]) == []
        assert receiver.fresh_link_ids(101.1) == [0]
        assert receiver.rejected_stale == 1
    finally:
        sender.close()
        receiver.close()


def test_udp_peer_delay_queue_is_bounded_and_keeps_newest_packets(monkeypatch):
    import sitl.peer_udp as peer_udp_module
    from sitl.peer_udp import UdpPeerStateTransport

    monkeypatch.setattr(peer_udp_module, "MAX_PENDING_PER_PEER", 4)
    monkeypatch.setattr(peer_udp_module, "MAX_PENDING_DATAGRAMS", 6)
    sender = UdpPeerStateTransport(0, 30410 + (os.getpid() % 1000), 3)
    try:
        for sequence in range(5):
            sender._queue_delivery(1, str(sequence).encode(), 1000.0 + sequence)
        assert len(sender.pending) == 4
        assert {entry[3] for entry in sender.pending} == {b"1", b"2", b"3", b"4"}

        sender._queue_delivery(2, b"peer-2-a", 1006.0)
        sender._queue_delivery(2, b"peer-2-b", 1007.0)
        sender._queue_delivery(1, b"peer-1-fresh", 1008.0)

        assert len(sender.pending) == 6
        assert sum(entry[2] == 1 for entry in sender.pending) <= 4
        assert sum(entry[2] == 2 for entry in sender.pending) <= 4
        assert {entry[3] for entry in sender.pending} == {
            b"2", b"3", b"4", b"peer-2-a", b"peer-2-b", b"peer-1-fresh",
        }
        assert sender.stats()["dropped_queue"] == 2
    finally:
        sender.close()


def test_udp_peer_jitter_is_seeded_and_changes_per_packet_delay():
    from sitl.peer_udp import UdpPeerStateTransport

    def scheduled(seed):
        sender = UdpPeerStateTransport(
            0, 30510 + (os.getpid() % 1000), 2,
            latency_mean=0.05, latency_std=0.02, seed=seed,
        )
        try:
            for index in range(8):
                sender._enqueue(1, str(index).encode(), 10.0)
            return ({entry[3]: entry[0] - 10.0 for entry in sender.pending},
                    sender.stats())
        finally:
            sender.close()

    first, first_stats = scheduled(331)
    second, second_stats = scheduled(331)
    assert first == second
    assert first_stats == second_stats
    assert len(set(first.values())) > 1
    assert all(delay >= 0 for delay in first.values())
    assert first_stats["injected_delay_samples"] == len(first)
    assert math.isclose(first_stats["injected_delay_mean_s"],
                        sum(first.values()) / len(first), rel_tol=1e-12)
    assert math.isclose(first_stats["injected_delay_min_s"], min(first.values()), rel_tol=1e-12)
    assert math.isclose(first_stats["injected_delay_max_s"], max(first.values()), rel_tol=1e-12)
    assert first_stats["injected_delay_std_s"] > 0


def test_udp_peer_rejects_nonfinite_delay_parameters():
    import pytest
    from sitl.peer_udp import UdpPeerStateTransport

    with pytest.raises(ValueError, match="finite and non-negative"):
        UdpPeerStateTransport(0, 30610, 2, latency_mean=float("nan"))
    with pytest.raises(ValueError, match="finite and non-negative"):
        UdpPeerStateTransport(0, 30610, 2, latency_std=float("inf"))


def test_udp_peer_rejects_nonfinite_fault_rates_and_reorder_delay():
    import pytest
    from sitl.peer_udp import UdpPeerStateTransport

    invalid = (
        ({"packet_loss_rate": float("nan")}, "packet loss rate"),
        ({"duplicate_rate": float("inf")}, "duplicate and reorder rates"),
        ({"reorder_rate": float("nan")}, "duplicate and reorder rates"),
        ({"reorder_delay": float("inf")}, "reorder delay"),
        ({"burst_start_rate": float("nan")}, "burst start and continuation rates"),
        ({"burst_continue_rate": 1.1}, "burst start and continuation rates"),
    )
    for options, message in invalid:
        with pytest.raises(ValueError, match=message):
            UdpPeerStateTransport(0, 30610, 2, **options)


def test_udp_peer_custom_burst_transitions_are_deterministic():
    from sitl.peer_udp import UdpPeerStateTransport

    transport = UdpPeerStateTransport(
        0, 30620, 1, burst_loss=True,
        burst_start_rate=1.0, burst_continue_rate=0.0, seed=1,
    )
    try:
        assert transport._drop_packet() is True
        assert transport._drop_packet() is False
        assert transport._drop_packet() is True
    finally:
        transport.close()


def test_udp_peer_state_propagates_local_landing_request():
    from sitl.peer_udp import UdpPeerStateTransport

    base = 35000 + (os.getpid() % 10000)
    sender = UdpPeerStateTransport(0, base, 2)
    receiver = UdpPeerStateTransport(1, base, 2)
    try:
        sender.publish([1.0, 2.0], [0.0, 0.0], 100.0, [1], 100.0, landing=True)
        states = receiver.receive(100.1, [0])
        assert len(states) == 1
        assert states[0]["landing"] is True
    finally:
        sender.close()
        receiver.close()


def test_udp_peer_state_age_is_corrected_across_host_clock_offsets():
    from sitl.peer_udp import UdpPeerStateTransport

    base = 36000 + (os.getpid() % 10000)
    sender = UdpPeerStateTransport(0, base, 2)
    receiver = UdpPeerStateTransport(1, base, 2)
    receiver.set_clock_offsets(20.0, {0: 60.0, 1: 20.0})
    try:
        sender.publish([1.0, 2.0], [0.0, 0.0], 160.0, [1], 160.0)
        states = receiver.receive(120.1, [0])
        assert len(states) == 1
        np.testing.assert_allclose(states[0]["age"], 0.1, atol=1e-9)
    finally:
        sender.close()
        receiver.close()


def test_udp_peer_accepts_small_clock_estimation_error_but_rejects_large_future_skew():
    from sitl.peer_udp import UdpPeerStateTransport

    base = 37000 + (os.getpid() % 10000)
    sender = UdpPeerStateTransport(0, base, 2)
    receiver = UdpPeerStateTransport(1, base, 2)
    try:
        # Separate clock probes can differ by milliseconds even on one host.
        receiver.set_clock_offsets(0.02, {0: 0.0, 1: 0.02})
        sender.publish_heartbeat([1], 100.0)
        sender.publish([1.0, 2.0], [0.0, 0.0], 100.0, [1], 100.0)
        states = receiver.receive(100.01, [0])
        assert len(states) == 1
        assert receiver.fresh_link_ids(100.01) == [0]

        receiver.set_clock_offsets(0.10, {0: 0.0, 1: 0.10})
        sender.publish_heartbeat([1], 101.0)
        sender.publish([2.0, 3.0], [0.0, 0.0], 101.0, [1], 101.0)
        assert receiver.receive(101.01, [0]) == []
        assert receiver.rejected_stale >= 2
    finally:
        sender.close()
        receiver.close()


def test_clock_offset_estimator_uses_four_timestamps_and_lowest_delay_sample():
    from sitl.clock_sync import best_clock_sample, estimate_clock_offset

    offset, round_trip = estimate_clock_offset(100.0, 102.1, 102.11, 100.21)
    np.testing.assert_allclose(offset, 2.0, atol=1e-9)
    np.testing.assert_allclose(round_trip, 0.2, atol=1e-9)
    best_offset, uncertainty = best_clock_sample([(1.5, 0.2), (2.0, 0.04)])
    assert best_offset == 2.0
    assert uncertainty == 0.02


def test_clock_offset_error_stays_within_half_rtt_under_asymmetric_path_delay():
    from sitl.clock_sync import best_clock_sample, estimate_clock_offset

    true_offset = 7.0
    operator_send = 100.0
    agent_processing = 0.003
    for forward_delay, reverse_delay in ((0.010, 0.190), (0.190, 0.010), (0.200, 0.0)):
        agent_receive = operator_send + forward_delay + true_offset
        agent_send = agent_receive + agent_processing
        operator_receive = (operator_send + forward_delay + agent_processing + reverse_delay)

        estimated_offset, network_rtt = estimate_clock_offset(
            operator_send, agent_receive, agent_send, operator_receive,
        )
        uncertainty = best_clock_sample([(estimated_offset, network_rtt)])[1]

        assert abs(estimated_offset - true_offset) <= uncertainty + 1e-9
        np.testing.assert_allclose(network_rtt, forward_delay + reverse_delay, atol=1e-9)


def test_clock_offset_estimator_rejects_agent_wall_clock_step_during_probe():
    from sitl.clock_sync import estimate_clock_offset

    with pytest.raises(ValueError, match="agent wall clock changed during probe"):
        estimate_clock_offset(
            100.0, 103.02, 103.08, 100.1, agent_processing_s=0.001,
        )


def test_mission_lease_uses_operator_clock_offset_when_validating_remote_intent(monkeypatch):
    import time
    from sitl.independent_agent_node import IndependentAgentNode
    from sitl.mission_intent import MissionIntent, MissionLease

    operator_now = time.time()
    local_now = operator_now + 75.0
    monotonic_now = 400.0
    intent = MissionIntent(1, operator_now - 1.0, operator_now + 20.0,
                           (3.0, -2.0)).to_dict()

    class Operator:
        def poll(self):
            return [{"version": 1, "type": "mission", "epoch": "operator-session-a",
                     "intent": intent}]

    node = object.__new__(IndependentAgentNode)
    node.operator = Operator()
    node.operator_clock_offset_s = 75.0
    node.clock_sync_epoch = "operator-session-a"
    node.last_mission_sequence = 0
    node.mission_lease = MissionLease()
    node.current_mission = None
    node.pending_intent = None
    node.landing = False
    node.airborne = False
    node.last_command = "ready"
    monkeypatch.setattr("sitl.independent_agent_node.time.time", lambda: local_now)
    monkeypatch.setattr("sitl.independent_agent_node.time.monotonic", lambda: monotonic_now)

    node._receive_operator_messages()

    assert node.last_mission_sequence == 1
    assert node.mission_lease.active(monotonic_now + 19.0) is not None
    assert node.mission_lease.active(monotonic_now + 20.0) is None


def test_independent_agent_guidance_uses_operator_clock_and_lands_on_invalid_intent():
    from unittest.mock import MagicMock
    from sitl.independent_agent_node import IndependentAgentNode
    from sitl.mission_intent import MissionIntent, MissionLease

    operator_now = 100.0
    local_now = 175.0
    intent = MissionIntent(1, operator_now - 1.0, operator_now + 20.0,
                           (3.0, -2.0))
    node = object.__new__(IndependentAgentNode)
    node.operator_clock_offset_s = 75.0
    node.agent = MagicMock()
    node.agent.command.return_value = np.array([1.0, 2.0])
    node.position = np.array([0.0, 0.0, -5.0])
    node.position_at = local_now
    node.slot = np.zeros(3)
    node.all_slots = [np.zeros(3)]
    node.peer_ids = []
    node.pending_intent = intent.to_dict()
    node.current_mission = intent.to_dict()
    node.mission_lease = MissionLease()
    node.mission_lease.accept(intent.to_dict(), operator_now, 50.0)
    node.safety_state = "normal"
    node.safety_reason = None
    node._send_position_target = MagicMock()
    node._command_land = MagicMock()

    node._run_guidance(intent, local_now, 10.1, 10.0, [])

    assert node.agent.command.call_args.kwargs["now"] == operator_now
    node._send_position_target.assert_called_once()
    node._command_land.assert_not_called()

    node.pending_intent = None
    node.agent.command.side_effect = ValueError("mission intent has expired")
    node._run_guidance(intent, local_now, 10.2, 10.1, [])

    node._command_land.assert_called_once_with("invalid_mission_intent")
    assert node.mission_lease.active(50.1) is None
    assert node.current_mission is None
    assert node.safety_state == "mission_failsafe"


def test_clock_sync_accepts_new_operator_epoch_and_rejects_retired_epoch():
    from unittest.mock import MagicMock
    from sitl.independent_agent_node import IndependentAgentNode

    offsets = {"0": 4.0, "1": -2.0}
    messages = [
        {"version": 1, "type": "clock_sync", "sequence": 1,
         "epoch": "operator-session-a", "offsets_s": offsets},
        {"version": 1, "type": "clock_sync", "sequence": 1,
         "epoch": "operator-session-b", "offsets_s": offsets},
        {"version": 1, "type": "clock_sync", "sequence": 2,
         "epoch": "operator-session-a", "offsets_s": {"0": 90.0, "1": 90.0}},
    ]

    class Operator:
        def poll(self):
            return messages

    node = object.__new__(IndependentAgentNode)
    node.drone_id = 0
    node.count = 2
    node.operator = Operator()
    node.last_clock_sync_sequence = 0
    node.clock_sync_epoch = None
    node.retired_clock_sync_epochs = []
    node.last_mission_sequence = 8
    node.last_operator_sequence = 9
    node.last_operator_action = "land"
    node.last_network_fault_sequence = 10
    node.operator_clock_offset_s = 0.0
    node.peer_clock_offsets_s = {0: 0.0, 1: 0.0}
    node.peers = MagicMock()

    node._receive_operator_messages()

    assert node.clock_sync_epoch == "operator-session-b"
    assert node.last_clock_sync_sequence == 1
    assert node.last_mission_sequence == 0
    assert node.last_operator_sequence == 0
    assert node.last_operator_action is None
    assert node.last_network_fault_sequence == 0
    assert node.operator_clock_offset_s == 4.0
    assert node.peer_clock_offsets_s == {0: 4.0, 1: -2.0}
    assert node.retired_clock_sync_epochs == ["operator-session-a"]


def test_restarted_operator_epoch_resets_sequences_and_rejects_old_commands(monkeypatch):
    import time
    from unittest.mock import MagicMock
    from sitl.independent_agent_node import IndependentAgentNode
    from sitl.mission_intent import MissionIntent, MissionLease
    from sitl.operator_udp import decode_message

    now = time.time()
    intent = MissionIntent(1, now - 0.1, now + 20.0, (1.0, -2.0)).to_dict()
    messages = [
        {"version": 1, "type": "clock_sync", "sequence": 1,
         "epoch": "replacement-console", "offsets_s": {"0": 0.0}},
        {"version": 1, "type": "mission", "epoch": "retired-console", "intent": {
            **intent, "sequence": 15,
        }},
        {"version": 1, "type": "mission", "epoch": "replacement-console",
         "intent": intent},
        {"version": 1, "type": "command", "sequence": 10,
         "epoch": "retired-console", "action": "launch",
         "issued_at": now - 0.1, "expires_at": now + 4.0},
        {"version": 1, "type": "command", "sequence": 1,
         "epoch": "replacement-console", "action": "land",
         "issued_at": now - 0.1, "expires_at": now + 4.0},
    ]

    class Operator:
        def poll(self):
            return messages

    acknowledgements = []

    class StatusSocket:
        def sendto(self, payload, endpoint):
            acknowledgements.append(decode_message(payload))

    node = object.__new__(IndependentAgentNode)
    node.drone_id = 0
    node.count = 1
    node.operator = Operator()
    node.operator_clock_offset_s = 0.0
    node.peer_clock_offsets_s = {0: 0.0}
    node.peers = MagicMock()
    node.clock_sync_epoch = "retired-console"
    node.retired_clock_sync_epochs = []
    node.last_clock_sync_sequence = 4
    node.last_mission_sequence = 14
    node.last_operator_sequence = 9
    node.last_operator_action = "launch"
    node.last_network_fault_sequence = 3
    node.mission_lease = MissionLease()
    node.current_mission = None
    node.pending_intent = None
    node.landing = False
    node.airborne = False
    node.last_command = "ready"
    node.operator_auth_key = None
    node.status_endpoint = ("127.0.0.1", 22000)
    node.status_socket = StatusSocket()
    landed = []
    node._command_land = landed.append

    node._receive_operator_messages()

    assert node.last_mission_sequence == 1
    assert node.current_mission["sequence"] == 1
    assert node.last_operator_sequence == 1
    assert landed == ["operator"]
    assert len(acknowledgements) == 1
    assert acknowledgements[0]["epoch"] == "replacement-console"
    assert node.retired_clock_sync_epochs == ["retired-console"]


def test_operator_command_expiry_uses_measured_clock_offset(monkeypatch):
    import time
    from sitl.independent_agent_node import IndependentAgentNode
    from sitl.operator_udp import decode_message

    operator_now = time.time()
    node_now = operator_now - 42.0
    messages = [{
        "version": 1, "type": "command", "sequence": 1, "epoch": "session-a",
        "action": "land",
        "issued_at": operator_now - 0.5, "expires_at": operator_now + 5.0,
    }]

    class Operator:
        def poll(self):
            return messages

    node = object.__new__(IndependentAgentNode)
    node.operator = Operator()
    node.operator_clock_offset_s = -42.0
    node.clock_sync_epoch = "session-a"
    node.last_operator_sequence = 0
    node.last_operator_action = None
    node.drone_id = 0
    node.operator_auth_key = None
    node.status_endpoint = ("127.0.0.1", 22000)
    acknowledgements = []

    class StatusSocket:
        def sendto(self, payload, endpoint):
            acknowledgements.append((decode_message(payload), endpoint))

    node.status_socket = StatusSocket()
    requested = []
    node._command_land = requested.append
    monkeypatch.setattr("sitl.independent_agent_node.time.time", lambda: node_now)

    node._receive_operator_messages()

    assert node.last_operator_sequence == 1
    assert requested == ["operator"]
    assert acknowledgements == [({"version": 1, "type": "command_ack", "drone_id": 0,
                                 "sequence": 1, "action": "land", "epoch": "session-a",
                                 "received_at": node_now},
                                node.status_endpoint)]


def test_peer_link_watchdog_debounces_missing_peer_and_resets_after_recovery():
    from sitl.peer_udp import PeerLinkWatchdog

    watchdog = PeerLinkWatchdog([1], timeout_s=2.0)
    assert watchdog.update(9.0, [1]) == []
    assert watchdog.update(10.0, []) == []
    assert watchdog.update(11.9, []) == []
    assert watchdog.update(12.0, []) == [1]
    assert watchdog.update(12.1, [1]) == []
    assert watchdog.update(13.9, []) == []


def test_peer_link_watchdog_monitors_required_peers_never_seen_before_takeoff():
    from sitl.peer_udp import PeerLinkWatchdog

    watchdog = PeerLinkWatchdog([1, 2], timeout_s=2.0)
    assert watchdog.update(10.0, []) == []  # Grounded preflight does not trigger.
    watchdog.deactivate()
    watchdog.activate(20.0, [1])
    assert watchdog.update(21.9, [1]) == []
    assert watchdog.update(22.0, [1]) == [2]
    assert watchdog.update(22.1, [1, 2]) == []


def test_peer_link_watchdog_applies_startup_grace_before_timeout():
    from sitl.peer_udp import PeerLinkWatchdog

    watchdog = PeerLinkWatchdog([1], timeout_s=2.0, startup_grace_s=5.0)
    watchdog.activate(10.0, [])
    assert watchdog.update(14.9, []) == []
    assert watchdog.update(16.9, []) == []
    assert watchdog.update(17.0, []) == [1]


def test_independent_agent_lands_after_required_peer_timeout_only_while_airborne():
    from sitl.independent_agent_node import IndependentAgentNode
    from sitl.peer_udp import PeerLinkWatchdog

    node = object.__new__(IndependentAgentNode)
    node.peer_watchdog = PeerLinkWatchdog([1], timeout_s=2.0)
    node.peer_timeout_ids = []
    node.airborne = False
    node.landing = False
    node.safety_state = "nominal"
    node.safety_reason = None
    landed_for = []
    node._command_land = landed_for.append

    node._check_peer_failsafe(1.0, [1])
    node._check_peer_failsafe(10.0, [1])  # Preflight absence does not age the timer.
    assert landed_for == []

    node.airborne = True
    node._check_peer_failsafe(20.0, [])
    node._check_peer_failsafe(21.9, [])
    assert landed_for == []
    node._check_peer_failsafe(22.0, [])
    assert landed_for == ["isolated_peer_timeout:2"]
    assert node.safety_state == "peer_isolation_failsafe"


def test_independent_agent_reconfigures_around_one_missing_peer_and_recovers():
    from sitl.independent_agent_node import IndependentAgentNode
    from sitl.peer_udp import PeerLinkWatchdog

    node = object.__new__(IndependentAgentNode)
    node.peer_ids = [1, 2]
    node.peer_watchdog = PeerLinkWatchdog([1, 2], timeout_s=2.0)
    node.peer_timeout_ids = []
    node.airborne = True
    node.landing = False
    node.safety_state = "nominal"
    node.safety_reason = None
    node.last_command = "airborne"
    landed_for = []
    node._command_land = landed_for.append

    node._check_peer_failsafe(10.0, [2])
    node._check_peer_failsafe(12.0, [2])
    assert landed_for == []
    assert node.peer_timeout_ids == [1]
    assert node.safety_state == "peer_link_degraded"
    assert node.safety_reason == "peer_link_degraded:2"

    node._check_peer_failsafe(12.1, [1, 2])
    assert node.safety_state == "nominal"
    assert node.safety_reason is None
    assert node.last_command == "peer_links_restored"


def test_independent_agent_launch_preflight_requires_mission_and_fresh_neighbors():
    import time
    from sitl.independent_agent_node import IndependentAgentNode
    from sitl.mission_intent import MissionIntent, MissionLease

    node = object.__new__(IndependentAgentNode)
    node.mission_lease = MissionLease()
    node.peer_ids = [0, 1, 2]

    class Peers:
        fresh = [0, 2]

        def receive(self, _now, _peer_ids):
            return []

        def fresh_link_ids(self, _now):
            return self.fresh

    node.peers = Peers()
    assert node._launch_preflight_failure() == "launch_rejected_no_valid_mission"

    wall_now = time.time()
    intent = MissionIntent(1, wall_now, wall_now + 30.0, (0.0, 0.0))
    node.mission_lease.accept(intent.to_dict(), wall_now, time.monotonic())
    assert node._launch_preflight_failure() == "launch_rejected_peer_links:2"
    node.peers.fresh = [0, 1, 2]
    assert node._launch_preflight_failure() is None


def test_agent_detects_existing_armed_airborne_vehicle_after_process_restart():
    from sitl.independent_agent_node import IndependentAgentNode
    from pymavlink import mavutil

    class Message:
        def __init__(self, kind, **fields):
            self.kind = kind
            self.__dict__.update(fields)

        def get_type(self):
            return self.kind

    class Connection:
        target_system = 1
        target_component = 1

        def __init__(self):
            self.messages = [
                Message("HEARTBEAT", base_mode=mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED),
                Message("GLOBAL_POSITION_INT", lat=0, lon=0, relative_alt=2000),
            ]

        def recv_match(self, blocking=False):
            return self.messages.pop(0) if self.messages else None

    node = object.__new__(IndependentAgentNode)
    node.conn = Connection()
    node.frame = type("Frame", (), {
        "gps_to_global_ned": lambda _self, *_args: np.array([0.0, 0.0, -2.0]),
    })()
    node.last_heartbeat_at = 0.0
    node.armed = False
    node.airborne = False
    node.position = None
    node.position_at = 0.0
    node.local_position = None
    node.global_origin = None
    node.landing = False
    node.recovery_state = "normal"
    node.safety_state = "nominal"
    node.safety_reason = None
    node.last_command = "booting"
    land_reasons = []
    node._command_land = land_reasons.append

    node._read_vehicle()

    assert node.armed
    assert node.airborne
    assert node.recovery_state == "landing_after_process_restart"
    assert node.safety_state == "agent_restart_failsafe"
    assert land_reasons == ["agent_process_restarted_while_airborne"]


def test_airborne_agent_observes_peer_landing_without_copying_land_command():
    from sitl.independent_agent_node import IndependentAgentNode
    from sitl.peer_udp import PeerLinkWatchdog

    node = object.__new__(IndependentAgentNode)
    node.peer_ids = [1]
    node.peer_watchdog = PeerLinkWatchdog([1])
    node.peer_timeout_ids = []
    node.airborne = True
    node.landing = False
    node.safety_state = "nominal"
    node.safety_reason = None
    node.last_command = "airborne"
    landed_for = []
    node._command_land = landed_for.append

    node._check_peer_failsafe(10.0, [1], [{"id": 1, "landing": True}])
    assert node.safety_state == "peer_landing_observed"
    assert node.safety_reason == "peer_landing_observed:2"
    assert landed_for == []


def test_new_mission_refresh_does_not_erase_landed_status():
    import time
    from sitl.independent_agent_node import IndependentAgentNode
    from sitl.mission_intent import MissionIntent, MissionLease

    now = time.time()
    intent = MissionIntent(1, now, now + 30.0, (2.0, 0.0)).to_dict()

    class Operator:
        def poll(self):
            return [{"version": 1, "type": "mission", "epoch": "session-a",
                     "intent": intent}]

    node = object.__new__(IndependentAgentNode)
    node.operator = Operator()
    node.operator_clock_offset_s = 0.0
    node.clock_sync_epoch = "session-a"
    node.last_mission_sequence = 0
    node.mission_lease = MissionLease()
    node.current_mission = None
    node.pending_intent = None
    node.landing = False
    node.airborne = False
    node.last_command = "landed"

    node._receive_operator_messages()

    assert node.last_mission_sequence == 1
    assert node.current_mission["goal_ne"] == [2.0, 0.0]
    assert node.last_command == "landed"


def test_hybrid_local_agent_degrades_and_recovers_with_hysteresis():
    from sitl.decentralized_guidance import LocalDroneAgent
    from swarm_core.controllers.hybrid import HybridMode

    agent = LocalDroneAgent(0, hybrid=True)
    agent.command(np.zeros(2), 100.0, np.array([1.0, 0.0]), [], {}, 0.1,
                  coordinator_connected=True, now=100.0)
    agent.command(np.zeros(2), 100.6, np.array([1.0, 0.0]), [], {}, 0.1,
                  coordinator_connected=False, now=100.6)
    assert agent.hybrid_controller.get_drone_mode(0) == HybridMode.DECENTRALIZED_FALLBACK

    for tick in range(1, 31):
        now = 100.6 + tick * 0.1
        agent.command(np.zeros(2), now, np.array([1.0, 0.0]), [], {}, 0.1,
                      coordinator_connected=True, now=now)
    assert agent.hybrid_controller.get_drone_mode(0) == HybridMode.CENTRALIZED


def test_settled_detector_waits_for_low_motion_dwell_but_does_not_hide_error():
    detector = SettledMotionDetector(speed_threshold_m_s=0.15, dwell_s=1.0, command_quiet_s=0.5)
    active = [True]
    fresh = [True]
    positions = [[3.0, 0.0, 0.0]]  # Can be still settled even if target is at the origin.
    assert not detector.update(1.0, 0.0, positions, [1.0], active, fresh)
    assert not detector.update(1.5, 0.0, positions, [1.5], active, fresh)
    assert not detector.update(2.0, 0.0, positions, [2.0], active, fresh)
    assert detector.update(3.1, 0.0, positions, [3.1], active, fresh)


def test_settled_detector_resets_after_motion_or_new_command():
    detector = SettledMotionDetector(speed_threshold_m_s=0.15, dwell_s=0.5, command_quiet_s=0.0)
    active = [True]
    fresh = [True]
    assert not detector.update(1.0, 0.0, [[0.0, 0.0]], [1.0], active, fresh)
    assert not detector.update(1.6, 0.0, [[0.0, 0.0]], [1.6], active, fresh)
    assert detector.update(2.2, 0.0, [[0.0, 0.0]], [2.2], active, fresh)
    assert not detector.update(2.3, 2.3, [[0.0, 0.0]], [2.3], active, fresh)
    assert not detector.update(2.4, 2.3, [[1.0, 0.0]], [2.4], active, fresh)


def test_hybrid_outage_freezes_altitude_and_slots():
    from sitl.decentralized_guidance import DecentralizedGuidance
    g = DecentralizedGuidance(1, control_mode='hybrid')
    g.step([np.array([0.,0.,-5.])],[10.],np.array([0.,0.,-5.]),
           [np.zeros(3)],[True],[True],now=10.)
    result = g.step([np.array([0.,0.,-5.])],[11.],np.array([50.,50.,-20.]),
                    [np.array([20.,20.,-10.])],[True],[True],now=11.,coordinator_connected=False)
    assert result[0][2] == -5.
    assert np.linalg.norm(result[0][:2]) < 1e-8


def test_process_agent_matches_local_and_closes():
    from sitl.decentralized_guidance import LocalDroneAgent
    from sitl.agent_process import ProcessDroneAgent
    from sitl.mission_intent import MissionIntent
    local = LocalDroneAgent(0,hybrid=True)
    remote = ProcessDroneAgent(0,hybrid=True)
    try:
        for tick in range(12):
            now = 10 + tick * .1
            args=dict(position=np.zeros(2),sample_time=now,goal=None,
                      neighbor_states=[],desired_offsets={},dt=.1,now=10+tick*.1,
                      coordinator_connected=tick<3,
                      mission_intent=MissionIntent(tick+1,now,now+1,(1.,0.)).to_dict(),
                      slot_offset=np.array([.5,0.]))
            np.testing.assert_allclose(remote.command(**args),local.command(**args))
        assert remote.hybrid_controller.get_drone_mode(0) == local.hybrid_controller.get_drone_mode(0)
    finally:
        remote.close()
    assert remote.process.poll() is not None


def test_mission_intent_is_shared_but_each_agent_applies_its_own_slot():
    from sitl.decentralized_guidance import LocalDroneAgent
    from sitl.mission_intent import MissionIntent

    common = MissionIntent(1, 10.0, 11.0, (10.0, 4.0)).to_dict()
    left = LocalDroneAgent(0)
    right = LocalDroneAgent(1)
    # Mission/slot mapping is checked while local peer awareness is available;
    # a truly isolated agent should hold/brake until peer state returns.
    peers = [{"id": 2, "position": np.array([0.0, 4.0]), "velocity": np.zeros(2)}]
    left_target = left.command(
        np.zeros(2), 10.0, None, peers, {}, 0.1, now=10.0,
        mission_intent=common, slot_offset=np.array([-2.0, 0.0]),
    )
    right_target = right.command(
        np.zeros(2), 10.0, None, peers, {}, 0.1, now=10.0,
        mission_intent=common, slot_offset=np.array([2.0, 0.0]),
    )
    assert left_target[0] < right_target[0]
    assert np.linalg.norm(left_target) > 0
    assert np.linalg.norm(right_target) > 0


def test_mission_intent_rejects_expired_and_replayed_messages():
    import pytest
    from sitl.decentralized_guidance import LocalDroneAgent
    from sitl.mission_intent import MissionIntent

    with pytest.raises(ValueError, match="expired"):
        MissionIntent.from_dict(MissionIntent(1, 1.0, 2.0, (0.0, 0.0)).to_dict(), 2.0)
    future = MissionIntent(1, 10.3, 20.0, (0.0, 0.0)).to_dict()
    with pytest.raises(ValueError, match="future"):
        MissionIntent.from_dict(future, 10.0)

    agent = LocalDroneAgent(0)
    message = MissionIntent(1, 10.0, 11.0, (1.0, 0.0)).to_dict()
    args = dict(position=np.zeros(2), sample_time=10.0, goal=None,
                neighbor_states=[], desired_offsets={}, dt=0.1, now=10.0,
                mission_intent=message, slot_offset=np.zeros(2))
    agent.command(**args)
    with pytest.raises(ValueError, match="replayed"):
        agent.command(**args)


def test_mission_lease_uses_monotonic_expiry_after_wall_clock_acceptance():
    from sitl.mission_intent import MissionIntent, MissionLease

    intent = MissionIntent(1, 100.0, 110.0, (2.0, 0.0))
    lease = MissionLease()
    lease.accept(intent.to_dict(), wall_now=100.0, monotonic_now=50.0)
    # A wall-clock rollback after receipt must not extend the accepted intent's lifetime.
    assert lease.active(59.9) == intent
    assert lease.active(60.0) is None
    assert lease.active(70.0) is None


def test_agent_keeps_an_accepted_mission_without_per_tick_refresh():
    from sitl.decentralized_guidance import LocalDroneAgent
    from sitl.mission_intent import MissionIntent

    agent = LocalDroneAgent(0)
    intent = MissionIntent(1, 10.0, 20.0, (5.0, 0.0)).to_dict()
    peers = [{"id": 1, "position": np.array([0.0, 4.0]), "velocity": np.zeros(2)}]
    first = agent.command(
        np.zeros(2), 10.0, None, peers, {}, 0.1, now=10.0,
        mission_intent=intent, slot_offset=np.array([1.0, 0.0]),
    )
    next_tick = agent.command(
        np.zeros(2), 10.1, None, peers, {}, 0.1, now=10.1,
    )
    assert agent.last_mission_sequence == 1
    assert np.linalg.norm(first) > 0 and np.linalg.norm(next_tick) > 0


def test_hybrid_local_agent_falls_back_on_coordinator_loss_and_recovers_with_hysteresis():
    from sitl.decentralized_guidance import LocalDroneAgent
    from sitl.mission_intent import MissionIntent

    agent = LocalDroneAgent(0, hybrid=True)
    intent = MissionIntent(1, 10.0, 40.0, (5.0, 0.0)).to_dict()
    for tick in range(8):
        now = 10.0 + tick * 0.1
        agent.command(
            np.zeros(2), now, None, [], {}, 0.1, now=now,
            mission_intent=intent if tick == 0 else None,
            slot_offset=np.zeros(2) if tick == 0 else None,
            coordinator_connected=True,
        )

    # A missing coordinator heartbeat outlives the controller's 0.5 s degrade timeout.
    for tick in range(6):
        now = 10.7 + (tick + 1) * 0.12
        agent.command(np.zeros(2), now, None, [], {}, 0.12, now=now,
                      coordinator_connected=False)
    assert agent.hybrid_controller.get_drone_mode(0).value == "decentralized_fallback"
    assert agent.hybrid_controller.get_alpha(0) < 1.0

    # Recovery uses fresh sequential coordinator heartbeats plus the minimum fallback dwell.
    for tick in range(30):
        now = 12.0 + tick * 0.1
        agent.command(np.zeros(2), now, None, [], {}, 0.1, now=now,
                      coordinator_connected=True)
    assert agent.hybrid_controller.get_drone_mode(0).value == "centralized"
    assert 0.0 <= agent.hybrid_controller.get_alpha(0) <= 1.0


def test_cached_mission_expires_and_altitude_is_bounded():
    import pytest
    from sitl.decentralized_guidance import LocalDroneAgent
    from sitl.mission_intent import MissionIntent

    agent = LocalDroneAgent(0)
    intent = MissionIntent(1, 10.0, 20.0, (5.0, 0.0)).to_dict()
    agent.command(np.zeros(2), 10.0, None, [], {}, 0.1, now=10.0,
                  mission_intent=intent, slot_offset=np.zeros(2))
    with pytest.raises(ValueError, match="expired"):
        agent.command(np.zeros(2), 20.1, None, [], {}, 0.1, now=20.1)
    invalid_altitude = MissionIntent(2, 10.0, 11.0, (0.0, 0.0), target_alt_m=26.0).to_dict()
    with pytest.raises(ValueError, match="altitude"):
        MissionIntent.from_dict(invalid_altitude, 10.1)


def test_udp_peer_agents_exchange_state_without_host_peer_bus_relay():
    from sitl.decentralized_guidance import DecentralizedGuidance

    guidance = DecentralizedGuidance(2, agent_runtime="process", peer_loss=0.1,
                                    peer_latency_ms=50.0, peer_seed=44)
    try:
        positions = [np.array([0.0, 0.0, -5.0]), np.array([4.0, 0.0, -5.0])]
        slots = [np.array([-2.0, 0.0, 0.0]), np.array([2.0, 0.0, 0.0])]
        first = guidance.step(positions, [10.0, 10.0], np.array([0.0, 0.0, -5.0]),
                              slots, [True, True], [True, True], now=10.0)
        second = guidance.step(positions, [10.1, 10.1], np.array([0.0, 0.0, -5.0]),
                               slots, [True, True], [True, True], now=10.1)
        assert all(target is not None for target in first)
        assert all(target is not None for target in second)
        assert guidance.peer_bus.channel.total_transmitted == 0
        assert sum(agent.last_peer_metrics.get("received", 0)
                   for agent in guidance.agents) > 0
    finally:
        guidance.close()


def test_direct_udp_transport_can_drop_one_directed_link_only():
    from sitl.peer_udp import UdpPeerStateTransport

    base = 30000 + (os.getpid() % 10000)
    agents = [
        UdpPeerStateTransport(0, base, 3, blocked_links={(0, 1)}),
        UdpPeerStateTransport(1, base, 3),
        UdpPeerStateTransport(2, base, 3),
    ]
    try:
        agents[0].publish([1.0, 2.0], [0.0, 0.0], 1.0, [1, 2], 1.0)
        assert agents[1].receive(1.0, [0, 2]) == []
        received = agents[2].receive(1.0, [0, 1])
        assert [entry["id"] for entry in received] == [0]
        assert agents[0].stats()["dropped_blocked"] == 1
        assert agents[0].stats()["sent"] == 1
        assert agents[0].set_link_blocked(0, 1, False)
        agents[0].publish([1.1, 2.0], [0.0, 0.0], 1.1, [1], 1.1)
        restored = agents[1].receive(1.1, [0, 2])
        assert [entry["id"] for entry in restored] == [0]
        assert agents[0].stats()["blocked_link_changes"] == 1
    finally:
        for agent in agents:
            agent.close()


def test_direct_udp_transport_duplicates_are_rejected_as_replays():
    from sitl.peer_udp import UdpPeerStateTransport

    base = 31000 + (os.getpid() % 10000)
    sender = UdpPeerStateTransport(0, base, 2, duplicate_rate=1.0, seed=31)
    receiver = UdpPeerStateTransport(1, base, 2)
    try:
        sender.publish([1.0, 2.0], [0.0, 0.0], 1.0, [1], 1.0)
        assert len(receiver.receive(1.0, [0])) == 1
        sender._flush(1.1)
        assert receiver.receive(1.1, [0]) == []
        assert receiver.stats()["rejected_replay"] == 1
        assert sender.stats()["duplicated_packets"] == 1
    finally:
        sender.close()
        receiver.close()


def test_direct_udp_transport_accepts_rejoining_agent_and_rejects_retired_session():
    import socket
    from sitl.peer_udp import FRAME, VERSION, UdpPeerStateTransport

    base = 34000 + (os.getpid() % 10000)
    receiver = UdpPeerStateTransport(1, base, 2)
    first_session = UdpPeerStateTransport(0, base, 2)
    restarted_session = None
    injector = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        first_id = first_session.session_id
        first_session.publish_heartbeat([1], 100.0)
        first_session.publish([1.0, 2.0], [0.0, 0.0], 100.0, [1], 100.0)
        assert len(receiver.receive(100.1, [0])) == 1
        assert receiver.fresh_link_ids(100.1) == [0]

        # A delayed packet from the old process must not regain authority after rejoin.
        old_packet = first_session._encode({
            "version": VERSION,
            "frame": FRAME,
            "drone_id": 0,
            "session_id": first_id,
            "sequence": 99,
            "sample_time": 100.15,
            "landing": False,
            "position": [99.0, 99.0],
            "velocity": [0.0, 0.0],
        })
        first_session.close()
        restarted_session = UdpPeerStateTransport(0, base, 2)
        assert restarted_session.session_id != first_id
        restarted_session.publish([3.0, 4.0], [0.0, 0.0], 100.2, [1], 100.2)
        rejoined = receiver.receive(100.2, [0])
        assert len(rejoined) == 1
        assert rejoined[0]["position"].tolist() == [3.0, 4.0]
        assert receiver.peer_session_ids[0] == restarted_session.session_id

        restarted_session.publish_heartbeat([1], 100.25)
        receiver.receive(100.25, [0])
        # Send from the expected peer endpoint so this specifically tests retired-session logic.
        restarted_session.socket.sendto(old_packet, receiver.bind_address)
        assert receiver.receive(100.3, [0]) == []
        assert receiver.stats()["rejected_replay"] == 1
        assert receiver.fresh_link_ids(100.3) == [0]
    finally:
        first_session.close()
        if restarted_session is not None:
            restarted_session.close()
        receiver.close()
        injector.close()


def test_direct_udp_transport_delayed_old_sequence_cannot_overwrite_newer_state():
    from sitl.peer_udp import UdpPeerStateTransport

    base = 32000 + (os.getpid() % 10000)
    sender = UdpPeerStateTransport(0, base, 2, reorder_rate=1.0,
                                   reorder_delay=0.2, seed=32)
    receiver = UdpPeerStateTransport(1, base, 2)
    try:
        sender.publish([1.0, 2.0], [0.0, 0.0], 1.0, [1], 1.0)
        sender.reorder_rate = 0.0
        sender.publish([2.0, 3.0], [0.0, 0.0], 1.01, [1], 1.01)
        newest = receiver.receive(1.01, [0])
        assert len(newest) == 1
        assert newest[0]["position"].tolist() == [2.0, 3.0]
        sender._flush(1.5)
        assert receiver.receive(1.5, [0]) == []
        assert receiver.stats()["rejected_replay"] == 1
        assert sender.stats()["reorder_delayed_packets"] == 1
    finally:
        sender.close()
        receiver.close()


def test_direct_udp_transport_supports_routed_addresses_and_hmac_authentication():
    from sitl.peer_udp import UdpPeerStateTransport

    base = 33000 + (os.getpid() % 10000)
    endpoints = {0: ("127.0.0.2", base), 1: ("127.0.0.3", base)}
    key = b"swarm-test-key-32-bytes-minimum-0001"
    sender = UdpPeerStateTransport(
        0, base, 2, bind_address=endpoints[0], peer_endpoints=endpoints, auth_key=key,
    )
    receiver = UdpPeerStateTransport(
        1, base, 2, bind_address=endpoints[1], peer_endpoints=endpoints,
        auth_key=b"wrong-test-key-32-bytes-minimum-0001",
    )
    try:
        sender.publish([1.0, 2.0], [0.1, 0.2], 1.0, [1], 1.0)
        assert receiver.receive(1.0, [0]) == []
        assert receiver.stats()["rejected_auth"] == 1
        receiver.auth_key = key
        sender.publish([2.0, 3.0], [0.2, 0.3], 1.1, [1], 1.1)
        received = receiver.receive(1.1, [0])
        assert len(received) == 1
        assert received[0]["position"].tolist() == [2.0, 3.0]
    finally:
        sender.close()
        receiver.close()


def test_direct_udp_transport_rejects_authenticated_packet_from_unrostered_source():
    import socket
    from sitl.peer_udp import FRAME, VERSION, UdpPeerStateTransport

    base = 33500 + (os.getpid() % 10000)
    key = b"swarm-source-test-key-32-bytes-minimum"
    sender = UdpPeerStateTransport(0, base, 2, auth_key=key)
    receiver = UdpPeerStateTransport(1, base, 2, auth_key=key)
    impostor = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    impostor.bind(("127.0.0.1", base + 2))
    try:
        packet = sender._encode({
            "version": VERSION,
            "frame": FRAME,
            "drone_id": 0,
            "session_id": "a" * 32,
            "sequence": 1,
            "sample_time": 100.0,
            "landing": False,
            "position": [1.0, 2.0],
            "velocity": [0.0, 0.0],
        })
        impostor.sendto(packet, receiver.bind_address)

        assert receiver.receive(100.0, [0]) == []
        assert receiver.stats()["rejected_source"] == 1
        assert receiver.stats()["rejected_auth"] == 0
    finally:
        sender.close()
        receiver.close()
        impostor.close()


def test_peer_roster_validates_ids_and_builds_default_local_addresses(tmp_path):
    import json
    import pytest
    from sitl.peer_roster import load_peer_roster

    default = load_peer_roster(None, 2, 34000)
    assert default[0] == {
        "bind_host": "127.0.0.1", "host": "127.0.0.1", "port": 34000,
        "command_bind_host": "127.0.0.1", "command_host": "127.0.0.1",
        "command_port": 21000, "status_host": "127.0.0.1", "status_port": 22000,
    }
    roster_path = tmp_path / "roster.json"
    roster_path.write_text(json.dumps({
        "version": 1,
        "agents": [
            {"id": 0, "bind_host": "0.0.0.0", "host": "10.0.0.2", "port": 20000},
            {"id": 1, "bind_host": "0.0.0.0", "host": "10.0.0.3", "port": 20000},
        ],
    }))
    loaded = load_peer_roster(roster_path, 2)
    assert loaded[1] == {
        "bind_host": "0.0.0.0", "host": "10.0.0.3", "port": 20000,
        "command_bind_host": "127.0.0.1", "command_host": "127.0.0.1",
        "command_port": 21001, "status_host": "127.0.0.1", "status_port": 22001,
    }
    with pytest.raises(ValueError, match="exactly 3 agents"):
        load_peer_roster(roster_path, 3)


def test_operator_udp_hmac_requires_valid_signature_and_rejects_replay_envelope():
    from sitl.operator_udp import decode_message, encode_message

    key = b"operator-test-key-32-bytes-minimum-0001"
    message = {"version": 1, "type": "command", "sequence": 4}
    signed = encode_message(message, key)
    assert decode_message(signed, key) == message
    assert decode_message(signed, b"wrong-key-32-bytes-minimum-00000000") is None
    assert decode_message(encode_message(message), key) is None
    assert decode_message(encode_message(message, key)) is None


def test_operator_udp_routes_signed_commands_to_rostered_agent_endpoint():
    from sitl.operator_udp import NodeOperatorSocket, send_to_nodes

    port = 35000 + (os.getpid() % 20000)
    key = b"operator-route-test-key-32-bytes-long"
    receiver = NodeOperatorSocket(0, bind_host="127.0.0.1", port=port, auth_key=key)
    try:
        count = send_to_nodes(
            {"version": 1, "type": "command", "sequence": 9},
            endpoints={0: ("127.0.0.1", port)}, auth_key=key,
        )
        assert count == 1
        import time
        deadline = time.monotonic() + 1.0
        messages = []
        while not messages and time.monotonic() < deadline:
            messages = receiver.poll()
            time.sleep(0.005)
        assert messages == [{"version": 1, "type": "command", "sequence": 9}]
    finally:
        receiver.close()


def test_five_rostered_agents_exchange_authenticated_operator_commands_and_status(tmp_path):
    import json
    import socket
    import time
    from sitl.operator_udp import NodeOperatorSocket, decode_message, encode_message, send_to_nodes

    base = 39000 + (os.getpid() % 2000)
    key = b"five-agent-operator-route-test-key-32bytes"
    agents = []
    status_sockets = []
    endpoints = {}
    rows = []
    for agent_id in range(5):
        host = f"127.0.0.{agent_id + 2}"
        command_port = base + agent_id
        status_port = base + 100 + agent_id
        endpoints[agent_id] = (host, command_port)
        agents.append(NodeOperatorSocket(agent_id, bind_host=host,
                                         port=command_port, auth_key=key))
        status_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        status_socket.bind(("127.0.0.1", status_port))
        status_socket.settimeout(1.0)
        status_sockets.append(status_socket)
        rows.append({
            "id": agent_id, "bind_host": host, "host": host, "port": base + 200 + agent_id,
            "command_bind_host": host, "command_host": host, "command_port": command_port,
            "status_host": "127.0.0.1", "status_port": status_port,
        })
    roster_path = tmp_path / "five-agent-network.json"
    roster_path.write_text(json.dumps({"version": 1, "agents": rows}))
    try:
        mission = {"version": 1, "type": "mission", "sequence": 7}
        assert send_to_nodes(mission, endpoints=endpoints, auth_key=key) == 5
        for agent in agents:
            deadline = time.monotonic() + 1.0
            messages = []
            while not messages and time.monotonic() < deadline:
                messages = agent.poll()
                time.sleep(0.005)
            assert messages == [mission]
        for agent_id, status_socket in enumerate(status_sockets):
            status = {"type": "status", "drone_id": agent_id, "sequence": 1}
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
                sender.sendto(encode_message(status, key), ("127.0.0.1", base + 100 + agent_id))
            raw, _ = status_socket.recvfrom(8192)
            assert decode_message(raw, key) == status
    finally:
        for agent in agents:
            agent.close()
        for status_socket in status_sockets:
            status_socket.close()


def test_peer_bus_rejects_nan_telemetry():
    bus=PeerStateBus(1)
    bus.publish([np.array([float('nan'),0])],[np.zeros(2)],[True],1.)
    assert bus.rejected_stale_source == 1


def test_process_protocol_rejects_replay_and_stale_source():
    import pytest
    from sitl.agent_process import validate, FRAME
    request=dict(version=1,frame=FRAME,drone_id=0,sequence=1,
                 args=dict(position=[0.,0.],goal=[0.,0.],now=5.,sample_time=3.,dt=.1,
                           neighbor_states=[],desired_offsets={}))
    with pytest.raises(ValueError,match='stale'):
        validate(request,0,0)
    request['args']['sample_time']=5.
    with pytest.raises(ValueError,match='replayed'):
        validate(request,0,1)


def test_dead_process_faults_only_its_agent():
    from sitl.decentralized_guidance import DecentralizedGuidance
    g=DecentralizedGuidance(2,agent_runtime='process')
    try:
        g.agents[0].process.terminate()
        g.agents[0].process.wait()
        out=g.step([np.array([0.,0.,-5.]),np.array([4.,0.,-5.])],[10.,10.],
                   np.array([0.,0.,-5.]),[np.zeros(3),np.array([4.,0.,0.])],
                   [True,True],[True,True],now=10.)
        assert out[0] is None and out[1] is not None
        assert 0 in g.agent_errors
    finally:
        g.close()
