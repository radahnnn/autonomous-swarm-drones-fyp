"""Retired 2026-11-04 with the legacy runtimes (user decision, item 5); moved unchanged from tests/test_decentralized_sitl.py. Not collected."""


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


def test_agent_detects_existing_armed_airborne_vehicle_after_process_restart():
    from sitl.independent_agent_node import IndependentAgentNode
    from pymavlink import mavutil

    class Message:
        def __init__(self, kind, **fields):
            self.kind = kind
            self.__dict__.update(fields)

        def get_type(self):
            return self.kind

        def get_srcSystem(self):
            return 1

        def get_srcComponent(self):
            return 1

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
    node.election = __import__('sitl.coordinator_election', fromlist=['CoordinatorElection']).CoordinatorElection(0)
    node.mavlink_system_id = 1
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
