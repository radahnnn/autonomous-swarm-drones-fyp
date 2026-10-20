import numpy as np

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
