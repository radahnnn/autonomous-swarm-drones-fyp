import math

import pytest

from sitl.obstacle_sensing import MAX_SHARED, ObstacleSensor, validate


def test_not_detected_when_the_surface_is_out_of_range():
    sensor = ObstacleSensor([(10.0, 0.0, 1.0)], range_m=5.0)
    assert sensor.update((0.0, 0.0)) == []                       # centre 10 m away = surface 9 m away
    assert sensor.update((3.9, 0.0)) == []                       # surface 5.1 m away: just outside


def test_boundary_and_each_obstacle_is_reported_once():
    sensor = ObstacleSensor([(10.0, 0.0, 1.0)], range_m=5.0)
    assert sensor.update((4.0, 0.0)) == [(10.0, 0.0, 1.0)]       # centre 6 m, radius 1 m: surface exactly 5 m
    assert sensor.update((5.0, 0.0)) == []                       # already known
    assert sensor.status()["detected"] == 1 and len(sensor.known) == 1


def test_far_obstacles_stay_unknown():
    sensor = ObstacleSensor([(30.0, 30.0, 1.0)], range_m=8.0)
    sensor.update((0.0, 0.0))
    assert sensor.known == []


def test_sharing_between_two_drones_and_dedupe():
    a, b = ObstacleSensor([(5.0, 5.0, 0.6)], range_m=3.0), ObstacleSensor([(5.0, 5.0, 0.6)], range_m=3.0)
    a.update((4.0, 4.0))
    assert b.known == []
    assert b.learn(a.shareable()) == [(5.0, 5.0, 0.6)]
    assert b.learn(a.shareable()) == []                          # same pole again
    assert b.learn([[5.3, 5.2, 0.6]]) == []                      # within the merge tolerance: same pole
    assert b.status() == {"known": 1, "detected": 0, "learned": 1, "range_m": 3.0, "items": [[5.0, 5.0, 0.6]]}


def test_malformed_and_oversized_peer_data_is_ignored():
    sensor = ObstacleSensor()
    bad = [[1.0, 2.0], [float("nan"), 0.0, 1.0], [0.0, 0.0, -1.0], "xyz", None, [0, 0, math.inf]]
    assert sensor.learn(bad) == []
    many = [[10.0 * i, 0.0, 0.5] for i in range(40)]
    assert len(sensor.learn(many)) == MAX_SHARED
    assert len(sensor.shareable()) <= MAX_SHARED


def test_validation_and_range():
    with pytest.raises(ValueError):
        validate((1, 2, 0))
    with pytest.raises(ValueError):
        ObstacleSensor(range_m=0)
    with pytest.raises(ValueError):
        ObstacleSensor([(1.0, 2.0)])
    assert ObstacleSensor().update((float("nan"), 0.0)) == []


def test_obstacle_message_round_trip_over_the_authenticated_peer_link():
    import os
    from tests.test_coordinator_election import make_pair
    base = 39000 + (os.getpid() % 3000)
    a, b = make_pair(base)
    import time
    try:
        now = time.time()

        def poll(node):
            for _ in range(60):
                node.receive(now, [0])
                got = node.take_obstacle_messages()
                if got:
                    return got
                time.sleep(0.02)
            return []

        a.publish_obstacles([1], now, [[5.0, 6.0, 0.6]])
        got = poll(b)
        assert len(got) == 1 and got[0]["sender"] == 0 and got[0]["items"] == [[5.0, 6.0, 0.6]]
        a.publish_obstacles([1], now, [[1.0, 1.0, 1.0]] * 17)                 # too many items: rejected
        assert poll(b) == [] and b.stats()["rejected_invalid"] >= 1
    finally:
        a.close(); b.close()


class _FakePeers:
    def __init__(self):
        self.inbox, self.sent = [], []

    def take_obstacle_messages(self):
        messages, self.inbox = self.inbox, []
        return messages

    def publish_obstacles(self, peer_ids, now, items):
        self.sent.append((list(peer_ids), items))


def _node(position, world):
    import numpy as np
    from sitl.independent_agent_node import IndependentAgentNode
    node = object.__new__(IndependentAgentNode)
    node.drone_id, node.obstacles = 0, []
    node.sensor = ObstacleSensor(world, 6.0)
    node._obstacles_published_count, node._obstacles_published_at = -1, 0.0
    node.peers, node.discovery_ids = _FakePeers(), [1, 2]
    node.position = np.array([position[0], position[1], -5.0])
    return node


def test_agent_detects_shares_and_avoids_a_pole(capsys):
    import numpy as np
    node = _node((0.0, 0.0), [(6.0, 0.0, 0.6)])
    assert node._all_obstacles() == []                                   # nothing known before sensing
    target = node._avoid_obstacles(np.array([0.0, 0.0]))
    np.testing.assert_allclose(target, [0.0, 0.0])                       # so no avoidance either
    node._tick_obstacles(100.0)
    assert node._all_obstacles() == [(6.0, 0.0, 0.6)]
    assert "obstacle detected" in capsys.readouterr().out
    assert node.peers.sent == [([1, 2], [[6.0, 0.0, 0.6]])]              # shared with the discovery peers
    node._tick_obstacles(100.5)
    assert len(node.peers.sent) == 1                                     # not re-sent until the refresh period
    node._tick_obstacles(102.5)
    assert len(node.peers.sent) == 2
    node.position = np.array([4.0, 0.0, -5.0])                           # 2 m from the pole centre: inside influence
    assert node._avoid_obstacles(np.array([4.0, 0.0]))[0] < 4.0          # pushed away from the pole (south)


def test_agent_learns_a_pole_it_never_saw_from_a_peer(capsys):
    node = _node((30.0, 30.0), [(6.0, 0.0, 0.6)])                        # far from the pole: cannot sense it
    node.peers.inbox.append({"sender": 3, "items": [[6.0, 0.0, 0.6]], "sequence": 1})
    node._tick_obstacles(10.0)
    assert node._all_obstacles() == [(6.0, 0.0, 0.6)]
    assert node.sensor.status()["learned"] == 1 and node.sensor.status()["detected"] == 0
    assert "learned from D4" in capsys.readouterr().out
