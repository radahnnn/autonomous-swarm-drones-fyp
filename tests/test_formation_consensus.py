import random

import numpy as np

from sitl.formation_consensus import FormationReference, formation_correction


def _ring(n):
    return {i: [(i - 1) % n, (i + 1) % n] for i in range(n)}


def _simulate(n=10, seconds=120.0, dt=0.1, loss=0.0, leader_path=None, leader=0, kill=None, seed=1):
    rng = random.Random(seed)
    refs = [FormationReference() for _ in range(n)]
    graph = _ring(n)
    alive = set(range(n))
    worst_late = 0.0
    t = 0.0
    for r in refs:
        r.update(0.0, False, (0.0, 0.0), [])
    while t < seconds:
        t += dt
        if kill and t >= kill[0] and kill[1] in alive:
            alive.discard(kill[1])
            leader = kill[2]
            graph = {i: [j for j in (sorted(alive)[(sorted(alive).index(i) - 1) % len(alive)],
                                     sorted(alive)[(sorted(alive).index(i) + 1) % len(alive)])]
                     for i in alive}
        goal = leader_path(t)
        snapshot = {i: (refs[i].ref.copy(), refs[i].rate.copy()) for i in alive}
        for i in alive:
            nbrs = [(snapshot[j][0], snapshot[j][1], dt) for j in graph[i] if j in alive and rng.random() >= loss]
            refs[i].update(t, i == leader, goal if i == leader else None, nbrs)
        if t > seconds * 0.6:
            worst_late = max(worst_late, max(float(np.linalg.norm(refs[i].ref - goal)) for i in alive))
    return worst_late


def test_followers_converge_to_a_static_leader_goal_they_never_see():
    assert _simulate(leader_path=lambda t: np.array([20.0, -5.0])) < 0.05


def test_followers_track_a_moving_leader_with_small_lag_under_packet_loss():
    path = lambda t: np.array([0.3 * t, 0.0])                       # transit at 0.3 m/s
    assert _simulate(leader_path=path, loss=0.05) < 0.1
    assert _simulate(leader_path=path, loss=0.3) < 0.2


def test_a_turning_leader_is_followed_with_bounded_second_order_lag():
    # 6 m circle at 0.3 m/s: the velocity estimate lags the rotation, ~0.8 m at five hops (documented limitation).
    assert _simulate(leader_path=lambda t: np.array([6 * np.cos(0.05 * t), 6 * np.sin(0.05 * t)])) < 1.0


def test_a_new_leader_after_the_old_one_dies_keeps_the_reference_continuous():
    path = lambda t: np.array([0.3 * t, 0.0])
    assert _simulate(leader_path=path, kill=(40.0, 0, 5)) < 0.3


def test_formation_correction_points_to_the_slot_implied_by_neighbours_and_is_bounded():
    c = formation_correction((0.0, 0.0), (0.0, 0.0), [((2.0, 0.0), (0.0, 0.0)), ((2.0, 0.0), (0.0, 0.0))])
    np.testing.assert_allclose(c, [1.0, 0.0])
    big = formation_correction((0.0, 0.0), (0.0, 0.0), [((50.0, 0.0), (0.0, 0.0))])
    assert abs(np.linalg.norm(big) - 1.0) < 1e-9
    assert np.allclose(formation_correction((1.0, 1.0), (1.0, 1.0), []), 0.0)


def test_peer_packet_reference_parsing_accepts_valid_and_rejects_bad_fields():
    from sitl.peer_udp import _parse_reference
    ref, rate, t = _parse_reference({"ref": [1.0, 2.0], "rate": [0.3, 0.0], "time": 5.0})
    assert np.allclose(ref, [1, 2]) and np.allclose(rate, [0.3, 0]) and t == 5.0
    assert _parse_reference(None) is None                                  # older agents send none
    assert _parse_reference({"ref": [1.0], "rate": [0, 0], "time": 1}) is None
    assert _parse_reference({"ref": [float("nan"), 0], "rate": [0, 0], "time": 1}) is None
