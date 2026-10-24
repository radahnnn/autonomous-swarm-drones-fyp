import itertools

import numpy as np

from sitl.adaptive_membership import AdaptiveMembership
from sitl.formation_library import MIN_SPACING


def test_ten_member_ring_is_deterministic_and_local():
    views = [AdaptiveMembership(i, 10, "v") for i in range(10)]
    assert views[0].neighbors_for() == [1, 9]
    assert views[5].neighbors_for() == [4, 6]
    assert all(view.leader_id == 0 for view in views)


def test_loss_repairs_ring_and_elects_lowest_live_member():
    view = AdaptiveMembership(4, 10, "ring", loss_timeout_s=3.0)
    view.update(0.0, range(10))
    assert not view.update(2.99, range(1, 10))
    assert view.update(3.0, range(1, 10))
    assert 0 not in view.active_ids
    assert view.leader_id == 1
    assert view.neighbors_for(1) == [2, 9]


def test_rejoin_requires_continuous_fresh_liveness():
    view = AdaptiveMembership(0, 3, "v", loss_timeout_s=1.0,
                              rejoin_stable_s=0.5)
    view.update(0.0, [1, 2])
    view.update(1.0, [2])
    assert 1 not in view.active_ids

    view.update(1.1, [1, 2])
    view.update(1.4, [2])
    view.update(1.7, [1, 2])
    assert 1 not in view.active_ids

    view.update(1.9, [1, 2])
    assert 1 not in view.active_ids
    assert view.update(2.2, [1, 2])
    assert 1 in view.active_ids


def test_slot_redistribution_is_finite_unique_and_keeps_spacing():
    view = AdaptiveMembership(0, 10, "ring", loss_timeout_s=1.0)
    view.update(0.0, range(10))
    view.update(1.0, [1, 2, 4, 5, 7, 8, 9])
    slots = view.slot_map()
    assert sorted(slots) == [0, 1, 2, 4, 5, 7, 8, 9]
    assert all(np.isfinite(slot).all() for slot in slots.values())
    distances = [np.linalg.norm(slots[a] - slots[b])
                 for a, b in itertools.combinations(slots, 2)]
    assert min(distances) >= MIN_SPACING - 1e-9


def test_single_member_has_no_neighbors_and_owns_a_valid_slot():
    view = AdaptiveMembership(3, 4, "triangle", loss_timeout_s=1.0)
    view.update(0.0, range(4))
    assert view.update(1.0, [])
    assert view.active_ids == {3}
    assert view.neighbors_for() == []
    np.testing.assert_allclose(view.slot_map()[3], [0.0, 0.0, 0.0])


def test_local_blocking_pause_does_not_look_like_total_network_failure():
    view = AdaptiveMembership(0, 4, "v", loss_timeout_s=3.0)
    view.update(10.0, [1, 2, 3])
    view.resume_after_local_pause(30.0)
    assert not view.update(30.1, [])
    assert view.active_ids == {0, 1, 2, 3}
    assert view.update(33.0, [])
    assert view.active_ids == {0}


def test_stale_loop_timestamp_cannot_move_liveness_anchor_backwards():
    view = AdaptiveMembership(0, 3, "v", loss_timeout_s=3.0)
    view.update(10.0, [1, 2])
    view.resume_after_local_pause(30.0)
    view.update(10.1, [1, 2])
    assert not view.update(30.1, [])
    assert view.active_ids == {0, 1, 2}


def test_obstacle_repulsion_is_finite_bounded_and_points_away():
    from sitl.independent_agent_node import IndependentAgentNode

    node = object.__new__(IndependentAgentNode)
    node.obstacles = [(0.0, 0.0, 2.0)]
    node.position = np.array([3.0, 0.0, -5.0])
    target = node._avoid_obstacles(np.array([4.0, 0.0]))
    np.testing.assert_allclose(target, [6.0, 0.0])

    node.position = np.array([0.0, 0.0, -5.0])
    target_at_center = node._avoid_obstacles(np.array([1.0, 0.0]))
    assert np.isfinite(target_at_center).all()
    assert np.linalg.norm(target_at_center - [1.0, 0.0]) <= 2.0 + 1e-9


def test_obstacle_outside_influence_does_not_change_target():
    from sitl.independent_agent_node import IndependentAgentNode

    node = object.__new__(IndependentAgentNode)
    node.obstacles = [(0.0, 0.0, 2.0)]
    node.position = np.array([5.0, 0.0, -5.0])
    np.testing.assert_allclose(node._avoid_obstacles([10.0, 1.0]), [10.0, 1.0])
