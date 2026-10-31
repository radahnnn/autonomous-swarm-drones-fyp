import numpy as np
import pytest
from scipy.optimize import linear_sum_assignment

from sitl.formation_library import assign_slots, slots_for
from swarm_core.assignment import assign, squared_cost
from swarm_core.formations import assign_slot_indices


def _reference(cost):
    rows, cols = linear_sum_assignment(cost)
    out = np.zeros(len(cost), dtype=int)
    out[rows] = cols
    return out


def test_matches_plain_hungarian_on_random_problems():
    rng = np.random.default_rng(0)
    for n in (2, 5, 10):
        current, slots = rng.normal(size=(n, 3)), rng.normal(size=(n, 3))
        np.testing.assert_array_equal(assign(squared_cost(current, slots)), _reference(squared_cost(current, slots)))


def test_axis_weights_change_the_cost_exactly():
    a, b = np.array([[0.0, 0.0, 1.0]]), np.array([[1.0, 2.0, 3.0]])
    assert squared_cost(a, b, (1, 1, 2))[0, 0] == pytest.approx(1 + 4 + 2 * 4)


def test_library_and_swarm_core_agree_on_planar_problems():
    rng = np.random.default_rng(5)
    for _ in range(20):
        current = np.column_stack([rng.normal(size=(8, 2)) * 5, np.zeros(8)])
        slots = slots_for("ring", 8)
        assert assign_slots(current, slots) == assign_slot_indices(current[:, :2], slots[:, :2]).tolist()


def test_good_previous_is_kept_and_bad_previous_is_replaced():
    cost = np.array([[1.0, 1.05], [1.05, 1.0]])
    assert assign(cost, previous=[1, 0], hysteresis=0.15).tolist() == [1, 0]     # 4% better only: latched
    assert assign(cost, previous=[1, 0], hysteresis=0.0).tolist() == [0, 1]
    bad = np.array([[0.0, 9.0], [9.0, 0.0]])
    assert assign(bad, previous=[1, 0], hysteresis=0.15).tolist() == [0, 1]


def test_invalid_input():
    with pytest.raises(ValueError):
        assign(np.zeros((2, 3)))
    cost = np.array([[0.0, 9.0], [9.0, 0.0]])
    assert assign(cost, previous=[0, 0]).tolist() == [0, 1]      # a malformed previous assignment is ignored
    assert assign(cost, previous=[0, 1, 2]).tolist() == [0, 1]
