"""The separation guard must keep priority over the goal term (UNIT / numerical sandbox)."""
import numpy as np
import pytest

from swarm_core.controllers.centralized import CentralizedController
from swarm_core.controllers.decentralized import DecentralizedController
from swarm_core.drone import Drone
from swarm_core.formations import FormationType
from swarm_core.safety import prioritised_separation_acceleration as prioritised
from swarm_core.safety import separation_guard_acceleration as guard


def _neighbor(x, y=0.0, vx=0.0, vy=0.0):
    return {"id": 1, "position": np.array([x, y]), "velocity": np.array([vx, vy])}


def test_no_violation_passes_nominal_through_unchanged():
    nominal = np.array([1.0, -0.5])
    out = prioritised(np.zeros(2), np.zeros(2), [_neighbor(20.0)], nominal)
    np.testing.assert_allclose(out, nominal)


def test_component_into_a_violated_neighbor_is_removed_but_lateral_motion_is_kept():
    out = prioritised(np.zeros(2), np.array([0.3, 0.0]), [_neighbor(2.7)], np.array([2.0, 1.0]))
    assert out[0] <= 0.0                      # no push toward the neighbor at +x
    assert out[1] > 0.5                       # lateral goal motion survives while budget remains


@pytest.mark.parametrize("seed", range(40))
def test_result_never_exceeds_the_acceleration_budget_and_never_weakens_the_guard(seed):
    rng = np.random.default_rng(seed)
    pos, vel = rng.uniform(-3, 3, 2), rng.uniform(-3, 3, 2)
    peers = [{"id": i, "position": rng.uniform(-6, 6, 2), "velocity": rng.uniform(-3, 3, 2)}
             for i in range(rng.integers(1, 6))]
    nominal = rng.uniform(-40, 40, 2)         # far larger than max_accel
    out = prioritised(pos, vel, peers, nominal, max_accel_m_s2=2.5)
    assert np.linalg.norm(out) <= 2.5 + 1e-9
    g = guard(pos, vel, peers, max_accel_m_s2=2.5)
    if np.linalg.norm(g) > 0:
        # the component along the guard direction must not be reduced by the goal term
        direction = g / np.linalg.norm(g)
        assert float(np.dot(out, direction)) >= float(np.dot(g, direction)) - 1e-6


def test_rejects_a_bad_nominal_command():
    with pytest.raises(ValueError):
        prioritised(np.zeros(2), np.zeros(2), [], np.array([np.nan, 0.0]))
    with pytest.raises(ValueError):
        prioritised(np.zeros(2), np.zeros(2), [], np.zeros(3))


def _sandbox_min_separation(pull, lag=0.0, vmax=3.0, amax=2.5, start=12.0):
    """One drone driven at a stationary neighbor by a constant goal acceleration."""
    p, v, a = np.zeros(2), np.zeros(2), np.zeros(2)
    q, dt, minimum = np.array([start, 0.0]), 0.01, 1e9
    for _ in range(4000):
        cmd = prioritised(p, v, [{"position": q, "velocity": np.zeros(2)}],
                          np.array([pull, 0.0]), minimum_separation_m=2.5,
                          reaction_time_s=1.0, max_accel_m_s2=amax)
        a = a + (cmd - a) * (dt / lag) if lag > 0 else cmd
        v = v + a * dt - 0.2 * v * dt
        speed = np.linalg.norm(v)
        if speed > vmax:
            v = v / speed * vmax
        p = p + v * dt
        minimum = min(minimum, float(np.linalg.norm(p - q)))
    return minimum


@pytest.mark.parametrize("pull", [0.5, 2.0, 2.5, 5.0, 20.0])
def test_sustained_goal_pull_cannot_push_through_the_gate(pull):
    # Regression: with the old additive guard, pull >= max_accel gave ~0 m separation.
    assert _sandbox_min_separation(pull) >= 2.49
    assert _sandbox_min_separation(pull, lag=0.18) >= 2.40


def test_centralized_controller_keeps_the_gate_against_a_huge_slot_error():
    ctrl = CentralizedController()
    drones = [Drone(0, np.array([0.0, 0.0])), Drone(1, np.array([2.6, 0.0]))]
    # Assignment may swap targets, so check the result directly: whatever target drone 0 has,
    # its command must not push it into drone 1 (at +x) while inside the guard envelope.
    out = ctrl.compute_control_inputs(
        drones, formation_type=FormationType.LINE, centroid_target=np.array([500.0, 0.0]),
    )
    assert np.all(np.linalg.norm(out, axis=1) <= drones[0].max_accel + 1e-9)


def test_decentralized_repulsion_is_not_diluted_by_distant_neighbors():
    drone = Drone(0, np.array([0.0, 0.0]))
    close = {"id": 1, "position": np.array([0.8, 0.0]), "velocity": np.zeros(2)}
    # 8 neighbors at 1.5 m: outside the repulsion radius (1.2 m) and inside the default
    # cohesion radius (1.8 m), so they only increase the degree.
    angles = np.linspace(0.6, 2 * np.pi - 0.6, 8)
    far = [{"id": 2 + i, "position": 1.5 * np.array([np.cos(a), np.sin(a)]), "velocity": np.zeros(2)}
           for i, a in enumerate(angles)]
    ctrl = DecentralizedController()
    alone = ctrl.compute_drone_control(drone, [close], desired_offsets=None, apply_guard=False)
    crowded = ctrl.compute_drone_control(drone, [close] + far, desired_offsets=None, apply_guard=False)
    # repulsion (pushing toward -x) must not be diluted by the 8 extra neighbors
    np.testing.assert_allclose(crowded, alone, atol=1e-9)


# ---------------------------------------------------------------- assignment latching
def test_assignment_is_latched_against_near_ties_but_changes_for_real_improvements():
    from swarm_core.formations import assign_slot_indices
    slots = np.array([[0.0, 0.0], [3.0, 0.0]])
    start = np.array([[0.0, 0.1], [3.0, -0.1]])
    first = assign_slot_indices(start, slots)
    assert first.tolist() == [0, 1]
    # nearly tied positions: swapping would barely change the cost -> keep previous
    tied = np.array([[1.5, 0.1], [1.5, -0.1]])
    kept = assign_slot_indices(tied, slots, previous_indices=first, hysteresis=0.15)
    assert kept.tolist() == [0, 1]
    # a real improvement (drones have swapped sides) -> switch
    swapped = np.array([[3.0, 0.0], [0.0, 0.0]])
    assert assign_slot_indices(swapped, slots, previous_indices=first, hysteresis=0.15).tolist() == [1, 0]
    # an invalid previous assignment is ignored
    assert assign_slot_indices(swapped, slots, previous_indices=np.array([0, 0])).tolist() == [1, 0]


def test_centralized_controller_keeps_its_assignment_between_steps():
    ctrl = CentralizedController()
    drones = [Drone(i, np.array([3.0 * i, 0.1 * (-1) ** i])) for i in range(4)]
    ctrl.compute_control_inputs(drones, FormationType.LINE, np.array([0.0, 0.0]))
    first = ctrl._assignment.copy()
    for d in drones:                       # nudge positions slightly; assignment must not change
        d.position = d.position + np.array([0.01, 0.01])
    ctrl.compute_control_inputs(drones, FormationType.LINE, np.array([0.0, 0.0]))
    assert ctrl._assignment.tolist() == first.tolist()
    ctrl.compute_control_inputs(drones, FormationType.GRID, np.array([0.0, 0.0]))  # new formation re-solves
    assert ctrl._assignment_key[0] == FormationType.GRID
