import numpy as np

from swarm_core.controllers.decentralized import DecentralizedController
from swarm_core.drone import Drone
from swarm_core.safety import separation_guard_acceleration
from simulator.engine import SwarmSimulation


def test_guard_brakes_two_closing_neighbors_before_minimum_clearance():
    peers = [{"position": np.array([2.8, 0.0]), "velocity": np.array([-0.5, 0.0])}]

    acceleration = separation_guard_acceleration(
        np.zeros(2), np.array([0.5, 0.0]), peers,
    )

    assert acceleration[0] < 0.0
    assert abs(acceleration[1]) < 1e-12
    assert np.linalg.norm(acceleration) <= 2.5


def test_guard_does_not_brake_a_receding_neighbor_outside_the_boundary():
    peers = [{"position": np.array([2.8, 0.0]), "velocity": np.array([0.5, 0.0])}]

    acceleration = separation_guard_acceleration(
        np.zeros(2), np.array([-0.5, 0.0]), peers,
    )

    assert np.allclose(acceleration, 0.0)


def test_guard_uses_the_requested_clearance_distance():
    peers = [{"position": np.array([2.8, 0.0]), "velocity": np.zeros(2)}]
    default = separation_guard_acceleration(np.zeros(2), np.zeros(2), peers)
    wider = separation_guard_acceleration(
        np.zeros(2), np.zeros(2), peers, minimum_separation_m=3.0,
    )

    assert np.allclose(default, 0.0)
    assert wider[0] < 0.0


def test_simulation_accepts_an_explicit_separation_guard_distance():
    drones = [Drone(0, np.zeros(2)), Drone(1, np.array([2.8, 0.0]))]
    sim = SwarmSimulation(drones, separation_minimum_m=3.0)

    assert sim.separation_minimum_m == 3.0


def test_simulation_rejects_invalid_separation_guard_distance():
    drones = [Drone(0, np.zeros(2))]

    try:
        SwarmSimulation(drones, separation_minimum_m=0.0)
    except ValueError as exc:
        assert "separation_minimum_m" in str(exc)
    else:
        raise AssertionError("expected invalid guard distance to be rejected")


def test_guard_returns_finite_bounded_acceleration_for_coincident_estimates():
    peers = [{"position": np.zeros(2), "velocity": np.zeros(2)}]

    acceleration = separation_guard_acceleration(
        np.zeros(2), np.zeros(2), peers,
    )

    assert np.isfinite(acceleration).all()
    assert 0.0 < np.linalg.norm(acceleration) <= 2.5


def test_decentralized_controller_brakes_instead_of_advancing_without_peer_state():
    drone = Drone(0, np.zeros(2), initial_velocity=np.array([0.8, -0.2]))
    controller = DecentralizedController(k_align=1.6)

    acceleration = controller.compute_drone_control(
        drone, neighbor_states=[], goal_pos=np.array([20.0, 0.0]),
    )

    assert np.allclose(acceleration, -1.6 * drone.velocity)
