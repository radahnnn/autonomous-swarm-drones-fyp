import numpy as np

from swarm_core.controllers.decentralized import DecentralizedController
from swarm_core.drone import Drone
from swarm_core.safety import separation_guard_acceleration


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
