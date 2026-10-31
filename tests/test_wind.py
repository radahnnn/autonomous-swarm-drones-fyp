import numpy as np
import pytest

from simulator.engine import SwarmSimulation
from swarm_core.drone import Drone
from swarm_core.wind import WindModel


def test_reproducible_from_seed():
    a, b = WindModel(gust_sigma=0.3, seed=4), WindModel(gust_sigma=0.3, seed=4)
    for _ in range(50):
        wa, wb = a.advance(0.05, [0, 1]), b.advance(0.05, [0, 1])
    np.testing.assert_array_equal(wa[1], wb[1])


def test_statistics_match_parameters():
    model = WindModel((0.2, 0.0), gust_sigma=0.3, corr_time_s=2.0, seed=1)
    samples = np.array([model.advance(0.1, [0])[0] for _ in range(20000)])
    assert abs(samples[:, 0].mean() - 0.2) < 0.05
    assert samples[:, 0].std() == pytest.approx(0.3, rel=0.15)


def test_common_component_correlates_drones():
    shared = WindModel(gust_sigma=0.3, common_fraction=1.0, seed=2)
    own = WindModel(gust_sigma=0.3, common_fraction=0.0, seed=2)
    rows_s = np.array([[w[0][0], w[1][0]] for w in (shared.advance(0.1, [0, 1]) for _ in range(5000))])
    rows_o = np.array([[w[0][0], w[1][0]] for w in (own.advance(0.1, [0, 1]) for _ in range(5000))])
    assert np.corrcoef(rows_s.T)[0, 1] > 0.95
    assert abs(np.corrcoef(rows_o.T)[0, 1]) < 0.1


def test_rejects_bad_parameters():
    for kwargs in ({"gust_sigma": -1}, {"corr_time_s": 0}, {"common_fraction": 2}, {"mean_accel": (float("nan"), 0)}):
        with pytest.raises(ValueError):
            WindModel(**kwargs)


def test_engine_applies_wind_to_drones():
    drones = [Drone(drone_id=i, initial_position=np.array([4.0 * i, 0.0])) for i in range(2)]
    sim = SwarmSimulation(drones, control_mode="decentralized", seed=1,
                          wind=WindModel((0.0, 0.5), 0.0, seed=1))
    sim.step()
    assert all(abs(d.wind_vector[1] - 0.5) < 1e-9 for d in drones)
