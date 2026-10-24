import numpy as np

from experiments.initial_positions import formation_start


def test_formation_start_is_reproducible_and_above_clearance_gate():
    first = formation_start(5, np.random.default_rng(123))
    second = formation_start(5, np.random.default_rng(123))
    np.testing.assert_allclose(first, second)
    distances = [np.linalg.norm(first[i] - first[j])
                 for i in range(5) for j in range(i + 1, 5)]
    assert min(distances) >= 2.5


def test_formation_start_supports_ten_drones():
    positions = formation_start(10, np.random.default_rng(999))
    assert positions.shape == (10, 2)
    assert np.isfinite(positions).all()
