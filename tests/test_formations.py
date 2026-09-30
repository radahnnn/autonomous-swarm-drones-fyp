import numpy as np
from swarm_core.formations import FormationGenerator, FormationType, assign_optimal_slots


def test_formation_shapes():
    n = 6
    # Line
    line = FormationGenerator.get_formation_offsets(FormationType.LINE, n, spacing=2.0)
    assert line.shape == (n, 2)
    # Centered at (0, 0)
    assert np.allclose(np.mean(line, axis=0), [0.0, 0.0], atol=1e-5)

    # V Shape
    v_form = FormationGenerator.get_formation_offsets(FormationType.V_SHAPE, n, spacing=2.0)
    assert v_form.shape == (n, 2)
    assert np.allclose(np.mean(v_form, axis=0), [0.0, 0.0], atol=1e-5)

    # Circle
    circle = FormationGenerator.get_formation_offsets(FormationType.CIRCLE, n, spacing=2.0)
    assert circle.shape == (n, 2)
    assert np.allclose(np.mean(circle, axis=0), [0.0, 0.0], atol=1e-5)

    # Grid
    grid = FormationGenerator.get_formation_offsets(FormationType.GRID, n, spacing=2.0)
    assert grid.shape == (n, 2)
    assert np.allclose(np.mean(grid, axis=0), [0.0, 0.0], atol=1e-5)


def test_hungarian_assignment():
    current_pos = np.array([
        [0.0, 0.0],
        [10.0, 10.0],
    ])
    # Target slots are swapped
    target_slots = np.array([
        [9.9, 9.9],
        [0.1, 0.1],
    ])
    # Hungarian should assign current[0]->target[1] (dist ~0.14) and current[1]->target[0] (dist ~0.14)
    assigned = assign_optimal_slots(current_pos, target_slots)
    assert np.allclose(assigned[0], [0.1, 0.1])
    assert np.allclose(assigned[1], [9.9, 9.9])


if __name__ == "__main__":
    test_formation_shapes()
    test_hungarian_assignment()
    print("Formation tests passed successfully!")
