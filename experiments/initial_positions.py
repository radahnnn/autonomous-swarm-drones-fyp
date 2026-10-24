"""Initial layouts for numerical Phase 2 trials."""

import numpy as np

from swarm_core.formations import FormationGenerator, FormationType


def formation_start(num_drones: int, rng: np.random.Generator, spacing: float = 3.0):
    """Place drones near valid V slots with a repeatable small perturbation."""
    slots = FormationGenerator.get_formation_offsets(
        FormationType.V_SHAPE, num_drones, spacing=spacing,
    )
    positions = slots + rng.normal(0.0, 0.04, slots.shape)
    if num_drones > 1:
        distances = [np.linalg.norm(positions[i] - positions[j])
                     for i in range(num_drones) for j in range(i + 1, num_drones)]
        if min(distances) < 2.5:
            raise ValueError("formation start violates the 2.5 m separation gate")
    return positions
