"""Formation templates and live transition planning must work up to the 10-drone target (UNIT)."""
import itertools
import random

import numpy as np
import pytest

from sitl.formation_library import MIN_SPACING, RING_SPACING_M, STYLES, slots_for
from sitl.formation_transition import plan_formation_transition


def _min_pair(points):
    return min(float(np.linalg.norm(points[i] - points[j]))
               for i in range(len(points)) for j in range(i + 1, len(points)))


@pytest.mark.parametrize("n", range(2, 11))
@pytest.mark.parametrize("style", STYLES)
def test_every_template_keeps_the_minimum_slot_spacing(style, n):
    assert _min_pair(slots_for(style, n)[:, :2]) >= MIN_SPACING - 1e-9


@pytest.mark.parametrize("n", range(3, 11))
def test_ring_has_room_for_every_drone_to_leave_it(n):
    # Regression: at 10 drones the ring used to be spaced 3.0 m, which made every transition out
    # of it fail the planner's path-clearance check.
    ring = slots_for("ring", n)[:, :2]
    adjacent = min(np.linalg.norm(ring[i] - ring[(i + 1) % n]) for i in range(n))
    assert adjacent >= RING_SPACING_M - 1e-6


def _plan(positions, target):
    plan = plan_formation_transition(positions, target, centroid_ne=positions.mean(axis=0))
    return plan, positions.mean(axis=0) + plan.slots_by_drone[:, :2]


@pytest.mark.parametrize("n", [6, 10])
@pytest.mark.parametrize("start", ["v", "ring"])
def test_from_the_demo_start_formations_every_other_formation_is_plannable(start, n):
    positions = slots_for(start, n)[:, :2] + np.array([3.0, -2.0])
    for target in (s for s in STYLES if s != start):
        plan, _ = _plan(positions, target)
        assert plan.minimum_path_separation_m >= MIN_SPACING + 0.35 - 1e-9
        assert plan.duration_s <= 45.0


@pytest.mark.parametrize("n", [6, 10])
def test_long_random_chains_of_formation_changes_never_dead_end(n):
    rng = random.Random(2026 + n)
    positions = slots_for("v", n)[:, :2] + np.array([3.0, -2.0])
    current = "v"
    for _ in range(14):
        target = rng.choice([s for s in STYLES if s != current])
        _, positions = _plan(positions, target)   # raises ValueError if the planner rejects it
        assert _min_pair(positions) >= MIN_SPACING + 0.35 - 1e-6
        current = target
