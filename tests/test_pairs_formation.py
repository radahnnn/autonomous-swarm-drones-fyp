"""World-anchored pair layouts for the survey patrol (UNIT)."""
import numpy as np
import pytest

from sitl.formation_library import MIN_SPACING, PAIR_PARTNER_SPACING_M, pair_sector_centres, slots_for
from sitl.formation_transition import plan_formation_transition


def test_pairs_are_partners_apart_and_sit_on_alternate_sectors():
    slots = slots_for("pairs", 6)[:, :2]
    centres = pair_sector_centres(3)
    for k in range(3):
        a, b = slots[2 * k], slots[2 * k + 1]
        assert np.linalg.norm(a - b) == pytest.approx(PAIR_PARTNER_SPACING_M)
        np.testing.assert_allclose((a + b) / 2, centres[2 * k], atol=1e-9)           # phase 0: sectors 0, 2, 4
    slots_b = slots_for("pairs_b", 6)[:, :2]
    for k in range(3):
        np.testing.assert_allclose((slots_b[2 * k] + slots_b[2 * k + 1]) / 2, centres[2 * k + 1], atol=1e-9)


def test_the_two_phases_are_different_position_sets_so_pairs_really_move():
    a = {tuple(np.round(p, 3)) for p in slots_for("pairs", 6)[:, :2]}
    b = {tuple(np.round(p, 3)) for p in slots_for("pairs_b", 6)[:, :2]}
    assert not a & b


@pytest.mark.parametrize("n", range(1, 11))
@pytest.mark.parametrize("style", ["pairs", "pairs_b"])
def test_spacing_for_every_swarm_size(style, n):
    pts = slots_for(style, n)[:, :2]
    if n > 1:
        assert min(np.linalg.norm(pts[i] - pts[j]) for i in range(n) for j in range(i + 1, n)) >= MIN_SPACING - 1e-9


def test_planner_keeps_world_anchored_orientation_and_screens_the_paths():
    start = slots_for("v", 6)[:, :2]
    plan = plan_formation_transition(start, "pairs", centroid_ne=np.zeros(2), max_transition_s=200.0)
    got = {tuple(np.round(p, 3)) for p in plan.slots_by_drone[:, :2]}
    want = {tuple(np.round(p, 3)) for p in slots_for("pairs", 6)[:, :2]}
    assert got == want                                       # no rotation search: sectors stay where the map says
    assert plan.minimum_path_separation_m >= 2.85
    follow = plan_formation_transition(plan.slots_by_drone[:, :2], "pairs_b", centroid_ne=np.zeros(2), max_transition_s=200.0)
    assert np.linalg.norm(follow.slots_by_drone[:, :2] - plan.slots_by_drone[:, :2], axis=1).max() > 3.0
