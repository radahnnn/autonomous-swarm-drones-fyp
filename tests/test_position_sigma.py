"""Tuning for position error (GPS wander): tolerances widen, planned clearance keeps a sigma above the abort line."""
import numpy as np
import pytest

from sitl.formation_library import slots_for
from sitl.formation_transition import ABORT_SEPARATION_M, plan_formation_transition
from sitl.transition_protocol import TransitionParams, scaled_params


def test_sigma_widens_position_tolerances_but_not_separation_thresholds():
    base, noisy = TransitionParams(), scaled_params(1.0, 1.0)
    assert noisy.start_tol_m == base.start_tol_m + 1.0 and noisy.arrival_tol_m == base.arrival_tol_m + 1.0
    assert noisy.peer_track_tol_m == pytest.approx(base.peer_track_tol_m + 1.5)
    assert noisy.abort_separation_m == base.abort_separation_m == ABORT_SEPARATION_M
    assert noisy.local_hold_separation_m == base.local_hold_separation_m
    assert scaled_params(1.0, 0.0) == base
    with pytest.raises(ValueError):
        scaled_params(1.0, -0.1)


@pytest.mark.parametrize("n, chain, ts", [(6, ["pairs", "pairs_b", "pairs", "v"], 0.2),
                                          (10, ["line", "v", "column", "v", "ring", "v", "diamond", "v"], 1.0)])
def test_demo_formation_chains_stay_plannable_with_a_sigma_margin(n, chain, ts):
    current = slots_for("v", n)[:, :2]
    for style in chain:
        plan = plan_formation_transition(current, style, centroid_ne=np.zeros(2), time_scale=ts, position_sigma_m=1.0)
        assert plan.minimum_path_separation_m >= ABORT_SEPARATION_M + 1.0 - 1e-9
        current = plan.slots_by_drone[:, :2]


def test_noisy_start_closer_than_the_margin_is_still_plannable_but_never_worse_than_now():
    rng = np.random.default_rng(4)
    start = slots_for("v", 10)[:, :2] * 1.1 + rng.normal(0, 0.4, (10, 2))
    closest = min(np.linalg.norm(start[i] - start[j]) for i in range(10) for j in range(i + 1, 10))
    plan = plan_formation_transition(start, "line", position_sigma_m=1.0)
    assert plan.minimum_path_separation_m >= min(ABORT_SEPARATION_M + 1.0, closest - 0.05) - 1e-9
