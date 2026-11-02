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


def test_sigma_rejects_a_plan_that_would_be_fine_without_noise():
    start = np.array([[0.0, 0.0], [3.3, 0.0]])                      # 3.3 m apart: fine at sigma 0, not at sigma 1
    plan_formation_transition(start, "line", centroid_ne=np.array([1.65, 0.0]), minimum_separation_m=2.0)
    with pytest.raises(ValueError):
        plan_formation_transition(start, "line", centroid_ne=np.array([1.65, 0.0]), minimum_separation_m=2.0,
                                  position_sigma_m=1.0)
