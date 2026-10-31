import numpy as np
import pytest

from sitl.clock_budget import compute_budget
from sitl.clock_sync import best_clock_sample, estimate_clock_offset


def test_budget_formula():
    budget = compute_budget({0: 0.010, 1: 0.004, 2: 0.002}, age_s=30.0, drift_ppm=100.0, tick_s=0.1,
                            latency_s=0.1, margin_s=0.25)
    assert budget.skew_bound_s == pytest.approx(0.010 + 0.004 + 2 * 100e-6 * 30.0 + 0.1)
    assert budget.required_lead_s == pytest.approx(budget.skew_bound_s + 0.35)


def test_single_agent_and_validation():
    assert compute_budget({3: 0.01}, age_s=0).skew_bound_s == pytest.approx(0.11)
    for bad in ({}, {0: float("nan")}, {0: -1.0}):
        with pytest.raises(ValueError):
            compute_budget(bad, age_s=1.0)
    with pytest.raises(ValueError):
        compute_budget({0: 0.01}, age_s=-1.0)


def test_default_lead_times_cover_the_budget_on_a_lan():
    from sitl.mode_switch import ModeParams
    from sitl.transition_protocol import TransitionParams
    budget = compute_budget({i: 0.005 for i in range(10)}, age_s=60.0)
    assert budget.required_lead_s < min(TransitionParams().lead_s, ModeParams().lead_s)


def test_bound_holds_in_a_simulated_estimation_with_asymmetric_delay_and_drift():
    """Monte-Carlo (SIM): estimate each offset with the real NTP code under random asymmetric delays, let the
    clocks drift for `age`, and check the realised start skew is inside the bound computed from the reported
    half-RTT uncertainty."""
    rng = np.random.default_rng(11)
    age, drift_ppm, tick = 30.0, 100.0, 0.1
    worst_ratio = 0.0
    for _ in range(300):
        n = 10
        true_offset = rng.uniform(-0.5, 0.5, n)
        drift = rng.uniform(-drift_ppm, drift_ppm, n) * 1e-6
        est, unc = [], {}
        for i in range(n):
            samples = []
            for _probe in range(8):
                out, back = rng.uniform(0.001, 0.03, 2)       # asymmetric one-way delays
                t0 = 100.0
                t1 = t0 + out + true_offset[i]
                t2 = t1 + 0.0005
                t3 = t2 - true_offset[i] + back
                samples.append(estimate_clock_offset(t0, t1, t2, t3))
            offset, half = best_clock_sample(samples)
            est.append(offset)
            unc[i] = half
        error = np.array(est) - true_offset + drift * age       # offset error at the time of the start
        skew = (error.max() - error.min()) + rng.uniform(0, tick)   # plus where in the tick the action lands
        budget = compute_budget(unc, age_s=age, drift_ppm=drift_ppm, tick_s=tick)
        assert skew <= budget.skew_bound_s + 1e-9
        worst_ratio = max(worst_ratio, skew / budget.skew_bound_s)
    assert worst_ratio > 0.2        # the bound is not vacuous
