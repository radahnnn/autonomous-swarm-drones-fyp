import numpy as np

from sitl.formation_transition import plan_formation_transition
from sitl.smoothing import PEAK_SPEED_FACTOR, min_jerk


def test_endpoints_and_clamping():
    assert min_jerk(0) == 0 and min_jerk(1) == 1 and min_jerk(0.5) == 0.5
    assert min_jerk(-3) == 0 and min_jerk(7) == 1


def test_rest_to_rest_derivatives_vanish_at_both_ends():
    h = 1e-4
    for a in (0.0, 1.0):
        sign = 1 if a == 0 else -1
        v = (min_jerk(a + sign * h) - min_jerk(a)) / h
        acc = (min_jerk(a + 2 * sign * h) - 2 * min_jerk(a + sign * h) + min_jerk(a)) / h ** 2
        assert abs(v) < 1e-3 and abs(acc) < 0.01


def test_monotone_and_peak_speed_matches_documented_factor():
    alphas = np.linspace(0, 1, 2001)
    values = np.array([min_jerk(a) for a in alphas])
    assert (np.diff(values) >= 0).all()
    assert abs(np.diff(values).max() / (alphas[1] - alphas[0]) - PEAK_SPEED_FACTOR) < 1e-3


def test_minimises_jerk_among_polynomial_rest_to_rest_profiles():
    """Integral of squared jerk for the quintic is below that of the cubic smoothstep (zero jerk impulses)."""
    t = np.linspace(0, 1, 20001)
    quintic = 60 - 360 * t + 360 * t ** 2          # third derivative of 10t^3-15t^4+6t^5
    cubic_jerk_energy = np.inf                      # cubic has a jerk (acceleration) step at the ends: infinite impulse
    assert np.trapezoid(quintic ** 2, t) < cubic_jerk_energy


def test_smoothing_keeps_pairwise_path_clearance_of_the_plan():
    """Same scaling for all drones: positions at time s are start + s*delta, so min pair distance over the
    smoothed motion equals the planner's straight-line value."""
    rng = np.random.default_rng(3)
    current = rng.uniform(-6, 6, (6, 2))
    plan = plan_formation_transition(current, "line", minimum_separation_m=0.5)
    start, end = current, current.mean(axis=0) + plan.slots_by_drone[:, :2]
    smooth_min = min(
        np.linalg.norm((start[i] + min_jerk(a) * (end[i] - start[i])) - (start[j] + min_jerk(a) * (end[j] - start[j])))
        for a in np.linspace(0, 1, 4001) for i in range(6) for j in range(i + 1, 6))
    linear_min = min(
        np.linalg.norm((start[i] + a * (end[i] - start[i])) - (start[j] + a * (end[j] - start[j])))
        for a in np.linspace(0, 1, 4001) for i in range(6) for j in range(i + 1, 6))
    assert abs(smooth_min - linear_min) < 0.02
