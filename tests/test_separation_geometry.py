import math

import pytest

from sitl.run_metrics import RunMetrics
from sitl.separation_geometry import horizontal_equivalent_gap, is_clear, layer_plan, required_horizontal_m


def test_special_cases_reduce_to_horizontal_and_sphere():
    assert horizontal_equivalent_gap(2.0, 5.0, 2.5, math.inf) == 2.0
    assert horizontal_equivalent_gap(3.0, 4.0, 2.5, 2.5) == pytest.approx(2.5 * 2.0)      # |(3,4)|=5 -> 5 m in a sphere
    assert horizontal_equivalent_gap(3.0, 0.0, 2.5, 7.0) == pytest.approx(3.0)


def test_downwash_ellipsoid_is_stricter_than_the_sphere_for_the_same_point():
    # one drone 2 m below another: sphere (Rv=Rh) lets it be 1.5 m aside; a taller ellipsoid (Rv=4) does not
    assert is_clear(1.6, 2.0, 2.5, 2.5)
    assert not is_clear(1.6, 2.0, 2.5, 4.0)
    assert required_horizontal_m(2.0, 2.5, 4.0) == pytest.approx(2.5 * math.sqrt(1 - 0.25))
    assert required_horizontal_m(5.0, 2.5, 4.0) == 0.0
    assert required_horizontal_m(1.0, 2.5, math.inf) == 2.5


def test_layer_plan_full_relief_only_when_spacing_reaches_vertical_radius():
    plan = layer_plan(3, 1.5, 2.5, 4.0)
    assert [r["required_horizontal_m"] for r in plan["rows"]][0] == 2.5
    assert not plan["full_relief_reached"]
    assert layer_plan(3, 4.0, 2.5, 4.0)["full_relief_reached"]
    assert not layer_plan(1, 9.0, 2.5, 4.0)["full_relief_reached"]
    with pytest.raises(ValueError):
        layer_plan(0, 1.0, 2.5, 4.0)


def _statuses(positions, now=100.0):
    return {i: {"airborne": True, "position": list(p), "operator_received_at": now} for i, p in enumerate(positions)}


def test_run_metrics_ellipsoid_gate_scores_vertical_neighbours():
    # 1.6 m apart horizontally, 2 m apart vertically
    positions = [(0.0, 0.0, -5.0), (1.6, 0.0, -7.0)]
    horizontal = RunMetrics(2); horizontal.update(100.0, _statuses(positions))
    sphere = RunMetrics(2, gate_metric="3d"); sphere.update(100.0, _statuses(positions))
    tall = RunMetrics(2, gate_metric="ellipsoid", vertical_radius_m=4.0); tall.update(100.0, _statuses(positions))
    assert horizontal.violations == 1 and sphere.violations == 0 and tall.violations == 1


def test_ellipsoid_gate_requires_a_radius():
    with pytest.raises(ValueError):
        RunMetrics(2, gate_metric="ellipsoid")
