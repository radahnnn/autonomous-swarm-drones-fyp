"""Keep-out geometry for separation: sphere, cylinder or ellipsoid (pure logic).

Two drones are clear if ``(h / Rh)^2 + (dz / Rv)^2 >= 1`` where ``h`` is the horizontal distance and ``dz`` the
vertical one. ``Rh`` is the project's horizontal gate (2.5 m); ``Rv`` is the vertical semi-axis:

* ``Rv = inf``  - horizontal-only gate (the project's acceptance definition; altitude never helps);
* ``Rv = Rh``   - a sphere (the plain 3-D distance);
* ``Rv > Rh``   - an ellipsoid that is taller than wide, the usual downwash-aware shape (a vehicle below another
  sits in its rotor wash even when it is not far below). Crazyswarm and EGO-Swarm style planners use ellipsoids.

``horizontal_equivalent_gap`` returns the pair's gap as a horizontal-equivalent distance
(``Rh * sqrt((h/Rh)^2 + (dz/Rv)^2)``), so it can be compared with the same gate in metres whatever the shape.

``Rv`` is an input, not a measurement: the real wash zone depends on the vehicle and has not been measured for
the team's quad. The default of the console is the horizontal-only gate.
"""
from __future__ import annotations

import math


def horizontal_equivalent_gap(horizontal_m: float, vertical_m: float, rh_m: float, rv_m: float) -> float:
    if not (rh_m > 0 and rv_m > 0) or not all(math.isfinite(v) for v in (horizontal_m, vertical_m, rh_m)):
        raise ValueError("radii must be positive and distances finite")
    if math.isinf(rv_m):
        return horizontal_m
    return rh_m * math.hypot(horizontal_m / rh_m, abs(vertical_m) / rv_m)


def is_clear(horizontal_m: float, vertical_m: float, rh_m: float, rv_m: float) -> bool:
    return horizontal_equivalent_gap(horizontal_m, vertical_m, rh_m, rv_m) >= rh_m - 1e-12


def required_horizontal_m(vertical_m: float, rh_m: float, rv_m: float) -> float:
    """Horizontal distance needed at a given vertical offset (0 once the vertical offset reaches ``Rv``)."""
    if math.isinf(rv_m):
        return rh_m
    return rh_m * math.sqrt(max(0.0, 1.0 - (abs(vertical_m) / rv_m) ** 2))


def layer_plan(layers: int, spacing_m: float, rh_m: float, rv_m: float) -> dict:
    """What an altitude-layer set buys under a keep-out shape.

    Drones are in layer ``id mod layers``. Same-layer pairs always need the full ``Rh`` horizontally. Pairs in
    layers ``k`` apart are ``k*spacing`` above each other and need ``required_horizontal_m`` of that. The planner and
    separation guard stay purely horizontal (conservative), so this only says how much *margin* the layers add.
    """
    if layers < 1 or spacing_m < 0:
        raise ValueError("layers must be >= 1 and spacing non-negative")
    rows = [{"layer_gap": k, "vertical_m": k * spacing_m, "required_horizontal_m": round(required_horizontal_m(k * spacing_m, rh_m, rv_m), 3)}
            for k in range(layers)]
    return {"rows": rows, "spacing_for_full_relief_m": None if math.isinf(rv_m) else rv_m,
            "full_relief_reached": (not math.isinf(rv_m)) and layers > 1 and spacing_m >= rv_m}
