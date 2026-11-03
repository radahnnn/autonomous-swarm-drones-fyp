"""Control-barrier-function safety filter for the commanded setpoint (pure logic).

For every nearby drone j the barrier is ``h_ij = |p_i - p_j|^2 - D^2`` (D = safety distance). Keeping
``dh/dt >= -alpha * h`` makes the set ``h >= 0`` forward invariant for a single-integrator vehicle
(Ames et al. 2017; Wang, Ames, Egerstedt 2017). With the neighbour assumed to keep its current velocity
``v_j`` this is one half-plane constraint on this drone's velocity ``u``:

    (p_i - p_j) . (u - v_j)  >=  -alpha * h_ij / 2

The filter returns the velocity closest to the nominal one (the velocity that would reach the anchored
setpoint) that satisfies every constraint and the speed limit: a 2-D quadratic program on a disc, solved exactly by
enumerating line projections, line-line and line-circle intersections and the circle point nearest the nominal
(at most ~10 neighbours). The caller converts the filtered velocity back into a setpoint.

What this is and is not: a *sampled, single-integrator* safety filter. The vehicle's real dynamics
(autopilot lag, wind, position noise) are not modelled, neighbours' future motion is assumed constant,
and the guarantee only holds under those assumptions and while the problem stays feasible. When it is
infeasible (surrounded) the filter falls back to the least-bad escape velocity and reports it.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Tuple

import numpy as np

_EPS = 1e-9


@dataclass(frozen=True)
class FilterResult:
    velocity: np.ndarray            # filtered velocity (m/s), same frame as the inputs
    active: bool                    # the filter changed the nominal velocity
    feasible: bool                  # all constraints could be satisfied together
    constraints: int                # neighbours that produced a binding-capable constraint
    closest_m: Optional[float]      # distance to the nearest considered neighbour


def _constraints(position: np.ndarray, neighbors,
                 distance: float, alpha: float, horizon_m: float):
    """Half-planes a.u >= b for neighbours inside the consideration radius (optional per-neighbour extra distance)."""
    rows: List[Tuple[np.ndarray, float]] = []
    closest = None
    for other_position, other_velocity, extra in neighbors:
        pair_distance = distance + extra
        delta = position - other_position
        d = float(np.linalg.norm(delta))
        if not math.isfinite(d):
            continue
        closest = d if closest is None else min(closest, d)
        if d > horizon_m:
            continue
        if d < 1e-6:                                   # coincident estimates: push along a fixed axis
            delta, d = np.array([1.0, 0.0]) * 1e-3, 1e-3
        h = d * d - pair_distance * pair_distance
        rows.append((delta, float(delta @ other_velocity) - 0.5 * alpha * h))
    return rows, closest


def _feasible(u: np.ndarray, rows, tol: float = 1e-7) -> bool:
    return all(float(a @ u) >= b - tol for a, b in rows)


def filter_velocity(position, nominal_velocity, neighbors, *, distance: float = 2.6, alpha: float = 1.5,
                    max_speed: float = 1.5, horizon_m: float = 8.0) -> FilterResult:
    """Closest velocity to ``nominal_velocity`` that keeps every pair outside ``distance`` (see module doc).

    ``neighbors`` is an iterable of ``(position, velocity)`` or ``(position, velocity, extra_distance)`` items (2-D);
    the extra distance (>= 0) widens the barrier for that neighbour, e.g. one whose position estimate is degraded.
    Non-finite or malformed neighbours are ignored; a non-finite own state or nominal velocity raises ``ValueError``.
    """
    position = np.asarray(position, dtype=float)[:2]
    nominal = np.asarray(nominal_velocity, dtype=float)[:2]
    if not (np.isfinite(position).all() and np.isfinite(nominal).all()):
        raise ValueError("position and nominal velocity must be finite")
    if distance <= 0 or alpha <= 0 or max_speed <= 0:
        raise ValueError("distance, alpha and max_speed must be positive")
    clean: List[Tuple[np.ndarray, np.ndarray]] = []
    for item in neighbors:
        try:
            p, v = np.asarray(item[0], dtype=float)[:2], np.asarray(item[1], dtype=float)[:2]
            extra = float(item[2]) if len(item) > 2 else 0.0
        except (TypeError, ValueError, IndexError):
            continue
        if (p.shape == (2,) and v.shape == (2,) and np.isfinite(p).all() and np.isfinite(v).all()
                and math.isfinite(extra) and extra >= 0.0):
            clean.append((p, v, extra))
    rows, closest = _constraints(position, clean, distance, alpha, horizon_m)
    speed = float(np.linalg.norm(nominal))
    start = nominal if speed <= max_speed else nominal * (max_speed / speed)
    if not rows or _feasible(start, rows):
        return FilterResult(start, False, True, len(rows), closest)

    # Exact QP on the disc |u| <= max_speed: the optimum is the nominal's projection onto the feasible set, so it lies
    # (a) on one constraint line, (b) at two lines' intersection, (c) where a line meets the speed circle, or (d) on
    # the circle closest to the nominal. Enumerate all of them (Codex review #7: (c) was missing, so some feasible
    # problems were reported infeasible) and keep the feasible one closest to the ORIGINAL nominal velocity.
    candidates: List[np.ndarray] = [start]
    for a, b in rows:
        norm2 = float(a @ a)
        candidates.append(nominal + (b - float(a @ nominal)) / norm2 * a)          # (a) projection onto the line
        foot = b / norm2 * a                                                         # (c) line-circle intersections
        reach = max_speed ** 2 - float(foot @ foot)
        if reach >= 0.0:
            along = np.array([-a[1], a[0]]) / math.sqrt(norm2)
            for sign in (-1.0, 1.0):
                candidates.append(foot + sign * math.sqrt(reach) * along)
    for i in range(len(rows)):                                                       # (b) line-line intersections
        for j in range(i + 1, len(rows)):
            (a1, b1), (a2, b2) = rows[i], rows[j]
            det = float(a1[0] * a2[1] - a1[1] * a2[0])
            if abs(det) < 1e-9:
                continue
            candidates.append(np.array([(b1 * a2[1] - b2 * a1[1]) / det, (a1[0] * b2 - a2[0] * b1) / det]))
    best = None
    for u in candidates:
        if float(np.linalg.norm(u)) > max_speed + 1e-9 or not _feasible(u, rows):
            continue
        cost = float(np.linalg.norm(u - nominal))
        if best is None or cost < best[0]:
            best = (cost, u)
    if best is not None:
        return FilterResult(best[1], True, True, len(rows), closest)

    # Truly infeasible within the speed limit (surrounded, or already deep inside a barrier): no velocity satisfies
    # every constraint, so this is a heuristic, not an optimum - move away from all neighbours at full speed, each
    # weighted by closeness. Reported with feasible=False.
    escape = sum((a / float(a @ a) ** 0.5) / max(float(a @ a) ** 0.5, 0.1) for a, _ in rows)
    norm = float(np.linalg.norm(escape))
    u = escape * (max_speed / norm) if norm > _EPS else np.zeros(2)
    return FilterResult(u, True, False, len(rows), closest)


def filtered_setpoint(position, setpoint, neighbors, *, horizon_s: float = 1.0, max_correction_m: float = 3.0,
                      **kwargs) -> Tuple[np.ndarray, FilterResult]:
    """Apply the filter to an anchored position setpoint.

    The nominal velocity is the one that reaches ``setpoint`` in ``horizon_s``; if the filter changes it, the
    new setpoint is ``position + u_safe * horizon_s`` (never more than ``max_correction_m`` from the
    position); otherwise the original setpoint is returned untouched.
    """
    position = np.asarray(position, dtype=float)[:2]
    setpoint = np.asarray(setpoint, dtype=float)[:2]
    nominal = (setpoint - position) / horizon_s
    result = filter_velocity(position, nominal, neighbors, **kwargs)
    if not result.active:
        return setpoint.copy(), result
    shift = result.velocity * horizon_s
    norm = float(np.linalg.norm(shift))
    if norm > max_correction_m:
        shift *= max_correction_m / norm
    return position + shift, result
