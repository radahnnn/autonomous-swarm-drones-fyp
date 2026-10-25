"""Experimental acceleration projection for Phase 2 pairwise spacing.

This is a discrete, radial look-ahead constraint, not a formal collision
guarantee. It assumes both peers apply the same local rule and uses the latest
received peer state. It is kept opt-in until repeated trials show its effect.
"""

from __future__ import annotations

from typing import Mapping, Sequence

import numpy as np


def project_acceleration(
    nominal: np.ndarray,
    own_position: np.ndarray,
    own_velocity: np.ndarray,
    neighbors: Sequence[Mapping],
    *,
    minimum_distance_m: float = 2.75,
    horizon_s: float = 1.0,
    max_accel_m_s2: float = 2.5,
) -> tuple[np.ndarray, bool]:
    """Project a 2D command onto pairwise radial acceleration constraints.

    For a peer at distance d with outward unit vector n and relative radial
    velocity vr, each side is asked to satisfy n·a >= (D-d)/T² - vr/T.
    If both sides obey, the constant-acceleration radial prediction at T is
    at least D. Tangential motion and acceleration lag make this a diagnostic
    filter rather than a complete safety controller.
    """
    u = np.asarray(nominal, dtype=float).copy()
    p = np.asarray(own_position, dtype=float)
    v = np.asarray(own_velocity, dtype=float)
    if u.shape != (2,) or p.shape != (2,) or v.shape != (2,):
        raise ValueError("predictive safety expects 2D position, velocity and acceleration")
    if not all(np.isfinite(x).all() for x in (u, p, v)):
        raise ValueError("predictive safety inputs must be finite")
    if minimum_distance_m <= 0 or horizon_s <= 0 or max_accel_m_s2 <= 0:
        raise ValueError("safety distance, horizon and acceleration limit must be positive")

    constraints = []
    for peer in neighbors:
        peer_p = np.asarray(peer["position"], dtype=float)
        peer_v = np.asarray(peer["velocity"], dtype=float)
        if peer_p.shape != (2,) or peer_v.shape != (2,) or not (
            np.isfinite(peer_p).all() and np.isfinite(peer_v).all()
        ):
            continue
        delta = p - peer_p
        distance = float(np.linalg.norm(delta))
        if distance < 1e-8:
            # No reliable avoidance direction exists for coincident estimates.
            return -max_accel_m_s2 * v / max(float(np.linalg.norm(v)), 1e-8), False
        n = delta / distance
        vr = float(np.dot(v - peer_v, n))
        bound = (minimum_distance_m - distance) / (horizon_s**2) - vr / horizon_s
        constraints.append((n, bound))

    speed = float(np.linalg.norm(u))
    if speed > max_accel_m_s2:
        u *= max_accel_m_s2 / speed
    for _ in range(24):
        old = u.copy()
        for n, bound in constraints:
            deficit = bound - float(np.dot(n, u))
            if deficit > 0:
                u += deficit * n
        norm = float(np.linalg.norm(u))
        if norm > max_accel_m_s2:
            u *= max_accel_m_s2 / norm
        if float(np.linalg.norm(u - old)) < 1e-7:
            break
    feasible = all(float(np.dot(n, u)) >= bound - 1e-4 for n, bound in constraints)
    return u, feasible
