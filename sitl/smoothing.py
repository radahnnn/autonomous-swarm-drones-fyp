"""Rest-to-rest time scaling shared by formation transitions (pure logic).

``min_jerk(alpha)`` is the quintic 10a^3 - 15a^4 + 6a^5. For a rest-to-rest move between two points it is
the minimum-jerk profile (Flash & Hogan 1985; the single-segment case of Mellinger & Kumar 2011), with zero
velocity and acceleration at both ends. Peak speed is ``PEAK_SPEED_FACTOR`` times the average speed, at the
midpoint.

Every drone uses the same scaling over the same duration, so the *set of points* each pair of straight paths
passes through is unchanged by the smoothing; only when it passes them changes. The planner's pairwise
path-clearance check (``sitl/formation_transition.py``) therefore stays valid for the smoothed motion, and the
duration is chosen from the peak speed.
"""
from __future__ import annotations

PEAK_SPEED_FACTOR = 1.875


def min_jerk(alpha: float) -> float:
    alpha = min(1.0, max(0.0, float(alpha)))
    return alpha ** 3 * (10.0 - 15.0 * alpha + 6.0 * alpha ** 2)
