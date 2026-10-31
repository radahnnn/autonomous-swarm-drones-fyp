"""Clock-synchronisation error budget for synchronized starts (pure logic).

The console estimates each agent's clock offset with the NTP four-timestamp method and keeps the lowest-delay
sample; half of that sample's network round trip bounds the offset error from path asymmetry
(``sitl/clock_sync.py``). Between resyncs each clock also drifts. Two agents that should start the same action
at one operator-clock instant can therefore disagree by at most

    skew_ij <= u_i + u_j + (|d_i| + |d_j|) * age + tick

where ``u`` is the offset-uncertainty bound, ``d`` the clock drift rate (ppm * 1e-6), ``age`` the time since the
last resync, and ``tick`` one agent control-loop period (an agent acts on its next tick after the instant).
The worst pair gives the swarm-wide bound. A synchronized start also needs the message that announces it to
arrive first, so the minimum safe lead (announcement to start) is the worst skew plus the worst one-way
latency plus a margin.

Assumptions (stated, not measured here): drift is bounded by ``drift_ppm`` (typical quartz is tens of ppm;
the default is conservative), the half-RTT bound holds (it needs non-negative one-way delays and stable clocks
during the probe), and clocks do not step between probes.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Mapping


@dataclass(frozen=True)
class ClockBudget:
    max_uncertainty_s: float
    drift_term_s: float
    tick_s: float
    skew_bound_s: float
    required_lead_s: float
    agents: int

    def as_dict(self) -> dict:
        return {k: round(v, 6) if isinstance(v, float) else v for k, v in self.__dict__.items()}


def compute_budget(uncertainty_s: Mapping[int, float], *, age_s: float, drift_ppm: float = 100.0,
                   tick_s: float = 0.1, latency_s: float = 0.1, margin_s: float = 0.25) -> ClockBudget:
    """Worst-case start skew and minimum safe announcement lead from per-agent offset uncertainty."""
    values = sorted((float(u) for u in uncertainty_s.values()), reverse=True)
    if not values or not all(math.isfinite(u) and u >= 0 for u in values):
        raise ValueError("need finite non-negative offset uncertainties for at least one agent")
    for name, value in (("age_s", age_s), ("drift_ppm", drift_ppm), ("tick_s", tick_s),
                        ("latency_s", latency_s), ("margin_s", margin_s)):
        if not math.isfinite(value) or value < 0:
            raise ValueError(f"{name} must be finite and non-negative")
    pair = values[0] + (values[1] if len(values) > 1 else 0.0)
    drift = 2.0 * drift_ppm * 1e-6 * age_s
    skew = pair + drift + tick_s
    return ClockBudget(values[0], drift, tick_s, skew, skew + latency_s + margin_s, len(values))
