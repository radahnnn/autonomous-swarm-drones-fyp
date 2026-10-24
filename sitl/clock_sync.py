"""Small NTP-style offset estimator for operator-to-agent clock checks."""

import math


def estimate_clock_offset(t0: float, t1: float, t2: float, t3: float,
                          agent_processing_s: float | None = None,
                          max_clock_step_s: float = 0.005) -> tuple[float, float]:
    """Return (agent-minus-operator offset estimate, network round-trip delay).

    ``t0``/``t3`` are operator wall-clock send/receive times; ``t1``/``t2`` are
    agent wall-clock receive/send times. This is the standard four-timestamp NTP estimate.
    With stable clocks and nonnegative one-way delays, half the measured network round-trip is a
    worst-case bound on offset error from path asymmetry; it is not an estimate that the path is
    symmetric or a guarantee against clock drift during the probe. When
    ``agent_processing_s`` is supplied from the agent's monotonic clock, reject a probe if the
    agent wall clock advanced by a materially different amount while handling it.
    """
    values = tuple(float(value) for value in (t0, t1, t2, t3))
    if not all(math.isfinite(value) for value in values):
        raise ValueError("clock probe timestamps must be finite")
    if not math.isfinite(max_clock_step_s) or max_clock_step_s < 0:
        raise ValueError("maximum clock-step tolerance must be finite and non-negative")
    t0, t1, t2, t3 = values
    if t3 < t0 or t2 < t1:
        raise ValueError("clock probe timestamps are not ordered within each clock")
    if agent_processing_s is not None:
        agent_processing_s = float(agent_processing_s)
        if not math.isfinite(agent_processing_s) or agent_processing_s < 0:
            raise ValueError("agent monotonic processing time must be finite and non-negative")
        if abs((t2 - t1) - agent_processing_s) > max_clock_step_s:
            raise ValueError("agent wall clock changed during probe")
    round_trip = (t3 - t0) - (t2 - t1)
    if round_trip < -0.001:
        raise ValueError("clock probe implies a negative network round-trip delay")
    round_trip = max(0.0, round_trip)
    offset = ((t1 - t0) + (t2 - t3)) / 2.0
    return offset, round_trip


def best_clock_sample(samples, max_round_trip_s: float = 1.0) -> tuple[float, float]:
    """Select the lowest-delay valid sample and return its offset and half-RTT bound."""
    if not math.isfinite(max_round_trip_s) or max_round_trip_s <= 0:
        raise ValueError("maximum clock probe delay must be positive and finite")
    valid = []
    for sample in samples:
        try:
            offset, round_trip = sample
            offset, round_trip = float(offset), float(round_trip)
        except (TypeError, ValueError):
            continue
        if (math.isfinite(offset) and math.isfinite(round_trip) and
                0.0 <= round_trip <= max_round_trip_s):
            valid.append((round_trip, offset))
    if not valid:
        raise ValueError("no clock sample met the round-trip bound")
    round_trip, offset = min(valid)
    return offset, round_trip / 2.0
