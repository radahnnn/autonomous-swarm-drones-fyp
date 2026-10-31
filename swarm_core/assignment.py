"""The one drone-to-slot assignment used everywhere (pure logic).

Minimum total *squared* travel (Hungarian algorithm, ``scipy.optimize.linear_sum_assignment``). The result is
deterministic for a given cost matrix, so every agent that computes it from the same inputs gets the same map.
Minimising summed squared distance does not by itself guarantee non-crossing straight paths (see
``docs/reports/ALGORITHM_REVIEW_2026-10-29.md`` 3.7); the transition planner screens the resulting paths.

``previous`` + ``hysteresis`` latch an earlier assignment: it is kept unless the new optimum is cheaper by more
than ``hysteresis`` (fraction of the old total cost), so near-tied solutions do not flip-flop while moving.
"""
from __future__ import annotations

from typing import Optional, Sequence

import numpy as np
from scipy.optimize import linear_sum_assignment


def squared_cost(current: np.ndarray, slots: np.ndarray, axis_weights: Optional[Sequence[float]] = None) -> np.ndarray:
    """cost[i, j] = weighted squared distance from drone i to slot j (any dimension)."""
    current, slots = np.asarray(current, dtype=float), np.asarray(slots, dtype=float)
    diff = current[:, None, :] - slots[None, :, :]
    weights = np.ones(diff.shape[-1]) if axis_weights is None else np.asarray(axis_weights, dtype=float)
    return (diff ** 2 * weights).sum(axis=-1)


def assign(cost: np.ndarray, previous: Optional[Sequence[int]] = None, hysteresis: float = 0.0) -> np.ndarray:
    """slot index for every row of a square cost matrix (see module doc for ``previous``/``hysteresis``)."""
    cost = np.asarray(cost, dtype=float)
    n = cost.shape[0]
    if cost.ndim != 2 or cost.shape[1] != n:
        raise ValueError("assignment needs a square cost matrix (one slot per drone)")
    rows, cols = linear_sum_assignment(cost)
    best = np.zeros(n, dtype=int)
    best[rows] = cols
    if previous is None:
        return best
    old = np.asarray(previous, dtype=int)
    if old.shape != (n,) or sorted(old.tolist()) != list(range(n)):
        return best
    if float(cost[np.arange(n), best].sum()) < (1.0 - float(hysteresis)) * float(cost[np.arange(n), old].sum()) - 1e-9:
        return best
    return old.copy()
