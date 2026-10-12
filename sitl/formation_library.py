"""
Formation slot templates and slot assignment for the swarm flight console.

Slots are offsets from the swarm centroid in the NED frame: [North, East, Down] metres.
Every style works for any number of drones (1..10+). For the sizes the project was
developed with (up to 5 drones) the hand-tuned slot tables are kept exactly; larger
swarms use generators that extend the same pattern. All templates keep neighbours at
least MIN_SPACING apart.
"""

import math
from typing import Dict, List

import numpy as np
from scipy.optimize import linear_sum_assignment

MIN_SPACING = 2.5  # metres, smallest allowed distance between two slots
STYLES = ("v", "line", "column", "diamond", "ring", "echelon", "triangle")

_V_TABLE: Dict[int, List[List[float]]] = {
    1: [[0.0, 0.0]],
    2: [[0.0, -2.5], [0.0, 2.5]],
    3: [[0.0, 0.0], [-3.0, -3.0], [-3.0, 3.0]],
    4: [[0.0, 0.0], [-3.0, -3.0], [-3.0, 3.0], [-5.0, 0.0]],
    5: [[0.0, 0.0], [-3.0, -3.0], [-3.0, 3.0], [-6.0, -6.0], [-6.0, 6.0]],
}
_DIAMOND_TABLE: Dict[int, List[List[float]]] = {
    1: [[0.0, 0.0]],
    2: [[0.0, -2.5], [0.0, 2.5]],
    3: [[2.5, 0.0], [-2.5, -3.0], [-2.5, 3.0]],
    4: [[3.5, 0.0], [0.0, -3.5], [0.0, 3.5], [-3.5, 0.0]],
    5: [[4.0, 0.0], [0.0, -4.0], [0.0, 4.0], [-4.0, 0.0], [0.0, 0.0]],
}


def _to_3d(rows) -> np.ndarray:
    pts = np.asarray(rows, dtype=float).reshape(-1, 2)
    return np.hstack([pts, np.zeros((len(pts), 1))])


def _v(n: int) -> np.ndarray:
    if n in _V_TABLE:
        return _to_3d(_V_TABLE[n])
    slots = [[0.0, 0.0]]  # apex, then wing pairs fanning out behind it, left before right
    rank = 1
    while len(slots) < n:
        slots.append([-3.0 * rank, -3.0 * rank])
        if len(slots) < n:
            slots.append([-3.0 * rank, 3.0 * rank])
        rank += 1
    return _to_3d(slots)


def _abreast(n: int, axis: int) -> np.ndarray:
    """Evenly spaced row (axis=1, East) or file (axis=0, North), centred on the centroid."""
    if n == 1:
        return _to_3d([[0.0, 0.0]])
    spacing = 5.0 if n == 2 else 3.0
    pts = np.zeros((n, 2))
    for k in range(n):
        offset = (k - (n - 1) / 2.0) * spacing
        pts[k, axis] = -offset if axis == 0 else offset  # column: leader (k=0) in front (+North)
    return _to_3d(pts)


def _diamond(n: int) -> np.ndarray:
    if n in _DIAMOND_TABLE:
        return _to_3d(_DIAMOND_TABLE[n])
    slots: List[List[float]] = []
    d = 4.0
    while len(slots) < n:
        slots += [[d, 0.0], [0.0, -d], [0.0, d], [-d, 0.0]]
        if d == 4.0:
            slots.append([0.0, 0.0])
        d += 4.0
    return _to_3d(slots[:n])


def _ring(n: int) -> np.ndarray:
    if n == 1:
        return _to_3d([[0.0, 0.0]])
    radius = max(4.0, MIN_SPACING * 1.2 / (2.0 * math.sin(math.pi / n)))
    ang = [2.0 * math.pi * k / n for k in range(n)]
    return _to_3d([[radius * math.cos(a), radius * math.sin(a)] for a in ang])


def _echelon(n: int) -> np.ndarray:
    step = 2.5
    return _to_3d([[((n - 1) / 2.0 - k) * step, -((n - 1) / 2.0 - k) * step] for k in range(n)])


def _triangle(n: int) -> np.ndarray:
    pts: List[List[float]] = []
    row = 0
    while len(pts) < n:
        for j in range(row + 1):
            if len(pts) < n:
                pts.append([-2.5 * row, (j - row / 2.0) * 3.0])
        row += 1
    arr = np.asarray(pts, dtype=float)
    arr[:, 0] -= arr[:, 0].mean()
    return _to_3d(arr)


_GENERATORS = {
    "v": _v,
    "line": lambda n: _abreast(n, axis=1),
    "column": lambda n: _abreast(n, axis=0),
    "diamond": _diamond,
    "ring": _ring,
    "echelon": _echelon,
    "triangle": _triangle,
}


def slots_for(style: str, n: int) -> np.ndarray:
    """Return the (n, 3) slot offsets [N, E, D] for `n` drones in the given formation style."""
    if style not in _GENERATORS:
        raise ValueError(f"Unknown formation style '{style}'. Valid: {list(STYLES)}")
    if n < 1:
        raise ValueError("A formation needs at least one drone")
    return _GENERATORS[style](n)


def assign_slots(current_offsets: np.ndarray, slots: np.ndarray, z_weight: float = 2.0) -> List[int]:
    """
    Optimal drone-to-slot matching (Hungarian algorithm) minimising total squared travel,
    which prevents crossing paths. Returns slot_index[i] for every drone i.
    """
    current = np.asarray(current_offsets, dtype=float)
    slots = np.asarray(slots, dtype=float)
    if len(current) != len(slots):
        raise ValueError("Need exactly one slot per drone")
    diff = current[:, None, :] - slots[None, :, :]
    cost = diff[..., 0] ** 2 + diff[..., 1] ** 2 + z_weight * diff[..., 2] ** 2
    rows, cols = linear_sum_assignment(cost)
    out = [0] * len(current)
    for r, c in zip(rows, cols):
        out[r] = int(c)
    return out
