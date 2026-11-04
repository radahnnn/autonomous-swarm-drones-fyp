"""Decentralized formation reference: leader-follower consensus tracking (pure logic).

User decision 2026-11-04 (PROJECT_STATE section 8, item 2): in decentralized mode a drone must keep formation from its
neighbours, with the goal coming from the elected leader rather than the operator.

Each drone i keeps a formation-reference estimate r_i (the shared North/East point its slot is measured from) and an
estimate v_i of the leader's goal velocity, and shares both with its ring neighbours over the peer link.

- Leader (the elected leader): r_L follows the mission goal (its cached mission/route, which it also re-broadcasts as
  acting coordinator when the operator is silent); v_L is that goal's rate.
- Follower: v_i' = -k_v (v_i - mean_j v_j) and r_i' = v_i - k (r_i - mean_j r_j), over fresh neighbours, with r_j
  extrapolated by v_j * age. Followers never read the operator's goal. This is consensus tracking with an estimated
  leader velocity (Ren & Beard 2008, ch. 3): if the leader is reachable from every drone through the neighbour graph,
  v_i -> v_L and r_i -> r_L exponentially, and the steady error behind a constant-velocity leader is zero.
  (Feeding the neighbours' full reference rate forward instead does not settle: followers keep exciting each other;
  found by the unit test on 2026-11-04.)
- Without fresh neighbours a follower dead-reckons r_i with v_i for HOLD_S, then holds it.

The drone then flies r_i + slot_i + a bounded formation correction from its neighbours' measured positions
(see `formation_correction`). Formation *transitions* remain planned by the coordinator (the plan's slots).
"""
from __future__ import annotations

from typing import Iterable, Optional, Sequence, Tuple

import numpy as np


class FormationReference:
    GAIN = 3.0                 # 1/s; consensus gain towards the neighbours' mean reference (stable for dt*GAIN < 1)
    VELOCITY_GAIN = 3.0        # 1/s; consensus gain of the leader-velocity estimate
    MAX_RATE_M_S = 1.5         # never move the reference faster than the drones can follow
    HOLD_S = 3.0               # dead-reckon this long without neighbours, then hold

    def __init__(self) -> None:
        self.ref: Optional[np.ndarray] = None
        self.rate = np.zeros(2)            # estimate of the leader's goal velocity (shared with neighbours)
        self._t: Optional[float] = None
        self._last_goal: Optional[Tuple[np.ndarray, float]] = None
        self._alone_since: Optional[float] = None

    def reset(self, ref: Sequence[float]) -> None:
        self.ref, self.rate = np.asarray(ref, dtype=float)[:2].copy(), np.zeros(2)

    def update(self, now: float, is_leader: bool, goal: Optional[Sequence[float]],
               neighbours: Iterable[Tuple[Sequence[float], Sequence[float], float]]) -> np.ndarray:
        """neighbours: (ref, leader_velocity_estimate, age_s) of each fresh neighbour. goal: the mission goal (used by the leader, and to
        initialise a drone that has no reference yet). Returns this drone's reference."""
        dt = 0.0 if self._t is None else max(0.0, min(0.5, now - self._t))
        self._t = now
        if goal is not None:
            goal = np.asarray(goal, dtype=float)[:2]
        if self.ref is None:
            if goal is None:
                raise ValueError("a formation reference needs a goal to start from")
            self.reset(goal)
        if is_leader and goal is not None:
            rate = np.zeros(2)
            if self._last_goal is not None and now - self._last_goal[1] > 1e-3:
                rate = (goal - self._last_goal[0]) / (now - self._last_goal[1])
            self._last_goal = (goal.copy(), now)
            self.ref, self.rate = goal.copy(), _limit(rate, self.MAX_RATE_M_S)
            self._alone_since = None
            return self.ref.copy()
        self._last_goal = None
        refs, rates = [], []
        for ref, rate, age in neighbours:
            rate = np.asarray(rate, dtype=float)[:2]
            refs.append(np.asarray(ref, dtype=float)[:2] + rate * max(0.0, float(age)))
            rates.append(rate)
        # Predict this drone's reference to `now` first, then compare it with the neighbours' references extrapolated
        # to the same instant (comparing a one-tick-old own value biased every hop; found by the unit test).
        predicted = self.ref + self.rate * dt
        if refs:
            self._alone_since = None
            mean_ref, mean_rate = np.mean(refs, axis=0), np.mean(rates, axis=0)
            self.rate = _limit(self.rate + min(1.0, self.VELOCITY_GAIN * dt) * (mean_rate - self.rate),
                               self.MAX_RATE_M_S)
            correction = _limit(-self.GAIN * (predicted - mean_ref), self.MAX_RATE_M_S)
            predicted = predicted + correction * dt
        else:
            self._alone_since = now if self._alone_since is None else self._alone_since
            if now - self._alone_since > self.HOLD_S:
                self.rate = np.zeros(2)
        self.ref = predicted
        return self.ref.copy()


def formation_correction(position: Sequence[float], slot: Sequence[float],
                         neighbours: Iterable[Tuple[Sequence[float], Sequence[float]]],
                         gain: float = 0.5, limit_m: float = 1.0) -> np.ndarray:
    """Bounded pull towards where the neighbours' measured positions put this drone's slot:
    gain * mean_j[(p_j - s_j) - (p_i - s_i)], clipped to limit_m. neighbours: (position_j, slot_j)."""
    own = np.asarray(position, dtype=float)[:2] - np.asarray(slot, dtype=float)[:2]
    implied = [np.asarray(p, dtype=float)[:2] - np.asarray(s, dtype=float)[:2] for p, s in neighbours]
    if not implied:
        return np.zeros(2)
    return _limit(gain * (np.mean(implied, axis=0) - own), limit_m)


def _limit(vector: np.ndarray, maximum: float) -> np.ndarray:
    norm = float(np.linalg.norm(vector))
    return vector * (maximum / norm) if norm > maximum else vector
