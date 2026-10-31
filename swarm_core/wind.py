"""Wind disturbance for the pure simulator (acceleration added to each drone).

Wind is a steady mean plus a gust. The gust has a swarm-wide component (the same air mass moves the whole
formation) and a per-drone component (turbulence decorrelates over a few metres), each a first-order
Gauss-Markov (Ornstein-Uhlenbeck) process, so it is smooth, bounded in variance and reproducible from a seed.

The values are accelerations in m/s^2 because ``Drone.step`` adds ``wind_vector`` to the net acceleration
(an aerodynamic drag model would turn a wind *speed* into an acceleration through the drag coefficient: use
``from_speed``). This is an assumed disturbance model, not a measured one: it has not been fitted to ArduPilot
SITL wind or to flight data.
"""
from __future__ import annotations

import math
from typing import Sequence

import numpy as np


class WindModel:
    def __init__(self, mean_accel: Sequence[float] = (0.0, 0.0), gust_sigma: float = 0.0,
                 common_fraction: float = 0.5, corr_time_s: float = 3.0, seed: int | None = None, dim: int = 2):
        mean = np.asarray(mean_accel, dtype=float)
        if mean.shape != (dim,) or not np.isfinite(mean).all():
            raise ValueError("mean wind acceleration must be finite with one value per axis")
        if not math.isfinite(gust_sigma) or gust_sigma < 0:
            raise ValueError("gust sigma must be finite and non-negative")
        if not 0.0 <= common_fraction <= 1.0:
            raise ValueError("common fraction must be within [0, 1]")
        if not math.isfinite(corr_time_s) or corr_time_s <= 0:
            raise ValueError("gust correlation time must be positive")
        self.mean, self.sigma, self.fraction, self.tau, self.dim = mean, float(gust_sigma), common_fraction, corr_time_s, dim
        self.rng = np.random.default_rng(seed)
        self._common = self.rng.normal(0, math.sqrt(common_fraction) * self.sigma, dim)
        self._own: dict[int, np.ndarray] = {}

    @classmethod
    def from_speed(cls, mean_speed_m_s: Sequence[float], gust_speed_m_s: float, drag_coeff: float, **kwargs):
        """Wind speeds (m/s) to equivalent accelerations through the linear drag coefficient (1/s)."""
        return cls(np.asarray(mean_speed_m_s, dtype=float) * drag_coeff, gust_speed_m_s * drag_coeff, **kwargs)

    def advance(self, dt: float, drone_ids: Sequence[int]) -> dict[int, np.ndarray]:
        """Advance one step and return the wind acceleration for each drone id."""
        phi = math.exp(-dt / self.tau)
        drive = math.sqrt(max(0.0, 1.0 - phi * phi))
        s_common, s_own = math.sqrt(self.fraction) * self.sigma, math.sqrt(1.0 - self.fraction) * self.sigma
        self._common = phi * self._common + drive * self.rng.normal(0, s_common, self.dim)
        out = {}
        for drone_id in drone_ids:
            previous = self._own.get(drone_id)
            if previous is None:
                previous = self.rng.normal(0, s_own, self.dim)
            self._own[drone_id] = phi * previous + drive * self.rng.normal(0, s_own, self.dim)
            out[drone_id] = self.mean + self._common + self._own[drone_id]
        return out
