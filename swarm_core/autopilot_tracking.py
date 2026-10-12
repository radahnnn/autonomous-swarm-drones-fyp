"""
Setpoint-tracking model of ArduPilot's GUIDED position controller (identified from SITL step responses).

The swarm software commands position setpoints to ArduPilot, so a vehicle model for the simulator should map
"setpoint -> position", not "acceleration -> position". Structure (per drone, horizontal plane, vector form):

    r_d(t)  = r(t - td)                                   transport delay
    v_des   = sqrt_controller(r_d - p; k_pos, a_max)      square-root position controller, speed-limited to v_max
    a_cmd   = clip(Kv * (v_des - v), a_max)               velocity P loop with acceleration limit
    da/dt   = (a_cmd - a) / tau_a                         acceleration lag
    dv/dt   = a ,  dp/dt = v

Parameters were fitted by experiments/fit_step_structures.py (structure S4) on 5 m north SITL steps and validated on 2 m, 8 m,
east and diagonal steps (docs/reports/HW_MATCHED_IDENTIFICATION_REPORT.md). They are effective values of the autopilot's
tracking behaviour in SITL, not physical airframe parameters.
"""

from dataclasses import dataclass
from typing import Deque, Tuple
from collections import deque

import numpy as np


@dataclass(frozen=True)
class AutopilotTrackingParams:
    k_pos: float = 0.923   # 1/s   (ArduPilot PSC_POSXY_P configured: 1.0)
    kv: float = 2.350      # 1/s   (PSC_VELXY_P configured: 2.0)
    a_max: float = 2.657   # m/s^2 (WPNAV_ACCEL configured: 2.5)
    tau_a: float = 0.122   # s     acceleration lag
    td: float = 0.303      # s     transport delay
    v_max: float = 3.0     # m/s   (WPNAV_SPEED configured: 3.0 in the default SITL file)


def sqrt_controller(err: np.ndarray, k: float, a_lim: float) -> np.ndarray:
    """ArduPilot's sqrt_controller on a 2D error vector: linear near the target, constant-deceleration far from it."""
    mag = float(np.linalg.norm(err))
    if mag < 1e-9:
        return np.zeros_like(err)
    lin = a_lim / (k * k)
    speed = k * mag if mag <= lin else np.sqrt(2.0 * a_lim * (mag - lin / 2.0))
    return err / mag * speed


class AutopilotTrackingModel:
    """One drone's setpoint tracking. Call step(dt, setpoint) repeatedly; read .position / .velocity."""

    def __init__(self, position, params: AutopilotTrackingParams = AutopilotTrackingParams(), dt_internal: float = 0.01):
        self.p = np.asarray(position, dtype=float).copy()
        self.v = np.zeros_like(self.p)
        self.a = np.zeros_like(self.p)
        self.params = params
        self.dt_internal = dt_internal
        self.t = 0.0
        self._history: Deque[Tuple[float, np.ndarray]] = deque()  # (time, setpoint), oldest first
        self._initial_setpoint = self.p.copy()

    @property
    def position(self) -> np.ndarray:
        return self.p

    @property
    def velocity(self) -> np.ndarray:
        return self.v

    def _delayed_setpoint(self) -> np.ndarray:
        """Setpoint that was commanded td seconds ago (zero-order hold; before any command, the initial position)."""
        t_query = self.t - self.params.td
        sp = self._initial_setpoint
        for ts, s in self._history:
            if ts <= t_query:
                sp = s
            else:
                break
        while len(self._history) > 1 and self._history[1][0] <= t_query:
            self._history.popleft()
        return sp

    def step(self, dt: float, setpoint) -> None:
        sp = np.asarray(setpoint, dtype=float)
        self._history.append((self.t, sp.copy()))
        prm = self.params
        n = max(1, int(round(dt / self.dt_internal)))
        h = dt / n
        for _ in range(n):
            r = self._delayed_setpoint()
            v_des = sqrt_controller(r - self.p, prm.k_pos, prm.a_max)
            speed = np.linalg.norm(v_des)
            if speed > prm.v_max:
                v_des = v_des / speed * prm.v_max
            a_cmd = prm.kv * (v_des - self.v)
            amag = np.linalg.norm(a_cmd)
            if amag > prm.a_max:
                a_cmd = a_cmd / amag * prm.a_max
            self.a = self.a + h * (a_cmd - self.a) / prm.tau_a
            self.v = self.v + h * self.a
            self.p = self.p + h * self.v
            self.t += h
