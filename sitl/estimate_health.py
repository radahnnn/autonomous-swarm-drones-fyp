"""Position-estimate health: detect glitches in a drone's own navigation estimate (pure logic).

Live finding (2026-11-03, realistic SITL): under GPS error plus vibration one drone's reported position jumped ~10 m/s
while its target stayed put. A swarm cannot trust such a drone: its own commands and everyone's separation checks use the
bad estimate. The check is deliberately simple and conservative: the speed implied by two consecutive position samples
must stay below ``max_speed_m_s`` (our flights command < 1 m/s; a multirotor in GUIDED does not do 4 m/s here),
judged over at least ``min_dt_s`` (0.5 s): over 0.05 s, centimetre jitter looked like >4 m/s and raised 20 false alarms
in a live 10-drone run, while real jumps in the estimate were only 0-2 per run.

States: ``ok`` -> ``degraded`` on a glitch (the agent freezes its target and peers keep extra distance) -> back to ``ok``
after ``recover_s`` without a glitch, or ``land`` when glitches keep coming for ``land_after_s``.
"""
from __future__ import annotations

import math
from typing import Optional, Sequence


class EstimateHealth:
    OK, DEGRADED, LAND = "ok", "degraded", "land"

    def __init__(self, max_speed_m_s: float = 4.0, recover_s: float = 5.0, land_after_s: float = 15.0,
                 min_dt_s: float = 0.5):
        if not (max_speed_m_s > 0 and recover_s > 0 and land_after_s > recover_s and min_dt_s > 0):
            raise ValueError("estimate-health thresholds must be positive and land_after_s > recover_s")
        self.max_speed_m_s, self.recover_s, self.land_after_s, self.min_dt_s = max_speed_m_s, recover_s, land_after_s, min_dt_s
        self.state = self.OK
        self.glitches = 0
        self._last: Optional[tuple] = None
        self._last_glitch_at: Optional[float] = None
        self._degraded_since: Optional[float] = None

    def update(self, position_ne: Sequence[float], t: float) -> str:
        north, east = float(position_ne[0]), float(position_ne[1])
        if not (math.isfinite(north) and math.isfinite(east) and math.isfinite(t)):
            return self.state
        if self._last is not None:
            dt = t - self._last[2]
            if dt >= self.min_dt_s:
                speed = math.hypot(north - self._last[0], east - self._last[1]) / dt
                if speed > self.max_speed_m_s:
                    self.glitches += 1
                    self._last_glitch_at = t
                    if self.state == self.OK:
                        self.state, self._degraded_since = self.DEGRADED, t
            elif dt < 0:
                self._last = (north, east, t)          # clock went backwards: restart the comparison
                return self.state
            else:
                return self.state                       # too close in time to judge; keep the older sample
        self._last = (north, east, t)
        if self.state == self.DEGRADED:
            if self._last_glitch_at is not None and t - self._last_glitch_at >= self.recover_s:
                self.state, self._degraded_since = self.OK, None
            elif self._degraded_since is not None and t - self._degraded_since >= self.land_after_s:
                self.state = self.LAND
        return self.state

    @property
    def degraded(self) -> bool:
        return self.state != self.OK

    def status(self) -> dict:
        return {"state": self.state, "glitches": self.glitches}
