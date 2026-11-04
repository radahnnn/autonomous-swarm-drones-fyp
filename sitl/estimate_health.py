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
    # Allowed lag behind a moving target: flyaway_m + LAG_ALLOWANCE_S x target speed (smoothed over ~SPEED_TAU_S).
    # Codex review 2026-11-04: the earlier rule skipped the check whenever the target moved > 0.5 m/s, so a real
    # flyaway during a move went unnoticed; a fixed 3 m on the other hand flagged a drone that was merely lagging a
    # 0.45 m/s target (tower-gap run, 2026-11-04 campaign d) and the freeze that followed squeezed the column.
    LAG_ALLOWANCE_S = 4.0
    SPEED_TAU_S = 2.0

    def __init__(self, max_speed_m_s: float = 4.0, recover_s: float = 5.0, land_after_s: float = 15.0,
                 min_dt_s: float = 0.5, flyaway_m: float = 3.0, flyaway_s: float = 1.0, flyaway_land_m: float = 10.0):
        if not (max_speed_m_s > 0 and recover_s > 0 and land_after_s > recover_s and min_dt_s > 0):
            raise ValueError("estimate-health thresholds must be positive and land_after_s > recover_s")
        self.max_speed_m_s, self.recover_s, self.land_after_s, self.min_dt_s = max_speed_m_s, recover_s, land_after_s, min_dt_s
        self.state = self.OK
        self.glitches = 0
        self._last: Optional[tuple] = None
        self._last_glitch_at: Optional[float] = None
        self._degraded_since: Optional[float] = None
        # 3 m / 1 s (was 4 m / 2 s): a flyaway at ~4 m/s covered 8 m before the old check fired (campaign 2026-11-04).
        # Flyaway: far from the commanded target for a sustained time (live finding 2026-11-04: under combined
        # realistic disturbances two drones left steady targets by 7 m and 18 m while the swarm logic was correct).
        self.flyaway_m, self.flyaway_s, self.flyaway_land_m = flyaway_m, flyaway_s, flyaway_land_m
        self._far_since: Optional[float] = None
        self._last_target: Optional[tuple] = None
        self._target_speed = 0.0
        self.flyaways = 0
        self.estimator_upsets = 0

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

    def check_tracking(self, position_ne: Sequence[float], target_ne: Optional[Sequence[float]], t: float) -> str:
        """Flag a drone that stays far from its commanded target (flyaway). Sustained > flyaway_m (+ the lag allowance
        for a moving target) for flyaway_s -> DEGRADED (peers keep extra distance); > flyaway_land_m (+ allowance)
        -> LAND (like an EKF failsafe)."""
        if target_ne is None or not all(math.isfinite(float(v)) for v in (*position_ne[:2], *target_ne[:2], t)):
            return self.state
        error = math.hypot(float(position_ne[0]) - float(target_ne[0]), float(position_ne[1]) - float(target_ne[1]))
        if self._last_target is not None and t - self._last_target[2] > 1e-3:
            dt = t - self._last_target[2]
            speed = math.hypot(float(target_ne[0]) - self._last_target[0],
                               float(target_ne[1]) - self._last_target[1]) / dt
            weight = min(1.0, dt / self.SPEED_TAU_S)
            self._target_speed += weight * (speed - self._target_speed)
        self._last_target = (float(target_ne[0]), float(target_ne[1]), t)
        allowance = self.LAG_ALLOWANCE_S * self._target_speed
        if error <= self.flyaway_m + allowance:
            self._far_since = None
            return self.state
        if self._far_since is None:
            self._far_since = t
            return self.state
        if t - self._far_since >= self.flyaway_s:
            if error > self.flyaway_land_m + allowance:
                self.state = self.LAND
            elif self.state == self.OK:
                self.flyaways += 1
                self.state, self._degraded_since = self.DEGRADED, t
            self._last_glitch_at = t                      # keep it degraded while it stays far away
        return self.state

    def note_estimator_upset(self, t: float) -> str:
        """The autopilot reported an EKF emergency yaw reset or lane switch (user-approved item 7c, 2026-11-04): in the
        realistic runs these preceded every flyaway, ~1 s before the tracking check could fire. Degrade at once (peers
        keep extra distance, the target is held) and recover after recover_s without a further upset."""
        if not math.isfinite(t):
            return self.state
        self.estimator_upsets += 1
        self._last_glitch_at = t
        if self.state == self.OK:
            self.state, self._degraded_since = self.DEGRADED, t
        return self.state

    @property
    def degraded(self) -> bool:
        return self.state != self.OK

    def status(self) -> dict:
        return {"state": self.state, "glitches": self.glitches, "flyaways": self.flyaways,
                "estimator_upsets": self.estimator_upsets}
