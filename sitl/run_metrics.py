"""Live run metrics and an end-of-run pass/fail report (pure logic, no I/O except report writing).

Fed with the agents' latest status messages. Separation is computed from the 1 Hz status positions,
so it is a *sampled* value: it can miss a dip between two samples and must never be described as a
continuous-time guarantee. The gate defaults to horizontal distance (the project's acceptance
definition); the 3-D gap is reported next to it because altitude layering only helps the latter.
"""
from __future__ import annotations

import json
import math
from collections import deque
from pathlib import Path
from typing import Any, Deque, Dict, List, Optional, Tuple

import numpy as np

from sitl.separation_geometry import horizontal_equivalent_gap

GATE_M = 2.5


class RunMetrics:
    def __init__(self, count: int, gate_m: float = GATE_M, warn_margin_m: float = 0.25,
                 history_s: float = 300.0, sample_period_s: float = 0.5,
                 gate_metric: str = "horizontal", max_status_age_s: float = 3.0,
                 vertical_radius_m: Optional[float] = None):
        if gate_metric not in ("horizontal", "3d", "ellipsoid"):
            raise ValueError("gate_metric must be 'horizontal', '3d' or 'ellipsoid'")
        if gate_metric == "ellipsoid" and not (vertical_radius_m and math.isfinite(vertical_radius_m) and vertical_radius_m > 0):
            raise ValueError("the ellipsoid gate needs a positive vertical radius")
        self.vertical_radius_m = vertical_radius_m
        self.count, self.gate_m, self.warn_margin_m = count, float(gate_m), float(warn_margin_m)
        self.gate_metric, self.max_status_age_s = gate_metric, max_status_age_s
        self.sample_period_s = sample_period_s
        self.history: Deque[Tuple[float, float]] = deque(maxlen=max(10, int(history_s / sample_period_s)))
        self.started_at: Optional[float] = None
        self.last_sample_at = -math.inf
        self.samples = 0
        self.min_gap_m = math.inf
        self.min_gap_3d_m = math.inf
        self.min_gap_at: Optional[float] = None
        self.gaps: List[float] = []
        self.violations = 0
        self.violation_time_s = 0.0
        self.warnings = 0
        self._state = "n/a"
        self._last_t: Optional[float] = None
        self.current: Dict[str, Any] = {"gap_m": None, "gap_3d_m": None, "pair": None}
        self.drones: List[Dict[str, Any]] = []
        self.tracking_errors: List[float] = []
        self.max_link_age_s = 0.0
        self.counters: Dict[str, int] = {}
        self.event_log: List[Dict[str, Any]] = []

    # ------------------------------------------------------------------ input
    def note_event(self, kind: str, now: float, **data: Any) -> None:
        self.counters[kind] = self.counters.get(kind, 0) + 1
        if len(self.event_log) < 500:
            self.event_log.append({"t": round(now - (self.started_at or now), 2), "kind": kind, **data})

    def update(self, now: float, statuses: Dict[int, Dict[str, Any]]) -> None:
        if self.started_at is None:
            self.started_at = now
        if now - self.last_sample_at < self.sample_period_s:
            return
        self.last_sample_at = now
        points: Dict[int, np.ndarray] = {}
        drones: List[Dict[str, Any]] = []
        for index in range(self.count):
            status = statuses.get(index)
            info: Dict[str, Any] = {"id": index + 1, "mode": None, "link_age_s": None,
                                    "tracking_error_m": None, "airborne": False,
                                    "coordinator_connected": None}
            if status:
                received = status.get("operator_received_at", status.get("timestamp"))
                try:
                    age = now - float(received)
                except (TypeError, ValueError):
                    age = None
                info["link_age_s"] = None if age is None else round(age, 2)
                info["mode"] = (status.get("mode_switch") or {}).get("mode") or status.get("control_mode")
                info["coordinator_connected"] = status.get("coordinator_connected")
                info["airborne"] = bool(status.get("airborne"))
                position = status.get("position")
                fresh = age is not None and 0.0 <= age <= self.max_status_age_s
                if fresh:
                    self.max_link_age_s = max(self.max_link_age_s, age)
                if (fresh and isinstance(position, (list, tuple)) and len(position) == 3 and
                        all(isinstance(v, (int, float)) and math.isfinite(v) for v in position)):
                    if info["airborne"]:
                        points[index] = np.asarray(position, dtype=float)
                    target = status.get("position_target_ne")
                    if (info["airborne"] and isinstance(target, (list, tuple)) and len(target) == 2):
                        try:
                            error = float(np.linalg.norm(np.asarray(position[:2]) - np.asarray(target, dtype=float)))
                            if math.isfinite(error):
                                info["tracking_error_m"] = round(error, 2)
                                self.tracking_errors.append(error)
                        except (TypeError, ValueError):
                            pass
            drones.append(info)
        self.drones = drones
        gap = gap3d = gap_shape = None
        pair = None
        ids = sorted(points)
        for a in range(len(ids)):
            for b in range(a + 1, len(ids)):
                delta = points[ids[a]] - points[ids[b]]
                horizontal = float(np.linalg.norm(delta[:2]))
                spatial = float(np.linalg.norm(delta))
                if gap is None or horizontal < gap:
                    gap, pair = horizontal, (ids[a] + 1, ids[b] + 1)
                gap3d = spatial if gap3d is None else min(gap3d, spatial)
                if self.gate_metric == "ellipsoid":
                    shaped = horizontal_equivalent_gap(horizontal, abs(float(delta[2])), self.gate_m, self.vertical_radius_m)
                    gap_shape = shaped if gap_shape is None else min(gap_shape, shaped)
        self.current = {"gap_m": gap, "gap_3d_m": gap3d, "pair": pair}
        if gap is None:
            self._state, self._last_t = "n/a", now
            return
        if self.gate_metric == "horizontal":
            measured = gap
        elif self.gate_metric == "3d":
            measured = gap3d if gap3d is not None else gap
        else:
            measured = gap_shape if gap_shape is not None else gap
        self.samples += 1
        self.gaps.append(measured)
        self.history.append((now - self.started_at, measured))
        if measured < self.min_gap_m:
            self.min_gap_m, self.min_gap_at = measured, now - self.started_at
        if gap3d is not None:
            self.min_gap_3d_m = min(self.min_gap_3d_m, gap3d)
        state = ("violation" if measured < self.gate_m else
                 "warn" if measured < self.gate_m + self.warn_margin_m else "ok")
        if state == "violation":
            if self._state != "violation":
                self.violations += 1
            elif self._last_t is not None:
                self.violation_time_s += max(0.0, now - self._last_t)   # only between violating samples
        elif state == "warn" and self._state not in ("warn", "violation"):
            self.warnings += 1
        self._state, self._last_t = state, now

    # ----------------------------------------------------------------- output
    def snapshot(self, max_points: int = 120) -> Dict[str, Any]:
        history = list(self.history)
        if len(history) > max_points:
            step = len(history) / max_points
            history = [history[int(i * step)] for i in range(max_points)]
        gap = self.current["gap_m"]
        return {
            "gate_m": self.gate_m, "gate_metric": self.gate_metric, "state": self._state,
            "gap_m": None if gap is None else round(gap, 2),
            "gap_3d_m": None if self.current["gap_3d_m"] is None else round(self.current["gap_3d_m"], 2),
            "margin_m": None if gap is None else round(gap - self.gate_m, 2),
            "pair": self.current["pair"],
            "min_gap_m": None if not math.isfinite(self.min_gap_m) else round(self.min_gap_m, 2),
            "violations": self.violations, "violation_time_s": round(self.violation_time_s, 1),
            "warnings": self.warnings, "samples": self.samples,
            "history": [[round(t, 1), round(g, 2)] for t, g in history],
            "drones": self.drones, "counters": dict(self.counters),
            "max_link_age_s": round(self.max_link_age_s, 2),
        }

    def report(self, ended_at: Optional[float] = None) -> Dict[str, Any]:
        gaps = np.asarray(self.gaps, dtype=float)
        duration = None if self.started_at is None or ended_at is None else round(ended_at - self.started_at, 1)
        has_data = self.samples > 0
        passed = bool(has_data and self.violations == 0)
        tracking = np.asarray(self.tracking_errors, dtype=float)
        return {
            "result": "PASS" if passed else ("NO DATA" if not has_data else "FAIL"),
            "gate_m": self.gate_m, "gate_metric": self.gate_metric, "vertical_radius_m": self.vertical_radius_m,
            "sampled_note": "separation is sampled from ~1 Hz status positions, not continuous",
            "duration_s": duration, "drones": self.count, "airborne_pair_samples": self.samples,
            "min_gap_m": None if not has_data else round(float(gaps.min()), 3),
            "min_gap_at_s": None if self.min_gap_at is None else round(self.min_gap_at, 1),
            "min_gap_3d_m": None if not math.isfinite(self.min_gap_3d_m) else round(self.min_gap_3d_m, 3),
            "p05_gap_m": None if not has_data else round(float(np.percentile(gaps, 5)), 3),
            "violation_episodes": self.violations, "violation_time_s": round(self.violation_time_s, 1),
            "warning_episodes": self.warnings, "warn_margin_m": self.warn_margin_m,
            "mean_tracking_error_m": None if tracking.size == 0 else round(float(tracking.mean()), 3),
            "p95_tracking_error_m": None if tracking.size == 0 else round(float(np.percentile(tracking, 95)), 3),
            "max_status_age_s": round(self.max_link_age_s, 2),
            "counters": dict(self.counters), "events": list(self.event_log),
        }

    def report_markdown(self, ended_at: Optional[float] = None) -> str:
        r = self.report(ended_at)
        lines = [f"# Run report — {r['result']}", "",
                 f"- Gate: {r['gate_m']} m ({r['gate_metric']} separation, **sampled**)",
                 f"- Duration: {r['duration_s']} s, drones: {r['drones']}",
                 f"- Minimum gap: {r['min_gap_m']} m at t={r['min_gap_at_s']} s (3-D minimum {r['min_gap_3d_m']} m, 5th percentile {r['p05_gap_m']} m)",
                 f"- Violation episodes: {r['violation_episodes']} ({r['violation_time_s']} s); warning episodes (< gate + {r['warn_margin_m']} m): {r['warning_episodes']}",
                 f"- Tracking error: mean {r['mean_tracking_error_m']} m, p95 {r['p95_tracking_error_m']} m",
                 f"- Oldest status seen: {r['max_status_age_s']} s",
                 f"- Events: {r['counters'] or 'none'}", ""]
        if r["events"]:
            lines += ["## Event log", ""] + [f"- t={e['t']} s `{e['kind']}`" for e in r["events"][:60]]
        return "\n".join(lines) + "\n"

    def write_report(self, directory: Path, ended_at: Optional[float] = None) -> Tuple[Path, Path]:
        directory = Path(directory)
        directory.mkdir(parents=True, exist_ok=True)
        json_path, md_path = directory / "run_report.json", directory / "RUN_REPORT.md"
        json_path.write_text(json.dumps(self.report(ended_at), indent=2) + "\n")
        md_path.write_text(self.report_markdown(ended_at))
        return json_path, md_path
