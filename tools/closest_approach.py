"""Closest-approach timeline and cause label for a run: `python3 -m tools.closest_approach <run_dir> [--timeline]`.

Codex review 2026-11-04: investigate each near-miss separately, from the time-aligned truth, estimate, target and
safety state of the two drones involved. For the closest true approach of a run this prints (with --timeline) one line
per status sample of both drones from WINDOW_S before to 3 s after, and a cause label:

- clear: the closest true approach stayed above the gate;
- autopilot_flyaway: either drone reported an autopilot EKF upset (yaw reset / lane switch) in the window, or was
  far (> FLYAWAY_M) from its own commanded target while that target was nearly still;
- estimate_error: the estimated positions were clear (> gate) while the true ones were not;
- controller: neither of the above (the commanded targets themselves brought the drones together).
"""
from __future__ import annotations

import datetime as dt
import itertools
import json
import sys
from pathlib import Path

import numpy as np

WINDOW_S = 20.0
FLYAWAY_M = 3.0
GATE_M = 2.5


def _load(run_dir: Path):
    path = sorted(Path(run_dir).glob("evidence/*/events.jsonl"))[0]
    rows, first = [], None
    for line in path.open():
        e = json.loads(line)
        if e.get("kind") != "agent_status":
            continue
        t = dt.datetime.fromisoformat(e["utc"])
        first = first or t
        e["_t"] = (t - first).total_seconds()
        rows.append(e)
    return rows


def closest(rows):
    latest, worst = {}, None
    for e in rows:
        if not (e.get("airborne") and e.get("true_position_ne")):
            latest.pop(e["drone_id"], None)
            continue
        latest[e["drone_id"]] = e
        if len(latest) < 2:
            continue
        for a, b in itertools.combinations(sorted(latest), 2):
            if abs(latest[a]["_t"] - latest[b]["_t"]) > 1.0:
                continue
            gap = float(np.linalg.norm(np.subtract(latest[a]["true_position_ne"], latest[b]["true_position_ne"])))
            if worst is None or gap < worst[0]:
                worst = (gap, e["_t"], a, b)
    return worst


def analyse(run_dir: Path, timeline: bool = False):
    rows = _load(run_dir)
    worst = closest(rows)
    if worst is None:
        return {"run": str(run_dir), "label": "no_data"}
    gap, t0, a, b = worst
    window = [e for e in rows if e["drone_id"] in (a, b) and t0 - WINDOW_S <= e["_t"] <= t0 + 3.0]
    upset = far_from_still_target = False
    before = {}
    lines = []
    est_at, true_at = {}, {}
    for e in window:
        d = e["drone_id"]
        health = e.get("estimate_health") or {}
        count = health.get("estimator_upsets", 0)
        if d in before and count > before[d]:
            upset = True
        before[d] = count
        target, pos, truth = e.get("position_target_ne"), e.get("position"), e.get("true_position_ne")
        if target and truth and pos:
            error = float(np.linalg.norm(np.subtract(truth, target)))
            if error > FLYAWAY_M and e["_t"] <= t0 + 0.5:
                prev = [r for r in window if r["drone_id"] == d and r["_t"] < e["_t"] and r.get("position_target_ne")]
                if prev and float(np.linalg.norm(np.subtract(target, prev[-1]["position_target_ne"]))) < 0.6:
                    far_from_still_target = True
            if abs(e["_t"] - t0) <= 1.0:
                est_at[d], true_at[d] = np.asarray(pos[:2]), np.asarray(truth)
        fr = e.get("formation_reference") or {}
        sf = e.get("safety_filter") or {}
        lines.append(f"{e['_t'] - t0:6.1f} D{d + 1:<2} true {np.round(truth, 1) if truth else '-'} "
                     f"est {np.round(pos[:2], 1) if pos else '-'} tgt {np.round(target, 1) if target else '-'} "
                     f"mode {e.get('control_mode')} health {health.get('state')} upsets {count} "
                     f"filter {sf.get('active')} corr {fr.get('correction_m')} lead {fr.get('lead_error_m')}")
    est_gap = (float(np.linalg.norm(est_at[a] - est_at[b])) if a in est_at and b in est_at else None)
    if gap >= GATE_M:
        label = "clear"
    elif upset or far_from_still_target:
        label = "autopilot_flyaway"
    elif est_gap is not None and est_gap > GATE_M:
        label = "estimate_error"
    else:
        label = "controller"
    at = min(window, key=lambda e: abs(e["_t"] - t0))
    result = {"run": str(run_dir), "true_gap_m": round(gap, 2), "t_s": round(t0, 1), "pair": [a + 1, b + 1],
              "estimated_gap_m": None if est_gap is None else round(est_gap, 2), "label": label,
              "autopilot_upset": upset, "far_from_still_target": far_from_still_target,
              "mode": at.get("control_mode"), "transition": at.get("formation_transition_active")}
    if timeline:
        result["timeline"] = lines
    return result


if __name__ == "__main__":
    show = "--timeline" in sys.argv
    for arg in (a for a in sys.argv[1:] if not a.startswith("--")):
        r = analyse(Path(arg), timeline=show)
        print(f"{Path(arg).name}: {r['label']} true {r.get('true_gap_m')} m est {r.get('estimated_gap_m')} m "
              f"D{r.get('pair', ['?', '?'])[0]}-D{r.get('pair', ['?', '?'])[1]} at {r.get('t_s')} s mode {r.get('mode')} "
              f"upset {r.get('autopilot_upset')} far {r.get('far_from_still_target')}")
        for line in r.get("timeline", []):
            print("   ", line)
