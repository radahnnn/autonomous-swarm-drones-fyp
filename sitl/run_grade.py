"""Strict run grade (user decision 2026-11-04, Codex review point 4): the live run report judges separation and evidence
coverage only; this grade also requires that the mission itself succeeded for the whole fleet.

PASS only if all of these hold:
- the live run report (evidence/*/run_report.json) is PASS (no separation violation, estimated or true; no ground
  contact; status coverage complete);
- every scenario step (wait/expect/poll) in driver.log succeeded and no agent died unexpectedly (the mission milestones);
- every configured drone flew, and at the end every drone is landed and disarmed;
- no drone was landed by anything other than the operator's LAND (e.g. the flyaway/estimate guard, a geofence breach,
  coordinator loss), unless the scenario explicitly allows that reason;
- true (simulator) positions cover at least TRUTH_COVERAGE of every drone's airborne status samples.

Verdict order FAIL > INCOMPLETE > PASS: a separation failure stays FAIL; everything else that is not a clean mission is
INCOMPLETE ("not PASS"), with the reasons listed. Usage: `python3 -m sitl.run_grade <run_dir>`.
"""
from __future__ import annotations

import json
import sys
from pathlib import Path
from typing import Dict, Iterable, List

TRUTH_COVERAGE = 0.95


def _events(path: Path) -> Iterable[dict]:
    with path.open() as handle:
        for line in handle:
            try:
                yield json.loads(line)
            except json.JSONDecodeError:
                continue


def grade_run(run_dir: Path, allowed_land_reasons: Iterable[str] = ()) -> Dict:
    run_dir = Path(run_dir)
    allowed = {"operator", *allowed_land_reasons}
    fail: List[str] = []
    incomplete: List[str] = []
    reports = sorted(run_dir.glob("evidence/*/run_report.json"))
    if not reports:
        return {"verdict": "NO DATA", "reasons": ["no run_report.json"], "run": str(run_dir)}
    report = json.loads(reports[0].read_text())
    live = report.get("result")
    if live == "FAIL":
        fail.append(f"live report FAIL (estimated min {report.get('min_gap_m')} m, true min "
                    f"{report.get('true_min_gap_m')} m, ground contacts {report.get('ground_contacts_while_armed')})")
    elif live != "PASS":
        incomplete.append(f"live report {live}")
    count = int(report.get("drones") or 0)

    driver = run_dir / "driver.log"
    if not driver.exists():
        incomplete.append("no driver.log: mission milestones not recorded")
    else:
        text = driver.read_text()
        timeouts = [line.split("expect ", 1)[-1].split(":")[0] if "expect" in line else line.strip()
                    for line in text.splitlines() if "TIMEOUT" in line]
        if timeouts:
            incomplete.append("milestones not reached: " + "; ".join(timeouts))
        if "unexpectedly_dead=[]" not in text:
            incomplete.append("agent liveness not confirmed (health check missing or an agent died)")
        if "driver done; failed=True" in text and not timeouts:
            incomplete.append("scenario driver reported a failed step")

    last: Dict[int, dict] = {}
    flew, airborne_samples, truth_samples = set(), {}, {}
    abnormal: Dict[int, str] = {}
    operator_landed = set()
    events_files = sorted(run_dir.glob("evidence/*/events.jsonl"))
    for event in (_events(events_files[0]) if events_files else ()):
        if event.get("kind") != "agent_status":
            continue
        drone = int(event.get("drone_id", -1))
        last[drone] = event
        command = str(event.get("command") or "")
        if command.startswith("landing:"):
            reason = command.split(":", 1)[1]
            # Only a landing that interrupts flight counts: after the operator's LAND, or once the drone is down and
            # disarmed, a later reason (e.g. telemetry_timeout at shutdown) changes nothing about the mission.
            if reason == "operator":
                operator_landed.add(drone)
            elif (reason not in allowed and drone not in abnormal and drone not in operator_landed
                  and (event.get("airborne") or event.get("armed"))):
                abnormal[drone] = reason
        if event.get("airborne"):
            flew.add(drone)
            airborne_samples[drone] = airborne_samples.get(drone, 0) + 1
            if event.get("true_position_ne") is not None:
                truth_samples[drone] = truth_samples.get(drone, 0) + 1
    if not events_files:
        incomplete.append("no events.jsonl")
    for drone, reason in sorted(abnormal.items()):
        incomplete.append(f"D{drone + 1} landed by {reason}, not by the operator")
    missing = [d + 1 for d in range(count) if d not in flew]
    if missing:
        incomplete.append(f"drones that never flew: {missing}")
    not_down = [d + 1 for d in range(count)
                if d not in last or last[d].get("armed") is not False or last[d].get("airborne") is not False]
    if not_down:
        incomplete.append(f"not landed and disarmed at the end: {not_down}")
    coverage = {d + 1: round(truth_samples.get(d, 0) / airborne_samples[d], 3) for d in sorted(airborne_samples)}
    low = {d: c for d, c in coverage.items() if c < TRUTH_COVERAGE}
    if low:
        incomplete.append(f"true-position coverage below {TRUTH_COVERAGE:.0%}: {low}")

    verdict = "FAIL" if fail else "INCOMPLETE" if incomplete else "PASS"
    return {"verdict": verdict, "reasons": fail + incomplete, "run": str(run_dir), "live_result": live,
            "drones": count, "min_gap_m": report.get("min_gap_m"), "true_min_gap_m": report.get("true_min_gap_m"),
            "true_coverage_min": min(coverage.values()) if coverage else None}


def write_grade(run_dir: Path, **kwargs) -> Dict:
    grade = grade_run(run_dir, **kwargs)
    (Path(run_dir) / "GRADE.json").write_text(json.dumps(grade, indent=2) + "\n")
    return grade


if __name__ == "__main__":
    for arg in sys.argv[1:]:
        g = write_grade(Path(arg))
        print(f"{g['verdict']:10s} {arg}" + "".join(f"\n    - {r}" for r in g["reasons"]))
