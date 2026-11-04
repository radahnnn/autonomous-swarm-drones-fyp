"""Summarise repeated runs: `python3 -m tools.campaign_report runs/independent/campaign-XXXX` -> table per scenario.

Each run folder (from `sitl.fleet_ctl`) holds driver.log (scenario steps) and evidence/*/run_report.json. Reports the
per-scenario run count, the strict grade (`sitl.run_grade`: whole fleet, milestones, normal landing, truth coverage),
the live run-report verdicts, scenario completion (every wait/expect/poll step OK), and the
distribution of the estimated and true (simulator) minimum gaps."""
from __future__ import annotations

import json
import re
import statistics
import sys
from collections import defaultdict
from pathlib import Path

from sitl.run_grade import grade_run


def summarise(root: Path) -> str:
    groups = defaultdict(list)
    for run in sorted(p for p in root.iterdir() if p.is_dir()):
        reports = list(run.glob("evidence/*/run_report.json"))
        if not reports:
            continue
        report = json.loads(reports[0].read_text())
        driver = (run / "driver.log").read_text() if (run / "driver.log").exists() else ""
        steps_ok = "TIMEOUT" not in driver and "unexpectedly_dead=[]" in driver
        report["_grade"] = grade_run(run)
        groups[re.sub(r"-\d+$", "", run.name)].append((report, steps_ok))
    lines = [f"# Campaign summary: `{root}`", "",
             "| Scenario | Runs | Strict grade PASS / FAIL / INCOMPLETE | Live report PASS / FAIL / INCOMPLETE | Steps all completed | Est. min gap median (worst) | True min gap median (worst) |",
             "|---|---|---|---|---|---|---|"]
    for name, rows in sorted(groups.items()):
        verdicts = [r["result"] for r, _ in rows]
        grades = [r["_grade"]["verdict"] for r, _ in rows]
        est = [r["min_gap_m"] for r, _ in rows if r.get("min_gap_m") is not None]
        true = [r["true_min_gap_m"] for r, _ in rows if r.get("true_min_gap_m") is not None]
        def fmt(values):
            return f"{statistics.median(values):.2f} ({min(values):.2f})" if values else "n/a"
        lines.append(f"| {name} | {len(rows)} | {grades.count('PASS')} / {grades.count('FAIL')} / "
                     f"{grades.count('INCOMPLETE')} | {verdicts.count('PASS')} / {verdicts.count('FAIL')} / "
                     f"{verdicts.count('INCOMPLETE')} | {sum(ok for _, ok in rows)}/{len(rows)} | {fmt(est)} | {fmt(true)} |")
    lines += ["", "## Runs that are not a strict PASS", ""]
    for name, rows in sorted(groups.items()):
        for r, _ in rows:
            g = r["_grade"]
            if g["verdict"] != "PASS":
                lines.append(f"- `{Path(g['run']).name}` {g['verdict']}: " + "; ".join(g["reasons"]))
    return "\n".join(lines) + "\n"


if __name__ == "__main__":
    print(summarise(Path(sys.argv[1])))
