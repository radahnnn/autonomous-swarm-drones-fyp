"""Summarize repeated three-mode numerical trials, including failures and clearance margins."""

import argparse
import json
from math import sqrt
from pathlib import Path
from statistics import mean


def wilson_interval(successes: int, total: int, z: float = 1.96):
    if total <= 0:
        raise ValueError("at least one trial is required")
    p = successes / total
    denominator = 1 + z * z / total
    center = (p + z * z / (2 * total)) / denominator
    half = z * sqrt(p * (1 - p) / total + z * z / (4 * total * total)) / denominator
    return [center - half, center + half]


def summarize(report, advisory_buffer_m: float = 0.25):
    min_sep = float(report["criteria"]["minimum_separation_m"])
    max_error = float(report["criteria"]["steady_p95_error_m"])
    result = {"scope": report["scope"], "source_revision": report["reproducibility"]["revision"],
              "criteria": report["criteria"], "advisory_clearance_buffer_m": advisory_buffer_m,
              "groups": {}}
    for scenario in sorted({r["scenario"] for r in report["results"]}):
        result["groups"][scenario] = {}
        for mode in sorted({r["mode"] for r in report["results"]}):
            rows = [r for r in report["results"] if r["scenario"] == scenario and r["mode"] == mode]
            if not rows:
                continue
            passing = sum(r["pass_criteria"] for r in rows)
            separations = [float(r["minimum_separation_m"]) for r in rows]
            errors = [float(r["steady_p95_error_m"]) for r in rows]
            result["groups"][scenario][mode] = {
                "n": len(rows), "passes": passing,
                "pass_rate_wilson95": wilson_interval(passing, len(rows)),
                "mean_minimum_separation_m": mean(separations),
                "worst_minimum_separation_m": min(separations),
                "worst_clearance_margin_m": min(separations) - min_sep,
                "advisory_buffer_shortfalls": sum(s < min_sep + advisory_buffer_m for s in separations),
                "mean_steady_p95_error_m": mean(errors),
                "failures": [{"seed": r["seed"],
                              "separation_m": r["minimum_separation_m"],
                              "steady_p95_error_m": r["steady_p95_error_m"],
                              "failed_gates": (["separation"] if r["minimum_separation_m"] < min_sep else []) +
                                              (["tracking"] if r["steady_p95_error_m"] > max_error else [])}
                             for r in rows if not r["pass_criteria"]],
            }
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    result = summarize(json.loads(args.input.read_text()))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2) + "\n")
    for scenario, groups in result["groups"].items():
        print(scenario)
        for mode, row in groups.items():
            print(f"  {mode}: {row['passes']}/{row['n']} passed; worst margin {row['worst_clearance_margin_m']:.3f} m; "
                  f"buffer shortfalls {row['advisory_buffer_shortfalls']}")


if __name__ == "__main__":
    main()
