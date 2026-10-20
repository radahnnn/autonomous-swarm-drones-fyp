#!/usr/bin/env python3
"""Coarse parameter sweep for decentralized 5-drone spacing/tracking tradeoffs."""

import argparse
import json
from pathlib import Path

import numpy as np

from compare_5drone_architectures import run_trial


CANDIDATES = [
    {"spacing": 2.5, "safe_radius": 1.5, "k_sep": 4.0, "k_form": 1.4},
    {"spacing": 3.0, "safe_radius": 1.5, "k_sep": 4.0, "k_form": 1.4},
    {"spacing": 3.5, "safe_radius": 1.5, "k_sep": 4.0, "k_form": 1.4},
    {"spacing": 4.0, "safe_radius": 1.5, "k_sep": 4.0, "k_form": 1.4},
    {"spacing": 3.5, "safe_radius": 3.0, "k_sep": 20.0, "k_form": 1.4},
    {"spacing": 4.0, "safe_radius": 3.0, "k_sep": 20.0, "k_form": 1.4},
]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--trials", type=int, default=8)
    parser.add_argument("--seed", type=int, default=20261020)
    parser.add_argument("--output", type=Path, default=None)
    args = parser.parse_args()
    if args.trials < 2:
        parser.error("--trials must be at least 2")

    results = []
    for params in CANDIDATES:
        controller_params = {key: value for key, value in params.items() if key != "spacing"}
        rows = [run_trial("decentralized", args.seed + i, controller_params,
                          formation_spacing=params["spacing"]) for i in range(args.trials)]
        spacing = np.asarray([row["minimum_separation_m"] for row in rows])
        error = np.asarray([row["steady_mean_error_m"] for row in rows])
        result = {
            "parameters": params,
            "trials": len(rows),
            "spacing_mean_m": float(np.mean(spacing)),
            "spacing_worst_m": float(np.min(spacing)),
            "trials_meeting_2_5m": int(np.sum(spacing >= 2.5)),
            "steady_error_mean_m": float(np.mean(error)),
            "steady_error_p95_across_trials_m": float(np.percentile(error, 95)),
            "collision_trials": int(sum(row["any_collision"] for row in rows)),
            "per_trial": rows,
        }
        results.append(result)
        print(f"{params}: min-sep={result['spacing_worst_m']:.3f}m, "
              f"mean-sep={result['spacing_mean_m']:.3f}m, "
              f"mean-steady-error={result['steady_error_mean_m']:.3f}m, "
              f"spacing-passes={result['trials_meeting_2_5m']}/{len(rows)}")

    results.sort(key=lambda row: (
        -row["trials_meeting_2_5m"], row["steady_error_mean_m"], -row["spacing_worst_m"]
    ))
    report = {
        "purpose": "coarse tuning only; the selected candidate requires full paired validation",
        "same_conditions_as": "compare_5drone_architectures.py",
        "trials_per_candidate": args.trials,
        "seeds": [args.seed + i for i in range(args.trials)],
        "ranking": "most trials meeting 2.5m first, then lowest mean steady tracking error",
        "results": results,
    }
    encoded = json.dumps(report, indent=2) + "\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(encoded, encoding="utf-8")
        print(f"Saved sweep: {args.output}")
    else:
        print(encoded)


if __name__ == "__main__":
    main()
