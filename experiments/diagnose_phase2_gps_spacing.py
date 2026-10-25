"""Isolate GPS position-noise effects in the corrected Phase 2 formation trial."""

import argparse
import json
from pathlib import Path
from statistics import mean

from experiments.test_network_sweep import run_single_trial
from swarm_core.config import set_active_profile


def wilson(successes: int, total: int, z: float = 1.96):
    p = successes / total
    den = 1.0 + z * z / total
    center = (p + z * z / (2.0 * total)) / den
    half = z * ((p * (1.0 - p) / total) + z * z / (4.0 * total * total)) ** 0.5 / den
    return [center - half, center + half]


def run(seeds: int = 20, noise_levels=(0.04, 0.25, 0.5, 1.5),
        predictive_safety: bool = False):
    profile = "sitl_corrected_s1"
    set_active_profile(profile)
    raw = []
    for noise in noise_levels:
        for mode in ("centralized", "decentralized", "hybrid_proposed"):
            for index in range(seeds):
                seed = 100 + 23 * index
                result = run_single_trial(
                    mode=mode, loss_rate=0.0, num_drones=5, seed=seed,
                    initial_layout="formation", gps_noise_std=noise,
                    predictive_safety=predictive_safety,
                )
                raw.append({
                    "profile": profile, "mode": mode, "gps_noise_std_m": noise,
                    "seed": seed,
                    "min_separation_m": result["min_recorded_distance_m"],
                    "final_error_m": result["final_formation_error_m"],
                    "steady_error_m": result["steady_state_error_m"],
                    "steady_p95_slot_error_m": result["steady_p95_slot_error_m"],
                    "safety_infeasible_steps": result["safety_infeasible_steps"],
                    "pass_criteria": bool(result["min_recorded_distance_m"] >= 2.5
                                           and result["steady_p95_slot_error_m"] <= 1.0),
                })
    summary = []
    for noise in noise_levels:
        for mode in ("centralized", "decentralized", "hybrid_proposed"):
            rows = [r for r in raw if r["gps_noise_std_m"] == noise and r["mode"] == mode]
            passed = sum(r["pass_criteria"] for r in rows)
            summary.append({
                "profile": profile, "mode": mode, "gps_noise_std_m": noise,
                "n": len(rows), "passes": passed,
                "pass_rate_wilson95": wilson(passed, len(rows)),
                "mean_min_separation_m": mean(r["min_separation_m"] for r in rows),
                "worst_min_separation_m": min(r["min_separation_m"] for r in rows),
                "mean_final_error_m": mean(r["final_error_m"] for r in rows),
                "mean_steady_error_m": mean(r["steady_error_m"] for r in rows),
                "mean_steady_p95_slot_error_m": mean(r["steady_p95_slot_error_m"] for r in rows),
                "total_safety_infeasible_steps": sum(r["safety_infeasible_steps"] for r in rows),
            })
    return {"scope": "five-drone Phase 2 numerical moving V-to-line trial; corrected S1 only",
            "initial_layout": "3 m V formation with 0.04 m deterministic jitter",
            "mission": "12 s; 0.85 m/s target; V-to-line morph at 5 s",
            "gates": {"minimum_separation_m": 2.5, "steady_p95_slot_error_m": 1.0},
            "predictive_safety": predictive_safety,
            "safety_distance_m": 2.75 if predictive_safety else None,
            "safety_horizon_s": 1.0 if predictive_safety else None,
            "interpretation": "Sensitivity diagnostic only; not physical-flight validation.",
            "seeds": [100 + 23 * i for i in range(seeds)],
            "summary": summary, "raw": raw}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seeds", type=int, default=20)
    parser.add_argument("--noise", type=float, nargs="+", default=[0.04, 0.25, 0.5, 1.5])
    parser.add_argument("--predictive-safety", action="store_true",
                        help="evaluate the experimental local acceleration projection")
    parser.add_argument("--output", type=Path,
                        default=Path("experiments/results/phase2_gps_spacing_diagnostic.json"))
    args = parser.parse_args()
    if args.seeds < 2 or any(value < 0 for value in args.noise):
        parser.error("need at least two seeds and non-negative GPS noise levels")
    report = run(args.seeds, tuple(args.noise), args.predictive_safety)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n")
    for row in report["summary"]:
        print(f"sigma={row['gps_noise_std_m']:.2f} {row['mode']}: "
              f"{row['passes']}/{row['n']} pass; worst spacing "
              f"{row['worst_min_separation_m']:.2f} m; "
              f"mean steady p95 error {row['mean_steady_p95_slot_error_m']:.2f} m")
    print("saved", args.output)


if __name__ == "__main__":
    main()
