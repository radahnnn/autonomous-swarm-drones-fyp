"""Paired Phase 2 sensitivity sweep after correcting the SITL timestamp fit.

The corrected S1 profile is still a simplified point-mass model. These runs
compare numerical-controller behavior, not physical-aircraft performance.
"""

import argparse
import json
from pathlib import Path

import numpy as np
from scipy.stats import t as student_t

from swarm_core.config import get_profile, set_active_profile
from experiments.test_network_sweep import run_single_trial
from experiments.test_burst_outage_sweep import run_outage_trial
from experiments.test_gps_noise_sweep import run_gps_noise_trial
from experiments.test_feedforward_ablation import run_feedforward_trial


def summarize(values):
    data = np.asarray(values, dtype=float)
    mean = float(np.mean(data))
    if len(data) < 2:
        return {"mean": mean, "ci95_low": None, "ci95_high": None}
    half = float(student_t.ppf(0.975, len(data) - 1) * np.std(data, ddof=1) / np.sqrt(len(data)))
    return {"mean": mean, "ci95_low": mean - half, "ci95_high": mean + half}


def run(seeds, profiles):
    results = []
    for profile in profiles:
        set_active_profile(profile)
        cfg = get_profile(profile)
        print(f"Profile {profile}: tau={cfg.get('attitude_tau')}, drag={cfg.get('drag_coeff')}", flush=True)
        for mode in ("centralized", "decentralized", "hybrid_proposed", "hybrid_naive"):
            for loss in (0.0, 0.5):
                for seed in seeds:
                    row = run_single_trial(mode, loss, num_drones=5, seed=seed,
                                           naive_switching=mode == "hybrid_naive")
                    results.append({"profile": profile, "study": "loss", "mode": mode,
                                    "condition": loss, "seed": seed,
                                    "error_m": row["final_formation_error_m"],
                                    "separation_m": row["min_recorded_distance_m"],
                                    "switches": row["total_mode_switches"]})
        for mode in ("centralized_hold", "hybrid_proposed"):
            for duration in (1.0, 3.0):
                for seed in seeds:
                    row = run_outage_trial(mode, duration, seed=seed)
                    results.append({"profile": profile, "study": "outage", "mode": mode,
                                    "condition": duration, "seed": seed,
                                    "error_m": row["final_error"],
                                    "separation_m": row["min_dist"],
                                    "switches": row["switches"]})
        for mode in ("centralized", "decentralized", "hybrid_proposed"):
            for sigma in (0.04, 1.5, 2.5):
                for seed in seeds:
                    row = run_gps_noise_trial(sigma, baseline=mode, seed=seed)
                    results.append({"profile": profile, "study": "gps_noise", "mode": mode,
                                    "condition": sigma, "seed": seed,
                                    "error_m": row["steady_error"],
                                    "separation_m": row["min_dist"],
                                    "collision": row["any_collision"]})
        for mode in ("centralized", "hybrid"):
            for feedforward in (False, True):
                for seed in seeds:
                    row = run_feedforward_trial(feedforward, mode=mode, seed=seed)["summary"]
                    results.append({"profile": profile, "study": "feedforward", "mode": mode,
                                    "condition": feedforward, "seed": seed,
                                    "error_m": row["steady_state_error_m"],
                                    "separation_m": row["min_recorded_distance_m"]})
    groups = {}
    for row in results:
        key = (row["profile"], row["study"], row["mode"], str(row["condition"]))
        groups.setdefault(key, []).append(row)
    summary = []
    for (profile, study, mode, condition), rows in sorted(groups.items()):
        entry = {"profile": profile, "study": study, "mode": mode,
                 "condition": condition, "n": len(rows)}
        for metric in ("error_m", "separation_m", "switches", "collision"):
            values = [row[metric] for row in rows if metric in row]
            if values:
                entry[metric] = summarize(values)
        summary.append(entry)
    return results, summary


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--seeds", type=int, default=20)
    parser.add_argument("--profiles", nargs="+", default=["sitl_default_quad", "sitl_corrected_s1"])
    parser.add_argument("--output", type=Path, default=Path("experiments/results/corrected_phase2.json"))
    args = parser.parse_args()
    seeds = [100 + 23 * i for i in range(args.seeds)]
    raw, summary = run(seeds, args.profiles)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps({"seeds": seeds, "profiles": args.profiles,
                                       "raw": raw, "summary": summary}, indent=2) + "\n")
    print(f"Saved {len(raw)} trials and {len(summary)} comparisons to {args.output}")


if __name__ == "__main__":
    main()
