"""Compare the two corrected S1 fit pipelines and S4 on identical train/holdout steps.

Train: two 5 m north SITL steps. Holdout: 2 m, 8 m, east, and diagonal steps.
The legacy S1 pipeline is refitted with its original objective and bounds;
the structure fits are loaded from their existing two-run training result.
No physical-airframe parameters are inferred by this comparison.
"""

import argparse
import json
from pathlib import Path

import numpy as np
from scipy.optimize import minimize

from experiments.fit_generalization import GROUPS, VMAX
from experiments import fit_step_structures as structures
from experiments.validate_step_response_sitl import simulate_swarm_core_step

ROOT = Path(__file__).resolve().parent.parent
DEFAULT_STEPS = ROOT / "docs/reports/hw_matched_identification/steps"
DEFAULT_FITS = DEFAULT_STEPS.parent / "generalization.json"


def legacy_predict(params, data):
    return simulate_swarm_core_step(
        data["t"], params[0], params[1], delay=params[2], step_dist=data["r"],
    )


def fit_legacy(train):
    def objective(params):
        scores = []
        for data in train:
            x, v = legacy_predict(params, data)
            scores.append(np.sqrt(np.mean((x - data["x"]) ** 2)) +
                          0.5 * np.sqrt(np.mean((v - data["v"]) ** 2)))
        return float(np.mean(scores))

    result = minimize(objective, [0.30, 0.30, 0.10],
                      bounds=[(0.05, 1.50), (0.01, 1.50), (0.0, 0.35)],
                      method="L-BFGS-B")
    if not result.success:
        raise RuntimeError(f"legacy optimizer failed: {result.message}")
    return result.x


def compare(steps_dir=DEFAULT_STEPS, fits_path=DEFAULT_FITS):
    datasets = {
        name: structures.load_csv(steps_dir / name / "sitl_step_response.csv")
        for names in GROUPS.values() for name in names
    }
    train_names = GROUPS["5 m north (train)"]
    legacy = fit_legacy([datasets[name] for name in train_names])
    fits = json.loads(fits_path.read_text())
    names = {
        "legacy_irregular_s1": ("tau", "c_d", "td"),
        "structure_s1": ("tau", "c_d", "td"),
        "s4_cascade": ("k_pos", "Kv", "a_max", "tau_a", "td"),
    }
    params = {
        "legacy_irregular_s1": dict(zip(names["legacy_irregular_s1"], map(float, legacy))),
        "structure_s1": fits["S1 repo model (PD+lag+drag)"]["params"],
        "s4_cascade": fits["S4 ArduPilot-like cascade"]["params"],
    }
    scores = {}
    for group, examples in GROUPS.items():
        scores[group] = {}
        for name in examples:
            data = datasets[name]
            scores[group][name] = {}
            for model in names:
                p = np.array(list(params[model].values()), dtype=float)
                if model == "legacy_irregular_s1":
                    xp, vp = legacy_predict(p, data)
                    x_rmse = float(np.sqrt(np.mean((xp - data["x"]) ** 2)))
                    v_rmse = float(np.sqrt(np.mean((vp - data["v"]) ** 2)))
                else:
                    structure = ("S1 repo model (PD+lag+drag)" if model == "structure_s1"
                                 else "S4 ArduPilot-like cascade")
                    x_rmse, v_rmse = structures.rmse(structure, p, data, VMAX)
                scores[group][name][model] = {"position_rmse_m": x_rmse, "velocity_rmse_mps": v_rmse}
    return {
        "training_sets": train_names,
        "holdout_groups": [group for group in GROUPS if group != "5 m north (train)"],
        "models": params,
        "scores": scores,
        "method_note": (
            "Legacy S1 uses each irregular observed interval, a discrete onset delay, "
            "Drone.step's lag update, and mean(position RMSE + 0.5 velocity RMSE). "
            "Structure S1/S4 use 0.01 s Euler integration, an interpolated delay, "
            "and least-squares residuals. The models share train and holdout recordings "
            "but their integration/objective definitions differ."
        ),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--steps-dir", type=Path, default=DEFAULT_STEPS)
    parser.add_argument("--fits", type=Path, default=DEFAULT_FITS)
    parser.add_argument("--output", type=Path, default=ROOT / "experiments/results/tracking_fit_reconciliation.json")
    args = parser.parse_args()
    result = compare(args.steps_dir, args.fits)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2) + "\n")
    print("model parameters:", result["models"])
    for group, rows in result["scores"].items():
        means = {model: round(100 * float(np.mean([row[model]["position_rmse_m"] for row in rows.values()])), 2)
                 for model in result["models"]}
        print(group, "position RMSE cm:", means)
    print("saved", args.output)


if __name__ == "__main__":
    main()
