#!/usr/bin/env python3
"""
Does each model structure generalise? Fit on 5 m north steps only, score on steps it never saw
(2 m, 8 m, east, diagonal). Data: experiments/collect_steps.py output (step_<size>m[_E|_NE]_r<k>/sitl_step_response.csv).

    python experiments/fit_generalization.py [steps_dir]
"""
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import fit_step_structures as F  # noqa: E402

ROOT = Path(__file__).resolve().parent.parent
VMAX = 3.0  # WPNAV_SPEED of the default SITL parameter file (m/s)

GROUPS = {
    "5 m north (train)": ["step_5m_r1", "step_5m_r2"],
    "2 m north": ["step_2m_r1", "step_2m_r2"],
    "8 m north": ["step_8m_r1", "step_8m_r2"],
    "5 m east": ["step_5m_E_r1", "step_5m_E_r2"],
    "5 m diagonal (NE)": ["step_5m_NE_r1", "step_5m_NE_r2"],
}


def main():
    steps_dir = Path(sys.argv[1]) if len(sys.argv) > 1 else ROOT / "docs" / "reports" / "hw_matched_identification" / "steps"
    data = {}
    for name in sum(GROUPS.values(), []):
        f = steps_dir / name / "sitl_step_response.csv"
        if f.exists():
            data[name] = F.load_csv(f)
    groups = {g: [n for n in ns if n in data] for g, ns in GROUPS.items()}
    train = groups["5 m north (train)"]
    assert train, f"no training data in {steps_dir}"

    result = {}
    cols = [g for g in GROUPS if groups[g]]
    print("RMSE of position (cm), fitted on the 5 m north steps only; mean over the repeats of each test set")
    print(f"{'structure':32s} " + " ".join(f"{c:>20s}" for c in cols))
    for model in F.MODELS:
        p, names, bounds = F.fit(model, [data[n] for n in train], [VMAX] * len(train))
        row = {g: float(np.mean([F.rmse(model, p, data[n], VMAX)[0] for n in groups[g]])) * 100 for g in cols}
        result[model] = {"params": dict(zip(names, map(float, p))), "rmse_x_cm": row}
        print(f"{model:32s} " + " ".join(f"{row[c]:20.1f}" for c in cols))

    print("\nParameters of the best structure refitted separately on each set (are they consistent?)")
    best = "S4 ArduPilot-like cascade"
    names = F.MODELS[best][1]
    print(f"{'set':22s} " + " ".join(f"{n:>9s}" for n in names))
    result["per_set_S4"] = {}
    for g in cols:
        p, _, _ = F.fit(best, [data[n] for n in groups[g]], [VMAX] * len(groups[g]))
        result["per_set_S4"][g] = dict(zip(names, map(float, p)))
        print(f"{g:22s} " + " ".join(f"{v:9.3f}" for v in p))

    out = steps_dir.parent / "generalization.json"
    out.write_text(json.dumps(result, indent=2))
    print(f"\nSaved {out}")

    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        p4 = np.array(list(result[best]["params"].values()))
        fig, axs = plt.subplots(1, len(cols), figsize=(4 * len(cols), 3.6), sharey=False)
        for ax, g in zip(np.atleast_1d(axs), cols):
            for n in groups[g]:
                d = data[n]
                ax.plot(d["t"], d["x"], ".", ms=3, color="k")
                xp, _ = F.predict(best, p4, d, VMAX)
                ax.plot(d["t"], xp, "-", lw=1.5, color="tab:red")
            ax.set(title=g, xlabel="t (s)")
        np.atleast_1d(axs)[0].set_ylabel("along-track x (m)")
        fig.suptitle("Autopilot-like cascade fitted on 5 m north only (red) vs SITL data (black)")
        fig.tight_layout()
        fig.savefig(steps_dir.parent / "generalization.png", dpi=140)
    except Exception as exc:
        print(f"(plot skipped: {exc})")


if __name__ == "__main__":
    main()
