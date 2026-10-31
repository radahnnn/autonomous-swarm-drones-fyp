"""One-at-a-time parameter sensitivity sweep (SIMULATION evidence, pure Python simulator).

Each controller gain is scaled by a factor while the others stay at their defaults; for every setting N seeded
10-drone trials (the stress harness, mild disturbance: 0.5 m GPS error, no wind) report the median minimum gap and
median steady-state formation error. A parameter is "sensitive" if moving it over the factor range changes the
median minimum gap by more than 0.25 m or the median steady error by more than 0.25 m. One-at-a-time sweeps do not
show interactions between parameters; they show which numbers matter and whether the defaults sit on a plateau.

    python3 -m experiments.sensitivity_sweep --seeds 8 --out experiments/results/sensitivity_20261031.json
"""
from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import numpy as np

from experiments import stress_wind_gps as stress

stress.LEVELS["mild_gps"] = (0.5, 0.0, 0.0)
FACTORS = (0.5, 0.75, 1.0, 1.5, 2.0)
PARAMS = {  # mode -> [(controller attribute, parameter)]
    "decentralized": [("decentral_ctrl", "k_sep"), ("decentral_ctrl", "k_align"), ("decentral_ctrl", "k_form"),
                      ("decentral_ctrl", "k_goal")],
    "centralized": [("central_ctrl", "kp"), ("central_ctrl", "kd"), ("central_ctrl", "k_repulse")],
    "hybrid": [("hybrid_ctrl", "degrade_timeout"), ("hybrid_ctrl", "ramp_duration")],
}
SENSITIVE_M = 0.25


def default_value(mode: str, attr: str, name: str) -> float:
    from swarm_core.controllers.centralized import CentralizedController
    from swarm_core.controllers.decentralized import DecentralizedController
    from swarm_core.controllers.hybrid import HybridController
    cls = {"central_ctrl": CentralizedController, "decentral_ctrl": DecentralizedController,
           "hybrid_ctrl": HybridController}[attr]
    return float(getattr(cls(), name))


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--seeds", type=int, default=8)
    parser.add_argument("--drones", type=int, default=10)
    parser.add_argument("--modes", default=",".join(PARAMS))
    parser.add_argument("--out", type=Path, default=Path("experiments/results/sensitivity.json"))
    args = parser.parse_args()
    out, started = {}, time.time()
    for mode in args.modes.split(","):
        for attr, name in PARAMS[mode]:
            base = default_value(mode, attr, name)
            rows = []
            for factor in FACTORS:
                trials = [stress.run_trial(mode, "mild_gps", 2000 + i, args.drones, overrides={(attr, name): base * factor})
                          for i in range(args.seeds)]
                rows.append({"factor": factor, "value": base * factor,
                             "min_gap_median": float(np.median([t["min_gap_m"] for t in trials])),
                             "steady_error_median": float(np.median([t["steady_error_m"] for t in trials])),
                             "collisions": sum(t["collision"] for t in trials)})
            gaps, errs = [r["min_gap_median"] for r in rows], [r["steady_error_median"] for r in rows]
            sensitive = (max(gaps) - min(gaps) > SENSITIVE_M) or (max(errs) - min(errs) > SENSITIVE_M)
            at_default = next(r for r in rows if r["factor"] == 1.0)
            best_gap = max(rows, key=lambda r: r["min_gap_median"])
            out[f"{mode}/{name}"] = {"default": base, "rows": rows, "sensitive": bool(sensitive),
                                     "best_gap_factor": best_gap["factor"],
                                     "default_is_within_0.1m_of_best": bool(at_default["min_gap_median"] >= best_gap["min_gap_median"] - 0.1)}
            print(f"{mode:<14}{name:<18} sensitive={sensitive!s:<5} gap {min(gaps):.2f}..{max(gaps):.2f} "
                  f"err {min(errs):.2f}..{max(errs):.2f} best factor {best_gap['factor']}", flush=True)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps({"seeds": args.seeds, "drones": args.drones, "level": "mild_gps",
                                    "results": out, "elapsed_s": time.time() - started}, indent=2))


if __name__ == "__main__":
    main()
