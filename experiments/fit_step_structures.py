#!/usr/bin/env python3
"""
Compare candidate model structures for the autopilot's position-step response (offline, no simulation needed).

Data: CSVs written by validate_step_response_sitl.py (columns time, x, vx, target_x).
What is modelled: the response of ArduPilot's GUIDED position controller to a position SETPOINT step, because that is
what the swarm software actually commands (setpoint -> position), not an acceleration command.

Structures (all include a pure transport delay td on the setpoint):
  S1  existing repo model : our PD controller (kp=1.8, kd=2.2) driving a lagged point mass with drag   (tau, c_d, td)
  S2  first-order closed loop                                                                          (T, td)
  S3  second-order closed loop                                                                         (wn, zeta, td)
  S4  ArduPilot-like cascade : sqrt position controller -> velocity P loop -> accel limit + accel lag    (k, Kv, amax, tau_a, td)

Usage:
    python experiments/fit_step_structures.py [run_dir ...]      # default: the four hw_matched_identification runs
"""

import json
import sys
from pathlib import Path
from typing import Callable, Dict, List, Tuple

import numpy as np
from scipy.optimize import least_squares

DT = 0.01
ROOT = Path(__file__).resolve().parent.parent
DEFAULT_RUNS = ["A_baseline", "B_hardware", "C_speedonly", "D_velponly"]
RUN_DIR = ROOT / "docs" / "reports" / "hw_matched_identification"
# WPNAV_SPEED (m/s) configured in each run; only matters if the speed limit is reached
VMAX = {"A_baseline": 3.0, "B_hardware": 16.0, "C_speedonly": 16.0, "D_velponly": 3.0}


# ----------------------------------------------------------------------------- data
def load_csv(path: Path) -> Dict[str, np.ndarray]:
    raw = np.genfromtxt(path, delimiter=",", names=True)
    # steps that are not along north carry along-track columns s / vs; otherwise use x / vx
    xc, vc = ("s", "vs") if "s" in raw.dtype.names else ("x", "vx")
    x0 = float(raw[xc][0])
    return {
        "t": raw["time"] - raw["time"][0],
        "x": raw[xc] - x0,
        "v": raw[vc],
        "r": float(raw["target_x"][0]) - x0,
    }


# ----------------------------------------------------------------------------- models
def _sqrt_controller(err: float, k: float, a_lim: float) -> float:
    """ArduPilot's sqrt_controller: linear near the target, square-root (constant decel) far from it."""
    lin = a_lim / (k * k)
    if abs(err) > lin:
        return np.sign(err) * np.sqrt(2.0 * a_lim * (abs(err) - lin / 2.0))
    return k * err


def _integrate(deriv: Callable, state0: np.ndarray, t_end: float, v_clip: float = None) -> Tuple[np.ndarray, np.ndarray]:
    n = int(round(t_end / DT)) + 1
    ts = np.arange(n) * DT
    out = np.zeros((n, len(state0)))
    s = state0.astype(float).copy()
    for i, t in enumerate(ts):
        out[i] = s
        s = s + DT * deriv(t, s)
        if v_clip is not None:  # speed saturation (swarm_core Drone.max_speed)
            s[1] = np.clip(s[1], -v_clip, v_clip)
    return ts, out


def sim_s1(p, r, t_end, vmax):
    tau, drag, td = p
    kp, kd, amax, vsat = 1.8, 2.2, 2.5, 3.0

    def f(t, s):  # s = [x, v, a]
        ref = r
        a_cmd = kp * (ref - s[0]) + kd * (0.0 - s[1])
        a_cmd = np.clip(a_cmd, -amax, amax)
        da = (a_cmd - s[2]) / tau
        dv = s[2] - drag * s[1]
        return np.array([s[1], dv, da])

    ts, out = _integrate(f, np.zeros(3), t_end, v_clip=vsat)
    return ts, out[:, 0], out[:, 1]


def sim_s2(p, r, t_end, vmax):
    T, td = p

    def f(t, s):
        ref = r
        return np.array([(ref - s[0]) / T])

    ts, out = _integrate(f, np.zeros(1), t_end)
    return ts, out[:, 0], np.gradient(out[:, 0], DT)


def sim_s3(p, r, t_end, vmax):
    wn, zeta, td = p

    def f(t, s):
        ref = r
        return np.array([s[1], wn * wn * (ref - s[0]) - 2.0 * zeta * wn * s[1]])

    ts, out = _integrate(f, np.zeros(2), t_end)
    return ts, out[:, 0], out[:, 1]


def sim_s4(p, r, t_end, vmax):
    k, kv, amax, tau_a, td = p

    def f(t, s):  # s = [x, v, a]
        ref = r
        v_cmd = np.clip(_sqrt_controller(ref - s[0], k, amax), -vmax, vmax)
        a_cmd = np.clip(kv * (v_cmd - s[1]), -amax, amax)
        return np.array([s[1], s[2], (a_cmd - s[2]) / tau_a])

    ts, out = _integrate(f, np.zeros(3), t_end)
    return ts, out[:, 0], out[:, 1]


# Informed first guess per structure (S4: the configured ArduPilot values PSC_POSXY_P=1, VELXY_P=2, accel 2.5 m/s^2)
INIT = {
    "S1 repo model (PD+lag+drag)": [0.3, 0.3, 0.1],
    "S2 first-order closed loop": [1.0, 0.5],
    "S3 second-order closed loop": [1.5, 0.8, 0.6],
    "S4 ArduPilot-like cascade": [1.0, 2.0, 2.5, 0.1, 0.5],
}

MODELS = {
    "S1 repo model (PD+lag+drag)": (sim_s1, ["tau", "c_d", "td"], [(0.05, 3.0), (0.01, 3.0), (0.0, 1.5)]),
    "S2 first-order closed loop": (sim_s2, ["T", "td"], [(0.1, 5.0), (0.0, 1.5)]),
    "S3 second-order closed loop": (sim_s3, ["wn", "zeta", "td"], [(0.2, 10.0), (0.2, 3.0), (0.0, 1.5)]),
    "S4 ArduPilot-like cascade": (sim_s4, ["k_pos", "Kv", "a_max", "tau_a", "td"],
                                  [(0.2, 5.0), (0.2, 8.0), (0.5, 10.0), (0.01, 1.0), (0.0, 1.5)]),
}


# ----------------------------------------------------------------------------- fitting
def predict(model: str, p, d: Dict[str, np.ndarray], vmax: float) -> Tuple[np.ndarray, np.ndarray]:
    """Un-delayed step response, shifted by the transport delay td (last parameter): x(t) = x0(t - td).
    For a time-invariant system this equals delaying the setpoint, and it keeps td a smooth fit parameter."""
    sim = MODELS[model][0]
    td = float(p[-1])
    ts, xs, vs = sim(p, d["r"], d["t"][-1] + 0.01, vmax)
    shifted = d["t"] - td
    return np.interp(shifted, ts, xs, left=0.0), np.interp(shifted, ts, vs, left=0.0)


def residuals(p, model, d, vmax):
    xp, vp = predict(model, p, d, vmax)
    return np.concatenate([xp - d["x"], 0.5 * (vp - d["v"])])


def fit(model: str, datasets: List[Dict[str, np.ndarray]], vmaxes: List[float], starts: int = 10, seed: int = 0):
    _, names, bounds = MODELS[model]
    lo, hi = np.array([b[0] for b in bounds]), np.array([b[1] for b in bounds])
    rng = np.random.default_rng(seed)
    best = None
    for s in range(starts):
        p0 = np.array(INIT[model], dtype=float) if s == 0 else lo + rng.random(len(lo)) * (hi - lo)
        try:
            res = least_squares(lambda p: np.concatenate([residuals(p, model, d, v) for d, v in zip(datasets, vmaxes)]),
                                p0, bounds=(lo, hi), x_scale=hi - lo, max_nfev=40)
        except Exception:
            continue
        if best is None or res.cost < best.cost:
            best = res
    return best.x, names, bounds


def rmse(model, p, d, vmax):
    xp, vp = predict(model, p, d, vmax)
    return float(np.sqrt(np.mean((xp - d["x"]) ** 2))), float(np.sqrt(np.mean((vp - d["v"]) ** 2)))


def at_bound(p, bounds, tol=0.02):
    return [i for i, (v, (lo, hi)) in enumerate(zip(p, bounds)) if v - lo < tol * (hi - lo) or hi - v < tol * (hi - lo)]


def main():
    runs = sys.argv[1:] or DEFAULT_RUNS
    data = {r: load_csv(RUN_DIR / r / "sitl_step_response.csv") for r in runs}
    result: Dict[str, dict] = {}

    print(f"{'structure':32s} {'run':12s} {'RMSE x (cm)':>12s} {'RMSE v (m/s)':>13s}  parameters (* = at a bound)")
    for model, (_, names, bounds) in MODELS.items():
        result[model] = {}
        for r in runs:
            p, _, _ = fit(model, [data[r]], [VMAX.get(r, 3.0)])
            ex, ev = rmse(model, p, data[r], VMAX.get(r, 3.0))
            flagged = at_bound(p, bounds)
            ptxt = ", ".join(f"{n}={v:.3f}{'*' if i in flagged else ''}" for i, (n, v) in enumerate(zip(names, p)))
            print(f"{model:32s} {r:12s} {ex * 100:12.1f} {ev:13.3f}  {ptxt}")
            result[model][r] = {"params": dict(zip(names, map(float, p))), "rmse_x_m": ex, "rmse_v_mps": ev,
                                "params_at_bound": [names[i] for i in flagged]}

    # cross-validation: fit on the first run only, evaluate on the others
    first, others = runs[0], runs[1:]
    if others:
        print(f"\nCross-validation: fit on {first}, test on the other runs (RMSE x in cm)")
        for model in MODELS:
            p = np.array(list(result[model][first]["params"].values()))
            cells = [f"{r}: {rmse(model, p, data[r], VMAX.get(r, 3.0))[0] * 100:5.1f}" for r in others]
            print(f"  {model:32s} " + "  ".join(cells))
            result[model]["cross_validation_rmse_x_cm"] = {r: rmse(model, p, data[r], VMAX.get(r, 3.0))[0] * 100 for r in others}

    out = RUN_DIR / "structure_fits.json"
    out.write_text(json.dumps(result, indent=2))
    print(f"\nSaved {out}")

    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        d = data[first]
        fig, ax = plt.subplots(1, 2, figsize=(12, 4.5))
        ax[0].plot(d["t"], d["x"], "k.", ms=4, label="SITL data")
        ax[1].plot(d["t"], d["v"], "k.", ms=4, label="SITL data")
        for model in MODELS:
            p = np.array(list(result[model][first]["params"].values()))
            xp, vp = predict(model, p, d, VMAX.get(first, 3.0))
            ax[0].plot(d["t"], xp, label=model.split(" (")[0])
            ax[1].plot(d["t"], vp, label=model.split(" (")[0])
        ax[0].set(title=f"Position step response ({first})", xlabel="time (s)", ylabel="x (m)")
        ax[1].set(title="Velocity", xlabel="time (s)", ylabel="vx (m/s)")
        ax[0].legend(fontsize=8)
        fig.tight_layout()
        fig.savefig(RUN_DIR / "structure_comparison.png", dpi=150)
        print(f"Saved {RUN_DIR / 'structure_comparison.png'}")
    except Exception as exc:  # plotting is optional
        print(f"(plot skipped: {exc})")


if __name__ == "__main__":
    main()
