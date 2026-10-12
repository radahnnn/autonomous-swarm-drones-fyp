#!/usr/bin/env python3
"""
Does the identified setpoint-tracking model predict a real multi-drone swarm?

  collect  (needs ArduPilot SITL, standalone):
      python experiments/swarm_replay_validation.py collect OUT_DIR [--drones 5]
      Flies a scripted mission with the swarm console (launch, move, morph, move, morph) and logs, at 10 Hz, every drone's
      commanded setpoint and its measured position (shared NED frame).

  analyze  (offline):
      python experiments/swarm_replay_validation.py analyze OUT_DIR
      Replays the logged setpoints through candidate vehicle models and compares predicted with measured positions:
        OLD : swarm_core Drone (tau 0.992 s, c_d 0.637 /s) driven by our PD controller   <- previous "fitted" profile
        S1  : same structure with parameters re-fitted correctly (tau, c_d, delay)
        S4  : identified autopilot setpoint-tracking model (swarm_core/autopilot_tracking.py)
        ZERO: assume the drone sits on its setpoint (no dynamics) - the do-nothing reference
"""

import argparse
import csv
import json
import sys
import threading
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

MISSION = [  # (console command, hold seconds after it)
    ("w 6", 14), ("l", 12), ("o", 12), ("d 5", 14), ("v", 12), ("s 6", 14), ("a 5", 14), ("x", 12), ("v", 12),
]


def collect(out_dir: Path, n: int):
    from sitl.swarm_console import SwarmConsole, handle_command

    out_dir.mkdir(parents=True, exist_ok=True)
    c = SwarmConsole(n=n, use_gazebo=False, force_restart=True)
    c.launch_sitl()
    c.connect_all()
    c.sync_frames()
    threading.Thread(target=c.guidance_loop, daemon=True).start()
    c.launch_swarm(wait=True)

    rows, events = [], []
    stop = threading.Event()
    t0 = time.time()

    def logger():
        while not stop.is_set():
            with c.lock:
                sp = c.centroid_pos[None, :] + c.current_offsets
                pos = np.array(c.last_pos)
                air = list(c.is_airborne)
            rows.append([time.time() - t0] + [v for i in range(n) for v in (*sp[i], *pos[i])] + [int(all(air))])
            time.sleep(0.1)

    threading.Thread(target=logger, daemon=True).start()
    time.sleep(10)  # settle in the initial V
    for cmd, hold in MISSION:
        events.append((time.time() - t0, cmd))
        print(f">> {cmd}", flush=True)
        handle_command(c, cmd)
        time.sleep(hold)
    stop.set()
    c.land_all()
    time.sleep(8)
    c.cleanup()

    header = ["t"] + [f"{k}{i}" for i in range(n) for k in ("spN", "spE", "spD", "pN", "pE", "pD")] + ["all_airborne"]
    with open(out_dir / "swarm_log.csv", "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(header)
        w.writerows(rows)
    (out_dir / "events.json").write_text(json.dumps(events))
    print(f"saved {len(rows)} samples to {out_dir}", flush=True)


# ----------------------------------------------------------------------------- analysis
def _delayed(history, t, td, initial):
    q = t - td
    sp = initial
    for ts, s in history:
        if ts <= q:
            sp = s
        else:
            break
    return sp


def replay_old_like(times, setpoints, p0, tau, drag, td=0.0, kp=1.8, kd=2.2):
    """swarm_core Drone driven by our PD controller toward the (optionally delayed) setpoint."""
    from swarm_core.drone import Drone

    d = Drone(drone_id=0, initial_position=np.array(p0, float), max_speed=3.0, max_accel=2.5,
              attitude_tau=tau, drag_coeff=drag, measurement_noise_std=0.0)
    hist, out, prev = [], [], times[0]
    for t, sp in zip(times, setpoints):
        hist.append((t, sp))
        dt = t - prev
        if dt > 0:
            n = max(1, int(round(dt / 0.01)))
            for _ in range(n):
                r = _delayed(hist, t, td, setpoints[0])
                d.set_control_input(kp * (r - d.position) + kd * (0.0 - d.velocity))
                d.step(dt / n)
        out.append(d.position.copy())
        prev = t
    return np.array(out)


def replay_s4(times, setpoints, p0, params):
    from swarm_core.autopilot_tracking import AutopilotTrackingModel

    m = AutopilotTrackingModel(p0, params)
    out, prev = [], times[0]
    for t, sp in zip(times, setpoints):
        dt = t - prev
        if dt > 0:
            m.step(dt, sp)
        out.append(m.p.copy())
        prev = t
    return np.array(out)


def analyze(out_dir: Path):
    from swarm_core.autopilot_tracking import AutopilotTrackingParams

    raw = np.genfromtxt(out_dir / "swarm_log.csv", delimiter=",", names=True)
    events = json.loads((out_dir / "events.json").read_text())
    n = sum(1 for k in raw.dtype.names if k.startswith("spN"))
    gen = json.loads((ROOT / "docs" / "reports" / "hw_matched_identification" / "generalization.json").read_text())
    s4 = gen["S4 ArduPilot-like cascade"]["params"]
    s1 = gen["S1 repo model (PD+lag+drag)"]["params"]
    s4p = AutopilotTrackingParams(k_pos=s4["k_pos"], kv=s4["Kv"], a_max=s4["a_max"], tau_a=s4["tau_a"], td=s4["td"])

    t_all = raw["t"]
    airborne = raw["all_airborne"] > 0.5
    alt_ok = np.all([-raw[f"pD{i}"] > 3.0 for i in range(n)], axis=0)
    start = np.argmax(airborne & alt_ok)
    sel = slice(start, None)
    t = t_all[sel] - t_all[start]
    ev = [(et - t_all[start], c) for et, c in events]
    t_end = ev[-1][0] + MISSION[-1][1] if ev else t[-1]
    keep = t <= t_end
    t = t[keep]

    models = {
        "OLD (tau .992, c_d .637)": lambda tt, sp, p0: replay_old_like(tt, sp, p0, 0.992, 0.637),
        "S1 refit (tau %.2f, c_d %.2f, td %.2f)" % (s1["tau"], s1["c_d"], s1["td"]): lambda tt, sp, p0: replay_old_like(tt, sp, p0, s1["tau"], s1["c_d"], s1["td"]),
        "S4 identified autopilot model": lambda tt, sp, p0: replay_s4(tt, sp, p0, s4p),
        "ZERO (drone sits on setpoint)": lambda tt, sp, p0: sp.copy(),
    }
    err = {m: [] for m in models}
    preds = {m: [] for m in models}
    meas_all, sp_all = [], []
    for i in range(n):
        sp = np.stack([raw[f"spN{i}"][sel][keep], raw[f"spE{i}"][sel][keep]], axis=1)
        pm = np.stack([raw[f"pN{i}"][sel][keep], raw[f"pE{i}"][sel][keep]], axis=1)
        meas_all.append(pm)
        sp_all.append(sp)
        for name, fn in models.items():
            pred = fn(t, sp, pm[0])
            preds[name].append(pred)
            err[name].append(np.linalg.norm(pred - pm, axis=1))

    print(f"Mission analysed: {t[-1]:.0f} s, {n} drones, {len(t)} samples at ~10 Hz, {len(ev)} commands")
    print("\nHorizontal position error between model prediction and measured SITL position (cm)")
    print(f"{'model':40s} " + " ".join(f"{'D%d' % (i + 1):>7s}" for i in range(n)) + f" {'mean':>7s} {'worst':>7s}")
    summary = {}
    for name in models:
        rm = [float(np.sqrt(np.mean(e ** 2))) * 100 for e in err[name]]
        mx = max(float(e.max()) for e in err[name]) * 100
        summary[name] = {"rmse_cm_per_drone": rm, "rmse_cm_mean": float(np.mean(rm)), "max_cm": mx}
        print(f"{name:40s} " + " ".join(f"{v:7.1f}" for v in rm) + f" {np.mean(rm):7.1f} {mx:7.1f}")

    print("\nBy manoeuvre (mean RMSE over drones, cm)")
    print(f"{'phase':12s} " + " ".join(f"{k[:14]:>14s}" for k in models))
    phases = {}
    for pi, lab in enumerate(["start"] + [c for _, c in ev]):
        lo = 0.0 if pi == 0 else ev[pi - 1][0]
        hi = ev[pi][0] if pi < len(ev) else t[-1]
        m = (t >= lo) & (t < hi)
        if m.sum() < 5:
            continue
        row = {name: float(np.mean([np.sqrt(np.mean(e[m] ** 2)) for e in err[name]]) * 100) for name in models}
        phases[lab + "@%.0fs" % lo] = row
        print(f"{lab + '@%.0fs' % lo:12s} " + " ".join(f"{row[k]:14.1f}" for k in models))

    (out_dir / "swarm_validation.json").write_text(json.dumps({"summary": summary, "phases": phases, "n_drones": n}, indent=2))

    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        fig, axs = plt.subplots(2, 3, figsize=(16, 8))
        for ax, i in zip(axs.ravel(), list(range(min(n, 5)))):
            ax.plot(meas_all[i][:, 1], meas_all[i][:, 0], "k-", lw=2, label="SITL measured")
            ax.plot(preds["S4 identified autopilot model"][i][:, 1], preds["S4 identified autopilot model"][i][:, 0], "r--", lw=1.4, label="S4 model")
            ax.plot(preds["OLD (tau .992, c_d .637)"][i][:, 1], preds["OLD (tau .992, c_d .637)"][i][:, 0], "b:", lw=1.4, label="OLD model")
            ax.set(title=f"Drone {i + 1} ground track", xlabel="East (m)", ylabel="North (m)")
            ax.axis("equal")
            ax.legend(fontsize=7)
        ax = axs.ravel()[5]
        for name in models:
            ax.plot(t, np.mean(err[name], axis=0) * 100, label=name[:24])
        ax.set(title="Mean error over drones", xlabel="time (s)", ylabel="cm")
        ax.legend(fontsize=7)
        fig.tight_layout()
        fig.savefig(out_dir / "swarm_validation.png", dpi=130)
    except Exception as exc:
        print(f"(plot skipped: {exc})")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("mode", choices=["collect", "analyze"])
    ap.add_argument("out_dir")
    ap.add_argument("--drones", type=int, default=5)
    a = ap.parse_args()
    (collect if a.mode == "collect" else lambda d, n: analyze(d))(Path(a.out_dir), a.drones)


if __name__ == "__main__":
    main()
