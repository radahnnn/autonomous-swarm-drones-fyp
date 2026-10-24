# Tracking-fit reconciliation and Phase 2 S4 sensitivity run — 2026-10-25

## Scope

This branch corrects the uneven-timestamp step fit and adds the identified S4
ArduPilot-like horizontal setpoint tracker to the older Phase 2 numerical
experiments. These experiments are **not** ten-drone SITL, MAVLink control, or
physical-aircraft tests. The S4 model was fitted to one SITL airframe's two
5 m north steps. It is used here to expose model sensitivity, not to claim a
validated vehicle model for the team's quad.

## Reconciled fits

`experiments/reconcile_tracking_fits.py` uses the same two 5 m north training
recordings and four held-out step groups for all three models. The two S1
pipelines differ in integration, delay handling, and objective, so their
parameters should not be interpreted as two estimates of one identical model.

| Model | Training position RMSE | Held-out 2 m north | 8 m north | 5 m east | 5 m diagonal |
| --- | ---: | ---: | ---: | ---: | ---: |
| S1 with observed, uneven time gaps | 10.96 cm | 8.70 cm | 19.60 cm | 9.58 cm | 9.06 cm |
| S1 fixed 0.01 s structure fit | 4.59 cm | 9.30 cm | 12.13 cm | **3.68 cm** | 3.14 cm |
| S4 cascade | **3.63 cm** | **8.47 cm** | **9.72 cm** | 4.25 cm | **2.76 cm** |

The old `tau=0.992 s, drag=0.637/s` profile came from the timestamp bug and
remains invalid. The corrected observed-interval S1 fit is `tau=0.401 s,
drag=0.231/s, delay=0.100 s`; the fixed-step S1 structure fit is
`tau=0.206 s, drag=0.010/s, delay=0.204 s`. S4's fitted position gain is
0.923/s, velocity gain 2.350/s, acceleration limit 2.657 m/s², acceleration
lag 0.122 s, and delay 0.303 s. The raw fit, grouping, and per-recording
scores are in `experiments/results/tracking_fit_reconciliation.json`.

On the separate 126 s five-drone SITL mission replay, mean horizontal error
was 30.4 cm for the invalid historical profile, 14.3 cm for corrected S1,
and 13.1 cm for S4. The worst single error was 61.8 cm for S1 versus 84.9 cm
for S4. S4 improves the mean but is not uniformly better. This is the existing
mission recording, re-analysed, not a new flight or independent airframe.

## Phase 2 integration and results

The Phase 2 controllers emit desired acceleration, whereas the identified S4
model accepts a position setpoint. `swarm_core/drone.py` now uses an explicit
inverse velocity/position-controller bridge to make that interface possible.
This synthetic bridge is **not** the live MAVLink command path. The tracker
is horizontal only; the five-drone Phase 2 trials are 2D point-mass studies.

For a controlled comparison, the new `formation` initial layout starts the
five drones in a 3 m-spaced V with small deterministic jitter and checks that
every starting pair is at least 2.5 m apart. Twenty paired seeds were run
per condition for the historical, corrected S1, and S4 bridge profiles: 500
trials per profile, 1,500 total. The four studies cover packet loss, coordinator
outages, GPS noise, and feedforward ablation. Raw trial values and 95% t
interval summaries are in `experiments/results/phase2_s4_formation_20seed.json`.

| Example condition | Historical error / separation | Corrected S1 | S4 bridge |
| --- | ---: | ---: | ---: |
| Centralized, 0% packet loss | 1.825 / 1.122 m | 1.826 / 1.115 m | 1.851 / 1.140 m |
| Hybrid, 50% packet loss | 1.822 / 1.136 m | 1.822 / 1.138 m | 1.853 / 1.147 m |
| Hybrid, 3 s coordinator outage | 1.836 / 1.131 m | 1.824 / 1.065 m | 1.858 / 1.153 m |
| Hybrid, 1.5 m GPS noise | 1.893 / 1.106 m | 1.867 / 1.144 m | 1.907 / 1.157 m |

Values are mean formation error / mean minimum sampled pair separation across
20 seeds. Across all 500 trials per profile, **500/500 violate the 2.5 m
separation gate**, even though the initial layout satisfies it. Mean error is
1.871 m historical, 1.832 m corrected S1, and 1.942 m S4 bridge. These
Phase 2 controllers and transitions therefore need work; replacing the
dynamics alone does not fix their spacing behavior. Do not use this study as
a safety validation or claim that the S4 bridge improves Phase 2 performance.
The separate ten-drone standalone SITL controller has a different path and
its own validation report.

For comparison with the branch's earlier Phase 2 output, the S4-only run
using the original `scattered` layout is preserved in
`experiments/results/phase2_s4_bridge_20seed.json`. That layout can start
vehicles closer than 2.5 m, so it is useful for numerical continuity only;
the controlled-layout run above is the clearer spacing diagnostic.

## Reproduce

```bash
python3 -m experiments.reconcile_tracking_fits
python3 -m experiments.run_corrected_phase2 --seeds 20 \
  --initial-layout formation \
  --output experiments/results/phase2_s4_formation_20seed.json
python3 -m pytest -q
```

Next, diagnose the Phase 2 formation/avoidance transitions and design a
native position-setpoint controller so S4 can be evaluated without the
synthetic inverse bridge. Refit with a separate training segment of a swarm
mission, reserve its remaining segment for validation, and report uncertainty
from more airframes and repeats. Physical parameters still require measured
hardware and real flight logs.
