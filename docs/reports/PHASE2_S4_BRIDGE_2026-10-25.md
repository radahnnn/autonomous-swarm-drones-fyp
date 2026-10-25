# Tracking-fit reconciliation and Phase 2 S4 sensitivity run — 2026-10-25

## Scope

This branch corrects the uneven-timestamp step fit and adds the identified S4
ArduPilot-like horizontal setpoint tracker to the older Phase 2 numerical
experiments. These experiments are **not** ten-drone SITL, MAVLink control, or
physical-aircraft tests. The S4 model was fitted to one SITL airframe's two
5 m north steps. It is used here to expose model sensitivity, not to claim a
validated vehicle model for the team's quad.
Shared software completion criteria and measurement definitions are in the
[software acceptance matrix](https://github.com/radahnnn/autonomous-swarm-drones-fyp/blob/work/decentralized-sitl-20261020/docs/SOFTWARE_ACCEPTANCE_MATRIX.md)
on the experimental controller branch.

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

### Spacing diagnosis: GPS noise and controller threshold

`experiments/diagnose_phase2_gps_spacing.py` held corrected S1 dynamics, the
three-metre V start, moving reference, and V-to-line morph constant while
varying position-noise standard deviation across 20 seeds per mode.

| Position noise | Centralized mean minimum / worst | Decentralized mean minimum / worst | Hybrid mean minimum / worst | Passes across 60 runs |
| ---: | ---: | ---: | ---: | ---: |
| 0.04 m | 2.288 / 2.213 m | 2.123 / 2.040 m | 2.288 / 2.213 m | 0/60 |
| 0.25 m | 2.190 / 1.930 m | 2.029 / 1.750 m | 2.190 / 1.930 m | 0/60 |
| 0.50 m | 2.000 / 1.503 m | 1.890 / 1.303 m | 2.000 / 1.503 m | 0/60 |
| 1.50 m | 1.115 / 0.038 m | 1.031 / 0.002 m | 1.115 / 0.038 m | 0/60 |

Even at 4 cm position noise, none of these 60 runs met both the 2.5 m
separation and 1 m steady-error gates. The steady-error gate passed in all 60
runs at 0.04 m and 0.25 m, 57/60 at 0.50 m, and 0/60 at 1.50 m; the spacing
gate passed in none. At the profile's default 1.5 m noise, mean steady error
rose to 1.87–2.00 m and worst sampled clearance nearly reached zero. Noise
makes the failure much worse, but it is not the only cause: the
current APF repulsion activates at 1.5 m, below the 2.5 m project gate, and
does not enforce stopping distance or an uncertainty margin.

A temporary additive/priority repulsion override was checked on one seed and
made spacing worse; it was discarded. A proper fix needs a tested pairwise
safety constraint with priority over tracking, delayed/noisy neighbor-state
handling, and a feasible formation path. Simply increasing the APF radius or
reducing sensor noise is not enough evidence of a fix. Raw noise-sweep data
are in `experiments/results/phase2_gps_spacing_diagnostic.json`.

### Experimental predictive projection and common tracking metric

The first opt-in candidate projects each vehicle's desired acceleration onto
radial pair constraints over a 1 s look-ahead with a 2.75 m requested distance.
It uses only received peer states and respects the drone's acceleration limit.
It does not model tangential closest approach, the other vehicle's future
command, actuator lag, or unbounded position error, so it is not a safety
guarantee. Infeasible projections are counted explicitly. The existing
controller default remains unchanged.

This rerun also added a **pooled steady p95 per-vehicle slot error** over
samples at `t >= 8.5 s`. Earlier Phase 2 `steady_state_error_m` is a mean of
per-step swarm RMS errors and cannot be compared directly with a p95 gate.
The table applies both the 2.5 m sampled separation gate and a 1 m pooled
steady p95 gate to the same 20 seeds per mode and noise level.

| Position noise | Mode | Baseline passes | Projection passes | Projection worst pair distance | Projection mean steady p95 |
| ---: | --- | ---: | ---: | ---: | ---: |
| 0.04 m | Centralized | 0/20 | 20/20 | 2.80 m | 0.26 m |
| 0.04 m | Decentralized | 0/20 | 0/20 | 2.79 m | 1.31 m |
| 0.04 m | Hybrid | 0/20 | 20/20 | 2.80 m | 0.26 m |
| 0.25 m | Centralized | 0/20 | 18/20 | 2.46 m | 0.52 m |
| 0.25 m | Decentralized | 0/20 | 0/20 | 2.40 m | 1.47 m |
| 0.25 m | Hybrid | 0/20 | 18/20 | 2.46 m | 0.52 m |
| 0.50 m | Centralized | 0/20 | 4/20 | 1.95 m | 0.93 m |
| 0.50 m | Decentralized | 0/20 | 0/20 | 1.87 m | 1.78 m |
| 0.50 m | Hybrid | 0/20 | 4/20 | 1.95 m | 0.93 m |
| 1.50 m | All three | 0/20 each | 0/20 each | 0.01–0.02 m | 2.87–3.66 m |

The candidate is a useful low-noise spacing intervention for centralized and
hybrid control, but it fails the common criteria for decentralized tracking
and does not tolerate moderate/high position noise. Raising the requested
distance in a five-seed sensitivity check improved some 0.25 m-noise spacing
results but produced hundreds of infeasible projected steps and, at 3.5 m,
lost tracking passes. This is not ready for live SITL or onboard use. Next,
design a feasible reconfiguration path and a safety supervisor that explicitly
handles estimation bounds, relative velocity, acceleration lag, and constraint
infeasibility. Raw paired reruns are in
`experiments/results/phase2_baseline_p95_20seed.json` and
`experiments/results/phase2_predictive_safety_p95_20seed.json`.

During this review, `build_controllers_from_profile` was found to ignore its
declared `k_goal=1.0` and use the decentralized class default `0.8`. The
profile value is now applied. A separate paired 20-seed decentralized check
after that correction still passed only **1/20** low-noise trials with the
projection and **0/20** at 0.25 m noise; the no-projection baseline passed
0/20 at both levels because of spacing. The lower-noise projection kept every
sampled pair above 2.79 m, while its mean steady p95 slot error was 1.06 m.
This confirms the remaining tracking/transition problem rather than resolving
it. The post-fix raw results are in
`experiments/results/phase2_decentralized_gain_fix_20seed.json`.

A five-seed exploratory test interpolated V-to-line targets over 2, 3, or 5 s
while the centroid continued to move. It did not improve the decentralized
passes: 0/5 at every duration, versus 0/5 in the same seeds with an immediate
transition. At 3 and 5 s it also worsened centralized and hybrid tracking.
That interpolation was removed from the implementation. The next transition
planner must account for a moving centroid and feasible individual routes,
not merely smooth the target coordinates.

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
python3 -m experiments.diagnose_phase2_gps_spacing --seeds 20
python3 -m experiments.diagnose_phase2_gps_spacing --seeds 20 \
  --predictive-safety \
  --output experiments/results/phase2_predictive_safety_p95_20seed.json
python3 -m pytest -q
```

Next, design and validate a pairwise safety controller and feasible formation
transition, then design a native position-setpoint controller so S4 can be evaluated without the
synthetic inverse bridge. Refit with a separate training segment of a swarm
mission, reserve its remaining segment for validation, and report uncertainty
from more airframes and repeats. Physical parameters still require measured
hardware and real flight logs.
