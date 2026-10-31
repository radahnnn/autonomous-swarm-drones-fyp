# Wind / GPS-noise stress sweep and parameter sensitivity (2026-10-31)

**Evidence label: SIMULATION only** (the pure-Python simulator in `simulator/engine.py`, not ArduPilot SITL, not
Gazebo, not flight). Raw results: `experiments/results/stress_wind_gps_20261031.json`,
`experiments/results/sensitivity_20261031.json`. Harnesses: `experiments/stress_wind_gps.py`,
`experiments/sensitivity_sweep.py`; wind model: `swarm_core/wind.py` (assumed Gauss-Markov acceleration model, not
fitted to SITL or flight data).

## 1. Stress sweep: 10 drones, 30 seeds per cell
Setup: random start (>= 3 m apart), V formation translating at 0.5 m/s, morph to LINE at t = 12 s, 24 s total; the
minimum *true* inter-drone distance after t = 6 s is judged against the 2.5 m project gate. GPS error is
first-order Gauss-Markov (60 % common mode, 40 % independent per drone); wind is a mean plus a swarm-wide and a
per-drone gust, given as speed and converted to acceleration with the drag coefficient.

| Level (GPS sigma / wind mean / gust) | Centralized median min gap | Decentralized | Hybrid |
|---|---|---|---|
| calm (0.04 m / 0 / 0) | 2.41 m, pass 0 % | **2.60 m, pass 100 %** | 2.42 m, pass 0 % |
| GPS 1.5 m | 0.41 m, 21/30 collisions | 0.74 m, 16/30 | 0.40 m, 22/30 |
| wind 3 m/s gust 1 | 2.41 m, pass 0 % | 2.59 m, pass 93 % | 2.42 m, pass 0 % |
| GPS 1.5 m + wind 3 | 0.42 m, 23/30 | 0.77 m, 15/30 | 0.42 m, 21/30 |
| GPS 2.5 m + wind 5 gust 2 | 0.24 m, 29/30 | 0.36 m, 26/30 | 0.27 m, 27/30 |

("collision" is the simulator's own threshold, 0.7 m by the active profile.)

What it shows:
1. **Wind is not the problem in this model; GPS error is.** Adding 3 m/s wind with gusts barely changes any gap
   (centralized/hybrid 2.41 -> 2.41/2.42; decentralized 2.60 -> 2.59). Adding 1.5 m GPS error collapses every mode to
   0.4-0.8 m and produces collisions in roughly half of the trials.
2. **Centralized and hybrid sit below the 2.5 m gate even when calm** (2.41-2.42 m, worst 2.35 m): their nominal
   formation spacing leaves less than the gate once the swarm is moving and morphing. Decentralized uses a wider
   nominal spacing (`decentralized_nominal_spacing`) and passes. This is a property of the simulator profile, not a
   statement about the live stack, whose live 10-drone runs are in `docs/reports/*_LIVE_SITL_*`.
3. The likely mechanism for the GPS result: each drone's separation guard works on *measured* positions, whose
   relative error between two drones is about 1-1.5 m per axis at this noise level, comparable to the 2.5 m gate.
   That is consistent with the data but was not isolated by an experiment (e.g. noise on the guard input only).
4. Consequence for hardware: the 2.5 m gate and the 2.8 m barrier distance cannot be justified with plain GNSS
   accuracy (Skybrush's documentation quotes about 2-3 m plain GNSS vs 1-10 cm with RTK). Either RTK/relative
   positioning or a gate re-derived from measured position error is needed (`docs/HARDWARE_READINESS_CHECKLIST.md`,
   gate 3.3).

Limits: one formation sequence; wind enters as an acceleration the controllers cannot see (no wind estimation or
feed-forward in the model); no packet loss/latency sweep here (existing experiments cover those); the live
stack's barrier filter is not in this simulator.

## 2. Parameter sensitivity: one-at-a-time, 8 seeds, mild GPS error (0.5 m)
Each gain scaled by 0.5, 0.75, 1, 1.5, 2; a parameter is called sensitive if the median minimum gap or the median
steady error moves by more than 0.25 m over that range.

| Mode / parameter | min gap range (m) | steady error range (m) | Sensitive? |
|---|---|---|---|
| decentralized k_sep | 2.12 - 2.12 | 1.62 - 1.62 | no (identical) |
| decentralized k_align | 2.10 - 2.13 | 1.61 - 1.67 | no |
| decentralized k_form | 2.10 - 2.13 | 1.62 - 1.67 | no |
| decentralized k_goal | 2.11 - 2.13 | 1.61 - 1.64 | no |
| centralized kp / kd / k_repulse | 1.74 - 1.78 | 1.31 - 1.48 | no |
| hybrid degrade_timeout / ramp_duration | 1.78 - 1.78 | 1.31 - 1.31 | no (identical) |

Reading it correctly:
- **No tested gain matters within 0.5x-2x** for these metrics. The defaults sit on a plateau, so the specific
  numbers are not fragile. This is the useful result for the thesis ("the chosen values are not tuned knife-edges").
- **Identical rows are not a bug and not a finding about robustness.** `k_sep` only acts inside the controller's
  1.2 m safe radius, and the 2.5 m separation guard acts first, so no drone gets there in these runs (verified:
  `k_sep` x0.1 and x10 give bit-identical output, `k_form` x10 does change the result). The hybrid parameters only
  matter when the coordinator link fails, which this sweep does not include; the outage sensitivity of the
  fallback is in `experiments/sweep_hybrid_fallback.py`.
- Range is narrow (4x) and one-at-a-time: interactions are not covered, and extreme values (x10) can matter
  (`k_form` x10 raised steady error from 1.88 to 2.44 m in one spot check).
- The median minimum gap here (1.8-2.1 m) is below the calm stress table because this sweep includes the
  start-up convergence from a random layout (no settling window).
