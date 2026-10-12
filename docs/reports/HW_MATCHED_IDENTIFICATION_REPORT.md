# Step-response identification of the ArduPilot position controller (software study)

Branch `hw-matched-identification` (master untouched). ArduPilot SITL (dev build), standalone, one drone, GUIDED position steps,
2026-10-12. Raw data, plots and fits: `hw_matched_identification/` (runs A-D, `steps/`, `structure_fits.json`, `generalization.json`).
Code: `experiments/fit_step_structures.py`, `fit_generalization.py`, `collect_steps.py`, `sitl/hardware_outer_loop.parm`.

## Correction of an earlier statement
An earlier version of this report said the first-order-lag structure "cannot produce" the SITL response. **That was wrong.**
The existing fit (`validate_step_response_sitl.py::simulate_swarm_core_step`) uses `dt = times[1] - times[0]` as a fixed step for the whole
simulation. The recorded timestamps are not evenly spaced (first interval 0.14 s, then ~0.055 s), so the simulated clock ran about 2.5x too fast
and the optimiser pinned every parameter at its bound (tau 1.5, c_d 1.5, 70 cm error). The previously reported tau = 0.992 s and c_d = 0.637 /s
are artefacts of this timing bug (on the older, nearly uniform dataset the same code gives 0.83 / 0.525; neither value is physical). They should not be quoted.
With correct time handling the existing model fits well, with tau ~ 0.2-0.3 s and c_d ~ 0 (pinned at its lower bound, i.e. not identifiable).
The Phase 2 "fitted" profile (tau 0.992, c_d 0.637) was built on those artefacts and should be reconsidered.

## What was compared
Position-setpoint step -> position response of the autopilot (what the swarm software actually commands). Every structure includes a
transport delay (applied by time shift, so it is a smooth fit parameter).

| | Structure | Parameters |
|---|---|---|
| S1 | existing repo model: our PD controller + lagged point mass + drag | tau, c_d, td |
| S2 | first-order closed loop | T, td |
| S3 | second-order closed loop | wn, zeta, td |
| S4 | ArduPilot-like cascade: sqrt position controller -> velocity P loop -> acceleration limit and lag | k_pos, Kv, a_max, tau_a, td |

## Result 1: fit quality (5 m north step, four parameter sets A-D; position RMSE)
S1 4.8-7.2 cm, S2 28-32 cm, S3 6-9 cm, **S4 3.1-3.7 cm**. Copying the real quad's outer-loop settings (PSC_VELXY_P 1.5, WPNAV_SPEED, ANGLE_MAX, ATC_INPUT_TC)
changes the step response only slightly (peak speed 2.82 -> 3.00 m/s, overshoot 7.6 -> 20 cm, rise time within 0.1 s).

## Result 2: generalisation (fit on 5 m north steps only; position RMSE, cm, mean of 2 repeats)
| Structure | 5 m N (train) | 2 m N | 8 m N | 5 m E | 5 m diagonal |
|---|---|---|---|---|---|
| S1 existing | 4.6 | 9.3 | 12.1 | 3.7 | 3.1 |
| S2 first-order | 23.7 | 25.5 | 80.3 | 21.9 | 22.8 |
| S3 second-order | 5.9 | 25.5 | 81.7 | 8.4 | 6.6 |
| **S4 autopilot-like** | **3.6** | **8.5** | **9.7** | **4.3** | **2.8** |

- The linear structures (S2, S3) fail on the 8 m step (about 80 cm) because they cannot represent the speed and acceleration limits.
- S4 and S1 generalise; S4 is best or equal everywhere except the 5 m east set (4.3 vs 3.7 cm).
- East and diagonal behave like north: the same fit predicts them to 3-4 cm, so the model works on both horizontal axes. In the east and diagonal steps the
  cross-track deviation stayed below 7 cm and the final along-track position was 5.03 m.

## Result 3: do the parameters map to the autopilot configuration? (S4 refitted per data set)
| Set | k_pos | Kv | a_max (m/s^2) | tau_a (s) | td (s) |
|---|---|---|---|---|---|
| 5 m N | 0.92 | 2.35 | 2.66 | 0.12 | 0.30 |
| 2 m N | 1.42 | 2.99 | 2.46 | 0.11 | 0.32 |
| 8 m N | 0.98 | 2.38 | 2.64 | 0.08 | 0.38 |
| 5 m E | 0.74 | 1.98 | 3.07 | 0.29 | 0.23 |
| 5 m NE | 0.83 | 2.17 | 2.75 | 0.21 | 0.24 |
| configured (default SITL file) | 1.0 (PSC_POSXY_P) | 2.0 (PSC_VELXY_P) | 2.5 (WPNAV_ACCEL) | - | - |

The acceleration limit (2.5-3.1 vs 2.5 configured) and position gain (0.7-1.4 vs 1.0) land close to the configured values, and the delay (0.23-0.38 s) is consistent;
Kv and the small lag tau_a vary more (the model omits the autopilot's velocity integral and derivative terms, and jerk limiting). Treat the parameters as
effective values of the autopilot's setpoint tracking, not as exact copies of the settings.

## Result 4: 5-drone swarm mission in SITL (validation of the new parameters on the swarm)
Standalone SITL, 5 drones, scripted mission through the swarm console (launch, move north 6 m, line, ring, east 5 m, V, south 6 m, west 5 m, diamond, V; 126 s).
The commanded setpoint and the measured position of every drone were logged at 10 Hz (`swarm5/`), then the logged setpoints were replayed through each vehicle model
(`experiments/swarm_replay_validation.py analyze`). Horizontal position error, model prediction vs measured SITL position:

| Model | D1 | D2 | D3 | D4 | D5 | mean RMSE | worst single error |
|---|---|---|---|---|---|---|---|
| OLD parameters (tau 0.992 s, c_d 0.637 /s) | 29.0 | 30.1 | 31.2 | 30.8 | 30.6 | **30.4 cm** | 152 cm |
| S1 refit (tau 0.21 s, c_d 0.01, delay 0.20 s) | 11.3 | 14.0 | 16.2 | 15.5 | 14.2 | 14.3 cm | 62 cm |
| **S4 identified autopilot model** | 11.5 | 13.1 | 14.4 | 13.9 | 12.8 | **13.1 cm** | 85 cm |
| ZERO (drone sits on its setpoint) | 131 | 132 | 134 | 133 | 132 | 132 cm | 602 cm |

- The new parameters more than halve the error of the old ones on a real 5-drone mission (30 -> 13-14 cm) and the five drones behave alike.
- By manoeuvre, S4 is best during formation morphs (8-16 cm vs 12-22 cm for S1) while S1 is better during the large 6 m translations (6-11 cm vs 12-19 cm for S4).
  The 5 m single-drone steps were predicted to 3-4 cm, so the swarm mission is clearly harder than the identification data.
- Checks: shifting the measured positions in time (+/-0.3 s) does not improve S4 (best at 0 s), and the simulated clock matches the wall clock within about 5% (best scale 0.95-1.0),
  so the remaining ~13 cm is genuine model mismatch under swarm commands, not a logging-time artefact. The cause of the extra error during translations is not yet explained
  (candidates: setpoints that change continuously during morphs, the commanded yaw in the swarm console vs none in the step test, position source GLOBAL_POSITION_INT vs LOCAL_POSITION_NED).
- Not yet done: refit on part of the swarm mission and validate on the rest, repeated missions for variability, 3D/altitude, Gazebo.

## Caveats
- One simulated drone on the default SITL airframe; two repeats per condition; no confidence intervals or bootstrap yet; altitude not covered.
- This is sim-to-sim identification of the autopilot's behaviour. It says nothing about the physical airframe (mass, thrust, inertia), which still needs real flight logs.
- The structure exists as `swarm_core/autopilot_tracking.py` and is validated against a 5-drone mission (Result 4), but the swarm simulation experiments (Phase 2) have not been rerun with it.
- `sitl_vehicle_params_dump.parm` and the PSC block printed by `validate_step_response_sitl.py` record the configured profile / hard-coded values, not values read back from the autopilot.

## Suggested next steps
1. Fix `simulate_swarm_core_step` / `fit_model_parameters` to use the true time stamps (or resample to a uniform grid) and add a regression test with an uneven first interval.
2. Add bootstrap confidence intervals; test repeat variability (more repeats).
3. Plug the S4 structure into the simulator as the vehicle's setpoint-tracking model (the swarm commands position setpoints to ArduPilot), then rerun the Phase 2 experiments.
4. Replace the "fitted" profile (tau 0.992 / c_d 0.637) with parameters re-derived from this pipeline, or retire it.
5. Hardware: repeat the step test from real flight logs (Matek) for the physical identification.
