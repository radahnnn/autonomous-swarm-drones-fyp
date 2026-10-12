# Hardware-matched step-response identification (software step)

Branch: `hw-matched-identification` (master untouched). Run on the Ubuntu PC, ArduPilot SITL (dev build), standalone, one drone,
5 m position step in GUIDED mode, 2026-10-12. Raw CSVs, plots and logs: `hw_matched_identification/<run>/`.

## What was done
The real quad's outer-loop settings (from `hardware/params/matek_h743_quad_20261006.param`) were layered over the SITL defaults
via `sitl/hardware_outer_loop.parm`, then the existing step-response test and (tau, c_d, delay) fit were rerun:

| Run | Settings changed vs SITL defaults |
|---|---|
| A_baseline | none (`swarm_params.parm`: PSC_VELXY_P 2.0, WPNAV_SPEED 300 cm/s) |
| B_hardware | all real-quad outer-loop values: PSC_VELXY_P 1.5, WPNAV_SPEED 1600, ANGLE_MAX 3000, ATC_INPUT_TC 0.15 (POS_P, VEL_I/D, jerk, accel already equal) |
| C_speedonly | WPNAV_SPEED 1600 only |
| D_velponly | PSC_VELXY_P 1.5 only |

Not copied: attitude/rate-loop gains (ATC_RAT_*, ATC_ANG_*), which are tuned for the 5-inch airframe and would be invalid on SITL's generic quad.

## Result 1: the fit does not work, in any run
| Run | tau (s) | c_d (1/s) | delay (s) | position RMSE | velocity RMSE |
|---|---|---|---|---|---|
| A | 1.500 | 1.500 | 0.100 | 69.8 cm | 0.77 m/s |
| B | 1.500 | 1.500 | 0.100 | 66.4 cm | 0.83 m/s |
| C | 1.500 | 1.500 | 0.100 | 53.5 cm | 0.69 m/s |
| D | 1.500 | 1.500 | 0.100 | 57.9 cm | 0.73 m/s |

Every fitted parameter sits on its bound (tau <= 1.5, c_d <= 1.5, delay >= 0.1 in the optimiser) and the error is 50-70 cm. These parameter values are therefore
not identified; they are optimiser limits. **The earlier reported values (tau 0.992 s, c_d 0.637 /s, 15.4 cm RMSE) did not reproduce on today's baseline run**,
so they should not be quoted as current without re-deriving them from a documented dataset.

## Result 2: why - the model structure cannot produce this response
The recorded SITL response (about 0.6-1.0 s dead time, bell-shaped speed profile peaking near 2.8-3.0 m/s at ~1.8 s, symmetric slow-down, 5 m reached at ~3-4 s)
is the autopilot's jerk- and acceleration-limited trajectory generator plus its cascaded position/velocity controller. A first-order lag with linear drag can only
approach a target as a monotone exponential, so it cannot match a bell-shaped velocity profile. The effective "tau" is dominated by trajectory shaping and limits
(PSC_JERK, WPNAV_ACCEL), not by vehicle dynamics.

## Result 3: effect of the real quad's settings on the step response
| Run | peak speed (m/s) | time of peak (s) | t to 50% (s) | t to 90% (s) | overshoot (cm) |
|---|---|---|---|---|---|
| A | 2.82 | 1.82 | 1.12 | 2.80 | 7.6 |
| B | 3.00 | 1.83 | 1.12 | 2.70 | 20.4 |
| C | 2.80 | 1.76 | 1.06 | 2.81 | 5.6 |
| D | 2.81 | 1.81 | 1.05 | 2.79 | 15.4 |

The real quad's outer loop changes the response only slightly (rise time within ~0.1 s); the lower velocity gain mainly increases overshoot (7.6 -> 15-20 cm).
The timing is dominated by dead time and acceleration/jerk limits, which are identical in both settings.

## Caveats
- Single run per variant, SITL's default quad physics (not the real airframe), one axis, one step size; no confidence intervals.
- `sitl_vehicle_params_dump.parm` and the PSC block printed by the script record the configured profile/hard-coded values, not values read back from the running autopilot.
- This is sim-to-sim calibration. It says nothing yet about the physical airframe.

## What this means for the supervisor discussion
- The lagged point-mass structure is not supported by the SITL step data; a structure with dead time and a second-order (or trajectory-generator) response should be compared.
- Do not present tau = 0.992 s / c_d = 0.637 /s as validated physical values.
- Next steps: (1) fit alternative structures to the saved CSVs (offline, no new simulation needed), (2) multiple seeds/step sizes with confidence intervals,
  (3) real flight logs from the Matek for the physical identification.
