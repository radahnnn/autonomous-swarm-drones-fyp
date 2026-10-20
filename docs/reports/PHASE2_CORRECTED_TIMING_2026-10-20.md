# Phase 2 rerun after the step-response timing correction

The original step-response integrator reused the first telemetry gap as a fixed time step. It now uses every recorded interval, and `tests/test_step_response_timing.py` covers an uneven first interval. The older `sitl_default_quad` profile is retained only to reproduce historical results; its `tau=0.992 s` and `drag=0.637/s` values are invalid as calibrated parameters.

This rerun is a **sensitivity comparison** using the existing point-mass swarm engine. The new `sitl_corrected_s1` profile uses the corrected S1 fit (`tau=0.21 s`, effective damping at the lower fit bound `0.01/s`). The S1 identification also estimated about 0.20 s transport delay, which this engine does not model. The more accurate S4 autopilot-style setpoint tracker has not yet been integrated into Phase 2; therefore neither profile is a validated physical vehicle model.

## Method and results

`python3 -m experiments.run_corrected_phase2 --seeds 20` ran 1,000 trials across two profiles with paired seeds, five simulated drones, and the current repository's packet-loss, outage, GPS-noise, and feedforward scenarios. The [raw results and all 50 condition summaries](../../experiments/results/corrected_phase2_20seed.json) include per-seed values. Intervals below are 95% Student-t intervals over seeds; they describe run-to-run variation under this simulation, not model uncertainty.

| Condition; metric | Historical profile | Corrected S1 profile |
|---|---:|---:|
| 0% loss, centralized; final formation error | 1.828 m [1.491, 2.165] | 1.826 m [1.501, 2.152] |
| 50% loss, proposed hybrid; final formation error | 1.822 m [1.480, 2.163] | 1.823 m [1.493, 2.152] |
| 50% loss, decentralized; final formation error | 1.844 m [1.501, 2.188] | 2.029 m [1.628, 2.430] |
| 3 s outage, centralized hold; final error | 1.834 m [1.497, 2.171] | 1.825 m [1.498, 2.151] |
| 3 s outage, proposed hybrid; final error | 1.831 m [1.494, 2.168] | 1.825 m [1.499, 2.151] |
| GPS noise 1.5 m, hybrid; closest separation | 0.906 m [0.736, 1.075] | 0.949 m [0.754, 1.143] |
| GPS noise 1.5 m, hybrid; trials with collision (<0.70 m) | 8/20 | 6/20 |

The corrected profile changes some results but does **not** make the current Phase 2 implementation meet a 2.5 m clearance or 1 m tracking target. The present script's large errors also differ markedly from the older Phase 2 report, which was generated from a different branch and statistical harness. These outputs must not be combined or presented as a like-for-like rerun of that historical report. A clean investigation of initialization, metric definitions, controller gains, and the missing transport delay is required before making controller rankings.

## Identification caveat

Rerunning the *older* `fit_model_parameters()` optimizer after its timing fix on the four recorded 5 m datasets gives `tau=0.40–0.48 s`, `drag=0.17–0.26/s`, and position RMSE 0.12–0.15 m. The separate S1 structure-fit pipeline in the identification report gives `tau=0.21–0.32 s`, `drag=0.01/s`, and position RMSE 0.05–0.07 m. They have different delay handling and fitting objectives. The latter was used for the sensitivity profile here, but those pipelines need to be reconciled before naming one final calibrated profile.
