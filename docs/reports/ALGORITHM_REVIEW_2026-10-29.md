# Control-algorithm review and test — 2026-10-29

Author: Claude. Commit under test: `b7688bd` (algorithms unchanged since `d1e7444`). Evidence level for everything below: **numerical simulation / unit-level experiments in the pure-Python engine** (`swarm_core`, `simulator`). Not SITL, not hardware. Profile `assumed_baseline` (the old fitted profile is known to be invalid). Raw outputs were written outside the repo (scratchpad); commands are reproducible.

## 1. What was run

| Check | Command | Result |
|---|---|---|
| Unit/integration suite | `python3 -m pytest -q tests` | 359 passed |
| 4-formation demo | `PYTHONPATH=. python3 experiments/run_demo.py --profile assumed_baseline --seed 123 --output-dir <dir>` | no collisions; **min distance 2.294 m**; final formation error 0.129 m |
| Three-mode comparison, 5 drones | `PYTHONPATH=. python3 experiments/compare_three_modes.py --trials 10 --drones 5 --output <file>` | 10/10 trials pass in every mode and scenario |
| Three-mode comparison, 10 drones | same with `--drones 10` | 10/10 pass in every mode and scenario |
| Separation-guard stress tests (my own, single pair, ideal point mass) | inline script (see §3.1) | see §3 |

Three-mode results (10 trials each; gates: min separation ≥ 2.5 m, steady p95 error ≤ 1 m). Means; worst single trial in brackets for min separation under burst outages:

| Drones | Scenario | Mode | steady p95 error (m) | min separation (m) |
|---|---|---|---|---|
| 5 | repeated outage | centralized / hybrid / decentralized | 0.135 / 0.135 / **0.264** | 2.78 / 2.78 / 2.79 |
| 5 | burst + repeated outage | centralized / hybrid / decentralized | 0.135 / 0.136 / 0.205 | 2.73 / 2.73 / 2.78 (worst 2.613 / 2.613 / 2.612) |
| 10 | repeated outage | centralized / hybrid / decentralized | 0.117 / 0.117 / **0.274** | 2.72 / 2.71 / 2.72 |
| 10 | burst + repeated outage | centralized / hybrid / decentralized | 0.117 / 0.117 / 0.206 | 2.69 / **2.63** / 2.70 (worst 2.609 / **2.502** / 2.549) |

## 2. What is good

- Clean separation of concerns (`swarm_core` controllers, network emulator, metrics); explicit parameter profiles with provenance; honest caveats printed in the result files.
- A velocity-aware local separation guard that uses only own-state and neighbor-state, shared by all three modes (`swarm_core/safety.py`).
- Hybrid supervision has the right ingredients: sequence/age validation, asymmetric hysteresis, minimum dwell, ramped blending.
- Hybrid matches centralized accuracy under outages (p95 0.135 vs decentralized 0.264 at 5 drones), so the fallback is working as designed in these scenarios.
- Paired-seed comparison with confidence intervals is the right methodology.

## 3. Findings (most important first)

### 3.1 The separation guard is a spring, not a barrier — it can be overridden by the goal term (real risk)
`separation_guard_acceleration` is capped at `max_accel` and is *added* to the nominal command. A single-pair sandbox (ideal actuation, stationary neighbor at 3 m, guard `min_sep=2.5`, `reaction=1.0`, `max_accel=2.5`) with a constant goal pull:

| Sustained goal acceleration (m/s²) | Steady separation (m) |
|---|---|
| 0.5 | 2.46 |
| 1.0 | 2.42 |
| 2.0 | 2.33 |
| 2.5 (= max_accel) | **0.26** (guard saturated; drone runs through) |

Head-on two-drone closing (0.5–3 m/s each, 15 m start, 0.5 m/s² goal pull) stays at 2.46 m ideal and with 0.18 s lag; with 0.18 s lag + 0.2 s command delay it was 2.79 m, and with the old 0.99 s lag + 0.2 s delay 2.51–2.60 m (these runs hold the gate only marginally). In the controllers the PD goal term `kp·error` is not limited before the guard is added (`CentralizedController.compute_control_inputs`), so a large slot error can exceed the guard. **Improvement:** apply safety *after* limiting nominal acceleration, e.g. clip the nominal command to `max_accel − guard magnitude`, or use a control-barrier-function projection that removes the goal component pointing into a neighbor inside its stopping envelope. Add a regression test: sustained goal pull = `max_accel` must never go below the gate in the sandbox.

### 3.2 The demo prints "Safety: PASSED" unconditionally and the value fails the project gate
`experiments/run_demo.py:170` prints the string literal `(Safety: PASSED)`. The run's minimum distance was 2.294 m, below the project's own 2.5 m acceptance gate (`docs/SOFTWARE_ACCEPTANCE_MATRIX.md`). **Fix:** compute the label from the gate (≥ 2.5 m) and print FAIL otherwise; apply the same gate in `metrics`.

### 3.3 Margins at 10 drones are razor thin
Hybrid, burst + repeated outage, 10 drones: worst trial **2.502 m** against the 2.5 m gate (2 mm). This agrees with the existing 30-seed report. Decentralized worst is 2.549 m. Separation is sampled, not continuous. **Improvements:** (a) raise the guard distance to 3.0 m (the repo's own sensitivity run reports ~10 cm worst margin), (b) lengthen `reaction_time_s` under loss, (c) report margin distributions and per-trial violations, not only pass/fail, (d) run ≥ 30 seeds per cell before claiming a result.

### 3.4 Hybrid is not better than centralized on safety, and nominal runs are identical
Paired difference hybrid − centralized is exactly 0 in `nominal_impaired` (hybrid never leaves centralized mode). Under burst outages at 10 drones hybrid min-separation (2.63 m mean) is *lower* than centralized (2.69 m) and decentralized (2.70 m). Plausible cause (not yet confirmed): each drone switches mode independently (`update_state_machine` is per-drone), so during partial loss neighbors can run different controllers and blend ramps (0.8 s) at different times. **Improvements:** make mode a swarm-level decision (propagate a mode flag over peer links, with the same hysteresis), log mode per drone per tick, test a heterogeneous-mode scenario deliberately, and give a stability argument for the blended switched system (dwell time ≥ ramp + settling) or at least an empirical sweep.

### 3.5 Hybrid fallback does not continue the mission by default
The default `fallback_strategy="consensus"` sets `v_cmd = 0` and a goal of `None`: on coordinator loss the swarm holds formation shape and brakes. That is safe, but it is not "decentralized mission execution". If the thesis claims the hybrid keeps flying on local control, the experiments must include a permanent coordinator loss with a goal pursued by the local controllers (the `dead_reckon` strategy is bounded to 1.5 s).

### 3.6 Decentralized force normalisation weakens repulsion with neighbor count
`DecentralizedController` divides the separation, alignment and formation forces by `max(1, effective_degree)`. In a fully connected 10-drone swarm the repulsion from one close neighbor is divided by up to 9. The guard compensates, which is why results still pass, but the APF part is effectively decorative: its radius (`safe_radius = 1.2 m`, centralized `collision_dist = 1.0 m`) is also well below the 2.5 m gate. **Improvement:** normalise alignment/cohesion only; sum repulsion; or drop the APF term and keep a single, documented separation layer.

### 3.7 Slot assignment: re-solved every step, no latching, three implementations
`assign_optimal_slots` (`swarm_core/formations.py`) re-runs Hungarian on measured positions each control step with squared-distance cost. That risks assignment flip-flop near ties and gives non-crossing straight-line paths only under a known spacing condition (the CAPT result by Turpin et al.; verify the exact bound before citing). With 2.5 m spacing that condition is probably not met. The SITL planner (`sitl/formation_transition.py`) instead screens pair paths explicitly, which is the better approach. **Improvements:** latch the assignment once per transition with hysteresis, reuse the SITL planner's screened-path check everywhere, and consolidate `swarm_core/formations.py`, `sitl/formation_library.py` and `sitl/formation_transition.py` (backlog item 10).

### 3.8 Smaller points
- Hybrid recovery uses `consecutive_hb ≥ 5 OR delivery_ratio ≥ 0.7` (docs describe a ratio requirement); decide which is intended and test it.
- `max_command_age = 0.15 s` is below plausible one-way delay at the top of the latency sweep; `set_nominal_latency` adapts it, so confirm every caller sets it.
- All three modes receive centrally supplied goals in these experiments ("decentralized" means local control only); say so in the thesis.
- 2D separation only: all vehicles share an altitude. Altitude staggering is a cheap extra safety layer for the first physical tests.
- Gains (`k_sep`, `k_align`, `k_form`, `kp`, `kd`) are hand-set; a sensitivity sweep around them would turn them from magic numbers into justified choices.
- The engine's dynamics (`assumed_baseline`) differ from the SITL cascade; SITL numbers must not be inferred from these.

## 4. Recommended next steps (in order)
1. Fix 3.1 (priority-aware safety) and add the sandbox regression test; re-run the 10-drone comparison with ≥ 30 seeds.
2. Fix 3.2 (honest pass/fail) — a small, safe change.
3. Decide on the hybrid mode-agreement design (3.4) and a permanent-coordinator-loss scenario (3.5).
4. Consolidate assignment/planning (3.7).
5. Re-evaluate the guard distance (3.3) after 1.
