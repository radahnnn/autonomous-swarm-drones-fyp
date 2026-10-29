# FYP Agent Handoff: Swarm Drones (BEE-60)

**Written:** 3 October 2026
**Purpose:** Give any new agent (or teammate) the full context, decisions, review findings and task list for this project, so it does not have to rediscover them.
**Read this first, then verify.** Items are tagged by how trustworthy they are:

| Tag | Meaning |
|---|---|
| `[code]` | Read directly in the source file |
| `[log]` | Taken from the project's own documents (log, reports, briefs); not independently verified |
| `[inference]` | The reviewer's reasoning; plausible but not proven; has a stated check |
| `[verify]` | A fact about ArduPilot, MAVLink or hardware stated from memory; confirm in official docs before relying on it |

Do not treat anything tagged `[inference]` or `[verify]` as established.

---

## 1. Project snapshot

- **Project:** Final year project, Dept. of Electrical Engineering. Autonomous swarm of multirotors flying formations (line, V, circle, grid), with centralised (master-slave), decentralised (mesh/consensus) and hybrid control. Must be simulated and, ideally, flown on real drones.
- **Team:** A group of students plus a hardware teammate; supervisor Dr. Abdul Ghafoor; an MS sub-supervisor/research assistant also reviews.
- **Deadline:** 31 Dec 2026 `[log]`.
- **Repo:** `github.com/radahnnn/autonomous-swarm-drones-fyp` (public, branch `master`). GitHub blocks automated access to code *folders*; individual `.md` files can be fetched, code files must be uploaded manually.
- **Tools in use:** Antigravity (agent doing most coding), GitHub Copilot (reasoning/review), plus Claude for review (this document's author). Host machine: Ubuntu 24.04, Ryzen 5 3500X, 15 GiB RAM, GTX 1660 Super `[log]`.
- **Department fleet:** 10 quadcopters; the team has 2 `[log]`. Model described as an "FPV P5 Voyager variant"; no public datasheet found.
- **Related prior project in the same department:** "Swarm Humanoid Robots for Cooperative Task Execution" (EZ-Robot JD humanoids, laptop relays everything over Wi-Fi/TCP). See section 11.

---

## 2. Hardware facts

### Confirmed or near-confirmed
- **Flight controller:** Matek H743-SLIM V3 (from board photos/PDF per the log; reported by the hardware teammate).
- **ArduPilot support:** Matek lists it as compatible with INAV, Betaflight and ArduPilot; ArduPilot Copter firmware target is `MatekH743` (and a `-bdshot` variant) `[verify]`.
- **Ports (retailer listing, `[verify]` against Matek's SLIM V3 wiring diagram):** 7 UARTs, 2 I2C, **1 CAN**, a JST-GH 4-pin connector (5V / CAN-H / CAN-L / GND), 6 ADC, USB-C, dual camera inputs.
- **Default ArduPilot serial map for the H743 family (wiki, family-wide, `[verify]` for SLIM V3):** SERIAL1=UART7 (Telem1), SERIAL2=USART1, SERIAL3=USART2 (GPS1), SERIAL4=USART3 (GPS2), SERIAL5=UART8, SERIAL6=UART4, SERIAL7=UART6.

### Unknown (blocking)
1. **What firmware is currently on the boards** (team mentioned Betaflight and Mission Planner).
2. **Whether a GPS and compass are fitted.** No GPS means no position control and no formations.
3. **ESC/motor protocol (DShot?) and actual airframe size.** The log says 5-inch in Task E but models a *cinewhoop* in Gazebo.
4. **Whether all 10 drones have the same FC/revision.**
5. **Whether the department allows reflashing, and whether a spare FC can be used as a bench board.**
6. **Receiver and protocol** (ELRS? 2.4 GHz?), transmitters available, soldering station and supervisor.

### Known constraints and ideas
- The team **cannot solder** on the drone. The drone has a ~26 V pad (battery, believed 6S); an ESP32 needs 5 V (VIN) or 3.3 V.
- **Do not use a cheap buck module from 6S**: many are rated ~28 V input; failure can short 25 V into the ESP32 and FC. Preferred: take 5 V from the FC (e.g. the JST-GH CAN connector or a UART connector via a pre-made pigtail), or power the ESP32 separately with common ground only.
- **Never power the ESP32 from the FC's 5 V and USB at the same time** (backfeed).
- **FC UARTs are normally 3.3 V logic** `[verify]`; ESP32 TX/RX cross to FC RX/TX; common ground required.
- **Supervisor asked for a "CAN bus receiver".** The team has an SN65HVD230 3.3 V CAN transceiver module. ESP32 has a built-in CAN controller (TWAI) but needs the transceiver. DroneCAN runs at 1 Mbit/s; 120 Ω termination at each bus end (check for onboard termination with a multimeter: ~60 Ω across CAN-H/CAN-L means both ends terminated). A lone node errors out without another node to ACK, so bench-test with a USB-CAN adapter (`candump`).
- **CAN is a wired in-vehicle bus and cannot link drones to each other.** It suits peripherals (GPS, ESCs, compass). The Task E agent report says ArduPilot does not expose GUIDED setpoint subscribers over DroneCAN `[log, verify]`, so the simplest route for commands is **MAVLink over UART**. **The supervisor's actual intent for CAN is unresolved; ask them.** Suggested framing: UART/MAVLink for the swarm link, CAN as a scoped extra if required.
- **ESP32 Wi-Fi is 2.4 GHz only (except the C5 variant)** `[verify]`. The Task E plan of a "5 GHz router with ESP32s to avoid ELRS interference" is therefore invalid. Alternatives: 900 MHz ELRS for RC, SiK telemetry radios, or accept 2.4 GHz coexistence and test it.
- **Plain GPS (e.g. u-blox M10) relative error ~1–2.5 m `[log]`.** Formation spacing must be scaled accordingly (reviewer recommends 8–10 m for real flights). RTK (dual F9P) is the upgrade path (~$300–400 per the log).

---

## 3. Architecture and key decisions

### Three tiers (decided 30 Sep 2026)
1. **Tier 1 `swarm_core` + `simulator`:** fast numerical engine for algorithm development and Monte Carlo sweeps.
2. **Tier 2 ArduPilot SITL (headless, MAVLink via pymavlink):** multi-vehicle validation. ROS 2/Gazebo plumbing was abandoned as a time sink; Gazebo is a visual layer only.
3. **Tier 3 hardware:** Matek H743 + ESP32 bridge. Not started.

### Control modes
- **Centralised:** laptop coordinator assigns slots (Hungarian), PD tracking, drag feedforward, APF safety.
- **Decentralised:** Laplacian velocity consensus, Reynolds/APF separation, relative-offset cohesion using 1-hop neighbours.
- **Hybrid (the proposed contribution):** per-drone state machine; degrade to decentralised after 0.5 s of silence/stale heartbeats; recover only after a sliding-window delivery ratio ≥70% over 20 ticks, a 2.0 s dwell, and a recent valid packet; continuous blending α(t); always-on APF.

### Hardware communication plan (recommended, not yet built)
- ESP32 per drone on a free UART, MAVLink2 ↔ UDP over Wi-Fi to the laptop (laptop = master), ArduPilot GUIDED mode, per-drone `SYSID_THISMAV` 1–10, telemetry stream rates configured.
- **UDP, not TCP**, for setpoints: a late setpoint is worse than a dropped one.
- **On hardware, the hybrid's "onboard decentralised fallback" cannot run** if the ESP32 is only a transparent bridge and the controller lives on the laptop. When the link dies, the realistic fallback is ArduPilot's own failsafe/hold. Options: add ESP-NOW peer broadcast plus onboard logic, or a companion computer. Until then, state that **the hybrid is validated in simulation/SITL only**.

### Localisation constraint
Formation accuracy is limited by position knowledge. Decide indoor vs outdoor early; plain GPS outdoors with generous spacing is the achievable path.

### Safety for real flights
Props off for all bench work; open field; one drone at a time until proven; minimum 10 m separation; transmitter kill switch per airframe; supervisor-approved test procedure. 6S 5-inch quads cause serious injuries.

---

## 4. Repo layout (from README and files read)

```
swarm_core/   drone.py, graph.py, network.py, formations.py, metrics.py, config.py
  controllers/ centralized.py, decentralized.py, hybrid.py
simulator/    engine.py, visualizer.py
sitl/         mavlink_swarm_adapter.py, common_frame.py, launch_*.sh, swarm_params.parm
experiments/  run_demo.py, test_network_sweep.py, validate_step_response_sitl.py,
              validate_3drone_scenario_sitl.py, results/
tests/        45 tests per the log (README still lists only 3 files)
docs/         BASELINE_AUDIT.md, SIMULATION_SITL_PARITY.md
root *.md     FYP_RUNNING_LOG, ACADEMIC_REVIEW_RESPONSE, TASK_B/C/E reports,
              SUB_SUPERVISOR_MEETING_BRIEF, CODEX_REVIEW_BRIEF, CLAUDE.md, FYP_SETUP_LOG
```

**Files read in full by the reviewer:** `network.py`, `hybrid.py`, `drone.py`, `engine.py`, `decentralized.py`, `centralized.py`, `mavlink_swarm_adapter.py`, `validate_step_response_sitl.py`, `validate_3drone_scenario_sitl.py`.
**Documents read:** `ACADEMIC_REVIEW_RESPONSE.md`, `FYP_RUNNING_LOG.md`, `SUB_SUPERVISOR_MEETING_BRIEF.md`, `TASK_B_EXPERIMENTAL_REPORT.md`.
**Not yet read:** `config.py` (incl. `build_controllers_from_profile`), `formations.py`, `common_frame.py`, `graph.py`, `metrics.py`, tests, TASK_C/E reports, CODEX brief, `run_demo.py`, `test_network_sweep.py`, launch scripts, `swarm_params.parm`.

---

## 5. Reported project state (from the log, unverified)

- `swarm_core` done: kinematics with first-order lag and drag, Hungarian slot assignment, three controllers, wireless channel emulator (Bernoulli loss, latency, range, Gilbert-Elliott burst, scheduled outages), metrics.
- Parameter profiles in `config.py`: `assumed_baseline` (τ=0.18 s, c_d=0.20 1/s, σ_gps=0.04 m) and `sitl_fitted` (τ=0.992 s, c_d=0.637 1/s, σ_gps=1.5 m, r_safe=1.5 m).
- SITL: 3 ArduCopter instances in GUIDED mode; MAVLink adapter; shared WGS84 datum (Lat0 −35.3632621, Lon0 149.1652374, Alt0 584 m); 3-drone V-formation flying.
- Task B (experiments), Task C (sim-vs-SITL), Task E (hardware research) reports written; 45 tests, CI on Python 3.10–3.12.
- Reported results (**provisional, see section 6**): 1.14 m steady error without feedforward, 0.018 m with; step-fit RMSE 0.1536 m; 3-drone sim-vs-SITL RMS 0.725 m (per drone 0.715/0.724/0.736).
- Self-assessed completion: engine 95%, SITL 65%, benchmarks 90%, hardware 40%. **The reviewer considers the 40% overstated; hardware is untouched** (a research document, not bench work). Honest hardware progress is roughly 10%.

---

## 6. Review findings (all open unless noted)

### 6A. Factual errors in the docs (verify each against official ArduPilot/MAVLink docs)

| ID | Claim in project docs | Reviewer's correction | Confidence |
|---|---|---|---|
| A1 | Type mask `0x0DF8` = "position + velocity" | Set bits mean *ignore*. `0x0DF8` is **position-only**. Position+velocity is `0x0DC0`; velocity-only `0x0DC7`. Code uses `0x0FF8` (position-only, with the FORCE bit set unnecessarily). | High |
| A2 | `FS_GCS_ENABLE = 2` is RTL, `1` is Land | Believed `1` = RTL, `5` = Land; `2` = continue mission in Auto (deprecated) | Moderate-high |
| A3 | `FS_OPTIONS = 32` = "continue in auto but failsafe in GUIDED" | Believed bit 5 = release gripper | Moderate |
| A4 | Setpoint-loss timeout governed by `WP_NAVALT_MIN` | Unrelated parameter. Look at `GUID_TIMEOUT`; believed to apply to velocity/acceleration targets (vehicle brakes), while a position target is simply held | Moderate |
| A5 | 5 GHz router + ESP32 to avoid 2.4 GHz ELRS | Standard ESP32 is 2.4 GHz only | High |
| A6 | "Initial 5-inch PIDs" (`ATC_ANG_PIT_P=4.5`, `ATC_RAT_PIT_P=0.08`, ...) | Do not fly pasted values; use ArduPilot's initial tuning calculator, props off, then autotune. Confirm the airframe (5-inch vs cinewhoop) | High |
| A7 | Brief says hardware "configured" and ArduPilot 4.6 "running" on the Matek | Nothing is flashed or configured; firmware is unknown | High |

### 6B. Documentation inconsistencies to reconcile

- Test count: brief says 15, log says 45.
- Hybrid recovery: brief says "recover within 0.85 s"; Task B table shows 1.65 s (1 s outage), 0.65 s (2 s), 0.34 s (3 s), dominated by the 2.0 s dwell.
- Ramp duration: brief says 0.5 s; `hybrid.py` default and log say 0.8 s.
- Safety radii: Task B config `apf_safe_radius`=2.5 m and `collision_threshold`=0.7 m; log `sitl_fitted` `r_safe`=1.5 m; brief says clearance 2.5 m and APF boundary 1.5 m; code defaults `DecentralizedController.safe_radius`=1.2 m and `CentralizedController.collision_dist`=1.0 m; `Drone.is_colliding_with` computes 0.35+0.35+0.1=0.8 m. Pick one set in `config.py` and print the active values in every experiment summary.
- V geometry: validation script `(-2.5, ±3.0)`; log section 6.3 `(-3.0, ±3.5)`; adapter `spacing=3.5` through `FormationGenerator`.
- Brief claims "a 1.52 m minimum separation safely exceeds our 1.50 m threshold": a 2 cm margin against 1.5–2.5 m GPS error is not safe.
- Brief says algorithms "run identically" in sim and SITL; the project had earlier removed "exact same" wording. Avoid either.
- Stale claims still in the log: "sub-centimeter formation accuracy", "Empirical Breakthrough", "3 weeks ahead of schedule", "N ≥ 5 consecutive heartbeats". Mark superseded. `file:///home/drone/...` links do not work for others.
- README is stale (directory tree omits `sitl/`, `docs/`; quick start lists three test files; no mention of hardware status).

### 6C. Simulation and experiment design (Task B)

| ID | Finding | Evidence |
|---|---|---|
| B1 | **GPS noise does not reach the centralised/hybrid controllers.** | `[code]` `engine.py` computes `measured_positions` only for the peer-broadcast payload. `CentralizedController.compute_control_inputs` and `DecentralizedController.compute_drone_control` have `measured_position(s)` hooks that `engine.py` never passes; `HybridController.compute_hybrid_control` has no such hook and uses `drone.position`. Task B shows Pure Centralized at 0.077 ± 0.000 m for every σ from 0.04 to 2.5 m, and a constant minimum distance. |
| B2 | Common-mode noise does not cancel for decentralised neighbours, because each drone's *own* position is true while neighbours' are noisy. This inflates decentralised error (to ~3 m) unfairly. | `[code]` |
| B3 | Noise is white (fresh `rng.normal` each call). Real GPS error is slowly varying with per-receiver bias; the 1 s lag filters white noise away. | `[code]` |
| B4 | **Gilbert-Elliott advances once per `send()` per recipient, not once per tick.** With n=5, each drone receives 4 peer packets + 1 heartbeat per tick, so the chain steps ~5×/tick. Per-tick persistence of the bad state becomes 0.2+0.8·0.75⁵ ≈ 0.39 instead of 0.8; mean burst ~1.6 ticks instead of 5; chance of a timeout-length burst ~10⁻⁴ instead of ~11%. The reported "0 fallback entries under burst loss" is an artefact. | `[code]` `network.py`, `engine.py`; arithmetic is the reviewer's |
| B5 | **Outage test cannot show hybrid value.** Outages start at t=3 s and last ≤3 s, ending before/at the morph (t=6 s), so hold-last never needs a missed command. In the table, Centralized (hold-last) gets 0.078 m with 0 switches, while the proposed hybrid gets 0.083/0.084/0.130 m. The chattering result is real but only against the naive hybrid. | `[log]` Task B report |
| B6 | `is_in_outage` blocks *all* sends, including peer broadcasts, so during an outage `perceived_neighbors` is empty: the APF has nothing to repel, and the fallback (no neighbours, `goal_pos=None`) brakes to hover. The reported ~1.38 m minimum distance comes from geometry, not active avoidance. Partial loss (e.g. 2 of 5 drones) is impossible to model. | `[code]` |
| B7 | **Hold-last baseline is unrealistic.** It repeats the last commanded *acceleration*, which for a constant-velocity target is roughly drag compensation, so the swarm keeps cruising blind. Real systems hold or stop at the last setpoint. The centralised controller also uses true instantaneous state (no telemetry latency/loss uplink). | `[code]` |
| B8 | **"Pure decentralised" isn't decentralised.** Goal, slots and offsets are computed by the engine from true positions via a global Hungarian assignment, with no comms. It has no drag feedforward and a low `k_goal` (0.8 vs centralised `kp` 1.8); drag alone gives ~c_d·v/k_goal ≈ 0.21 m steady error with perfect comms. Its 0.56 m error is not noise propagation (0.561 m already at σ=0.04). | `[code]` |
| B9 | The hybrid receives information the link does not carry: `target_velocity` and formation offsets come from live engine state, not heartbeat contents. | `[code]` |
| B10 | Double separation in fallback: decentralised `f_sep` at weight (1−α) plus a full APF on top (effective gain ≈ k_sep·(2−α)). | `[code]` |
| B11 | Controller gains sum over neighbours (no degree normalisation), so tuning at n=5 will not carry to n=3 (SITL) or n=10. | `[code]` |
| B12 | Slot assignment is re-run every tick (near-ties can flip slots; error measured against re-optimised slots). Dropped packets remove neighbours entirely for that tick (no memory/age-out). | `[code]` |
| B13 | `recovery_times` stores time since fallback entry (≥ dwell), not recovery after link restoration; `avg_recovery_time_s` is misleading. Recovery rule is an OR of "5 consecutive" and "≥70% delivery" although docs say the former was replaced. `set_coordinator_link(False)` does not record missed heartbeats. Coordinator range check uses distance to the formation centroid. Latency is quantised to dt (0.05 s). | `[code]` |
| B14 | The 1.14 m → 0.018 m result is an **analytical check of the code against its own equations**, not validation. Call it that. | `[log]` + reasoning |
| B15 | Velocity in the peer broadcast is sent true and noiseless while position is noisy. | `[code]` |

**Net assessment:** the hybrid architecture is sound and the state machine is clean, but the current experiments do not yet demonstrate its benefit over simple baselines. The headline experiment still missing is a **ground-link-only outage (peer links alive) of 1–10 s overlapping a morph and a trajectory turn, plus partial-partition cases**, compared against realistic baselines.

### 6D. SITL validation and adapter (Task C)

| ID | Finding | Evidence / check |
|---|---|---|
| C1 | **The 72.5 cm sim-vs-SITL RMS is probably a missing-feedforward artefact.** The SITL side sends position-only setpoints along a 1 m/s ramp; ArduPilot's position loop is P-type, so it should lag a moving target by about v/`PSC_POSXY_P` ≈ 1.0 m. The sim side uses PD with velocity feedforward and lags c_d·v/kp = 0.637·1/1.8 ≈ 0.35 m. The difference ≈ 0.65 m, close to 0.72 m, and similar across all three drones. The brief's explanation ("motor spool-up and EKF lag") is unverified. | `[inference]` Check: plot signed North error vs time (expect a constant lag during the ramp, ~0 East). Fix: send position+velocity (`0x0DC0`) with velocity = centroid velocity; report before/after. |
| C2 | The SITL leg of the validation script never runs `swarm_core`; it sends absolute waypoints. The sim leg hand-codes PD (no APF, no Hungarian, no drag FF) and only reads kp/kd from `CentralizedController`. So it validates a plant model on one ramp, not the controllers. | `[code]` |
| C3 | **Adapter loop gain is ~8× lower than the sim's.** `acceleration_to_position_setpoint` sends P_sp = p + a·dt·γ (= 0.2·a m at dt=0.1, γ=2). Steady speed ≈ `PSC_POSXY_P`×offset ≈ 0.2·a. In the sim, a sustained acceleration gives a/c_d ≈ 1.57·a. "lookahead_scale=2" is documented as lead compensation but acts as a gain. Never compared. | `[inference]` from `[code]`; assumes `PSC_POSXY_P` ≈ 1.0. Fix: velocity setpoints (`0x0DC7`) integrating a_cmd through the same plant as `engine.py`, with a clamp; overlay adapter vs engine on one scenario. `GUID_TIMEOUT` may then act as a free failsafe `[verify]`. |
| C4 | **Frame origin risk.** Setpoint local = global − `home_global_ned`, where home is the first telemetry sample (adapter) or the position at hover time (validation script). ArduPilot's local origin is the spawn point, so any takeoff drift, or connecting to an already-airborne drone, shifts every setpoint. | `[code]`. Fix: origin = `global_ned − local_ned` from simultaneous messages (the adapter already reads `LOCAL_POSITION_NED` but never uses it). |
| C5 | Validation script fabricates telemetry: if no `GLOBAL_POSITION_INT` arrives in a tick, the record is set to the commanded target with zero velocity, biasing RMS downward. | `[code]`. Use NaN and report the count. |
| C6 | One-sample timing offset: sim logs state *after* `step()`, SITL logs latest telemetry, so the sim is ~0.1 s ahead (~0.1 m at 1 m/s). Start conditions also differ (sim starts exactly in slots; SITL has had 4 s to assemble from 5 m/10 m offsets). | `[code]` |
| C7 | **Adapter modes do not test what is claimed.** Centralised sends fresh slot waypoints and bypasses `CentralizedController`; decentralised ignores heartbeats; so packet loss only affects hybrid. Hybrid calls `compute_hybrid_control` without `target_velocity`, so it runs with no feedforward. Peer states are perfect telemetry, not routed through the channel. | `[code]` |
| C8 | Heartbeat age margin: `send` and `receive` occur in the same cycle, so every heartbeat is delivered one cycle later at age ≈ 0.1 s against a 0.15 s stale limit (`max(3·0.03, 0.15)`). Loop jitter can cause false stale rejections and false fallback. | `[inference]` from `[code]` |
| C9 | **Fit script:** τ=0.992 s and c_d=0.637 are lumped parameters that make a PD-driven double integrator imitate ArduPilot's position loop; they are not attitude lag or physical drag, and they depend on the script's kp/kd. Two free parameters, no delay, position-only loss, one 5 m step, no hold-out. The response is also nonlinear (speed/acceleration limits). | `[code]` |
| C10 | **Hardware readiness gaps in the adapter:** no GUIDED-mode check, no arm/takeoff sequence, no setpoint distance clamp (the `w/s/a/d` keys jump 5 m), no stale-telemetry watchdog, no geofence or kill command, `ARMING_CHECK=0` appears in scripts (must not carry to hardware), `centroid_target` mutated across threads without the lock, loop sleeps a fixed 0.1 s ignoring compute time. | `[code]` |

### 6E. What is good (keep)

- Fit-then-validate on *different* scenarios; the 72.5 cm was reported honestly rather than hidden.
- Parameter provenance tags; shared formation code between sim and SITL; 45 tests; CI; packaging.
- Task B's explicit disclosure that chattering reduction is only against the naive hybrid (its own Q3 answer).
- Clean, readable controllers and state machine; α-ramp blending; always-on APF; per-recipient channel state.
- Reproducible validation scripts (`--refit-only`, `--sim-only`, `ARDUCOPTER_BIN`), import guards.

---

## 7. Claims to reword or retract until fixed

1. "Hybrid holds formation via consensus" during outages (neighbours are empty in the outage model).
2. "0 switches under burst loss because bursts <0.5 s are filtered" (channel advanced ~5× per tick).
3. GPS noise tolerance / "safely exceeding 1.50 m" / "common-mode rejection" (noise does not reach the controllers).
4. "72.5 cm due to motor spool-up and EKF lag" (unverified; likely feedforward).
5. "Identical scenario in swarm_core and SITL" / "algorithms run identically" (SITL leg runs no swarm_core).
6. Any present-tense hardware claims (firmware, failsafe parameters, "configured").
7. 99.5–100% chattering reduction (naive baseline only).
8. Avoid scripted "cite these numbers confidently" talking points; students must be able to explain every result themselves.

---

## 8. Consolidated task list for agents

Do tasks in priority order. Each is a small, separate commit.

**P0: Hardware (human-led, this week)**
- H1. Get answers to the unknowns in section 2 (firmware, GPS/compass, ESCs, airframe, reflash permission, spare FC, receiver).
- H2. Bench: back up the Betaflight config (`diff all`), flash ArduPilot Copter (`MatekH743`) on one spare FC, props off, read MAVLink heartbeat.
- H3. ESP32↔ESP32 (ESP-NOW/UDP) loss and range test, then ESP32↔FC over MAVLink UART, then (if still required) CAN with a USB-CAN adapter.
- H4. Ask the supervisor what the CAN receiver is for and whether it is a required deliverable.

**P1: Fix simulator correctness**
1. `network.py`: advance Gilbert-Elliott once per tick per recipient (cache last-update time). Add a test: per-recipient bursts have mean ≈5 ticks and P(burst ≥ 11) ≈ 0.107 with 5 drones. Rerun burst experiments.
2. `engine.py`: give every controller each drone's own *measured* position and an EKF-like velocity (not differenced positions); use true positions for metrics only. Make GPS error time-correlated (Gauss-Markov, correlation time ~30 s; common and independent parts). Rerun the GPS sweep with true-position separation.
3. Split outages: ground-link-only, peer-link-only, per-recipient partitions (`recipients` argument in `add_outage`). Make `set_coordinator_link(False)` record missed heartbeats.
4. Put what controllers use (target velocity, next waypoint, formation spec) in the heartbeat payload; hybrid and decentralised use only the last received values (freeze offsets at last heartbeat).
5. Replace hold-last-acceleration baseline with **hold-last-target** using the same onboard tracker as the hybrid; keep the old one as a labelled extra.
6. Lock slot assignment at morph start; add neighbour memory with age-out.
7. Pass `measured_position/velocity` into every controller call (hooks already exist in centralised/decentralised; add to hybrid).
8. Bounded fallback options: hover-on-loss, hold-last-target, dead-reckon-for-T-then-brake, current behaviour. Compare.
9. Headline experiment: ground-link-only outage of 1–10 s overlapping the morph and a trajectory turn, plus partial partitions of 2 of 5 drones. Report fallback behaviour, path deviation, min separation, time to regain formation.
10. Rename "Pure Decentralized" or make its goals arrive over the channel; add drag feedforward so the comparison is fair.
11. Remove the double APF in fallback; degree-normalise `k_sep`, `k_align`, `k_form`; re-verify at n = 3, 5, 10.
12. Define recovery metrics properly (link restoration → full centralised authority); unify safe-radius/collision constants from `config.py` and print them in every summary; fix docstrings (recovery rule).

**P2: Fix SITL validation and adapter**
13. Validation script: send position+velocity (`0x0DC0`) with velocity = centroid velocity; plot signed North/East error vs time; NaN for missing frames and report the count; align timestamps; one V geometry from `config.py`.
14. Run the real `swarm_core` controllers through the adapter for the same scenario and overlay with `engine.py`; remove the hand-coded PD from the validation script.
15. Adapter: replace `acceleration_to_position_setpoint` with velocity setpoints (`0x0DC7`) integrated through the same plant as `engine.py`, with a clamp; verify closed-loop equivalence on one scenario.
16. Compute frame origin as `global_ned − local_ned` from simultaneous messages; test with a drone that has drifted from spawn.
17. Adapter: route peer states through the channel, pass `target_velocity`, make centralised use `CentralizedController` with hold-last-target, call `receive` after advancing time, add a stale-age margin test at 10 Hz with jitter.
18. Fit script: add a delay term, fit position and velocity, cross-validate on 2 m and 8 m steps, report parameters per `PSC_*` setting; rename to "effective position-loop parameters".
19. Adapter safety layer: GUIDED check, arm/takeoff sequence, setpoint distance clamp, telemetry watchdog, geofence, kill command, logging, dry-run mode; remove `ARMING_CHECK=0` from anything hardware-facing; guard shared state with the lock.

**P3: Report and defence**
20. Claims-versus-evidence table (claim, file, test, result); honest limitations list; examiner Q&A as **questions only** (students write the answers).
21. Update README, log, brief per section 6B and 7; move agent briefs and logs into `docs/`; keep the repo root clean.
22. Regenerate all Task B tables after P1; scale to 5-drone SITL with in-flight morphs through the adapter.

---

## 9. Rules for any agent working on this repo

1. Inspect existing code first and report what you find before changing it.
2. Small changes, one commit per task, with a short description of what and why.
3. No new algorithm features and no cosmetic Gazebo work until P1 and P2 are done.
4. Report raw numbers, including bad ones. If a test fails, report it; **do not tune parameters until it passes.**
5. Report mean, std and number of seeds. No superlatives or "100%" claims.
6. Label every parameter `assumed`, `fitted from SITL`, or `measured on hardware`.
7. For every change, add a plain-English explanation of why, so a student can defend it.
8. For any ArduPilot/MAVLink/DroneCAN claim, cite the official documentation URL and say when unsure. **Do not guess parameter names or bit values.**
9. Do not carry simulator conveniences (`ARMING_CHECK=0`, perfect GPS) to anything hardware-facing.
10. Do not use "the previous agent verified it" as evidence. Verify.

---

## 10. Notes for the human team

- **AI-assisted work:** much of the code and several reports are agent-generated, and agents reviewing agents will share blind spots (the errors in 6A show this). Verify against official documentation, run things yourselves, and make sure every member can explain every result without notes. Check the department's policy on AI assistance and disclose use.
- **Public repo:** consider making it private until submission to avoid copying and similarity-check problems.
- **Timeline sketch (approx., ~3 months):** weeks 1–2 hardware answers, bench flash, ESP32 link, P1 fixes; weeks 3–6 P2 fixes, 5-drone SITL, one real drone in position hold, then GUIDED setpoints; weeks 7–10 two real drones, wide spacing, logging and sim-vs-real overlay; weeks 11–13 report and defence preparation. A clean 2-drone formation with honest analysis beats a 10-drone system that only works in simulation.
- **Spacing:** with plain GPS, scale real-flight formations to 8–10 m.

---

## 11. Reference notes

### Prior department thesis (EZ-Robot JD humanoids)
- Laptop is a star hub over Wi-Fi/TCP; "master-slave" there means the laptop relays the master robot's confirmed state to followers; the robots cannot talk to each other. Its future work lists peer-to-peer, laptop-independent operation and GPS tracking, which this drone project provides.
- Reusable patterns: static IPs on one AP, master-confirm-then-follow, startup sequence with grace period, watchdogs, global emergency stop, measuring command latency (42 ms DLL vs 67 ms bridge).
- Do not copy: "100% success" and "94% detection" without run counts or definitions; inconsistent IPs/roles in its documentation; TCP for time-critical setpoints; cloud AI calls in the loop.

### MAVLink bit masks (`SET_POSITION_TARGET_LOCAL_NED`, set bit = ignore) `[verify]`
- Position-only: `0x0DF8` (bits 3–8 ignore vel/accel, 10–11 ignore yaw/yaw-rate)
- Position + velocity: `0x0DC0`
- Velocity-only: `0x0DC7`

### Reported hybrid parameters (`config.py`/log; `hybrid.py` defaults differ in places)
- `degrade_timeout` 0.5 s; `min_dwell_time` 2.0 s; sliding window 20 ticks, ≥70%; `ramp_duration` 0.8 s (`hybrid.py`); `max_command_age` = max(3·latency, 0.15 s); `recovery_consecutive_hb` 5 (still in code).

### Reported results to treat as provisional until P1/P2 are rerun
- Outage/burst/GPS sweeps (Task B), steady error 1.14 → 0.018 m, step fit τ=0.992 s / c_d=0.637 / RMSE 0.1536 m, 3-drone RMS 0.725 m.

### Reviewer's open-question list
- `config.py`/`build_controllers_from_profile`, `formations.py`, `common_frame.py` (to confirm B-series safe radii and C4 frame semantics), `swarm_params.parm` (which `PSC_*` values are in force), the real firmware/GPS state of the drones, and the supervisor's intent for the CAN receiver.
