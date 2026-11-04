# Project state — start here

**Last verified:** 2026-11-04, branch `wip/decentralized-sitl-handoff-20261029`,  unit/integration tests collected, all passing with no simulator running (`python3 -m pytest -q tests`).
**Rule for this file:** it is the single entry point. If it disagrees with an older document, this file wins; if it is wrong, fix it in the same commit that changes the facts.

Any assistant (Claude, Codex, or other) or teammate should read, in order:
1. this file
2. `docs/history/agent-handoffs/CLAUDE_CODEX_COLLABORATION_2026-10-29.md` (live work log — append, never rewrite)
3. `docs/SOFTWARE_ACCEPTANCE_MATRIX.md` (what counts as done) and `docs/AGENT_PROTOCOL.md` (message formats)

## 0. If you are an AI agent (Codex, Claude, or other): do this first

1. Read this file fully, then the collaboration log's rules and its **latest entry** (the "next step" there is your starting task).
2. Verify before trusting: `git rev-parse --abbrev-ref HEAD && git rev-parse HEAD && git status --short`, `df -h /`, and `python3 -m pytest -q tests`. If branch, HEAD or status differ from section 7's "Last known state", **stop and report the difference** before editing.
3. Check no other assistant is currently writing to this checkout. Only one writer at a time.
4. Work from the plan in section 8, one focused commit per fix. Describe results with the evidence labels (UNIT … PHYSICAL); never present mocked tests as SITL evidence.
5. **Before you finish, update the docs** (see "How to keep this file current" below) and append a session entry to the collaboration log. Leave the tree clean where practical.
6. Reviewer role (Codex while Claude writes): read the diff and the latest log entry, run the tests, and add review findings to the log as a new entry; do not edit code unless the user hands you the writer role.

### How to keep this file current

Update this file **in the same commit** whenever a change alters any of: branch/HEAD or test count (section 7), a defect's status (section 5), an evidence level or objective status (section 4), file/folder locations (section 2), or the plan (section 8). Rules:
- Change facts, don't append commentary. Delete a defect from section 5 when fixed and note it in the changelog at the bottom with the commit hash.
- Keep "Last verified" at the top accurate (date, branch, test count). Only write a status you have actually checked; mark anything unverified as such.
- Raise an objective's status only with the evidence label that supports it, and cite the report or log path.
- Don't copy long content here; link to the report. Keep this file under ~250 lines.
- Session-by-session detail belongs in `docs/history/agent-handoffs/CLAUDE_CODEX_COLLABORATION_2026-10-29.md`, not here.

## 1. The project in one paragraph

Final Year Project BEE-60 (Dept. of Electrical Engineering, supervisor Dr. Abdul Ghafoor, deadline 31 Dec 2026): software for a swarm of multirotors flying line/V/ring/grid-style formations under **centralized**, **decentralized** and **hybrid** control, validated in a numerical simulator, ArduPilot SITL, Gazebo, and across a host/VM network boundary. Target scale 5–10 vehicles. Physical flight is a stretch goal and has not happened.

**Accurate wording for the current formation work:** distributed *execution* (each drone's agent flies its own assigned trajectory) with *centralized* transition planning. Do not claim decentralized formation negotiation yet.

## 2. Where things live

| What | Location | Rule |
|---|---|---|
| **Active development checkout (this repo)** | `/home/drone/Documents/Codex/FYP_Consolidated_2026-10-12/working-copies/decentralized-sitl-20261020`, branch `wip/decentralized-sitl-handoff-20261029` | Only place code is edited. One writer at a time. |
| Frozen demo baseline | `.../working-copies/demo-master` (`master`) | Read-only. Defense fallback. |
| Research lines (read-only archives) | `.../working-copies/hw-matched-identification`, `identification-phase2-20261020`, `experimental-hybrid-20261020` | Do not edit. Identification results are preliminary. |
| History shelf (checkpoints, backups, dependencies, references) | `/home/drone/Documents/Codex/FYP_Consolidated_2026-10-12/` (not a Git repo) | Archive only. Index: `START_HERE.md` there + `docs/history/ARCHIVE_INDEX.md` here. |
| Heavy evidence (ArduPilot `.BIN` logs, raw run output) | `/media/drone/SSDSAM/FYP_Archive/` (NTFS external SSD) | Checksummed; see archive index. |
| SITL VM (Ubuntu 24.04, QEMU/KVM) | `/media/drone/SSDSAM/drone-swarm-vm/` | Single copy; not yet backed up. |
| Matek H743 parameter exports and diagnostics | `hardware/` (repo; Oct 23 export and summary copied in 2026-10-29) | Do not change hardware files unless assigned. |
| GitHub | `https://github.com/radahnnn/autonomous-swarm-drones-fyp` | Do not push/merge/PR without the user's instruction. |

Git branches (remote): `master` (demo), `work/decentralized-sitl-20261020` (parent of this branch, `cb7fdcc`), `wip/decentralized-sitl-handoff-20261029` (this branch, WIP checkpoint `d1e7444`), `work/corrected-phase2-20261020`, `hw-matched-identification`, `archive/*`, `checkpoint/2d-working-savepoint`. This branch is 24 commits ahead of / 4 docs-only commits behind `origin/master`; README conflicts are expected on merge.

## 3. Code map (this checkout)

- `swarm_core/` — numerical foundation: config profiles, drone model, graph, network emulator, formations, the shared Hungarian assignment (`assignment.py`), wind model, controllers, safety.
- `simulator/` — fast pure-Python engine and plots.
- `sitl/` — live system (ArduPilot SITL / Gazebo). Independent runtime (used by all current work):
  - console: `independent_swarm_console.py` (lifecycle, status, operator REPL) + mixins `console_clock.py`, `console_protocols.py` (mode switch, formation transitions, settling), `console_mission.py` (patrol, goto, transit + tower gap, capture, geofence), `console_metrics.py`;
  - agent: `independent_agent_node.py` (MAVLink I/O, guidance, status) + mixins `agent_safety.py` (barrier neighbours, estimate health, geofence, obstacles, peer failsafe), `agent_protocols.py`, `agent_mission.py` (leases, election, coordinator-loss policy, routes, membership);
  - pure logic: `transition_protocol.py`, `mode_switch.py`, `coordinator_election.py`, `cbf_filter.py`, `estimate_health.py`, `obstacle_sensing.py`, `gap_manoeuvre.py`, `route.py`, `geofence.py`, `formation_library.py`, `formation_transition.py`, `adaptive_membership.py`, `run_metrics.py`, `clock_budget.py`, `separation_geometry.py`;
  - tools: `swarm_console.py` (entry point), `fleet_ctl.py` (scripted scenarios), `operator_dashboard.py`, `replay.py`, `generate_gazebo_world.py`, `gz_camera.py`, `patrol_layout.py`, launch scripts.
  - Older runtimes still present (`--agent-runtime local|process`: `decentralized_guidance.DecentralizedGuidance`, `agent_process.py`, `peer_state_bus.py`, `mode_supervisor.py`, `mavlink_swarm_adapter.py`); retiring them is pending the user's decision.
- `experiments/` — sweeps and validation scripts. `scenarios/` — scripted live missions for `sitl.fleet_ctl`.
- `tools/parameter_table.py` → `docs/PARAMETERS.md` (constants, `*Params` fields and selected constructor defaults; not CLI defaults or profile dictionaries; a test keeps it current).
- `docs/history/legacy-scripts/` — 13 archived early scripts (not maintained).
- `tests/` —  tests (UNIT/INTEGRATION; they do not prove live behaviour); CI also runs `pyflakes` (0 warnings).

## 4. Progress against the proposal (evidence-labelled)

Labels: UNIT, INTEGRATION, SITL, QGC, GAZEBO, VM-NETWORK, PHYSICAL (see collaboration log). No completion percentage is given (Codex review 2026-11-03: it had no stated basis); see the rows and the open items in section 8.

| Objective | Status | Highest evidence | Gap |
|---|---|---|---|
| Mathematical model / simulator | Done | UNIT/INTEGRATION | Fitted SITL parameters are preliminary; old τ=0.992 s fit was a timestamp artifact; S4 cascade model not integrated into Phase 2. |
| Centralized control | Working | SITL, GAZEBO (5 drones, 20 Oct), QGC (15 Oct) | Repeated matched live runs. |
| Decentralized control | Working in SITL | SITL (same host), VM-NETWORK (5 and 10 agents, one clean mission each) | Repeated split-host fault runs; radio. |
| Hybrid switching + in-flight mode switching | Working in SITL | SITL (same host; 10 drones, 5 live switches, 2026-10-30) | Repeated live and separate-host evidence. Hybrid violated the 2.5 m guard in 1 of 10 burst-outage seeds. |
| Formations + reconfiguration | Working in SITL/Gazebo | SITL (10 drones; synchronized transition protocol, pairs, tower-gap column), GAZEBO (6 drones) — single runs | Repeats; under `--realistic` some transitions are cancelled/retried; Codex review findings 1-2, 4, 6, 8 open. |
| Gazebo 6-drone, independent runtime | Working (single runs) | GAZEBO: survey patrol under `--realistic` PASS 4.81 m (2026-11-03); tower gap with real towers PASS 3.42 m (2026-11-03); RTF ~0.19 at the 1.25 ms step | One run each; no 10 drones / wind in Gazebo; see `docs/reports/REALISTIC_SIMULATION_2026-11-03.md`. |
| Scaling 5→10 | Partial | 30-seed point-mass pass (2 mm worst margin); one 10-agent VM mission | Thin margin; repeated live/fault runs. |
| Physical prototype | Bench-checked only | PHYSICAL (USB bench: FC, sensors, GPS fix; props off) | Never flown. ESP32/CAN link is a proposal; CAN enabled in params but no command path confirmed. |

## 5. Known defects and risks (current)

Tracked as the backlog in `docs/history/agent-handoffs/CLAUDE_ONE_WEEK_PROMPT_2026-10-29.md`. Confirmed on 2026-10-29 (the stale/out-of-order `MissionIntent` defect was fixed in the same session; see changelog):

1. **Runaway log on dropped SITL link** — run `external-sitl-20261022T193321` wrote ~54 M lines/agent (4.6 GB) of `EOF on TCP socket`; no backoff or log limit in the retry path.
2. **CI cannot install pymavlink** — `.github/workflows/ci.yml` installs `.[dev]` only, while tests import `sitl`. Python 3.10–3.12 matrix is unverified.
3. Synchronized transition protocol (`sitl/transition_protocol.py`, agent + console wiring, opt-in `--transition-protocol`): UNIT + in-memory INTEGRATION tests, and **one live 5-drone standalone SITL run** (`docs/reports/TRANSITION_PROTOCOL_LIVE_SITL_2026-10-30.md`) in which a LINE transition completed and a COLUMN transition with a killed agent ended in a coordinated HOLD (no LAND). The run exposed and fixed two bugs the tests missed. Not repeated, no loss/latency/coordinator-kill/VM runs, not the default. Remaining limits: agent aborts reach peers through the coordinator relay; command gating covers REPL/dashboard `handle` and `_set_formation`.
4. Dashboard: `session.count` vs `SwarmConsole.n` mismatch; commands lack structured results.
5. Mode supervisor has no child-readiness check or rollback; process cleanup is broad rather than ownership-based.
6. ~~Several slot-assignment implementations~~ — the Hungarian cost/latching code is now one function (`swarm_core/assignment.py`) used by the simulator, library and planner (2026-11-01). Formation *shape* generators are still two sets (`swarm_core/formations.py` for the simulator, `sitl/formation_library.py` for live); merging them would change recorded simulator results, so it is deliberately not done.
7. Repo hygiene: some raw `.log` evidence files under `docs/reports/evidence/VM_SPLIT_*` are tracked in Git, against the "keep raw logs out of Git" rule.
8. Algorithm review (`docs/reports/ALGORITHM_REVIEW_2026-10-29.md`): guard priority, demo label, repulsion, assignment latching, recovery-rule pin and the hybrid defaults (`hybrid_degrade_timeout` 1.2 s, fallback `dead_reckon`) are done (numerical simulation only). **Closed 2026-10-31:** missions can now continue without a coordinator when the swarm carries a route (`sitl/route.py`; live SITL, see `docs/reports/ROUTE_CONTINUATION_LIVE_SITL_2026-10-31.md`). **Open:** altitude staggering, gain sweeps, SITL/hardware validation of all of it.
9. **Membership-change near-collisions (found 2026-10-31, mitigated 2026-11-01):** after a drone's agent died, rank-based slot redistribution sent survivors across each other (min gap 0.10 m). Barrier filter (`sitl/cbf_filter.py`) + minimum-motion Hungarian slot reassignment (`sitl/adaptive_membership.py`): live 10-drone SITL, one dead agent (the V apex), min gap **3.62 m, PASS** (`docs/reports/SAFETY_FILTER_AND_MEMBERSHIP_LIVE_SITL_2026-10-31.md`). One run per configuration; other losses / two losses / rejoin are UNIT only.
11. **Position error dominates safety (SIMULATION, 2026-11-01):** with 1.5 m GPS error all three modes fell to 0.4-0.8 m median minimum gap with collisions in about half of 30-seed trials; wind of 3-5 m/s changed little (`docs/reports/STRESS_AND_SENSITIVITY_2026-10-31.md`). The 2.5 m gate and 2.8 m barrier distance are not justified for plain GNSS; re-derive from measured error (`docs/HARDWARE_READINESS_CHECKLIST.md`, gate 3.3).
10. Data at risk: the VM image has a single copy on an external NTFS drive; the VM's fallback console password is in its setup note.

## 6. Hardware facts

Verified over USB on 2026-10-06 (`hardware/README.md`, bench, propellers off, disarmed): Matek H743-SLIM V3 running **ArduCopter 4.6.2**; gyro, accel, compass, baro healthy; GPS healthy with a 3D fix (23 satellites); X-frame quad, DShot600. Expected pre-arm notices: RC not found, 3D accel calibration needed. The Oct 23 parameter export (`hardware/params/matek_h743_quad_20261023.param`) enables the CAN port (`CAN_P1_DRIVER=1`, DroneCAN node 10) — this is *configuration only*; no CAN message path to a companion has been demonstrated.

On 2026-10-29 the real ESP32 board enumerated over USB as a CP2102 UART bridge (`/dev/ttyUSB0`) (**PHYSICAL, passive enumeration only**). No Linux CAN interface was exposed, the Matek was not attached by USB, and no serial port or CAN traffic was opened/read. The active test stopped because propellers were installed and the flight battery was connected. The Oct 23 export further records DroneCAN at 250 kbit/s and `CAN_D1_UC_SER_EN=0`; node 10 is the autopilot's DroneCAN node ID, not evidence of an ESP32 node or MAVLink command channel. Detailed evidence and the propellers-off plan: `hardware/reports/PHYSICAL_ESP32_CAN_BENCH_PLAN_2026-10-29.md`.

Not verified / open:
- No physical flight has been attempted; RC link and accel calibration not done. One quad is assembled; the department has 10 drones, the team has 2.
- ESP32 role (MAVLink relay vs onboard agent) and the link between the ESP32 and the flight controller (UART vs CAN) are undecided. An MCP2551 alone is only a CAN transceiver.
- Current recommendation: prove a connectorized UART/MAVLink command path first if an approved no-solder harness is available; validate CAN separately as an experimental DroneCAN link. This is a design recommendation, not a verified physical path.
- Free ports per the port-mapping PDF: UART4/6/7/8, I2C1/2, one CAN (from notes, not re-checked on the board).
- Department fleet count remains team-reported (10 total, team has 2); no per-aircraft inventory has been completed. Inventory and bench-check each aircraft before extrapolating results.
- Hardware files are configuration records, not approval to fly. Do not modify them unless assigned.
- A propellers-off bench test of the command path is planned (target: week of 2026-10-30), run in a separate Codex hardware chat.
- Details: `hardware/README.md`, `docs/HARDWARE_SOFTWARE_MAPPING.md`, `docs/PHYSICAL_NETWORK_VALIDATION_PLAN.md`; the 30 Sep / 3 Oct notes in `docs/history/agent-handoffs/` are *superseded* where they say firmware/GPS are unknown.

## 7. Working agreements

**Last known state:** branch `wip/decentralized-sitl-handoff-20261029`, pushed to origin (check `git log -1` against GitHub; auth via the `gh` CLI login on this PC). Older checkpoint tag: `wip-checkpoint-20261029`.

- Claude is primary writer for the week of 2026-10-29; Codex reviews. One writer at a time. Log every session in the collaboration log.
- Focused commits; clean tree at handoff; no push/merge/PR/history rewrite without instruction.
- Never `git reset --hard`, `git clean`, force-push, auto-rebase, broad `pkill -9 -f`. Don't delete runs, logs, evidence, backups, branches or hardware files.
- Check disk (`df -h /` — about 35 GB free after the 2026-10-29 move) before full tests, SITL, Gazebo, VM runs. Confirm process/port ownership first.
- Never present mocked/unit results as live SITL evidence.

## 8. Plan

Rule set by the user (2026-11-03): this section lists only what the user asked for or what testing found. Agent
recommendations are NOT written here; new items are added only after consulting the user, and results are recorded
only once tested and proven (with their evidence label). Hardware is on hold by the user's decision.

### Not done yet (open items, 2026-11-03)

Requested by the user, not finished:
- Leader-decided tower-gap / formation decisions (today the console decides).
- Gazebo: trees as real obstacles (now scenery only); wind in Gazebo.
- Campus world (Military College of Signals, Rawalpindi; EE and SE departments) — waiting for the user's photos.
- Hardware (on hold): propellers-off bench test of the PC/ESP32 -> Matek link; RC + failsafes; first hover; then
  two drones; then the 5 new ones. Gated by `docs/HARDWARE_READINESS_CHECKLIST.md`.

Codex review findings accepted by the user for fixing (2026-11-04; details in the collaboration log entry "2026-11-03 22:20 PKT — Codex"):
- #1 mode-switch partial commit can split modes; #8 transition self-close can strand a lagging drone (terminal-decision agreement).
- #2 console mission state has several writers (duplicate sequences, stale retries after LAND, retries lose gap heading/spacing); #4 formation start time taken before waits; #11 dashboard reports rejected commands as accepted.
- #3 safety filter is not the last step on the transmitted target; #7 barrier solver misses circle-boundary solutions; #6 gap check uses tower spacing, not the swept path; #9 LAND cannot interrupt arming/takeoff; #10 acting-coordinator limit renews itself.
- #5 run-report PASS does not require fleet coverage, mission completion or fresh positions; true simulator positions not logged.
- Done 2026-11-04: #12 CI extra, #13 vacuous test, #14 parameter table coverage/label, #15 this file, #16 report wording.

Found by testing, not yet resolved:
- Under `--realistic`, ~1 run in 3 dips below the 2.5 m gate (true worst 1.7-2.0 m, final campaign 2026-11-04), driven by ArduPilot EKF3 yaw resets/flyaways (see `docs/reports/OVERNIGHT_CAMPAIGN_2026-11-04.md`); calm runs 3/3 PASS.
- Formation changes require all drones airborne: after the flyaway guard lands a drone, the remaining ones cannot change formation (partial-fleet planning is a design decision for the user).
- Realistic 10-drone 2D mission: recurring near-miss in the decentralized patrol (~190 s): 0.92 m, 2.48 m, then 0.995 m after the refactor (2026-11-03). Separation is from estimated positions, so not yet distinguishable from estimate error (Codex #16).
- Battery endurance cannot be simulated with SITL's generic quad (drain disabled); must be measured on hardware.
- Disturbance values in `--realistic` are assumptions until measured on the team's drones.
- Most results are single runs.

Waiting on the user:
- Drone details: battery (cells, mAh), GPS module, who flies RC, RC receivers per drone, drone-to-PC link.
- Campus photos; the demo date; the idea the user mentioned and has not shared yet.

## Changelog (newest first; one line per change to this file)

- 2026-11-04 — Claude — Overnight: root cause of realistic near-misses (EKF yaw resets from SITL GPS position-only wander + wind), GPS wander 0.5 m, onboard flyaway guard, gap re-plan, 4 repeat campaigns + Gazebo re-check; report `docs/reports/OVERNIGHT_CAMPAIGN_2026-11-04.md`.

- 2026-11-03 — Claude — Code efficiency (no behaviour change): 13 unused scripts archived; console and agent split into mixin modules; repo lint-clean (pyflakes in CI); generated parameter table.

- 2026-11-03 — Claude — Removed the agent's 10-step recommendation plan at the user's request; section 8 now holds only user-requested or test-found items; new items only after consulting the user.

- 2026-11-03 — Claude — Added the open backlog ("Not done yet") under section 8.

- 2026-11-03 — Claude — Tower-gap manoeuvre PASS in 2D (10) and Gazebo (6, real towers) under --realistic after widening the gap column (8 m), slower pass, 180 s cap, refusal handling.

- 2026-11-03 — Claude — Estimation-health guard, transient-cancellation retry, dashboard upgrade (mission buttons, camera gallery, sensed obstacles, per-drone status, realism badge), tower-gap manoeuvre (`transit`). Gazebo 6-drone survey patrol under --realistic PASS (4.81 m); 10-drone tower gap works up to the V re-form but FAIL (0.85 m between column members). See the report update.

- 2026-11-03 — Claude — `--realistic` simulation + attribution matrix (GPS error is the disturbance that matters); fixed: Gazebo arming (1.25 ms step), position-sigma tuning, formation scale 1.4, settle-before-plan, battery model (drain off, endurance to be measured on hardware), run report fails on ground contact. Final realistic 10-drone run: 3/4 formation changes completed, FAIL from one estimate-glitch near-miss -> estimation-health guard is next. Report `docs/reports/REALISTIC_SIMULATION_2026-11-03.md`.

- 2026-11-01 — Claude — Gazebo survey-patrol built (UNIT-tested, NOT yet flown end to end): drone cameras + PNG capture (`sitl/gz_camera.py`, console `capture`), ground/markers/poles/trees in generated worlds, simulated range sensing of obstacles shared over the peer link (`sitl/obstacle_sensing.py`), world-anchored `pairs`/`pairs_b` formations, `goto`, `--time-scale`, 2.5 ms physics step (RTF 0.14 -> 0.3-0.4), `sitl/patrol_layout.py`, `scenarios/gazebo_patrol.txt`. **Open blocker:** in the patrol world the drones would not arm: "Arm: Gyro 0 rate 403Hz < loop rate*1.8 720Hz" (run 4 at the same 2.5 ms step armed fine; cause unknown). Trees are scenery only; wind not in Gazebo yet.
- 2026-11-01 — Claude — Barrier filter + minimum-motion slot reassignment validated live (defect 9: min gap 0.10 -> 3.62 m, PASS); added wind model + 30-seed stress sweep, parameter-sensitivity sweep, min-jerk smoothing consolidation, clock-sync error budget (leads that cannot cover it are refused), ellipsoid separation gate, single assignment function, scenario runner `sitl/fleet_ctl.py`, hardware-readiness checklist; fixed relative `--evidence-dir` start failure. Test count 671. Gazebo 6-drone: world generator N/E swap fixed + 2.5 ms physics step (RTF 0.14 -> 0.3-0.4); run 4 PASS (3.22 m). Remaining: repeat Gazebo runs, routes/kill in Gazebo, 10 drones.
- 2026-10-31 — Claude — Acting-coordinator failover (`sitl/coordinator_election.py`) validated live with 10 drones (centralized survives operator loss; leader death hands over with a higher term); live run exposed near-collisions after a membership change (min gap 0.10 m among survivors) -> new defect 9; cached-intent expiry bug fixed; roadmap features 2, 3, 4, 5, 7 of 7 done (remaining: control-barrier filter, wind/GPS stress tests). Test count 618.
- 2026-10-31 — Claude — Mission routes (`sitl/route.py`, `patrol`/`route` commands): decentralized and hybrid swarms keep patrolling without a coordinator, centralized holds; live run exposed and fixed a route-speed vs setpoint-slew flaw (cap 0.4 m/s); roadmap features 2, 4, 5, 7 of 7 done (remaining: coordinator election, control-barrier filter, wind/GPS stress tests). Test count 595.
- 2026-10-31 — Claude — Geofence + altitude layers (`sitl/geofence.py`), operator altitude floor (2 m), metrics/replay validated live with 10 drones; roadmap features 4, 5, 7 of 7 done (remaining: mission continuation, coordinator election, control-barrier filter, wind/GPS stress tests). Test count 573.
- 2026-10-31 — Claude — Added live run-metrics panel + run report (`sitl/run_metrics.py`) and read-only replay (`python3 -m sitl.replay`); roadmap features 4 and 5 of 7 done (remaining: geofence + altitude staggering, mission continuation, coordinator election, control-barrier filter, wind/GPS stress tests). Test count 559.
- 2026-10-30 — Claude — In-flight synchronized control-mode switching (`sitl/mode_switch.py`, console `mode <name>`, dashboard buttons) validated live with 10 drones; per-mode coordinator-loss policy; 10-drone ring/CI/link-loss hardening; research report added; test count 543.
- 2026-10-30 — Claude — First live SITL run of the transition protocol; two fixes (peer memory, goal drift/pre-start abort); report + compact evidence added.
- 2026-10-30 — Claude — Wired the transition protocol into agent and console (opt-in flag) with INTEGRATION tests; defect 3 updated; test count 417.
- 2026-10-30 — Claude — Hybrid defaults changed (timeout 1.2 s, fallback dead_reckon) on sweep evidence; defect 8 updated; test count 412.
- 2026-10-30 — Claude — Algorithm fixes 3.1/3.2/3.6 committed (`17ad625`); hybrid mixed-mode diagnosis recorded; test count 409.
- 2026-10-29 — Claude — Added `docs/reports/ALGORITHM_REVIEW_2026-10-29.md` (numerical-simulation evidence) and its findings as defect 8.
- 2026-10-29 — Claude — Added `sitl/transition_protocol.py` + 41 UNIT/simulated-channel tests (not yet wired into the runtime). Test count 359.
- 2026-10-29 — Claude — Added `docs/TRANSITION_PROTOCOL_SPEC.md` (draft); sharpened defect 3 with the verified gating gap.
- 2026-10-29 — Codex — Recorded PHYSICAL passive ESP32 USB enumeration, clarified what the Oct 23 DroneCAN settings do not prove, selected UART/MAVLink as the first command-path recommendation, and linked the propellers-off CAN/fleet bench plan.
- 2026-10-29 — Claude — Fixed stale/out-of-order `MissionIntent` handling in `sitl/independent_agent_node.py` (sequence check moved before any state change); added `tests/test_mission_intent_ordering.py` (UNIT). Removed the defect from section 5.
- 2026-10-29 — Claude — Corrected hardware facts (firmware ArduCopter 4.6.2 and GPS fix were bench-verified Oct 6; CAN enabled in Oct 23 params); copied the Matek Oct 23 export and summary into `hardware/`.
- 2026-10-29 — Claude — Created. Consolidated scattered project state; archived the Antigravity checkout and moved heavy logs/runs to the SSD (see `docs/history/ARCHIVE_INDEX.md`).
