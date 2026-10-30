# Claude–Codex Development Log

## Purpose

This file is the shared record for the one-week Claude development period beginning 2026-10-29. Claude is the primary code writer. Codex reviews changes, tests, risks, and integration progress when requested. The user decides when work is committed, pushed, merged, or demonstrated.

## Repository baseline

- Repository: `radahnnn/autonomous-swarm-drones-fyp`
- Working branch: `wip/decentralized-sitl-handoff-20261029`
- Parent development commit: `cb7fdcc1e1537edea08a7d3ea976d671c34733e1`
- WIP checkpoint commit: `d1e7444`
- Verified external backup: `/home/drone/Documents/Codex/preserved-software-20261027`
- The checkpoint is preserved development work. It is not release-ready and must not be merged into `master` without review and testing.

## Collaboration rules

1. Only one assistant writes to this checkout at a time.
2. Claude records every meaningful work session in the log below.
3. Each entry states the exact files changed, tests run, results, known limitations, commit hash, and next step.
4. Do not claim success from mocked tests alone. Distinguish unit, integration, SITL, QGroundControl, Gazebo, VM, and physical-hardware evidence.
5. Do not delete evidence, logs, backups, branches, or hardware files.
6. Do not use `git reset --hard`, `git clean`, force-push, broad `pkill -9 -f`, or automatic rebasing.
7. Do not merge `origin/master` casually. The development branch diverges and README conflicts are expected.
8. Keep raw runs, generated logs, caches, keys, and large evidence files out of commits.
9. Prefer focused commits. Do not hide known failures in commit messages or handoffs.
10. Before live SITL or full tests, check disk space and confirm no other session owns the relevant processes or ports.

## Current engineering priorities

1. Reject stale or replayed `MissionIntent` messages before mutating agent state.
2. Add regression tests for stale and out-of-order mission messages.
3. Design a prepare/acknowledge/common-start/abort protocol for formation transitions.
4. Add live transition supervision to the central/local runtime: telemetry freshness, armed state, separation, timeout, abort, actual arrival, and dwell.
5. Define safe behavior for peer-link loss, membership changes, coordinator loss, and failed agents during a transition.
6. Fix the dashboard session interface and structured command-result reporting.
7. Add mode-supervisor readiness checks and rollback.
8. Correct clean-CI dependency installation for Python 3.10–3.12.
9. Replace broad process cleanup with ownership-based shutdown in a separate change.
10. Validate five- and ten-drone centralized, decentralized, and hybrid operation after unit and integration tests pass.

## Evidence language

Use one of these labels for every result:

- `UNIT`: isolated functions/classes, mocks permitted.
- `INTEGRATION`: multiple real project components, simulated transport permitted.
- `SITL`: real ArduPilot SITL instances.
- `QGC`: observed in QGroundControl.
- `GAZEBO`: observed in Gazebo.
- `VM-NETWORK`: traffic crosses the host/VM boundary.
- `PHYSICAL`: real flight controller, radio, or airframe evidence.

## Session log template

Copy this block for every work session. Append new entries; do not rewrite old results.

```markdown
### YYYY-MM-DD HH:MM PKT — Claude

- Objective:
- Starting commit:
- Files inspected:
- Files changed:
- Design decisions:
- Tests and evidence level:
  - Command:
  - Result:
  - Evidence label:
- Failures or unresolved risks:
- Commit created:
- Pushed branch/commit:
- Recommended Codex review:
- Next step:
```

## Session log

### 2026-10-29 — Codex checkpoint and handoff preparation

- Objective: Preserve the 13 unpublished source/test files as a Git checkpoint and establish a shared collaboration record.
- Starting commit: `cb7fdcc1e1537edea08a7d3ea976d671c34733e1`
- Files changed: the reviewed 13-file WIP change set was committed without modifying its contents; this collaboration log was then added separately.
- Verification:
  - Repository, branch, remote, status, diff, and backup checksums were verified.
  - Backup checksum verification passed for all 13 files and both patch files.
  - `git diff --check` passed before the checkpoint.
  - Available root filesystem space was approximately 26 GiB at the time of checkpointing.
- Tests and evidence level: no full or live test suite was run during checkpoint creation.
- Checkpoint commit: `d1e7444`
- Known status: WIP; critical correctness and runtime-safety issues remain as listed above.
- Next step: Claude performs a read-only baseline inspection, then fixes stale/out-of-order mission validation with focused regression tests.

### 2026-10-29 — Claude — read-only baseline inspection and repository/machine cleanup

- Objective: verify the handoff baseline, consolidate scattered project context, free disk space, and make the repo self-describing for any assistant. No source code changed.
- Starting commit: `c97c56308eea9dcc1e4f9097170ab73c7c6b02ed` on `wip/decentralized-sitl-handoff-20261029`, clean tree, remote `origin` = radahnnn/autonomous-swarm-drones-fyp.
- Files inspected: `sitl/independent_agent_node.py` (mission handler), `.github/workflows/ci.yml`, `pyproject.toml`, `README.md`, `docs/SOFTWARE_ACCEPTANCE_MATRIX.md`, `runs/README.md`, the FYP_Consolidated shelf docs, the free-Claude notes, the SSDSAM archive manifests.
- Files changed (docs only): added `docs/PROJECT_STATE.md`, `docs/history/ARCHIVE_INDEX.md`, root `AGENTS.md` and `CLAUDE.md`; copied two free-Claude notes into `docs/history/agent-handoffs/`; edited `docs/history/agent-handoffs/README.md` and `README.md` (pointer lines).
- Findings:
  - First defect CONFIRMED: `independent_agent_node.py` ~lines 405–441 mutates formation/slot/transition state before the `intent.sequence <= self.last_mission_sequence` check. Not fixed yet.
  - CI installs `.[dev]` only; `pymavlink` (extra `sitl`) is missing although tests import `sitl`.
  - Run `external-sitl-20261022T193321` produced ~54 M "EOF on TCP socket" lines per agent (4.6 GB): unbounded retry/log loop on a dropped SITL link.
  - Some raw `.log` evidence under `docs/reports/evidence/VM_SPLIT_*` is tracked in Git.
- Machine actions (non-repo, all verified before removal):
  - Verified `/media/drone/SSDSAM/FYP_Archive/ArduPilot_Logs_2026-10-29` with `sha256sum -c` (72 objects OK).
  - Archived the Antigravity checkout (git bundle of all refs, ignored logs, interim-report drafts; logs/reports compared identical) to `SSDSAM/FYP_Archive/Antigravity_Original_2026-10-29/`, then removed `~/.gemini/antigravity/scratch/swarm_drones_fyp` at the user's instruction. Its two tarballs were hash-identical to shelf copies.
  - Moved the five largest raw runs (~5.9 GB) and 501 `logs/*.BIN` (2.5 GB) to `SSDSAM/FYP_Archive/Repo_Runs_Logs_2026-10-29/` with SHA-256 manifests; the runs were replaced by symlinks (excluded in `.git/info/exclude`). Root disk free: 26 GB → 35 GB.
- Tests and evidence level:
  - Command: `python3 -m pytest -q tests` — 312 passed. Evidence label: UNIT/INTEGRATION only.
- Failures or unresolved risks: all backlog items remain open; VM image has a single copy on the external NTFS SSD; Matek parameter exports in `~/Documents/Codex/2026-10-12/...` are outside Git.
- Commit created: docs cleanup commit (see `git log`); local tag `wip-checkpoint-20261029` on `d1e7444`.
- Pushed branch/commit: nothing pushed.
- Recommended Codex review: accuracy of `docs/PROJECT_STATE.md` sections 4–5 against the code and reports; confirm the stale-intent diagnosis.
- Next step: fix stale/out-of-order `MissionIntent` handling (validate before any mutation) with regression tests in `tests/test_decentralized_sitl.py`; then the transition-protocol spec.

### 2026-10-29 — Claude — stale/out-of-order MissionIntent fix

- Objective: validate `MissionIntent` sequence before any agent state mutation (backlog items 1–2).
- Starting commit: `70b120d` (pushed to origin by the user's instruction, docs only).
- Files inspected: `sitl/independent_agent_node.py` (mission handler ~L380–460), `sitl/mission_intent.py` (`from_dict`, `MissionLease.accept`), `sitl/decentralized_guidance.py` (L60–95), `tests/test_decentralized_sitl.py` (existing mission tests).
- Files changed: `sitl/independent_agent_node.py` (+3/−2: sequence check moved to immediately after `MissionIntent.from_dict`, old check removed); new `tests/test_mission_intent_ordering.py`; `docs/PROJECT_STATE.md`.
- Design decisions: reject `intent.sequence <= last_mission_sequence` right after parse/validation and before formation/slot/transition changes. Semantics unchanged for accepted messages (replay of the same sequence is still rejected; epoch change still resets the counter). `DecentralizedGuidance.step` already validates before mutating, so it needed no change. `MissionLease.accept` still runs after the state update, but it re-validates with the same `wall_now`, so it cannot reject an intent the handler already accepted (only a non-finite `time.monotonic()` could); not changed.
- Tests and evidence level (UNIT; node built with `object.__new__` and a fake operator feed, no SITL):
  - `python3 -m pytest -q tests/test_mission_intent_ordering.py` before the fix: 5 failed (out-of-order and replayed intents altered grounded formation, restarted an airborne transition, and overwrote operator slots), 2 passed (newer message still applies; baseline).
  - Same command after the fix: 7 passed.
  - `python3 -m pytest -q tests`: all pass, 318 tests.
- Failures or unresolved risks: no live SITL run of this path; the airborne-transition test checks the agent state only. Items 3–12 of the backlog are open. A same-sequence retransmit is deliberately dropped (idempotent for the sender, not re-acked).
- Commit created: see `git log` (fix + tests + docs).
- Pushed branch/commit: not pushed.
- Recommended Codex review: confirm no caller relied on state changes from a stale intent; check whether the sender (`independent_swarm_console._publish_intent`) ever re-sends an intent with the same sequence but changed contents.
- Next step: transition-protocol spec (prepare/ack/common-start/abort) for review, then implementation.

### 2026-10-29 12:45 PKT — Codex — physical ESP32/CAN readiness review

- Objective: establish honest physical evidence for the connected ESP32, interpret the Oct 23 CAN configuration, choose the first flight-controller command-path direction, and define a propellers-off plan for one aircraft and the reported 10-drone department fleet.
- Starting commit: `1b700b8a1eacd509accd7d64e200ad409d53bc22` on `wip/decentralized-sitl-handoff-20261029`; tree clean.
- Files inspected: `AGENTS.md`; `docs/PROJECT_STATE.md`; this collaboration log; `hardware/README.md`; both Matek parameter exports; all three hardware scripts; `docs/HARDWARE_SOFTWARE_MAPPING.md`; `docs/PHYSICAL_NETWORK_VALIDATION_PLAN.md`; superseded 30 Sep/3 Oct handoffs.
- Files changed: `hardware/README.md`; new `hardware/reports/PHYSICAL_ESP32_CAN_BENCH_PLAN_2026-10-29.md`; section 6 and changelog only in `docs/PROJECT_STATE.md`; this appended session entry.
- Design decisions:
  - Recommend connectorized UART/MAVLink as the first command path if an approved no-solder UART harness exists.
  - Treat CAN as an experimental DroneCAN link until traffic, node identity, error state, and an ESP32 application protocol are demonstrated.
  - The Oct 23 values (`CAN_P1_DRIVER=1`, `CAN_D1_PROTOCOL=1`, 250 kbit/s, autopilot node 10) enable/configure a DroneCAN bus; they do not establish ESP32 traffic or a MAVLink command bridge. `CAN_D1_UC_SER_EN=0` records DroneCAN serial passthrough disabled.
- Tests and evidence level:
  - Command: `python3 -m pytest -q tests` before edits.
  - Result: 318 tests passed; no source/test files changed.
  - Evidence label: UNIT/INTEGRATION baseline only.
  - PHYSICAL: a real ESP32 USB interface enumerated as Silicon Labs CP2102 (`10c4:ea60`) at `/dev/ttyUSB0`, with a persistent by-id path and no port owner. No SocketCAN interface or USB-CAN adapter appeared.
- Failures or unresolved risks:
  - Propellers were installed and the flight battery was connected, contrary to the bench-test gate. No serial port was opened and no active CAN test was attempted.
  - ESP32 firmware source/binary/hash, GPIO mapping, TWAI bitrate, startup behavior, MCP2551 module schematic/logic levels, termination, and CAN traffic are unknown.
  - Matek was not connected by USB in this session; no CAN node discovery, FC acknowledgement, UART link, physical command, motor action, or flight was tested.
  - `docs/HARDWARE_SOFTWARE_MAPPING.md` still says the saved Matek parameter has `CAN_P1_DRIVER=0`. That is true for the Oct 6 export but stale if read as current; request a future permitted documentation edit to distinguish Oct 6 from Oct 23. No change made because this session's write scope excluded that file.
  - The department count (10, team has 2) remains team-reported; no per-aircraft physical inventory exists.
- Commit created: none; user controls whether these hardware documentation changes are committed.
- Pushed branch/commit: nothing pushed.
- Recommended Codex review: verify the Matek connector/pin mapping from the board manual and review the ESP32 firmware/source before any serial monitor or CAN transmission.
- Next step: obtain authorization to remove propellers; then run the report's logic-only USB and passive CAN stages with the flight battery absent where possible.

### 2026-10-29 — Claude — transition-protocol design spec (no code)

- Objective: draft the prepare/acknowledge/common-start/abort design (backlog item 3) with supervision and failure semantics (items 4–6) for review before implementation.
- Starting commit: `1b700b8` (pushed). Code unchanged this session.
- Files inspected: `sitl/independent_swarm_console.py` (`_formation_command`, `_publish_intent`, `_formation_transition_abort_reason`, `_formation_transition_arrived`, `_intent_loop`, `_operator_command`, `handle`), `sitl/independent_agent_node.py` (mission handler, slot interpolation), `sitl/formation_transition.py` (headers), `sitl/operator_dashboard.py` (transition flags), `docs/AGENT_PROTOCOL.md`.
- Files changed: new `docs/TRANSITION_PROTOCOL_SPEC.md`; `docs/PROJECT_STATE.md` (plan, defect 3, changelog).
- Findings (all UNIT-level code reading, no runs): no ack of the plan (a missed UDP intent leaves one agent stationary during a morph); start time is publication time so late receivers jump; supervision and abort exist only in the coordinator and the only action is LAND ALL; `_operator_command` has no transition gating and the dashboard's gating is client-side JS, its server flag reads `session.morph_steps_remaining` rather than `formation_transition_deadline`.
- Design decisions proposed: per-transition `transition_id` + `plan_hash`; agents STAGE without moving and ack; coordinator sends `go` with a common `start_at`; abort means HOLD (freeze slot), not LAND; each agent supervises peers and separation locally; membership/coordinator-loss rules in spec §7; pure state-machine module first.
- Tests and evidence level: none run (design only); existing suite unaffected.
- Failures or unresolved risks: thresholds are uncalibrated; HOLD-vs-planned-timing safety and coordinator-loss continuation are open questions (spec §10).
- Commit created / pushed: see `git log`; not pushed.
- Recommended Codex review: spec §10 questions; whether HOLD at `freeze_at` can violate screened clearance; whether any code path other than `_formation_command` starts a morph.
- Next step: after review, implement `sitl/transition_protocol.py` (pure state machine) with UNIT tests, then wire the agent and coordinator.

### 2026-10-29 — Claude — transition state machines (pure logic, not yet wired)

- Objective: implement backlog item 3's core as testable pure logic before touching the runtime.
- Starting commit: `8a339ee` (pushed).
- Files changed: new `sitl/transition_protocol.py` (`TransitionPlan`, `AgentTransition`, `CoordinatorTransition`, `TransitionParams`); new `tests/test_transition_protocol.py`; `docs/TRANSITION_PROTOCOL_SPEC.md` (status, message list, replay-protection wording); `docs/PROJECT_STATE.md`.
- Design decisions: defaults for the spec's open questions = abort is HOLD (freeze slot at common `freeze_at`), coordinator loss during a transition = HOLD. Agent validates the whole plan before any state change; STAGED agents never move; `go` carries the common `start_at`; late `go` or missing `go` discards the plan; ARRIVED agents self-close after `arrived_close_s`; a HOLD agent accepts only a newer `transition_id`.
- Tests and evidence level: UNIT plus INTEGRATION-with-simulated-channel (in-memory, deterministic seeds; **not** SITL, not real UDP). `python3 -m pytest -q tests/test_transition_protocol.py`: 41 passed. Includes 4 loss levels (0/10/30/60%) × 60 seeds asserting no agent moves without a full ack set and no agent/coordinator is left mid-transition, and a lost-`go` case that must end in HOLD for the agents that started. Full suite: 359 passed.
- Test-design note: my first test geometry crossed drone paths at 2.1 m and correctly triggered the separation abort; geometry changed to a pure translation (3 m clearance). The real planner (`plan_formation_transition`) is unchanged and its clearance screening is not exercised by these tests.
- Unresolved risks: thresholds uncalibrated; HOLD-at-`freeze_at` timing vs screened clearance still the open question for review; agent/console wiring, message transport (epoch/HMAC), structured command results and server-side gating not done; no live evidence.
- Commit / push: see `git log`; not pushed after `8a339ee`.
- Recommended Codex review: invariants in the lossy-channel test; `AgentTransition._local_violation` thresholds; whether `CoordinatorTransition.abort` correctly distinguishes "moved" from "not moved" when `go` was sent but agents had not yet started.
- Next step: wire the agent side (message kinds, slot ownership, status fields) and the coordinator side.

### 2026-10-29 — Claude — algorithm review and tests (no algorithm code changed)

- Objective: test and critique the control algorithms at the user's request.
- Starting commit: `b7688bd`.
- Files inspected: `swarm_core/controllers/{centralized,decentralized,hybrid}.py`, `swarm_core/safety.py`, `swarm_core/formations.py`, `experiments/run_demo.py`, `experiments/compare_three_modes.py`.
- Files changed: new `docs/reports/ALGORITHM_REVIEW_2026-10-29.md`; `docs/PROJECT_STATE.md`. Outputs written to the scratchpad, not the repo.
- Tests and evidence level: numerical simulation (pure-Python engine, `assumed_baseline`), not SITL. `run_demo --seed 123`: min distance 2.294 m. `compare_three_modes` 10 trials at 5 and 10 drones: 10/10 pass in all cells; 10-drone hybrid worst trial 2.502 m. Own single-pair sandbox: guard overridden at goal pull = `max_accel` (0.26 m).
- Findings: see the report (guard is a spring not a barrier; hardcoded "Safety: PASSED"; 2 mm margin; per-drone hybrid switching; degree-normalised repulsion; per-step Hungarian).
- Failures or unresolved risks: findings are not fixed; the sandbox is single-pair/ideal, so its severity inside the full engine is unquantified.
- Commit / push: see `git log`; not pushed.
- Recommended Codex review: sandbox in report §3.1 (is the model of "goal term added on top of the guard" faithful to the engine?); whether `Drone` clips acceleration before or after the guard.
- Next step: fix 3.1 and 3.2 with regression tests, or resume wiring the transition protocol — the user to choose.

### 2026-10-30 — Claude — algorithm fixes (guard priority, demo label, repulsion) and hybrid diagnosis

- Objective: fix the problems found in the algorithm review.
- Starting commit: `1995b57` (local). Pushed so far: up to `8a339ee`.
- Files changed: `swarm_core/safety.py` (new `prioritised_separation_acceleration`), `swarm_core/controllers/{centralized,decentralized,hybrid}.py`, `simulator/engine.py`, `experiments/run_demo.py`, tests (`test_safety_priority.py` new; `test_hybrid_features.py`, `test_simulation.py` updated), docs (`PROJECT_STATE.md`, algorithm review addendum).
- Design decisions: the guard now has priority (components of the nominal command into a violated neighbor or against the net escape direction are removed; nominal scaled so guard + nominal ≤ max_accel); hybrid applies one guard at full weight after blending (decentralized part uses `apply_guard=False`); decentralized repulsion is a capped sum; the `test_degree_normalisation` expectations changed deliberately (repulsion no longer divided by degree); the 5 s centralized convergence test now allows 10 s because drones start inside the guard distance.
- Tests and evidence level: UNIT + numerical simulation. Full suite passes. Sandbox: old guard 0.00 m vs new 2.50 m for goal pull ≥ 2.5 m/s². `compare_three_modes` 10 trials at 5/10 drones: 10/10 pass; worst 10-drone hybrid burst 2.502 → 2.552 m. Demo seeds 123/1/2/3 pass the 2.5 m gate (2.61–2.67 m) with 3.0 m spacing.
- Failures or unresolved risks: findings 3.3–3.5, 3.7, 3.8 still open (see report addendum); hybrid mixed-mode diagnosis is correlational; changes are numerical-engine only, SITL adapter paths (`sitl/decentralized_guidance.py`) not re-verified with the new guard; no live evidence.
- Commit / push: `17ad625` plus a docs commit; not pushed.
- Recommended Codex review: confirm the SITL/agent path (`sitl/decentralized_guidance.py`, `sitl/independent_agent_node.py`) does not rely on the old additive guard or the degree-divided repulsion; review the `prioritised_separation_acceleration` semantics (removal of the opposing component is a conservative choice).
- Next step: implement optional hybrid mode agreement and compare 30 seeds; read the 30-seed guard-distance runs; then resume wiring the transition protocol.

### 2026-10-30 — Claude — hybrid defaults change, margin runs, assignment latching

- Objective: act on the recommendations (defaults, evidence), after the user approved "lets do these".
- Starting commit: `3ce1a0a` (pushed).
- Files changed: `swarm_core/config.py` (`hybrid_degrade_timeout` 0.50 → 1.20 with provenance note), `swarm_core/controllers/hybrid.py` and `simulator/engine.py` (default fallback `dead_reckon`), `swarm_core/formations.py` + `controllers/centralized.py` (assignment hysteresis, commit `5da15b0`), tests (`test_config_and_safety.py`, `test_simulation.py`, `test_hybrid_features.py`, `test_safety_priority.py`), `experiments/sweep_hybrid_fallback.py`, report addendum, state file.
- Design decisions: change the tracked profile parameter in one place so the SITL adapter stays in parity; keep the 2.5 m guard (3.0 m worsens tracking); leave swarm-level mode agreement unimplemented (timeout/strategy fixed the observed violation).
- Tests and evidence level: numerical simulation + UNIT. Sweep (10 drones, 12 seeds): `dead_reckon`/1.2 s → mean 2.717 m, worst 2.613 m, 41 switches vs old default 2.640/2.560, 77. 30-seed runs: 2.5 m guard all cells 30/30, worst hybrid 2.552 m. Permanent loss: separation held, mission not continued (lag 5.4–6.0 m). Full suite passes.
- Failures or unresolved risks: no SITL/hardware evidence; dead reckoning for up to 1.5 s is blind motion; hybrid does not continue the mission; the SITL agent path (`sitl/decentralized_guidance.py`, default timeout 0.5, `hold_target`) was not changed.
- Commit / push: see `git log`; the previous commits were pushed at the user's request.
- Recommended Codex review: the defaults change (is blind dead reckoning acceptable before physical tests?); whether `sitl/decentralized_guidance.py` should adopt the new timeout/strategy.
- Next step: wire the transition protocol into the agent and console.

### 2026-10-30 — Claude — transition protocol wired into agent and console (opt-in)

- Objective: connect `sitl/transition_protocol.py` to the runtime without changing default behavior.
- Starting commit: `4acdc66`.
- Files changed: `sitl/independent_agent_node.py` (transition message handling, context snapshot, slot ownership while RUNNING/ARRIVED/HOLD, `track_formation`, `transition` status field), `sitl/independent_swarm_console.py` (`--transition-protocol` path in `_set_formation`, ack/abort intake in `_status_loop`, `_tick_transition_protocol` in the intent loop, server-side gating in `handle`/`_set_formation`, land aborts to HOLD, airborne intents carry no formation/slots in protocol mode), `sitl/swarm_console.py` (flag), docs, new `tests/test_transition_wiring.py`.
- Design decisions: opt-in flag because there is no live evidence yet; agents never move while STAGED; abort = HOLD; airborne protocol-mode intents use `formation="configured"` and no slots so a stale intent can never overwrite frozen slots; agent aborts reach peers through the coordinator relay.
- Tests and evidence level: INTEGRATION (real coordinator + real agent logic over an in-memory channel with a controllable clock; no SITL/UDP/MAVLink): success path (all start together, finish in target formation, nobody moves before `start_at`), missing prepare (cancelled, nobody moved), lost go (others HOLD, agent 2 never starts, no LAND, re-plan possible), server-side gating (`transition_active`), land during morph. Full suite passes.
- Failures or unresolved risks: no live SITL run yet; peer-to-peer abort broadcast not implemented (coordinator relays); gating covers REPL/dashboard `handle` + `_set_formation` only; clock-sync uncertainty cap from the spec is not enforced; thresholds are initial values.
- Commit / push: see `git log`; not pushed after `3ce1a0a`.
- Recommended Codex review: `_publish_intent` protocol-mode branch; whether any dashboard endpoint bypasses `handle`/`_set_formation`; the HOLD semantics when the swarm centre goal is held.
- Next step: live 5-drone standalone SITL run with `--transition-protocol` (formation change, killed agent, dropped go if possible), record evidence.

### 2026-10-30 — Claude — first live SITL run of the transition protocol (3 runs, 2 bugs fixed)

- Objective: validate the opt-in protocol in live SITL (recommendation 3) and fix what it exposes.
- Starting commit: `52added`; end: `0c02b7c` (fixes `8e6e4a6`, `0c02b7c`).
- Method: 5-drone standalone console, driven by a script in the scratchpad (launch, `formation line`, `formation column` + SIGTERM to agent 3 mid-morph, `land`). Process/port ownership and disk checked first; the console's own broad `pkill arducopter` ran (nothing else was running). Teardown verified: no leftover processes or ports.
- Findings (SITL): run 1 `peer_stale` abort (supervision saw only the current tick's packets) + HOLD-after-pre-start-abort drift; run 2 ~3 m swarm drift because the goal moved to the centroid at prepare time; run 3 success. Fixes: per-peer observation memory; pre-start abort/violation discards the plan (IDLE); slots and a goal override (plan centroid) applied only after the common start and while HOLDING; coordinator never moves the goal in protocol mode; movement commands rejected while HOLDING. Regression tests added and mutation-checked.
- Result (run 3): LINE complete 19.05 s after prepare (commit 0.21 s, common start 4.02 s), sampled min separation 3.62 m, line spacing 4.0 m; COLUMN + killed agent: peers detected in ~1 s, all 4 survivors HOLD at progress 0.406–0.408, no LAND, 3.85 m sampled min separation, survivors landed on command.
- Evidence level: SITL (host-only, loopback, standalone 2D). One run per scenario; 1 Hz status sampling; no impairment. Full list of limits in the report.
- Files: `docs/reports/TRANSITION_PROTOCOL_LIVE_SITL_2026-10-30.md`, `docs/evidence/2026-10-30/transition_protocol_live_summary.json`; raw logs in `runs/independent/transition-protocol-live-20261030/` (not in Git, checksummed).
- Recommended Codex review: the two root-cause analyses; whether pre-start abort = discard is right when an agent already applied a `go` but the coordinator aborts for a non-safety reason; `_transition_goal` semantics when HOLDING and an operator changes the mission goal.
- Next step: repeat runs (10 drones, loss/latency, dropped go, coordinator kill, VM split) and decide on making the protocol the default.

### 2026-10-30 — Claude — hardening, research, in-flight mode switching (live 10-drone SITL)

- Objective: user asked for hardening, 10 drones in 2D, 6 in 3D, in-flight mode switching (Claude drives, user watches), and extensive research vs state of the art.
- Commits: `e897eaf` (link-loss spin fix, 10-drone ring spacing, CI installs `.[all]`, 10-drone coverage tests), `35791b1` (`sitl/mode_switch.py` state machines + research report + loopback-UDP test made patient), `eb38cf0` (agent/console wiring, per-mode coordinator-loss policy, dashboard).
- Findings: (1) `wait_ekf_ready` spun on a dead TCP link (the 54 M-line log incident); (2) 12 of 42 formation changes at 10 drones were rejected because the ring template was spaced 3.0 m (fixed to 4.0 m; realistic chains of 96 random changes at 6 and 10 drones all plan directly); (3) lossy-channel simulation exposed a split-mode hole in the first mode-switch design (lost abort -> some agents self-commit): abort/done now retransmitted and the coordinator's verify deadline is shorter than the agents' self-commit timeout; (4) in SITL the three modes fly almost identically while healthy (target anchored to the slot): they differ in fault policy, so the policy was made explicit (centralized hold then land; decentralized/hybrid continue on peers up to 120 s).
- Live evidence (SITL, 10 drones, 2D): 5 in-flight switches completed in 7.3-8.2 s, ring change 34 s, no aborts, sampled min separation 2.65 m (ring morph), 10/10 landed, mean 94 % of one core, 1.0 GB RAM. Report `docs/reports/MODE_SWITCH_LIVE_SITL_2026-10-30.md`; compact summary `docs/evidence/2026-10-30/`.
- Research: `docs/reports/RESEARCH_BENCHMARK_2026-10-30.md` (AI-produced; claims tagged verified/snippet/unverified).
- Limits: one run, no injected failures during a switch, centralized 20 s landing and live rollback not exercised, no 3D yet, no Python 3.10/3.11 runtime test (grammar-checked only).
- Recommended Codex review: `sitl/mode_switch.py` (abort/done retransmission, verify deadlines), the per-mode fault policy in `independent_agent_node.py`, and whether the dashboard supervisor (`ModeSupervisor`) should forward in-flight `/mode` requests to the child console.
- Next step: Gazebo 6-drone run, failure-injection repeats, ring-morph margin.

### 2026-10-31 — Claude — roadmap features 4, 5, 7 (metrics, replay, geofence + altitude layers)

- Objective: the user asked for all seven research-recommended features; started with the cheap, demo-visible ones.
- Commits: `19a0cfa` (`sitl/run_metrics.py`, dashboard panel, run report), `84acf5e` (`sitl/replay.py`, dashboard replay bar), `efc9e15` (`sitl/geofence.py`, altitude layers, console pre-checks, fence drawn on the map), `031ea48` (2 m altitude floor, 3-D gap in the panel).
- Live evidence (SITL, 10 drones): `docs/reports/GEOFENCE_LAYERS_METRICS_LIVE_2026-10-31.md`. Replay verified on the earlier mode-switch recording. Run 1 exposed that the swarm could be commanded down to 0.5 m (fixed). Run report PASS: min horizontal gap 3.108 m, 3-D 3.452 m.
- Limits: real fence breach not exercised live; layers not shown to improve the horizontal minimum; one run per configuration; software fence only.
- Pitfalls hit (for the next session): `pkill -f` patterns that appear in the same shell command kill the shell itself — stop servers by PID; the browser pane can show the live dashboard (port 8765) and the replay (8766).
- Not yet done from the roadmap: mission continuation after coordinator loss, coordinator election, control-barrier safety filter, wind/GPS-noise stress tests; 6-drone Gazebo run still waiting on the user's choice.
- Recommended Codex review: `sitl/run_metrics.py` (violation accounting), `sitl/replay.py` metric rebuild, the `_send_position_target` choke point (clamp + layer + ceiling), console `_movement_blocked`.
- Next step: mission continuation (feature 2), then coordinator election.
