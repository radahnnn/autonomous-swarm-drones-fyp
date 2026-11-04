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

### 2026-10-31 — Claude — roadmap feature 2 (mission continuation) and a live-found flaw

- Commits: `e473ce4` (`sitl/route.py`, intent fields, agent goal override, `patrol`/`route` commands, map drawing), `ad79cd5` (route speed cap 0.4 m/s).
- Live evidence (SITL, 10 drones): `docs/reports/ROUTE_CONTINUATION_LIVE_SITL_2026-10-31.md`. Controlled A/B: 0.6 m/s FAIL (min gap 1.7 m, tracking error 3.07 m) vs 0.3 m/s PASS (min gap 3.80 m, 0.73 m). Decentralized/hybrid kept patrolling with the stream off for 30 s; centralized held.
- Root cause of run A: route speed above the 0.45 m/s setpoint slew makes the setpoint lag grow without bound; the new run metrics report and the setpoint-vs-position tracking error made it diagnosable.
- Limits: coordinator loss simulated by pausing its stream; centralized 20 s landing not exercised; one run per configuration.
- Recommended Codex review: the interaction between `LocalDroneAgent.POSITION_TARGET_SPEED_M_S` (0.45) and any future faster motion (transitions use the same slew path with `track_formation`); whether the route cap should be derived from that constant instead of duplicated in `sitl/route.py`.
- Next step: coordinator election, then the control-barrier filter and wind/GPS stress tests; 6-drone Gazebo run still waiting on the user's choice; 17 commits are local only.

### 2026-10-31 — Claude — roadmap feature 3 (acting-coordinator election) and a safety finding

- Commits: `cb278e5` (`sitl/coordinator_election.py`, `publish_coordinator` / `take_coordinator_messages` on the peer transport, agent wiring, dashboard coordinator line, takeover counter), `c977a04` (election pause handling).
- Live evidence (SITL, 10 drones, centralized, patrol): `docs/reports/COORDINATOR_ELECTION_LIVE_SITL_2026-10-31.md`. Takeover 2.7 s after operator silence, hand-over to D2 (term 2) after D1's agent was killed, operator return ends the acting role. Mutation-checked unit/simulation tests (three mutants caught).
- Findings: (1) false takeover when an agent is blocked in arm/takeoff (fixed); (2) latent cached-intent expiry during long autonomous periods (fixed); (3) **near-collisions after membership change: survivors' min gap 3.90 m -> 0.10 m** (open, new defect 9; guard authority in the SITL setpoint path is ≤ 0.25 m).
- Limits: operator loss simulated by pausing its stream; no partition test; acting coordinator republishes the mission only.
- Recommended Codex review: `CoordinatorElection.on_acting_message` ordering rules, the interplay between `COORDINATOR_TIMEOUT_S` (2.5 s) and `election_timeout_s` (3 s), and how adaptive membership redistributes slots on member loss.
- Next step: control-barrier safety filter on the commanded setpoint (feature 1), motivated by finding (3); then wind/GPS stress tests; Gazebo; push (user's order: election, CBF, Gazebo, push).

### 2026-11-01 — Claude — safety follow-up (defect 9) and roadmap features 7, 9-15

- Order given by the user: finish the election/barrier work, then features #7, #9, #10, #11, #12, #13, #14, #15, then Gazebo, then push.
- Commits (see `git log`): `fa8c2d0` barrier filter; `6a94db1` minimum-motion slot reassignment; wind model + stress harness; `sitl/smoothing.py`; `sitl/clock_budget.py` (+ console refuses starts whose lead is shorter than the clock budget); `swarm_core/assignment.py`; `sitl/separation_geometry.py` + `--gate-metric ellipsoid --vertical-radius`; `sitl/fleet_ctl.py` + `scenarios/election_10.txt`; sensitivity sweep; evidence-dir fix.
- Live evidence (SITL, 10 drones): `docs/reports/SAFETY_FILTER_AND_MEMBERSHIP_LIVE_SITL_2026-10-31.md` — survivors' min gap 0.10 m (no filter) -> 1.92 m (filter) -> **3.62 m PASS** (filter + slot reassignment). One run per configuration.
- SIMULATION evidence: `docs/reports/STRESS_AND_SENSITIVITY_2026-10-31.md` (GPS error, not wind, is what breaks separation; no tested gain is sensitive within 0.5-2x; two of the nine rows are identical because the parameter is inactive, explained there).
- Feature #9 turned out to be already implemented (quintic smoothstep = minimum jerk); consolidated into one tested function. #13: only the assignment code was merged; the two shape-generator sets stay (merging changes recorded simulator results). #14 is a document only.
- Honest gaps: ellipsoid vertical radius is an input, not measured; wind model is assumed, not fitted; clock budget assumes 100 ppm drift and has only Monte-Carlo (SIM) support; `fleet_ctl` scenario's landing expectation did not match console output (landing completion not verified in the last run); an unexplained SITL start failure occurred once right after a previous run.
- Pitfalls hit: relative `--evidence-dir` broke start (fixed); `pgrep/pkill -f <pattern>` in the same shell command kills the shell (use PID-only filters via `ps | awk`); `timeout` on the console leaves SITL and agents running (stop by PID).
- Recommended Codex review: `sitl/cbf_filter.py` constraint derivation, `AdaptiveMembership.slot_map` (baseline = full-count template; behaviour when the count of survivors is 1), `sitl/clock_budget.py` assumptions, `Runner.agents()` process-ancestry matching.
- Next step: Gazebo 6-drone run. Needs a measured real-time factor with 6 SITL first, then a time-scale for slew speed, route speed, transition/ramp durations and telemetry-age limits; then push (34 commits local only).

### 2026-11-01 — Claude — Gazebo 6-drone bring-up (partial)

- Commits: `--time-scale` (slew speed, transition duration, arrival timeouts), `sitl/launch_gazebo_world.sh`, `scenarios/gazebo_6.txt`, `c04cfe9` (plan bound 60 -> 600 s, found live).
- Evidence (GAZEBO, headless, RTF ~0.16): `docs/reports/GAZEBO_6DRONE_INDEPENDENT_2026-11-01.md`. Independent runtime runs in Gazebo; mode switching works; **no formation transition completed**; run 2 report FAIL (1.598 m) with a transition cancelled by a separation check. Cause of the dip not isolated.
- Not verified: landing completion, anything with routes, kills, GUI.
- Recommended Codex review: whether time-scaling is complete (mode-switch ramp, route speeds, `LOCAL_PAUSE_S`, peer/telemetry staleness all remain in wall time), and the pre-start separation check against the V spacing.
- Next step: isolate the dip (position log around the abort, wider Gazebo V), repeat the transition, then 10-drone/route tests. Push is the user's ordered last step.

### 2026-11-01 (later) — Claude — Gazebo fixed: mirrored world and slow physics step

- Root causes found from run-2 data: generator wrote north,east into Gazebo x,y (V mirrored; `generate_gazebo_world.py`, test now compares to the checked-in 5-drone world); lockstep at 1 ms limits RTF to 0.137 -> `--step-ms` / `STEP_MS=2.5` gives 0.29-0.39.
- Run 4 (GAZEBO, GUI, 6 drones): PASS 3.223 m, both transitions complete, 2 mode switches, landing verified from events; 440 s wall. One run. Raw: `runs/independent/gazebo-6drone-20261101-run4-step25/` (not in Git).
- `fleet_ctl`: console exit after `land`/`q` is no longer reported as a failure.
- Open: repeat for variance; routes (speeds are not time-scaled), kill/coordinator-loss, 10 drones in Gazebo; the 2.5 ms step is validated only by run 4.
- Not pushed yet (commits after `b10bf34` are local).

### 2026-11-01 (end of session) — Claude — Gazebo survey patrol built, first flight blocked at arming

- Asked for by the user: Gazebo 3D patrol, centralized first, then three decentralized pairs, then hybrid, then regroup at the origin; surveillance/obstacle detection/camera capture; 6 drones in 3D (must be 3D); trees to be added; wind and campus look discussed.
- Built (all UNIT-tested, see git log): world generator cameras/ground/markers/poles/trees (`--camera --obstacle --marker --tree`, `--step-ms`); `sitl/gz_camera.py` (PNG from gz-transport, verified live: aerial frame in `docs/evidence/2026-11-01/gazebo_camera_aerial_probe.png`); `sitl/obstacle_sensing.py` + peer message `obstacles` + agent wiring (SIMULATED range sensor, not image detection); `pairs`/`pairs_b` formations (hexagon of survey sectors; world-anchored so the planner does not rotate them); console `goto`, `capture`; `--sensed-obstacle`/`--sense-range`; route minimum speed 0.02 m/s; `sitl/patrol_layout.py`; `sitl/launch_gazebo_patrol.sh`; `scenarios/gazebo_patrol.txt`.
- **Not flown**: the one patrol attempt (`runs/independent/gazebo-patrol-20261101-run1`, not in Git) never took off: all 6 refused to arm with `Arm: Gyro 0 rate 403Hz < loop rate*1.8 720Hz` for 23 minutes. Run 4 (no cameras, same 2.5 ms step) armed. Hypotheses, none tested: the cameras/rendering change the IMU timing; the 400 Hz gyro was never enough and run 4 got lucky; something in the patrol world. Next: start the plain world and the patrol world with the same step and compare the gyro rate message; try `STEP_MS=1`; or lower `SCHED_LOOP_RATE` for Gazebo; check if `--no-camera` arms.
- Also: the survey-patrol scenario is a 20-minute wall-clock script: do not start it until arming works. Trees are visual only (user wants them as obstacles: give them collision + put them in `patrol_layout.POLES`-style sensed list, keep them off the pair sectors). Wind in Gazebo: not done (try 1 then 3 m/s). Campus look: waiting for the user's photos (memory note `project_gazebo_campus_demo`).
- Not pushed: the commits after `b10bf34` are local only.

### 2026-11-02 — Claude — Gazebo arming fix, --realistic mode, plan update

- Gazebo arming blocker: 2.5 ms physics step gives a 400 Hz gyro; ArduCopter needs >= 720 Hz (1.8 x 400 Hz loop) and refused to arm with or without cameras. Step capped at 1.38 ms, default 1.25 ms (all 6 armed, RTF 0.19). Commit `ec40c9d`.
- `--realistic` (commit `625723a`): SITL profile with GPS wander/lag, wind+gusts, vibration, sensor noise, battery model + land-on-low failsafe, and a lossy bursty peer link. Params verified by readback inside ArduPilot. SITL quad simulates a 3S pack (SIM_BATT_VOLTAGE ignored) - low-voltage threshold set per cell for 3S. 10-drone 2D realistic run: PASS, min gap 3.56 m, mean tracking 0.33 m; formation change refused by the 45 s cap -> cap 90 s. Values are ASSUMPTIONS until the team measures battery, GPS and link.
- Gazebo survey patrol with --realistic (in progress at time of writing): pole detected by D1 and learned by the other five over the peer link; 6/6 camera captures; both pair transitions aborted by protocol supervision (separation < 2.8 m; peer off plan) -> swarm held. Finding: protocol thresholds tuned on clean positions do not survive ~1 m GPS wander.
- Plan updated with the user (PROJECT_STATE section 8): noise tuning, the user's tower-gap manoeuvre (V -> column through two towers -> V), patrol end to end, repeats, physical flight within a month.

### 2026-11-03 (overnight) — Claude — realistic simulation, attribution matrix, fixes; push and power-off on the user's instruction

- Report: `docs/reports/REALISTIC_SIMULATION_2026-11-03.md` (matrix table, five fixes, final run, open problem).
- **Withdrawn results:** the four realistic 10-drone "PASS" runs of 2026-11-02 (drones were sinking/bouncing; battery model).
- Next step: estimation-health guard (position-estimate glitch caused the remaining near-miss); then the 6-drone Gazebo patrol under `--realistic`; then the tower-gap manoeuvre (user's idea, planned, not started).
- The user asked that the PC be powered off after the work and push.

### 2026-11-03 (day) — Claude — guard, retries, dashboard, Gazebo patrol PASS, tower gap partial

- See the update section of `docs/reports/REALISTIC_SIMULATION_2026-11-03.md`. Next: widen the gap column, rerun the gap (2D, then Gazebo with towers); repeats for variance.
- Recommended Codex review: `sitl/estimate_health.py` thresholds, `IndependentSwarmConsole._tick_gap` state machine (threads started from the console loop), the retry timer interplay with operator commands, `operator_dashboard` capture serving (filename regex).

### 2026-11-03 (end of day) — Claude — plan agreed: strengthen the software (hardware on hold)

- Results today (all pushed): Gazebo 6-drone survey patrol PASS under --realistic; tower-gap manoeuvre PASS in 2D (10) and Gazebo (6, real towers); estimation-health guard; transient-cancellation retry; dashboard upgrade. Details: `docs/reports/REALISTIC_SIMULATION_2026-11-03.md`.
- Panel-style self-critique and the 10-step plan are in PROJECT_STATE section 8. Weakest points: single-run evidence, no baselines, estimate-based sampled separation, assumed realism values, never flown.
- Next step: Step 1, ground-truth separation logging. Codex (software chat): good review targets are `sitl/estimate_health.py`, `IndependentSwarmConsole._tick_gap` / `_gap_formation`, the retry timer, and `operator_dashboard` capture serving.

### 2026-11-03 — Claude — user rule: no agent recommendations in the project files

- At the user's request the 10-step recommendation plan was removed from PROJECT_STATE section 8 (the entry above that mentions it is superseded). From now on: only items the user asked for or that testing found are recorded; results only once tested and proven; new plan items only after consulting the user. Applies to every agent writing these files.

### 2026-11-03 (evening) — Claude — code efficiency (no behaviour change) + one robustness fix

- Archived 13 unused early scripts to `docs/history/legacy-scripts/` (-2,200 lines). Split `independent_swarm_console.py` (2,186 -> 1,231 + `console_clock/protocols/mission/metrics.py`) and `independent_agent_node.py` (1,464 -> 1,045 + `agent_safety/protocols/mission.py`) as mixins with a mechanical, behaviour-preserving move. Repo lint-clean (pyflakes 70 -> 0, now in CI). `docs/PARAMETERS.md` generated by `tools/parameter_table.py`, kept current by a test.
- Verification (SITL, 10 drones, same short mission): calm profile after the refactor PASS 5.19 m (before: 5.18 m), all steps completed. A `--realistic` run after the refactor FAILED (0.995 m at ~192 s during the decentralized patrol): the same moment and pattern as pre-refactor realistic runs (0.92 m, 1.0 m) - recorded as the open realistic-mode near-miss, not attributed to the refactor.
- Fix: the airborne formation pre-check now waits up to 5 s for momentarily stale peer links before refusing (a single dropped link refused a whole V re-form under the lossy-radio model).
- Pending the user's decision: retiring the older `--agent-runtime local|process` code (~2-3k lines).
- Codex: the user will ask you to review; prompt is in the chat. Write findings as a new entry here.

### 2026-11-03 22:20 PKT — Codex — software review and thesis-panel critique

#### Top five findings (ranked)

1. **HIGH — A partly delivered mode commit can leave the fleet in different modes.** `sitl/mode_switch.py:202` (`AgentModeSwitch.on_done`), `:386` (`CoordinatorModeSwitch.tick`), `:431` (`_verify`). A received `mode_done` closes the agent's plan permanently. The coordinator can still roll back on stale status or verification timeout, but that closed agent ignores the abort while an agent that missed DONE rolls back. **Reproduced, UNIT:** two agents switching centralized -> decentralized; deliver DONE only to D1, then make D2's status stale and deliver the resulting abort to both: final modes are `[decentralized, centralized]`. The coordinator eventually returns to IDLE with rollback unverified. **Fix:** define an irreversible commit decision and recovery protocol; after any commit can have been delivered, retransmit/reconcile that decision rather than issue an incompatible rollback. Keep unresolved membership/mode agreement explicitly blocked. Test lost DONE, lost status, delayed abort, restart, and partition recovery.

2. **HIGH — Console mission mutation has multiple writers; retries lack cancellation and lose plan options.** `sitl/independent_swarm_console.py:569` (`_publish_intent`), `sitl/console_protocols.py:181` (`_on_transition_event`), `sitl/console_mission.py:176` (`_gap_formation`) and `:212` (`_tick_gap`). The intent thread, REPL, HTTP handlers, timer callbacks and gap workers share mutable mission fields without a common ownership mechanism. The dashboard lock covers only some HTTP actions and does not serialize these other writers. **Reproduced, UNIT:** pause two publication calls at their clock reads after incrementing the counter; both emit sequence `2`. Other fields can also come from different mission revisions. A delayed old formation retry can run after a new operator request or LAND, and only retains `style`, dropping the gap's pinned heading and 8 m spacing. `_gap_formation` reads the shared `last_formation_result`, so another command can overwrite its result. **Fix:** one mission-command executor/queue, immutable request snapshots with a generation ID, cancellation on replacement/LAND/shutdown, and an atomic sequence allocation plus payload snapshot. Retain heading/spacing on every retry. Reserve a planning state before any waits; emergency LAND must preempt it. Test real interleavings with barriers/events, including retry-after-LAND and retry-after-new-transit.

3. **HIGH — The safety filter is not the final constraint on the transmitted target.** `sitl/independent_agent_node.py:694` (`_run_guidance`), `:625` (`_send_position_target`), `sitl/agent_safety.py:126` (`_avoid_obstacles`). After `LocalDroneAgent.command` applies its pairwise barrier filter, obstacle repulsion, estimate-target freezing and fence clamping can change that target without checking the pair constraints again. **Reproduced, UNIT:** own position `(0,0)`, neighbour `(3,0)`, nominal target `(1,0)`, D=2.8 and alpha=3 produce a filtered north target `0.58`; an obstacle at `(-1,0,r=1)` then moves it to `2.58`, toward the neighbour and beyond the filter's allowed component. This demonstrates incompatible command composition, not an observed physical collision. **Fix:** combine obstacle/fence/pair constraints in the final command arbitration step, or explicitly revalidate the final transmitted target and enter a defined infeasibility policy. Test constraints on the captured MAVLink output, not only the intermediate filter return.

4. **MEDIUM — Formation start times are computed before blocking waits and planning.** `sitl/console_protocols.py:353`, `:402`, `:413`, `:436` (`_set_formation`), `:237` (`_begin_protocol_transition`). `now` is captured before up to 30/time_scale seconds of settling, five seconds of link recovery, and geometric planning; the plan still starts at that old `now + lead_s`. Positions are also captured before the link-recovery wait and are not refreshed afterward. **Reproduced, INTEGRATION with the existing in-memory harness:** a five-second settle moves the clock from 5000 to 5005, but the announced start is 5004; with fresh valid agent context it rejects `start_too_soon`. **Failure:** a genuinely settling fleet repeatedly refuses a valid formation; concurrent command changes during the wait are not rechecked before begin. **Fix:** after waits/planning, revalidate the request generation, flight state, telemetry, route/mode ownership and current positions; allocate the common start using the current clock immediately before PREPARE. Treat missing/non-finite settling data as unknown, not the zero error currently used by `_settle_errors` at line 477. Add a clock-advancing test rather than only fixed-clock settling tests.

5. **HIGH for evidence integrity — Run-report PASS does not establish fleet or mission completion.** `sitl/run_metrics.py:92` (`update`), `:187` (`report`). Any positive pair-sample count with no observed violation/ground contact passes; fleet coverage, mission completion, telemetry loss and confirmed landing are not required. Stale/missing vehicles disappear from the gap; fresh status can still contain old vehicle-position data because `position_age_s` is not checked. `max_status_age_s` in the report excludes statuses older than the accepted limit. **Reproduced, UNIT:** a ten-drone report with only two fresh airborne drones once, followed by no telemetry, says PASS with `drones=10` and one sample. **Fix:** separate sampled-separation verdict, mission outcome and evidence completeness. Record expected/observed aircraft, stale intervals, position freshness, synchronized sample times, fault membership, required milestones and landing. Missing safety evidence should be INCOMPLETE/UNKNOWN. A failed agent's flight controller may remain airborne; its disappearance from telemetry must not establish clearance from that aircraft.

#### Baseline and verification

- Role: reviewer. Only this appended log entry was written; no source/test edits, commit, push, merge or live simulation. The user explicitly limited documentation edits to this log, overriding the general PROJECT_STATE update instruction.
- Initial inspection was at `6e1770727c61b95d68e559d2c3670005c026693f` plus uncommitted software changes. Claude finished committing/pushing during inspection. Review paused on the user's request and resumed after the user confirmed Claude was paused. Final reviewed baseline: `33b97a37efafe607b9dd4253520e5a61178c96f5`, branch `wip/decentralized-sitl-handoff-20261029`, matching its locally recorded upstream. No pull was needed for this local checkout.
- Pre-existing changes at resume: `hardware/README.md`, untracked `hardware/reports/`, and untracked run/evidence directories. Preserved without editing. Root had about 20 GB available. The host process check showed no active ArduCopter/independent-agent simulation; no process was stopped.
- Required files read: AGENTS and all PROJECT_STATE; collaboration rules and entries from Oct 31 onward; the four requested reports; independent console/agent and their mixins (including legacy `agent_process.py`); transition, mode, election, barrier, estimate health, gap, formation planner, membership, metrics and dashboard modules; PARAMETERS and its generator. Supporting guidance, tests, CI and packaging were inspected to trace runtime behaviour.
- First suite on the changing tree: 805 passed, 1 failed (`tests/test_parameter_table.py`, stale generated table). That result is superseded by the paused baseline below.
- Final command: `PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -q tests -p no:cacheprovider -o addopts='-ra'`: **806 passed in 63.94 s**. Cache/bytecode suppression protects the checkout; the addopts override exposes the normal summary. Host execution permission allowed localhost test sockets and the SITL-process guard to work.
- `PYTHONDONTWRITEBYTECODE=1 python3 -m pyflakes sitl swarm_core simulator experiments tests`: **exit 0, no diagnostics**.
- Evidence labels for this review: **UNIT/INTEGRATION only**, including deterministic in-memory probes run from stdin. No new SITL, QGC, GAZEBO, VM-NETWORK or PHYSICAL validation. Probes were not added to the repository; the writer should turn them into regressions.

#### Further correctness and design findings

6. **HIGH — Gap width is not clearance along the actual route.** `sitl/gap_manoeuvre.py:50` (`find_gap`), `:81` (`passable_single_file`), `sitl/console_mission.py:159` (`_resume_transit`). The distance between obstacle centres minus radii does not prove that the current route passes safely between them; the route is not steered through the midpoint. **Reproduced, UNIT:** northbound at E=0, towers `(20,-0.2,0.6)` and `(20,10,0.6)` yield 9 m free width and `passable=True` with sigma=1, even though the route intersects the first tower. Staggered towers have the same issue. Local repulsion may deflect the drone but does not validate the admitted corridor. **Fix:** calculate the swept path's clearance against each obstacle, choose a corridor centreline and bounded approach/rejoin path, and verify the whole manoeuvre. Test asymmetric, staggered and rotated towers. Also make a same-name COLUMN request respect changed heading/spacing: `_set_formation` currently returns unchanged at `console_protocols.py:346` solely on the shape name.

7. **MEDIUM — Barrier QP falsely declares some feasible problems infeasible.** `sitl/cbf_filter.py:95` (`filter_velocity`). Candidate enumeration includes line projections and line-line intersections, but not line-speed-circle intersections. **Reproduced, UNIT:** own `(0,0)`, nominal `(0,1.5)`, neighbour at `(-2.8,0)` moving `(0.5,0)`, D=2.8, alpha=3, speed limit 1.5. The constraint is `u_north >= 0.5`; `(0.5,1)` is feasible, yet the implementation returns escape `(1.5,0)` with `feasible=False`. This causes an unnecessary large command and invalidates the description of an exact bounded optimizer. **Fix:** include the circle boundary candidates or use a verified convex solver; minimize distance to the original nominal velocity, not only its clipped version. Add adversarial circle-boundary tests and comparison against an independent oracle. Do not describe the fallback as optimal/least-bad without a defined objective.

8. **HIGH — Transition self-close can discard an unresolved arrival/abort.** `sitl/transition_protocol.py:308` (`AgentTransition.tick`), `sitl/agent_protocols.py:150` (`_tick_transition`). ARRIVED means the trajectory clock reached its end, not that this vehicle arrived. At duration + 7/time_scale seconds, the agent closes before local safety validation and calls `_finish_transition`, while the coordinator's overrun allowance is 12/time_scale seconds. **Scenario:** a lagging agent has not satisfied coordinator arrival+dwell; it self-closes and adopts the target. The coordinator then sees IDLE instead of ARRIVED and aborts, but this agent has discarded the plan and ignores the abort while other agents hold. **Fix:** retain unresolved transition state and completion identity until an acknowledged terminal decision or a defined recovery/HOLD policy; require actual arrival and dwell before any success close. Test a lagging aircraft, lost DONE, late abort and coordinator loss around the self-close boundary.

9. **HIGH — Arming/takeoff blocks handling of a later LAND command.** `sitl/independent_agent_node.py:559` (`_receive_operator_messages`). The same loop that reads operator commands synchronously calls `arm_with_retry(timeout=120)` and then takeoff; its callback sends heartbeats, not emergency-command handling. **Scenario:** some vehicles fail to arm promptly, the operator cancels with LAND, but those agents cannot process LAND until the helper returns; its five-second message expiry may have passed by then. They could finish the old takeoff after cancellation. **Fix:** implement cancellable flight-operation states under the agent's single I/O owner, with immediate LAND priority; acknowledge execution outcomes separately from receipt. Test cancellation while waiting to arm and while climbing using a fake MAVLink clock, without hardware.

10. **MEDIUM — Acting-role expiry is not a hard autonomy limit.** `sitl/coordinator_election.py:102` and `:113` (`tick`), `sitl/agent_mission.py:74` (`_publish_acting_intent`). Expiry resets `last_operator_at` to now; after another election timeout the same node can take over again and renew missions. **Reproduced, UNIT:** acting_max=10, operator last seen at 0: takeover at 4, expiry at 15, takeover again at 19. With the default 20-second centralized landing delay, this short interruption does not enforce the advertised bounded acting period. **Fix:** define whether the limit is per term or per operator-loss episode. If it is a safety bound, retain the true last operator receipt and latch exhaustion until a valid operator return; test multiple expiry/election cycles. Separately, `_handle_acting_intent` at `agent_mission.py:82` mutates election state before checking the mission's validity/order; validate the complete candidate before refreshing leader liveness.

11. **MEDIUM — Dashboard replies can claim acceptance for a rejected command.** `sitl/operator_dashboard.py:474` (`Handler.do_POST` fallback), `sitl/independent_swarm_console.py:904` (`handle`). Several command gates only print and return without setting a structured rejection. The HTTP handler then substitutes `accepted=True`. **Scenario:** click a movement command during a formation transition: the console rejects it, while the UI says it was accepted. The `/mode` handler also bypasses `action_lock` used by `/formation` and `/command`, contributing to finding 2. **Fix:** return one typed result from every command path, propagate rejection codes, and route every control endpoint through the same executor. Test actual HTTP response semantics during busy/HOLD states. Capture filenames are tightly allowlisted and the server binds loopback; no ordinary path-traversal issue was found in the reviewed capture path.

12. **MEDIUM — Clean CI does not declare its new lint dependency.** `.github/workflows/ci.yml:31` and `:35`, `pyproject.toml:22` and `:30`. CI installs `.[all]`, but pyflakes was added only to the separate `dev` extra; `all` is a literal independent list, not an inclusion of `dev`. **Scenario:** a clean Python job has no pyflakes and fails before tests, although this workstation passes lint. **Fix:** install `.[dev,sitl]` in CI or keep an actually complete all extra. Validate the clean matrix; local Python success does not prove Python 3.10/3.11/3.12 CI success. No dependency installation or remote CI run was performed in this review.

13. **MEDIUM — Test quantity hides gaps at runtime boundaries.** `tests/test_cbf_filter.py:37`, `tests/test_gap_manoeuvre.py:70`, `tests/test_transition_wiring.py:547`. The barrier assertion ends in `or True` and cannot fail; gap tests replace threads with objects that only record arguments; retry tests record timer arguments rather than exercising callbacks against later commands. These are useful unit tests but do not test the interleavings above. **Scenario:** all 806 tests pass while duplicate publication and partial mode commits still reproduce. **Fix:** remove vacuous assertions; add deterministic thread scheduling, terminal-message-loss and final-MAVLink-output tests. Keep fast pure-state tests and the existing simulated-channel integration harness; extend them rather than replacing them with more demonstration scripts.

14. **LOW — The generated parameter inventory is not every tunable parameter.** `tools/parameter_table.py:14` (`_constants`), `docs/PARAMETERS.md:1`. It only collects public uppercase literal assignments; dataclass fields (`ModeParams`, `TransitionParams`, `ElectionParams`), constructor defaults (`EstimateHealth`), CLI defaults, arithmetic constants and profile dictionaries are omitted. The matching test proves reproducibility of this subset, not completeness. **Scenario:** a reviewer misses the 7-versus-12-second transition timeout mismatch because neither appears in the table. **Fix:** label the subset honestly or extend generation to typed configurations/defaults, include units, scaling rules and evidence/provenance, and test representative coverage.

#### Evidence and thesis claims

15. **MEDIUM — The entry-point status contradicts current code and recent evidence.** `docs/PROJECT_STATE.md:3`, `:67`, `:75`, `:114`, and section 8's realistic-run item. It still says 766 tests at the top (about 820 elsewhere), an unbuilt transition protocol, an older pushed state, and a 2.48 m last realistic failure. The latest collaboration entry instead records a subsequent ~0.995 m failure after the refactor. The 70–75% completion estimate has no stated acceptance-weight calculation. **Scenario:** a defense or new agent uses the designated source of truth and reports superseded progress. **Fix:** have the writer reconcile state against the acceptance matrix, latest run IDs and current commit/test count; retain earlier results as history. This review did not edit PROJECT_STATE because the user allowed only the log.

16. **MEDIUM — Some real-world and robustness wording exceeds the experiment.** `docs/reports/REALISTIC_SIMULATION_2026-11-03.md:24` and `:34`; `docs/reports/STRESS_AND_SENSITIVITY_2026-10-31.md:59`; `sitl/transition_protocol.py:26`. GPS is the largest observed effect in the tested attribution cells; one run per cell does not identify all interactions or establish what matters on hardware. The hardware-hover comparison and roughly five-minute mission advice are not supported by a measured flight/endurance result in the reviewed evidence. The sensitivity report correctly notes inactive gains, so its blanket statement that no tested gain matters should be limited to activated gains, these metrics, this scenario and this sweep. Transition checks use estimates and therefore do not directly protect a known true gap. **Fix:** use those narrower statements; distinguish assumed, configured, read-back and physically measured quantities. Report true simulator positions alongside estimated positions, with timestamps and coverage, before interpreting a noisy estimated near-miss as physical contact or a physical safety margin. Repeat paired scenarios with equal spacing, speed and disturbances and report distributions rather than selecting the best run. These are recommendations, not additions to PROJECT_STATE's approved plan.

#### What the project does well

- It has a substantial executable software stack: individual agent processes and MAVLink ownership, real peer transport, formation planning, mode transitions, membership repair, failover, mission routes, a usable dashboard and an evidence workflow. The reports document real SITL/Gazebo runs; this reviewer did not independently rerun them.
- The pure protocol/math modules and in-memory channel tests are a good foundation. Splitting the large files makes navigation easier, and the final local lint/test baseline is clean.
- The research record includes failed runs, limitations and withdrawn battery-contaminated results. Membership repair is compared with earlier failing configurations, and the stress report admits both model assumptions and below-gate calm baselines. That honesty strengthens a thesis.
- The reports generally label single-run evidence, sampled separation and the absence of physical flight. The top-level explanation of centrally planned formations with distributed execution is appropriately cautious.

#### What the architecture means, and its disadvantages

The honest description is **distributed per-aircraft execution and peer-based guidance, with centralized mission/formation/gap/mode planning and limited acting-coordinator continuation**. Each agent owns its own flight-controller link and can continue a preloaded route under the stated conditions. The console still chooses the formation, assigns transition paths, decides when to squeeze through a gap and requests mode changes. An elected coordinator republishes the cached mission; it does not implement the missing autonomous planning decisions. User input to start a mission is compatible with decentralized control; reliance on one console for subsequent planning is the actual limitation.

`sitl/decentralized_guidance.py:48` explicitly implements the independent runtime's centralized label as a local hybrid controller that never degrades. All three modes still execute aboard agent processes; in normal position tracking their differing acceleration corrections are bounded to 0.25 m, and timed formation tracking bypasses that correction path. Thus demonstrations of switching labels are not by themselves proof of three fundamentally different deployed computing architectures or of hybrid superiority. Define the compared control laws, information available to each, and loss policies in the thesis. Numerical-simulator results cannot automatically validate the independent SITL adapter, especially because the simulator lacks the live barrier filter and uses different spacing in some comparisons.

The largest maintainability cost is shared implicit state across mixins and threads. File splitting reduces file length but does not establish ownership of a mission transaction. Old local/process runtimes remain additional maintenance surfaces; label their support level explicitly before deciding, with the user, whether to retire them. The remaining work is mainly coherent safety/lifecycle behaviour and defensible evaluation, rather than more visual features.

#### Recommended order for Claude (advisory; no implementation or plan change authorized here)

1. Give mission commands one owner, cancellation generations and fresh planning timestamps; prioritize LAND. Preserve complete retry options. Turn the reproduced race/timing cases into regressions.
2. Resolve terminal commit/abort agreement for modes and formations; test loss around every terminal decision and unresolved-state recovery.
3. Make the final transmitted command satisfy the combined safety constraints; correct the bounded barrier solver and corridor geometry.
4. Make PASS explicitly cover required mission milestones and complete evidence; add independent simulator ground-truth logging, separate from estimated navigation state.
5. Fix CI packaging, reconcile status documentation, then run a fixed, repeated 5/10-drone comparison with predeclared acceptance criteria. Demonstrate robustness after the fixes rather than adding more features first.

Panel assessment: a strong and ambitious research prototype with credible partial demonstrations, but not yet a validated general-purpose safe swarm system or a fully decentralized mission planner. There is enough substantive work for a defensible FYP if the claims, failure cases and scope remain precise. A completion percentage would obscure the unresolved acceptance gates.

- Commit created: none. Push/merge/PR: none. Hardware and raw evidence: untouched.
- Next handoff: Claude remains the writer. Review the ranked findings with the user, choose focused fixes, then request regression review; no live flight/simulation was launched by this review.

### 2026-11-04 — Claude — fixes for the Codex review (all 16 findings), live check

- Commits `77e74f2`..`b701be3` (pushed). Each finding fixed with a regression test built from Codex's reproduction where one was given: #12 CI extra; #13 vacuous barrier test; #14 parameter table coverage + honest label; #15 PROJECT_STATE reconciled; #16 report wording; #7 exact barrier QP on the speed disc (oracle-tested); #3 barrier re-applied to the final transmitted target, hold if fence and barrier conflict; #9 LAND cancels arm/takeoff; #6 gap column via the gate midpoint, staggered towers not a gate, same-shape request with heading/spacing is planned; #4 positions/clock read after waits + re-check before PREPARE; #2 atomic publish, mission-command generations drop superseded retries (LAND cancels the gap), retries keep heading/spacing; #11 dashboard reports print-only rejections; #10 acting period a hard limit, acting missions validated first; #1 mode switch irreversible after DONE (test fails on the old code); #8 agent self-close only after the coordinator window and only when arrived, else HOLD; #5 run report FAIL > INCOMPLETE > PASS with evidence coverage, stale positions excluded, simulator truth (SIMSTATE) logged and judged.
- Live (SITL, 10 drones, `--realistic`, short mission): every step completed (2 mode switches, LINE, V). Run report FAIL: estimated min 2.07 m, **true (simulator) min 2.02 m**, 2 true violation episodes - the recurring near-miss is real, not estimate noise. It occurs right after the decentralized-patrol -> centralized switch (~185-200 s): some drones were up to ~7 m from their centralized targets, so the formation re-converges abruptly. Root cause not yet fixed (candidate: drift of the slewed setpoint during barrier activations in decentralized mode under noise). A follow-up fix (landing drones were counted as "missing") is in `b701be3`.
- Request: Codex, please re-review the fixes (same scope as your review entry) and add findings as a new entry.

### 2026-11-03 23:30 PKT — Codex — re-review of review fixes `77e74f2`..`1dd853c`

- Role and scope: REVIEWER only. Inspected the latest two log entries, the fix diff, relevant source, tests, project state, and the new live report. No source, hardware, or evidence files changed; no live simulation was launched. HEAD `1dd853c`; branch `wip/decentralized-sitl-handoff-20261029`. The checkout already had unrelated hardware and untracked-run changes.
- Verification (UNIT/INTEGRATION): `PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -q tests -p no:cacheprovider -o addopts='-ra'` -> **827 passed**; `PYTHONDONTWRITEBYTECODE=1 python3 -m pyflakes sitl swarm_core simulator experiments tests` -> no warnings. No Python 3.10/3.11 CI, SITL, Gazebo, VM, or physical rerun by this reviewer. The prior SITL report JSON at `runs/independent/review-fixes-realistic-20261104/evidence/20261103T180044.871392Z/run_report.json` says FAIL: estimated minimum 2.065 m, simulator-truth minimum 2.016 m, two truth violation episodes, 10/10 observed airborne. It was generated before `b701be3`; its missing-airborne count of 10 reflects the landing-completeness bug fixed afterward. Truth is still sampled, so it does not bound between-sample minima.

#### Highest-priority residual findings

1. **HIGH — #5 remains open: PASS can cover only two of ten configured drones.** `sitl/run_metrics.py:173-192` (`_track_completeness`) only tracks `ever_airborne`, and `:244-252` (`report`) never requires that set to equal `self.count`, required mission milestones, or verified landing. Reproduced with three fresh samples containing only D1/D2 in a ten-drone run: `{'result':'PASS','drones':10,'drones_observed_airborne':2,'evidence_coverage':1.0}`. It can also PASS without any simulator truth samples, because truth coverage is not required. **Fix:** define per-scenario required participants and milestones, demand fresh coverage while flight is expected and a terminal landed/disarmed state; label truth coverage separately and require it for a truth-backed safety claim. Keep FAIL > INCOMPLETE > PASS.

2. **HIGH — #2/#4 remain partly open: an older formation can start after LAND during a wait.** `sitl/console_protocols.py:326-354` increments/checks a mission generation only before delayed callbacks; `_set_formation` at `:425-470` never rechecks it after settle/link waits or before `transition.begin`. `sitl/independent_swarm_console.py:569-605` locks publication but not mutations of goal, route, formation, gap, or generation. **Reproduced, INTEGRATION with the in-memory `Swarm` harness:** an operator LAND during `_wait_settled` changed generation and disabled intent; `_set_formation('line')` nevertheless returned accepted and prepared a new transition. It reported `generation=2`, `intent_enabled=False`, `transition_started=True`. **Fix:** route every command and gap/timer event through one serialized mission owner; associate planning with a request ID; recheck that ID, LAND state, flight/safety freshness, and mode/transition ownership after every blocking wait and immediately before begin. Add this forced interleaving as a regression.

3. **HIGH — #8 remains partly open: self-close can bypass coordinator-loss HOLD.** `sitl/transition_protocol.py:313-333` checks the close timeout and own position before `_local_violation`. **Reproduced, UNIT:** an ARRIVED agent at its target with `coordinator_age_s=100` at `start + duration + 15.1` closed to IDLE with only `arrived_timeout_close`, instead of detecting coordinator loss. Lost DONE/abort and a coordinator partition can therefore leave one agent adopting the new layout while others HOLD. The new 15 s threshold and arrival check fix the original lagging-agent case, but do not establish terminal agreement. **Fix:** evaluate liveness/safety before any self-close; retain an unresolved terminal state or explicitly reconcile it; test lost DONE/abort and coordinator loss exactly at this boundary.

4. **MEDIUM — #1 prevents the reproduced post-DONE rollback but still permits an unconfirmed commit to become the console's active mode.** `sitl/mode_switch.py:437-451` assigns `self.mode=to_mode`, `state=IDLE` after a timeout even if a drone has no fresh confirming view. A paused/restarted/partitioned agent can still be in the old mode when the console accepts a subsequent switch. The event is honestly named `mode_commit_unconfirmed`, but IDLE does not preserve that uncertainty. **Fix:** expose an unresolved state, block dependent mode changes until every agent reconciles or is removed through an explicit membership decision, and test restart/partition recovery. The original two-agent lost-DONE regression does pass.

5. **MEDIUM — #3's final filter closes the demonstrated post-filter obstacle push, but the fence conflict branch is not a barrier guarantee.** `sitl/independent_agent_node.py:651-676` returns the current position when the fence clamp reactivates a pair constraint. With a neighbour inside the 2.8 m barrier and approaching, zero commanded motion can violate the required barrier inequality; `tests/test_cbf_filter.py` even accepts this hold case. The function also forwards the filter's heuristic escape without checking `result.feasible`. **Fix:** record feasible/infeasible status on the final MAVLink command; define an explicit emergency policy for incompatible constraints, and do not describe HOLD as satisfying a mathematically infeasible barrier. Test the captured transmitted target and safety state.

#### Disposition of all 16 original findings

| # | Re-review disposition | Evidence and remaining limit |
|---|---|---|
| 1 | **Partial** | No rollback after DONE and focused lost-DONE test; unconfirmed commitment can enter IDLE (finding 4 above). |
| 2 | **Partial** | Publication sequences are locked, retry options retained, and generations added; shared mission state is still multi-writer and waits are not cancellation-safe (finding 2). |
| 3 | **Partial** | Final pair filter now follows obstacle/freeze/fence changes; infeasible/fence conflict policy and output-level proof remain (finding 5). |
| 4 | **Partial** | Clock and positions refreshed after waits; command generation and all state/safety/telemetry conditions are not revalidated before PREPARE (finding 2). `_settle_errors` still treats missing data as zero (`sitl/console_protocols.py:511-523`). |
| 5 | **Partial** | FAIL/INCOMPLETE ordering, stale-position exclusion and SITL SIMSTATE truth are valuable; fleet participation, milestones, landing and truth coverage are not PASS gates (finding 1). |
| 6 | **Partial** | The original straight-line tower clip is routed via the gate midpoint and staggered towers are refused; no swept-path/whole-column clearance check against all obstacles is implemented (`sitl/gap_manoeuvre.py:82-87`, `sitl/console_mission.py:162-172`). |
| 7 | **Closed for the reported QP defect** | Line-circle candidates and cost relative to the original nominal are present; targeted and randomized oracle tests pass. Sampled single-integrator assumptions remain. |
| 8 | **Partial** | Longer timeout plus arrival check closes the lagging case; coordinator-loss and lost-terminal-message state remain (finding 3). |
| 9 | **Improved, not fully end-to-end verified** | Arm/takeoff helpers now poll a cancellation callback and defer operator messages. Tests check callback and helper separately, not receipt -> cancellation -> LAND mode on a fake MAVLink vehicle; no live evidence. |
| 10 | **Closed for the reported renewal case** | Acting exhaustion is latched until a new operator message; mission validity/order is checked before leader refresh. The previously reproduced re-election no longer occurs. |
| 11 | **Partial** | `/mode` shares the HTTP action lock; print-only rejection detection works for one message. `_run_capturing` redirects process-wide `sys.stdout` in a multithreaded app (`sitl/operator_dashboard.py:135-153`); other threads can contaminate the reply, and unrecognized rejection wording still defaults to accepted. Return typed command results instead. |
| 12 | **Closed at packaging-source level** | `.[all]` now declares pyflakes and the local lint passes. Clean 3.10-3.12 CI was not checked; `tests/test_packaging.py` skips its assertion on Python 3.10 because `tomllib` is absent. |
| 13 | **Partial** | Vacuous `or True` removed; new targeted tests, an oracle and one in-memory concurrency test help. Retry, LAND, protocol-loss and final MAVLink output interleavings still lack full integration coverage (findings 2, 3, 5). |
| 14 | **Closed as a scoped inventory** | Generator includes `*Params` and selected constructor defaults and explicitly names excluded categories; it should not be presented as every tunable. |
| 15 | **Partial; current docs need another update** | `docs/PROJECT_STATE.md:3,60` now contains blank test counts; section 8 still says the reviewed findings are open and that the near-miss cannot be distinguished from estimate error, although the new SITL truth report found 2.016 m. Reconcile with 827 local passing tests and the new FAIL, retaining earlier runs as history. |
| 16 | **Partial** | Old report language was narrowed and true SITL positions are now logged. `docs/reports/REALISTIC_SIMULATION_2026-11-03.md:94-98` still calls truth logging an open item and does not include the new FAIL. The one-run, assumed-disturbance and sampled-evidence limits remain. |

#### Decision on the drift investigation

**Yes: Claude should proceed as writer after this review handoff.** The new SITL run measures a true 2.016 m minimum against a 2.5 m gate, so drift after decentralized patrol -> centralized is a real, higher-priority failure. Investigate before claiming a successful realistic ten-drone mission. Compare time-aligned simulator truth, estimated position, planned slot, guidance setpoint before/after slew, final MAVLink target, barrier activity, route position, mode-switch timing and membership for the affected drones. Separate pre-switch drift from the post-switch convergence command, reproduce with/without GPS noise and the barrier, make a focused fix, then rerun matched scenarios with multiple seeds and truth coverage. Do not infer the cause from the candidate setpoint-drift hypothesis alone. Coordinate with the user before adding new plan items to `PROJECT_STATE.md`, per its section 8 rule. This review does not authorize a master merge or physical flight.

- Commit created by Codex: none. Push/merge/PR: none. Next handoff: Claude may resume the drift investigation as sole code writer; reviewer findings above are ready for prioritization.

### 2026-11-04 (overnight) — Claude — near-miss root cause, flyaway guard, repeat campaigns

- Report: `docs/reports/OVERNIGHT_CAMPAIGN_2026-11-04.md` (root cause from dataflash logs, decisions with reasons, campaign table).
- Commits `fd3b1a8`..(this) pushed. Final code: realistic 2/3, tower gap 2/3 (true worst 1.71 / 2.04 m), calm 3/3; Gazebo patrol and gap PASS (1 run each).
- Open (user decision): partial-fleet formation changes after a drone lands.
- Codex: please include the overnight changes in your re-review (`sitl/estimate_health.py` flyaway check, `console_mission._tick_gap` re-plan, `run_metrics` truth/completeness).

### 2026-11-04 evening — Claude — tower-gap mechanism, moving-target guard, strict run grade

- Objective: Codex points 1, 3, 4 (separate the tower-gap failure; moving-target guard blind spot; PASS gates). User approved the work list now in PROJECT_STATE section 8.
- Tower-gap failure (campaign d gap-realistic-3, D1-D2 true 2.04 m), from the event timeline: no yaw reset. After the column transition completed, agents resumed the pre-transition goal ~11 m from the plan centroid; targets slewed at 0.45 m/s, D1 lagged > 3 m, the guard flagged a flyaway and froze it ~11 s, and D2 closed from 8 m to 5 m planned spacing.
- Fix `039b44c`: console re-bases the goal to the plan centroid on transition_complete; agents hold the centroid until an intent issued after completion arrives; flyaway allowance = 3 m + 4 s x smoothed target speed (replaces the skip above 0.5 m/s). Unit + integration tests.
- Strict grade (this commit): `sitl/run_grade.py` — PASS needs live PASS, all driver steps OK and agents alive, every drone flew and ends landed + disarmed, no non-operator landing while flying, truth coverage >= 95 % per drone. Late reasons after an operator LAND (Gazebo shutdown telemetry_timeout) do not count.
- Evidence (SITL, 10 drones, --realistic, 3 runs per cell, not matched seeds): campaign f tower-gap strict 2 PASS / 1 INCOMPLETE, true worst 3.32 m; realistic 1 PASS / 2 FAIL (true 2.07, 1.83 m), the EKF-related mechanism is untouched. Campaign e stopped (mixed code) and kept as `campaign-20261104e-mixed-code-stopped`.
- Known limit: kill-agent scenarios (election_10) will grade INCOMPLETE until a scenario can declare an expected landing reason.
- Next: item 2 (real decentralized control).
