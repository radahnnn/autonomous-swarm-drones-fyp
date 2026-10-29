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
