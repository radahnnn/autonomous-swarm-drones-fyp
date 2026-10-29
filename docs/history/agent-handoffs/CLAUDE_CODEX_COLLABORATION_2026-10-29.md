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
