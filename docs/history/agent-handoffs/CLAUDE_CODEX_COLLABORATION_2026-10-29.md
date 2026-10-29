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
