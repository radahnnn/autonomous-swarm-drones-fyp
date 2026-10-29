# Claude One-Week Development Prompt

Paste the text below into Claude at the beginning of the development week.

---

You are the primary implementer for one week on my swarm-drone FYP software.

## Repository

- GitHub: `https://github.com/radahnnn/autonomous-swarm-drones-fyp`
- Local checkout: `/home/drone/Documents/Codex/FYP_Consolidated_2026-10-12/working-copies/decentralized-sitl-20261020`
- Branch: `wip/decentralized-sitl-handoff-20261029`
- WIP source checkpoint: `d1e7444`
- Collaboration-log commit: `2efa526`
- Verified external backup: `/home/drone/Documents/Codex/preserved-software-20261027`

Do not work on `master`. Do not merge `origin/master` unless the user and reviewer explicitly plan that integration.

## Shared development record

Read this file before changing code:

`docs/history/agent-handoffs/CLAUDE_CODEX_COLLABORATION_2026-10-29.md`

Append a session entry to that file during every meaningful work session and before handing control back. Record:

- objective and starting commit
- files inspected and changed
- design decisions
- exact test commands and results
- evidence level: UNIT, INTEGRATION, SITL, QGC, GAZEBO, VM-NETWORK, or PHYSICAL
- failures, limitations, and unresolved risks
- commit and push status
- what Codex should review
- next step

Do not rewrite previous entries or report mocked tests as live SITL evidence.

## Collaboration model

- Claude is the primary writer during this week.
- Codex is the reviewer and may inspect diffs, tests, risks, and integration status between Claude sessions.
- Only one assistant may edit this checkout at a time.
- Before editing, confirm that no other assistant is currently writing to it.
- Keep each completed fix in a focused commit.
- Leave the working tree clean at handoff whenever practical.
- Do not push, merge, rewrite history, or open a pull request without the user's instruction.

## Safety rules

- Preserve the external backup.
- Do not use `git reset --hard`, `git clean`, force-push, automatic rebase, or destructive history rewriting.
- Do not delete runs, logs, evidence, backups, branches, or hardware files.
- Do not use broad `pkill -9 -f` cleanup commands.
- Keep credentials, caches, generated logs, raw runs, and large evidence files out of Git.
- Check disk space before full tests, SITL, QGroundControl, Gazebo, or VM experiments.
- Confirm process and port ownership before starting or stopping live simulations.
- Do not modify physical-drone parameters or hardware files unless explicitly assigned.

## Current project position

The project supports centralized, decentralized, and hybrid swarm-control research with ArduPilot SITL, QGroundControl, Gazebo, a browser dashboard, independent per-vehicle agents, formation control, failure experiments, and operation intended to scale from five to ten simulated drones.

The current formation-transition implementation uses centralized planning even when independent agents execute their own assigned trajectories. Describe this accurately as distributed execution with centralized transition planning. Do not claim fully decentralized formation negotiation yet.

Physical swarm communication and physical flight have not been validated. One drone is assembled but has not been flight-tested. ESP32/CAN work remains proposed hardware integration, and an MCP2551 alone is only a CAN transceiver.

## Current WIP contents

The checkpoint includes:

- collision-screened formation-transition planning
- formation fields in `MissionIntent`
- independent-agent transition execution
- independent transition monitoring and arrival checks
- central/local airborne formation morphing
- dashboard preflight, formation, failure, recovery, and mode controls
- grounded mode supervision
- MAVLink source filtering in the independent agent
- private SITL runtime directories and QGroundControl mirroring
- SITL sensor profiles
- associated unit tests

It is a preservation checkpoint, not release-ready software.

## Priority backlog

Work in this order unless repository evidence shows a more urgent blocker:

1. Fix stale/out-of-order `MissionIntent` handling so validation happens before any state mutation.
2. Add regression tests proving stale or replayed messages cannot alter transition state.
3. Design and implement prepare/acknowledge/common-start/abort behavior for synchronized formation transitions.
4. Add central/local transition supervision for telemetry freshness, vehicle state, live separation, timeout, abort, actual arrival, and arrival dwell.
5. Block conflicting commands during transitions.
6. Define safe behavior for link loss, membership change, coordinator failure, and killed agents.
7. Fix the dashboard's `session.count` versus `SwarmConsole.n` mismatch and structured command-result reporting.
8. Add mode-supervisor child-readiness checks and rollback.
9. Fix clean-CI dependency installation for Python 3.10, 3.11, and 3.12.
10. Consolidate or clearly separate the multiple formation and slot-assignment implementations.
11. Replace broad process cleanup with ownership-based shutdown in a separate change.
12. Validate five- and ten-drone centralized, decentralized, and hybrid behavior after the lower-level tests pass.

## First session

Start with a read-only inspection:

1. Verify repository root, branch, HEAD, remote, status, disk space, and backup readability.
2. Read the collaboration log, relevant source, tests, and recent Git history.
3. Confirm or challenge the stated first defect with file and symbol references.
4. Explain the proposed fix and tests in plain language.
5. Then implement only the stale/out-of-order mission correction and focused regression tests.
6. Run targeted tests that do not generate large artifacts.
7. Review the diff for unrelated changes.
8. Append the complete result to the collaboration log.
9. Report the result to the user and identify what Codex should review next.

If the checkout, branch, backup, or status differs materially from this prompt, stop before editing and report the difference.

---
