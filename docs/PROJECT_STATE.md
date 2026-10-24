# Project state and where to work

Last reconciled: 25 October 2026.

## GitHub branches

All seven published branches belong to [one GitHub repository](https://github.com/radahnnn/autonomous-swarm-drones-fyp). The two local Git databases described below both use that same remote.

| Branch | Meaning |
|---|---|
| [`master`](https://github.com/radahnnn/autonomous-swarm-drones-fyp/tree/master) | Stable five-drone demo and primary landing page. |
| [`work/decentralized-sitl-20261020`](https://github.com/radahnnn/autonomous-swarm-drones-fyp/tree/work/decentralized-sitl-20261020) | Current 10-drone 2D experimental control line. |
| [`hw-matched-identification`](https://github.com/radahnnn/autonomous-swarm-drones-fyp/tree/hw-matched-identification) | Original SITL controller-identification study. |
| [`work/corrected-phase2-20261020`](https://github.com/radahnnn/autonomous-swarm-drones-fyp/tree/work/corrected-phase2-20261020) | Corrected step timing and Phase 2 sensitivity results. |
| [`checkpoint/2d-working-savepoint`](https://github.com/radahnnn/autonomous-swarm-drones-fyp/tree/checkpoint/2d-working-savepoint) | Historical working 2D state. |
| [`archive/live-hybrid-checkpoint-20261020`](https://github.com/radahnnn/autonomous-swarm-drones-fyp/tree/archive/live-hybrid-checkpoint-20261020) | Earlier live hybrid prototype and records. |
| [`archive/hw-identification-working-copy-20261024`](https://github.com/radahnnn/autonomous-swarm-drones-fyp/tree/archive/hw-identification-working-copy-20261024) | Preserved identification working copy and raw runs. |

Older local branch tips such as `milestone/phase1-stable`, `unified-swarm-console`, and `verified-3d-20261020` are ancestors of `master`. Their commits remain in GitHub history. The historical tags below mark the useful restore points without seven more branch names.

## How the project reached this state

| Stage | Git checkpoint | Where to read more |
|---|---|---|
| Baseline audit and Phase 1 foundations | `v0-baseline`, `v1.0-phase1-complete` | [`docs/BASELINE_AUDIT.md`](BASELINE_AUDIT.md) |
| Early 2D and three-drone formation work | `v1.0-savepoint-2d-working`, `v2.0-3drones-vformation` | [`docs/logs/FYP_RUNNING_LOG.md`](logs/FYP_RUNNING_LOG.md) |
| Gazebo digital-twin assets | `v2.1-gazebo-3d-digital-twins` | [`docs/CONSOLE_GUIDE.md`](CONSOLE_GUIDE.md) |
| Unified five-drone console, 12 October | `demo-ready-2026-10-12` | [`docs/CONSOLE_GUIDE.md`](CONSOLE_GUIDE.md) |
| Verified five-drone 2D checkpoint, 14 October | `demo-2d-verified-2026-10-12` | [`docs/RUN_EVIDENCE.md`](RUN_EVIDENCE.md) |
| Five-drone Gazebo flight, 20 October | `master` history | [3D validation report](reports/GAZEBO_5DRONE_VALIDATION_2026-10-20.md) |
| Independent agents and 10-drone 2D modes, 24 October | `work/decentralized-sitl-20261020` | [10-drone report](https://github.com/radahnnn/autonomous-swarm-drones-fyp/blob/work/decentralized-sitl-20261020/docs/reports/TEN_DRONE_2D_VALIDATION_2026-10-24.md) |
| Identification timing correction and Phase 2 rerun | `work/corrected-phase2-20261020` | [corrected Phase 2 report](https://github.com/radahnnn/autonomous-swarm-drones-fyp/blob/work/corrected-phase2-20261020/docs/reports/PHASE2_CORRECTED_TIMING_2026-10-20.md) |

Tags and branches are retained for recovery. A tag identifies a historical commit; it does not mean every feature at that stage passed a later acceptance test.

The older Claude/session handoffs are preserved under [`docs/history/agent-handoffs/`](history/agent-handoffs/README.md). They describe past states and are separate from the current run instructions.

## The short version

This project is stored in several Git working copies under `working-copies/`. Each is a separate checkout, and the hardware-identification checkout has its own Git repository. Do not edit an old backup when you mean to change the current project.

| Local folder | Branch | Use |
|---|---|---|
| `working-copies/demo-master/` | `master` | Stable, demo-oriented baseline. Start here for normal project work and the five-drone 2D/verified 3D baseline. |
| `working-copies/decentralized-sitl-20261020/` | `work/decentralized-sitl-20261020` | Experimental 10-drone control/dashboard work and centralized, decentralized, and hybrid comparisons. Kept separate from `master`. |
| `working-copies/experimental-hybrid-20261020/` | `archive/live-hybrid-checkpoint-20261020` | Preserved live hybrid checkpoint and run records. Research snapshot, not the stable demo. |
| `working-copies/hw-matched-identification/` | `archive/hw-identification-working-copy-20261024` | Preserved working copy including raw hardware-matched identification runs. Its Git repository is separate from the main checkout. |
| `working-copies/identification-phase2-20261020/` | `work/corrected-phase2-20261020` | Corrected uneven-timestamp handling, regression coverage, and corrected Phase 2 profile results. |

The folders are not automatically synchronized. A commit exists only in the Git repository containing that checkout. The matching branch must then be pushed to GitHub before it is backed up there.

## Which branch is the reference?

- `master` is the stable reference for the demo console and verified five-drone baseline.
- `work/decentralized-sitl-20261020` is the current experimental line for scaling and comparing control modes. It is not merged into `master`.
- `hw-matched-identification` is the published identification study. `work/corrected-phase2-20261020` contains follow-up corrections and results; the archive branch preserves the separate dirty working copy and its raw runs.
- `archive/*` branches are snapshots kept for traceability. Do not develop on them unless intentionally recovering that snapshot.

## Evidence and limits

Use the validation reports linked from each branch README and `docs/reports/`. A test or SITL result supports only the code, configuration, and scenario actually run. The 10-drone branch is software/SITL work; separate controller processes on one laptop are not the same as independent computers aboard aircraft. None of these results establish physical flight readiness.

Generated logs and machine-specific artifacts may be ignored by Git. They can remain in the local folders without being on GitHub. Check each branch's `runs/README.md` and `.gitignore` before assuming every raw run is part of the GitHub copy. Do not commit credentials, private keys, or generated secrets.

## Quick orientation

```bash
cd ~/Documents/Codex/FYP_Consolidated_2026-10-12/working-copies/demo-master
git status --short --branch
git branch --show-current
```

For the experimental 10-drone work, change directory to `decentralized-sitl-20261020/` and confirm the branch before editing. For hardware-matched identification, use the dedicated identification checkout and confirm its remote and branch; it is a separate Git repository.
