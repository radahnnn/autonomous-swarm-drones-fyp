# Project state and where to work

Last reconciled: 24 October 2026.

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
