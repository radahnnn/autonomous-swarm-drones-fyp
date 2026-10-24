# Validation evidence policy

This directory separates compact, reviewable evidence from local simulator output.

## Kept in Git

- Profile-checked JSON batch summaries directly under `runs/independent/`.
- Small reproducible configuration files under `runs/independent/configs/`.
- Compact, multi-seed comparison reports under `runs/comparison/`.
- Selected centralized SITL `summary.json` files under `runs/centralized/`.
- This policy file.

## Kept locally

Timestamped `runs/independent/<timestamp>/`, `external-sitl-*`, and `multihost-sitl-*`
directories contain raw status streams, process manifests, console output, and simulator logs.
They are intentionally ignored because repeated validation generates many files and potentially
very large logs. The reports name the local source directory for every stated result. Ignoring a
directory does not delete it.

Large centralized `events.jsonl` traces are also local and ignored; selected compact
`runs/centralized/<run>/summary.json` files are retained. The ten-drone baseline and comparison
results from 2026-10-24 are documented in
`docs/reports/TEN_DRONE_2D_VALIDATION_2026-10-24.md`.

Per-run `*.key` files are generated HMAC test secrets and must not be published. Runtime `*.log`,
`*.bin`, and `*.raw` files are also ignored globally.

## Reproduce an aggregate

Run `experiments/summarize_external_sitl_runs.py` with source
`external_agent_sitl_summary.json` paths. The summarizer refuses to combine different swarm sizes,
control modes, impairment profiles, or incompatible operator-loss/restart modes. A batch summary
records its source paths, seeds, outcome counts, Wilson interval, landing checks, separation, and
transport counters.

Raw local evidence should be copied into an external archive when long-term retention is required;
Git is not the storage layer for multi-gigabyte SITL logs.
