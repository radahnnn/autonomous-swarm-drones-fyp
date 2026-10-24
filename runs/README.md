# Validation evidence policy

This directory separates compact, reviewable evidence from local simulator output.

## Kept in Git

- Profile-checked JSON batch summaries directly under `runs/independent/`.
- Small reproducible configuration files under `runs/independent/configs/`.
- This policy file.

## Kept locally

Timestamped `runs/independent/<timestamp>/`, `external-sitl-*`, and `multihost-sitl-*`
directories contain raw status streams, process manifests, console output, and simulator logs.
They are intentionally ignored because repeated validation generates many files and potentially
very large logs. The reports name the local source directory for every stated result. Ignoring a
directory does not delete it.

Per-run `*.key` files are generated HMAC test secrets and must not be published. Runtime `*.log`,
`*.bin`, and `*.raw` files are also ignored globally.

## Reproduce an aggregate

Run `experiments/summarize_external_sitl_runs.py` with the five source
`external_agent_sitl_summary.json` paths. The summarizer refuses to combine different impairment
profiles or incompatible operator-loss/restart modes. A batch summary records its source paths,
seeds, outcome counts, Wilson interval, landing checks, separation, and transport counters.

Raw local evidence should be copied into an external archive when long-term retention is required;
Git is not the storage layer for multi-gigabyte SITL logs.
