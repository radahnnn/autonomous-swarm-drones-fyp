# Swarm console run evidence

Each invocation of `sitl/swarm_console.py` creates a new UTC-named folder under `runs/console/` before it launches or connects to SITL. The console supports 1–10 drones. Use `--evidence-dir PATH` to choose another parent folder. The folder contains `events.jsonl` and, when the console exits, `summary.json`. The runtime folder is ignored by Git; archive chosen results deliberately with the report.

`events.jsonl` records the Git commit, requested fleet size and mode, connections, EKF readiness, pilot commands, launch/formation/fault/patrol/landing events, and 10 Hz samples. Each sample carries timestamp, actual and commanded global NED positions, active state, telemetry freshness, heartbeat age, formation, leader, slot errors, and the closest observed horizontal separation. Errors use a position fix no older than one second. The p95 tracking result uses only samples at least five seconds after a target/topology change and after morphing finishes. `patrol_commands_complete` means the four legs were issued; it does not by itself prove accurate patrol tracking.

## Criteria fixed before a run

The summary marks a run **pass** only if all of these hold:

1. Every requested vehicle connected and the EKF readiness check succeeded.
2. All vehicles were observed airborne, with at least 50 samples (about five seconds) of fresh full-fleet telemetry.
3. LAND completed and every vehicle reported disarmed.
4. Minimum observed horizontal separation between fresh, active vehicle positions was at least **2.5 m**.
5. The 95th percentile of settled 3D slot error was at most **1.0 m**.
6. Maximum observed heartbeat age for an active vehicle stayed within that run's watchdog budget (**3.5 s** standalone, **25 s** Gazebo), with no unexpected watchdog timeout.

These are project evaluation thresholds, not certified physical-flight limits. A run without enough observations fails instead of silently passing. The summary reports each check separately, so an incomplete run can still be diagnosed. Intentional `f <id>` injection remains recorded; it is distinct from an unexpected heartbeat timeout. The Gazebo timing budget is longer because simulation may run below real time; its other thresholds have not been validated by a completed Gazebo flight.

## Reading the result

Start at `summary.json`, then inspect `events.jsonl` for the timeline and individual vehicles. Each position, target, and error array uses drone order 1 through N; absent or stale errors are `null`. A pass describes only that specific run and its logged data. Logs come from the console's received MAVLink telemetry; they do not prove collision freedom between samples, onboard decentralized fallback, real radio performance, or physical aircraft safety. Check `git_head` and `working_tree_dirty` when comparing runs.
