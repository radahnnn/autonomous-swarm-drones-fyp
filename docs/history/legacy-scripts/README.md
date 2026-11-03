# Legacy scripts (archived 2026-11-03)

Early experiments from the first months of the project (Oct 2026), moved out of `sitl/` because nothing in the
current system imports, launches or tests them (checked: only old docs and the name list in `sitl/stop_all.sh` mention
them). Kept for history; they are not maintained and may not run against the current code.

| Script | What it was |
|---|---|
| `swarm_3_drones.py` | first 3-drone SITL swarm controller |
| `autonomous_wingman.py`, `qgc_swarm_wingman.py`, `qgc_swarm_wingman_5drones.py` | leader-follower "wingman" demos with QGroundControl |
| `interactive_swarm_flight.py` | early interactive flight console (superseded by `sitl/swarm_console.py`) |
| `sitl_bridge.py` | early SITL bridge (superseded by the independent agent runtime) |
| `live_radar.py`, `sync_hud.py` | early live views (superseded by `sitl/operator_dashboard.py`) |
| `diag_takeoff.py`, `diag_telemetry.py` | one-off takeoff / telemetry diagnostics |
| `test_5drones_takeoff.py`, `verify_5drones.py`, `verify_swarm.py` | early manual verification scripts (not part of the test suite) |
