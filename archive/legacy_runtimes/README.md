# Legacy runtimes (retired 2026-11-04)

User decision (PROJECT_STATE section 8, item 5): the independent runtime (`sitl/independent_agent_node.py` +
`sitl/independent_swarm_console.py`) is the only one. The older host-side runtimes are archived here, not deleted:

- `swarm_console_legacy.py` — the former `sitl/swarm_console.py` with the "local" and "process" runtimes
  (`SwarmConsole`, `handle_command`). Its SITL start-up/clean-up was extracted unchanged to `sitl/sitl_launcher.py`;
  its argument parsing and `--realistic` helpers stay in the new `sitl/swarm_console.py` entry point.
- `mavlink_swarm_adapter.py`, `mode_supervisor.py`, `agent_process.py`, `peer_state_bus.py` — legacy-only modules.
- `decentralized_guidance_harness.py` — the host-side `DecentralizedGuidance` harness class (the live code keeps
  `LocalDroneAgent` and `one_hop_graph` in `sitl/decentralized_guidance.py`).
- `tests/` — the tests of these paths, moved unchanged; not collected (pytest `testpaths = tests`).

These files are not imported by the live code and may not run as-is (imports point at moved modules). Git history
has every version.
