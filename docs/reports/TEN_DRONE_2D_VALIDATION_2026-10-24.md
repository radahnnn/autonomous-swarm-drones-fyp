# Ten-drone 2D/SITL software validation — 2026-10-24

## Scope

Ten standalone ArduPilot SITL vehicles were exercised through the 2D MAVLink path. This validates
software and SITL behavior on one host; it does not validate a physical radio network, companion
computers, or physical flight. A generated ten-model Gazebo world was loaded in a headless
one-iteration smoke test; a full 10-drone Gazebo-to-ArduPilot flight was not run.

## Results

| Mode / experiment | Result |
| --- | --- |
| Centralized console, 10 vehicles | Pass. All ten connected, became EKF-ready, took off, formed, moved the centroid 2 m north, then landed and disarmed. Minimum sampled separation was 4.17 m; settled slot-error p95 was 0.044 m. |
| Independent decentralized SITL, 10 vehicles | Pass on a fresh rerun. All agents accepted the shared north goal, converged within 0.5 m horizontal and vertical tolerance, recovered from a directed peer-link interruption, then landed and disarmed. |
| Independent hybrid SITL, 10 vehicles | Pass on a fresh burst-loss rerun. All agents converged, switched to local fallback on coordinator loss, recovered after restoration, stayed airborne for a 20 s soak, then landed and disarmed. Soak minimum sampled separation was 4.22 m against the 2.5 m gate. |
| Matched three-mode SITL mission | Pass. One 10-drone run per mode used V formation, 5 m altitude, a 2 m north goal, and a 0.5 m convergence tolerance. Final aligned p95 3D slot error: centralized 0.040 m, decentralized 0.309 m, hybrid 0.243 m. Minimum sampled separations: 4.170 m, 4.126 m, and 4.158 m respectively. All converged and landed disarmed. |
| Paired three-mode simulation, 10 vehicles | 10 seeds per condition, 30 s per trial, paired seeds/configuration, 95% t intervals. All three modes passed 10/10 trials in each of the nominal-impaired, repeated-outage, and burst-repeated-outage conditions. The lowest sampled pair separation was 2.502 m, just above the 2.5 m project threshold. |

Under burst repeated outages, mean p95 formation error was 0.117 m for hybrid, 0.206 m for
decentralized, and 0.117 m for centralized. Mean minimum separation was 2.632 m for hybrid
(95% CI 2.585–2.678 m), 2.696 m for decentralized (2.638–2.753 m), and 2.689 m for centralized
(2.649–2.729 m). The narrowest observed clearance was 2.502 m in one hybrid trial, so the pass
margin is small. See every per-trial result in `runs/comparison/three_modes_10drones_20261024.json`.

## Interpretation

The live 10-drone 2D path is operational in all three modes. The hybrid path now uses the
independent per-agent runtime, and the live test verified coordinator-loss fallback and recovery.
The repeated-burst comparison passed the project’s sampled 2.5 m gate after fixing formation
target initialization and adding a velocity-aware local separation intervention. The pure
decentralized controller uses a 1.5 s anticipation window; centralized and hybrid use 1.0 s.
The narrowest observation was only 2 mm above the threshold, so this is not a robust safety
margin. The gate is based on sampled point-mass positions and is not a continuous collision
guarantee.

The live SITL comparison uses one run per mode. Decentralized and hybrid used the same seed and
nominal peer channel (2% packet loss, 25 ms mean latency, 5 ms jitter); centralized has no
peer-to-peer channel by design. The terminal error is a one-snapshot p95 across ten vehicles, so
these three values are descriptive, not statistically significant. The separate pure-Python
comparison supplies ten paired seeds per mode and condition with 95% intervals. The independent
SITL path uses per-agent processes and direct loopback UDP, not separate computers or real drone
radios. Raw independent-run traces remain local under timestamped `runs/independent/` folders.
The centralized event stream is retained locally at
`runs/centralized/standalone-10-20261024/events.jsonl`; its compact summary is kept with the
repository. The normalized matched SITL result is
`runs/comparison/three_mode_sitl_10drones_20261024.json`.

The current raw independent run folders are:

- Decentralized 10-drone rerun: `runs/independent/external-sitl-20261024T101746.150977Z/`
- Hybrid nominal matched run: `runs/independent/external-sitl-20261024T102501.767100Z/`
- Hybrid burst-loss, coordinator-switch, and 20 s soak rerun:
  `runs/independent/external-sitl-20261024T101921.729225Z/`
- Centralized console run: `runs/centralized/standalone-10-20261024/`

The full Python test suite passed after these changes. The generated ten-model Gazebo world loaded
in a headless one-iteration smoke test, but the ten ArduPilot SITL instances were not connected to
it for takeoff or movement. The measured mission results here are for 2D SITL and the point-mass
engine comparison.

## Reproduction

```bash
python -m experiments.compare_three_modes --drones 10 --trials 10 \
  --output runs/comparison/three_modes_10drones_20261024.json

python -m experiments.validate_external_agent_sitl --drones 10 \
  --control-mode decentralized --seed 1024

python -m experiments.validate_external_agent_sitl --drones 10 \
  --control-mode hybrid --seed 1024

python -m experiments.summarize_three_mode_sitl_comparison \
  --central-summary runs/centralized/standalone-10-20261024/summary.json \
  --central-events runs/centralized/standalone-10-20261024/events.jsonl \
  --decentralized-summary runs/independent/external-sitl-20261024T101746.150977Z/external_agent_sitl_summary.json \
  --hybrid-summary runs/independent/external-sitl-20261024T102501.767100Z/external_agent_sitl_summary.json \
  --output runs/comparison/three_mode_sitl_10drones_20261024.json

python -m experiments.validate_external_agent_sitl --drones 10 \
  --control-mode hybrid --seed 20261024 --peer-burst-loss \
  --test-coordinator-switch --soak-seconds 20

python -m sitl.swarm_console --drones 10 --standalone \
  --control-mode centralized --auto-launch
```

Each independent run writes a configuration hash, SITL-parameter hash, Git revision and dirty-tree
state, software environment, JSON summary, per-agent logs, and operator event stream. The generated
HMAC key is local-only and must not be published.
