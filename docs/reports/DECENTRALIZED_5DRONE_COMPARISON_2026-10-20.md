# Five-drone decentralized progress and comparison — 2026-10-20

## Scope

This records the first five-drone decentralized SITL validation, the correction to its settling
measurement, and a paired comparison against centralized control. It is evidence for software
simulation only. It does not validate onboard execution, a real peer radio, or physical flight.

## What changed

1. **Settling evidence:** A sample is now counted as settled only after all active vehicles have
   fresh position data, the last command has been quiet for at least 2 seconds, and measured fleet
   speed has remained below 0.15 m/s for a 1.5-second dwell. Tracking error is measured separately;
   a stationary but inaccurate formation can still fail.
2. **Agent input boundary:** Peer states now pass through `PeerStateBus`. Each local controller gets
   its own telemetry-derived state and only fresh, one-hop neighbor packets. The bus emulates
   message delay, loss, range drops, stale expiry, and directed link blocking.
3. **Fault conditions:** The SITL console exposes `--peer-loss`, `--peer-latency-ms`, `--peer-seed`,
   and `--peer-block SENDER:RECEIVER` for repeatable peer-network experiments.
4. **Matched comparison:** A repeatable 10-seed experiment runs both controller modes with the
   same initial positions, mission, fitted dynamics profile, sensor noise, radio loss, and latency.
5. **Onboard stage:** The deployment boundary and required next work are documented in
   `docs/DECENTRALIZED_SITL.md`; no onboard agent runtime or hardware path was implemented.

The harness still reads all SITL telemetry, prepares the peer graph and slot configuration, and
broadcasts the common mission goal. Both the agent code and peer bus run in one host process. This
is decentralized controller logic with a simulated message boundary, not a fully distributed
drone system.

## Five-drone SITL fault run

Command configuration: standalone ArduPilot SITL, five vehicles, 10% independent peer packet loss,
100 ms mean peer latency, seed 426, and a blocked directed peer link from Drone 2 to Drone 4. After
a 4 m north movement, Drone 3 was deliberately failed through the console; the simulator commanded
it to land while four remaining agents continued in their fixed slots. The operator then landed
the group.

| Measure | Result | Check |
|---|---:|---|
| Vehicles connected, EKF-ready, airborne | 5/5 | Pass |
| Minimum measured separation | 4.139 m | Pass, ≥2.5 m criterion |
| 95th-percentile settled slot error | 0.277 m | Pass, ≤1.0 m criterion |
| Maximum heartbeat age | 0.963 s | Pass, ≤3.5 s criterion |
| Watchdog timeouts | 0 | Pass |
| Landing confirmed | All vehicles disarmed | Pass |
| Peer packets | 2,392 sent; 1,852 delivered; 185 lost; 352 range-dropped | Logged |

The `f 3` action is a controlled SITL fault injection that commands Drone 3 to land; it is not an
unannounced real radio loss or a hardware failure test. The run summary and full timeline are in
`runs/console/20261020T174629.974360Z/`.

The earlier five-drone run reported 1.999 m p95 error because it marked the formation settled
after a fixed delay even while it was still moving. That run did not satisfy the tracking criterion.
After changing the detector, this fault run measured 0.277 m. The criterion now reflects a
low-motion dwell; it does not guarantee safe trajectories between telemetry samples.

## Paired software simulation

Ten paired trials (seeds 20261020–20261029) used the same initial state within each controller
pair. Both modes used the SITL-fitted dynamics profile, a five-drone V formation, 0.08 m simulated
GPS noise, 10% packet loss, 50 ms mean channel latency, and a mission that holds, moves north at
0.6 m/s for 6 seconds, then holds. Confidence intervals below are 95% Student-t intervals across
the ten trial means; the delta interval is computed from paired per-seed differences.

| Metric | Centralized mean (95% CI) | Decentralized mean (95% CI) | Paired difference: decentralized − centralized (95% CI) |
|---|---:|---:|---:|
| Steady-window mean tracking error | 0.114 m (0.095–0.132) | 0.148 m (0.118–0.179) | +0.035 m (+0.017–+0.052) |
| Steady-window p95 tracking error | 0.152 m (0.125–0.178) | 0.192 m (0.157–0.227) | +0.040 m (+0.018–+0.063) |
| Minimum separation across each trial | 2.313 m (2.278–2.348) | 1.933 m (1.873–1.993) | −0.380 m (−0.430–−0.329) |
| Trials with simulated contact | 0/10 | 0/10 | — |

### Reading the comparison

- Under these conditions, decentralized control tracked less accurately and allowed closer
  approaches than centralized control. The intervals for the paired differences exclude zero for
  the steady tracking metrics and minimum separation.
- Neither mode had simulated contact, but **both missed the project’s 2.5 m minimum-spacing
  target** in this software experiment. The decentralized mode needs spacing/control tuning before
  it can meet that target consistently.
- These results are preliminary: ten seeds, one five-drone geometry, one mission, and one radio
  impairment setting. Do not generalize them to other formations, larger swarms, Gazebo, or hardware.

Raw paired data is in `experiments/results/decentralized_5drone_paired_20261020.json`. Rerun with:

```bash
python3 experiments/compare_5drone_architectures.py --trials 10 --seed 20261020
```

## Verification and current boundary

- Full Python test suite: **166 passed**.
- Five-drone SITL with peer impairments and the controlled vehicle-failure scenario: all logged
  acceptance checks passed.
- The SITL agent process remains on the host. The high-level goal, slot assignments, and neighbor
  graph are still configured by the console. Physical peer communications and onboard deployment
  remain unbuilt and untested.

## Recommended next work

1. Tune the decentralized controller and/or formation spacing until the paired trials meet the
   2.5 m target while preserving tracking performance.
2. Sweep packet loss, burst loss, delay, stale-message timeout, and multiple directed link cuts;
   retain failed runs and report intervals.
3. Keep formation morphing and automatic rejoin disabled until collision-aware paths and recovery
   behavior are validated.
4. Specify a versioned peer protocol: drone identity, coordinate frame, units, sequence number,
   timestamp, state age, and behavior when peer data or mission goals expire.
5. Only after software results meet requirements, implement a separate onboard agent per aircraft,
   test its approved flight-controller interface and radio on a bench, and then plan controlled
   flight tests. The project is not at that hardware-validation stage yet.
