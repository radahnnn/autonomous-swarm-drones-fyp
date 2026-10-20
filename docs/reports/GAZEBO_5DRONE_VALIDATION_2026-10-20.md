# Five-drone Gazebo 3D SITL validation — 20 October 2026

This is a record of one completed **simulated** flight on the Ubuntu development laptop. It does not validate the physical quad, the experimental live hybrid controller, or every supported fleet size.

## Result

The five-drone Gazebo world ran with one server, five ArduCopter JSON SITL instances, and the unified console in its default centralized mode. All five vehicles connected and reached EKF ready. The operator then commanded launch, north/south movement, line and V formations, a failure of drone 1, its revive, and landing. The console confirmed all five were airborne after launch and disarmed after landing.

The [unchanged run summary](evidence/2026-10-20-gazebo-5drones-summary.json) reports:

| Measure | Recorded result |
|---|---:|
| Outcome | landed; `pass: true` |
| Five-drone samples | 2,660 |
| Closest recorded horizontal separation | 2.594 m (criterion: at least 2.5 m) |
| 95th percentile settled slot error | 0.211 m (criterion: at most 1 m) |
| Maximum recorded heartbeat age | 1.566 s (criterion: at most 25 s) |
| Unexpected watchdog timeouts | 0 |

The local run folder is `runs/console/20261020T150642.843011Z/`. Its `events.jsonl` holds the full timestamped command and telemetry trace; generated run folders are ignored by Git because they can be large. The summary above is preserved here as a small, reviewable record.

## Startup finding

The first attempts were affected by two Gazebo servers owning the same five FDM ports. After closing the duplicate, the first clean run reached EKF ready after about 211 wall-clock seconds. A second SITL attempt against that same Gazebo process stalled after SITL disconnected. Restarting Gazebo and SITL together produced this completed flight; EKF ready was logged at 219.8 seconds. The Gazebo server ran at roughly 0.16–0.22 real-time factor during startup. A 120-second EKF cutoff was therefore insufficient on this machine.

The console previously left failed TCP connection candidates open during retries, filling ArduPilot's small listen backlog and producing connection timeout errors. The connection retry fix closes unsuccessful candidates. The five-drone Gazebo launcher now refuses to open another world when drone port 9002 is already in use.

## Provenance and limits

The run's `run_start` event recorded Git HEAD `92797eb53a29f890cf3c37b0c184db0177705de7` and `working_tree_dirty: true`. That checkout also contained experimental hybrid-control changes, but this flight used the default **centralized** mode. This update includes the validated TCP retry and explicit SysID changes; it keeps the unfinished hybrid work separate. The preserved summary is evidence for the observed run, not a claim that this exact new commit was independently reflown.

The spacing and tracking figures come from the console's recorded telemetry and acceptance criteria. They do not prove a formal collision-free guarantee or physical-airframe performance. No real drone was flown.
