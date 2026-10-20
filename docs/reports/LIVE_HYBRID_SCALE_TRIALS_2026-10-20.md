# Live hybrid guidance and scale trials — 20 October 2026

## Scope

These are standalone ArduCopter SITL runs using the experimental host-side hybrid adapter.
They do not validate real aircraft, independent onboard coordination, Gazebo physics, or
physical radio/network failure. The unchanged criteria are minimum measured horizontal
separation of 2.5 m and p95 settled 3D slot error no greater than 1.0 m, plus full connection,
takeoff, heartbeat, and confirmed landing checks.

## Change made

The previous live adapter estimated velocity by dividing each position change by the 10 Hz
control-loop interval, even when the underlying MAVLink position sample was older. It also
converted the research acceleration output to a short position waypoint during healthy
coordinator operation. In the 14 October five-drone run, this path produced a roughly 26 m
slot excursion after rejoin and failed the separation and slot-error criteria.

The adapter now estimates velocity using changes in telemetry sample time and ignores duplicate
position samples. With a healthy coordinator it sends the nominal formation position target
directly, which matches the ArduPilot GUIDED position interface. The hybrid supervisor still
handles coordinator-loss state and its bounded fallback controller. Unit regression checks cover
sample timing and the healthy-coordinator setpoint.

## Results

| Fleet | Trial exercises | Result | Minimum separation | Settled slot-error p95 |
|---:|---|---|---:|---:|
| 5 | coordinator off/on, triangle morph, fail/revive/rejoin, 6 m move, land | Pass | 2.668 m | 0.750 m |
| 6 | line morph, 6 m move, coordinator off/on, land | Pass | 2.961 m | 0.192 m |
| 10 | V formation to ring morph, land | Fail: spacing criterion | 2.079 m | 0.451 m |

All three trials connected, became EKF ready, observed the whole fleet airborne with fresh
telemetry, and confirmed landing. There were no unexpected watchdog timeouts. The ten-drone
transition briefly compressed drones 8 and 10 to 2.079 m, below the 2.5 m criterion. Therefore
the console can launch ten simulated drones, but ten-drone safety through formation changes is
not validated. Do not present ten as a proven safe operating size.

The five-drone live trial's complete machine summary is
`docs/reports/evidence/2026-10-20-live-hybrid-5drones-summary.json`; similarly named files hold
the six- and ten-drone summaries. Full timestamped sample streams remain under the ignored
`runs/console/` directory in this working copy, including failed and interrupted attempts.

## Software test results

The complete Python test suite passed after the adapter change, including the added live-adapter
regression tests. The ten-drone runtime criterion still failed as described above; unit tests do
not replace that safety result.

## Interpretation and next work

The five- and six-drone results show that the corrected adapter can pass the chosen SITL mission
criteria in individual runs, including coordinator fallback and recovery. Two passing flights
are preliminary evidence, not repeatability or statistical validation. The ten-drone result
exposes a formation-transition safety gap. The current straight-line morph interpolates slot
coordinates independently; it needs a coordinated, collision-aware transition planner with
explicit separation enforcement and repeated trials before larger formations are claimed.

Next: preserve the failed 10-drone case as a regression scenario; plan collision-free transitions
between formations; rerun at least 20 randomized SITL seeds for 5 and 6 drones and the transition
scenarios for 10; compare hybrid and centralized with paired seeds; report confidence intervals;
then test Gazebo on the available GPU machine. Keep the current default centralized mode for
demonstrations until those results are in. No physical-flight claim follows from these SITL tests.
