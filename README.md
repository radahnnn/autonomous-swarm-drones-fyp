# Autonomous Swarm Drones

Final Year Project (BEE-60), Department of Electrical Engineering · Supervisor: Dr. Abdul Ghafoor · Status summarized: 24 October 2026

This project combines a multi-drone swarm simulator, ArduPilot Software-in-the-Loop (SITL) experiments, and a separate hardware-matched controller-identification study. Simulation and SITL results are not evidence of physical-airframe flight performance.

## Experimental branch: decentralized and hybrid SITL

This branch (`work/decentralized-sitl-20261020`) adds per-agent decentralized/hybrid guidance,
reserved-slot recovery safeguards, and optional separate controller processes. The stable demo
branch remains separate. Read the [hardening report](docs/reports/HYBRID_HARDENING_2026-10-20.md)
for implementation limits, preserved failed/passing SITL runs, and the 90-trial software comparison.
In the burst-loss/outage comparison, centralized passed 8/10, decentralized 10/10, and hybrid 9/10;
hybrid still violated the 2.5 m spacing criterion in one seed. This is not a hardware result.

```bash
python3 -m sitl.swarm_console --drones 5 --standalone --control-mode hybrid --agent-runtime process
```

The console still owns transport and mission distribution; separate processes are not independent
onboard deployment. See [the protocol](docs/AGENT_PROTOCOL.md) and [console guide](docs/CONSOLE_GUIDE.md).

## Current implementation

The default `master` branch is the demo-oriented baseline. It includes:

- A unified interactive ArduPilot SITL console in `sitl/swarm_console.py`, supporting 1–10 vehicles in standalone mode and Gazebo mode when a matching world is available.
- EKF-gated launch, coordinated takeoff and landing, status reporting, movement commands, square patrol, and failure/revive handling with leader re-election.
- Seven console formations: V, line, column, diamond, ring, echelon, and triangle. The formation library tests slot spacing and assignment; these tests do not establish a formal collision-free guarantee.
- A separate swarm simulator with centralized, decentralized, and hybrid control experiments, plus network loss and delay emulation.
- ArduPilot/Gazebo and Matek H743 support files. Hardware configuration files are project artifacts and must not be treated as approval for real-aircraft flight.

The console usage guide is [`docs/CONSOLE_GUIDE.md`](docs/CONSOLE_GUIDE.md). The unified console now records per-run commands and telemetry under `runs/console/`; see the [evidence criteria](docs/RUN_EVIDENCE.md). A five-drone standalone SITL demo was also shown in QGroundControl on 15 October 2026. On 20 October, a five-drone Gazebo 3D run completed takeoff, pilot-commanded movement and formation changes, failure/rejoin, and landing, with all recorded criteria passing; see the [3D validation record](docs/reports/GAZEBO_5DRONE_VALIDATION_2026-10-20.md).

## Branches and project state

For the plain-language map of local folders, branches, and which copy to use, read the [project state guide on `master`](https://github.com/radahnnn/autonomous-swarm-drones-fyp/blob/master/docs/PROJECT_STATE.md). This experimental branch remains separate from stable `master`. For the ten-drone evidence, read the [live 2D validation report](docs/reports/TEN_DRONE_2D_VALIDATION_2026-10-24.md) and the [30-seed repeated comparison](docs/reports/TEN_DRONE_2D_REPEATED_VALIDATION_2026-10-25.md). The repeated comparison passes its numerical gates but has only a 2 mm worst clearance margin under burst outages.

| Branch or tag | Purpose | State |
|---|---|---|
| `master` | Standalone 2D demo baseline and run evidence | The earlier `demo-ready-2026-10-12` tag remains at `2620f585`; `demo-2d-verified-2026-10-12` marks the consolidated five-drone demo checkpoint. |
| `work/decentralized-sitl-20261020` | Experimental per-agent control, hybrid recovery and process isolation | Changes and evidence committed separately from the stable demo. |
| `work/corrected-phase2-20261020` | Corrected identification Phase 2 timing and profile study | Separate identification checkout; inspect its report for corrected metrics and limitations. |
| `archive/*` | Preserved experimental working snapshots | Historical/recovery branches; not stable demo baselines. |
| `hw-matched-identification` | SITL position-controller identification study | Kept separate from the demo branch. Its report identifies a timing bug that made the old `tau=0.992 s`, `c_d=0.637 s^-1` fit an artifact. Do not present those values as validated parameters. |
| `checkpoint/2d-working-savepoint` | Earlier 2D working state | Preserved on GitHub as a historical savepoint; not part of the current demo baseline. |

The identification report compares several software models. An ArduPilot-like position/velocity cascade fit the recorded step responses best among those compared. In a five-drone SITL replay, it reported 13.1 cm mean horizontal RMSE versus 30.4 cm for the old fitted parameters, with an 85 cm worst single error. The corrected S1 timing profile was used in a separate Phase 2 sensitivity rerun. The more accurate S4 cascade model has not yet been integrated into Phase 2; see the [corrected Phase 2 report](https://github.com/radahnnn/autonomous-swarm-drones-fyp/blob/work/corrected-phase2-20261020/docs/reports/PHASE2_CORRECTED_TIMING_2026-10-20.md).

The identification results are preliminary: one default SITL airframe, two repeats per step condition, no confidence intervals, and horizontal motion only. The study does not identify physical mass, thrust, or inertia and has not been validated against real-flight logs. See [identification report on the research branch](https://github.com/radahnnn/autonomous-swarm-drones-fyp/blob/hw-matched-identification/docs/reports/HW_MATCHED_IDENTIFICATION_REPORT.md) for methods, raw-data locations, comparisons, and limitations.

## Verification record (12–20 October 2026)

These are results from local consolidated copies on one development machine. The 12 October flight result predates the run-evidence instrumentation:

- The consolidated demo copy's Python suite passed 161 tests; the identification copy's suite passed 168 tests. The identification count includes a local timing regression test. Two preserved checkpoint suites also passed 10 tests each.
- A five-drone standalone SITL session completed takeoff, formation change, movement, square patrol, failure/re-election, revive/rejoin, and landing. The consolidated-copy landing fix waits for all vehicles to report disarmed before stopping SITL.
- In the earlier Gazebo check, the five-drone world rendered on the NVIDIA GPU and all five SITL instances connected. The console then remained at EKF flags `0xa7` (GPS fix 6), so the EKF readiness gate blocked flight in that run.
- On 20 October, after using one clean Gazebo server and allowing about 220 seconds for EKF convergence, all five drones took off and landed in a pilot-commanded 3D SITL run. The recorded minimum separation was 2.594 m and the 95th percentile settled slot error was 0.211 m. The run summary passed every recorded criterion. This validates that specific simulated run, not physical flight or every formation and failure scenario.
- No physical-airframe flight was performed as part of these checks.
- Later instrumented five-drone standalone runs on 14 October landed but did not pass all [recorded acceptance criteria](docs/RUN_EVIDENCE.md). The 15 October defense demonstration was visual evidence of a working 2D SITL display and control path, not a measured safety or tracking result.

## Work kept separate

Earlier experimental work and presentation drafts are historical artifacts. The current experimental implementation is committed on the separate branch named above, with both failed and passing evidence retained. It has not been merged into the stable demo baseline. The corrected timing work is on its separate identification branch. Physical-aircraft communication and flight tests remain future work.

## Quick start

Create an environment, install project dependencies, and run the tests:

```bash
git clone https://github.com/radahnnn/autonomous-swarm-drones-fyp.git
cd autonomous-swarm-drones-fyp
python3 -m pip install -e ".[dev]"
python3 -m pip install -e ".[sitl]"  # needed for live MAVLink/SITL use
python3 -m pytest -q tests
```

Start the standalone five-drone console:

```bash
python3 -m sitl.swarm_console --drones 5 --standalone
```

For Gazebo, first start the matching Gazebo world/plugin as described in the console guide, then run:

```bash
python3 -m sitl.swarm_console --drones 5 --gazebo
```

The five-drone Gazebo run passed the console's EKF readiness gate after convergence on 20 October. See the validation report above for that run's results and limits.

## Main directories

```text
sitl/          ArduPilot console, launchers, fleet layout, and formation library
swarm_core/    Swarm models and control logic
simulator/     Numerical simulation engine and visualization
experiments/   Identification, simulation, and SITL experiment scripts
hardware/      Matek H743 diagnostics, parameters, and hardware notes
docs/          Console guide, research reports, and project records
tests/         Automated project tests
```
