# Autonomous Swarm Drones

Final Year Project (BEE-60), Department of Electrical Engineering  
Supervisor: Dr. Abdul Ghafoor  
Status summarized: 25 October 2026

> **Repository state:** This branch is checked out from a separate local Git database that uses the same GitHub repository. Use `hw-matched-identification` for the original study, this branch for the uneven-timestamp correction and rerun, and `archive/hw-identification-working-copy-20261024` for the preserved raw-run snapshot. See the [project-wide branch and folder map](https://github.com/radahnnn/autonomous-swarm-drones-fyp/blob/master/docs/PROJECT_STATE.md).

> **Important correction:** The old `sitl_fitted` profile (`tau=0.992 s`, `c_d=0.637 s^-1`) was fit using a time-step assumption invalid for unevenly spaced timestamps. Do not describe it as a validated ArduPilot parameter set. See the [corrected timing report](docs/reports/PHASE2_CORRECTED_TIMING_2026-10-20.md).

This project combines a multi-drone swarm simulator, ArduPilot Software-in-the-Loop (SITL) experiments, and a separate hardware-matched controller-identification study. Simulation and SITL results are not evidence of physical-airframe flight performance.

## Current implementation

The default `master` branch is the demo-oriented baseline. It includes:

- A unified interactive ArduPilot SITL console in `sitl/swarm_console.py`, supporting 1–10 vehicles in standalone mode and Gazebo mode when a matching world is available.
- EKF-gated launch, coordinated takeoff and landing, status reporting, movement commands, square patrol, and failure/revive handling with leader re-election.
- Seven console formations: V, line, column, diamond, ring, echelon, and triangle. The formation library tests slot spacing and assignment; these tests do not establish a formal collision-free guarantee.
- A separate swarm simulator with centralized, decentralized, and hybrid control experiments, plus network loss and delay emulation.
- ArduPilot/Gazebo and Matek H743 support files. Hardware configuration files are project artifacts and must not be treated as approval for real-aircraft flight.

The console usage guide is [`docs/CONSOLE_GUIDE.md`](docs/CONSOLE_GUIDE.md).

## Branches and project state

| Branch or tag | Purpose | State |
|---|---|---|
| `master` | Demo baseline | Current five-drone 2D and verified 3D baseline. The earlier `demo-ready-2026-10-12` tag remains at `2620f585`. |
| `hw-matched-identification` | SITL position-controller identification study | Original study; its old `tau=0.992 s`, `c_d=0.637 s^-1` fit is a timing artifact. |
| `work/corrected-phase2-20261020` | Timing correction and sensitivity rerun | This branch; contains the regression test, corrected run data, and Phase 2 report. |
| `checkpoint/2d-working-savepoint` | Earlier 2D working state | Preserved as a historical savepoint, not the current demo baseline. |

The identification report compares several software models. An ArduPilot-like position/velocity cascade fit the recorded step responses best among those compared. In a five-drone SITL replay, it reported 13.1 cm mean horizontal RMSE versus 30.4 cm for the old fitted parameters, with an 85 cm worst single error. This branch reruns Phase 2 with a corrected S1 timing profile as a sensitivity comparison. The more accurate S4 cascade model has not yet been integrated into Phase 2.

The identification results are preliminary: one default SITL airframe, two repeats per step condition, no confidence intervals, and horizontal motion only. The study does not identify physical mass, thrust, or inertia and has not been validated against real-flight logs. See [`docs/reports/HW_MATCHED_IDENTIFICATION_REPORT.md`](docs/reports/HW_MATCHED_IDENTIFICATION_REPORT.md) for methods, raw-data locations, comparisons, and limitations.

## Verification status (12 October 2026)

These are results from the local consolidated copies on one development machine, not GitHub Actions results and not a claim that the exact GitHub commits were independently rerun:

- The consolidated demo copy's Python suite passed 161 tests; the identification copy's suite passed 168 tests. The identification count includes a local timing regression test. Two preserved checkpoint suites also passed 10 tests each.
- A five-drone standalone SITL session completed takeoff, formation change, movement, square patrol, failure/re-election, revive/rejoin, and landing. The consolidated-copy landing fix waits for all vehicles to report disarmed before stopping SITL.
- Gazebo rendered the five-drone world on the NVIDIA GPU and all five SITL instances connected. The console then remained at EKF flags `0xa7` (GPS fix 6), so the EKF readiness gate blocked flight. Gazebo rendering and connection were checked; Gazebo flight was not verified.
- No physical-airframe flight was performed as part of these checks.

## Follow-up work on other branches

The consolidated launch-path updates and landing-shutdown fix are in the current `master` history. This branch contains the timing correction and uneven-first-interval regression test. Earlier raw identification runs and local changes are preserved on `archive/hw-identification-working-copy-20261024`. These branches are published on GitHub and remain separate from one another.

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
python3 sitl/swarm_console.py --drones 5 --standalone
```

For Gazebo, first start the matching Gazebo world/plugin as described in the console guide, then run:

```bash
python3 sitl/swarm_console.py --drones 5 --gazebo
```

The five-drone Gazebo flight was later verified on the `master` branch. See its [validation report](https://github.com/radahnnn/autonomous-swarm-drones-fyp/blob/master/docs/reports/GAZEBO_5DRONE_VALIDATION_2026-10-20.md) for the run and its limits.

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
