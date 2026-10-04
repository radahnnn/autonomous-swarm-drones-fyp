# Autonomous Swarm Drones FYP: New Chat & Model Handoff Brief

> **Purpose:** Give this file to any new AI session or model (Claude, GPT, Gemini, etc.) to immediately establish full project context, architecture understanding, recent debugging breakthroughs, and active task directions without redundant onboarding.

---

## 1. Executive Summary & Repository Status

- **Project:** Autonomous Swarm Drones: Coordinated Controlled Movement (Final Year Project, BEE-60, Electrical Engineering).
- **GitHub Repository:** [radahnnn/autonomous-swarm-drones-fyp](https://github.com/radahnnn/autonomous-swarm-drones-fyp.git)
- **Branch:** `master`
- **Working Tree:** Clean, all updates pushed to `origin/master`.
- **Test Suite:** **73/73 unit tests passing** via `PYTHONPATH=. pytest` (covering configs, graph Laplacians, Hungarian assignment, hybrid APF controllers, MAVLink adapters, and flight prep verifications).
- **Environment:** Ubuntu Linux, ROS 2 Jazzy, Gazebo Harmonic (gz sim 8.x), ArduPilot SITL (`ardupilot-gazebo` plugin), Python 3.12 (`pymavlink`, `scipy`, `numpy`, `matplotlib`).

---

## 2. System Architecture

The repository is structured into three main operational layers:

```
swarm_drones_fyp/
├── swarm_core/              # Algorithmic foundation (independent of simulation backend)
│   ├── config.py            # Explicit profiles ('assumed_baseline' vs 'sitl_fitted')
│   ├── drone.py             # 3D multirotor kinematics, lag models, rotor drag
│   ├── graph.py             # Communication topology, Laplacian, algebraic connectivity
│   ├── network.py           # Simulated packet drop (0-50%), latency, burst outages
│   ├── formations.py        # Line, V-Formation (Chevron), Circle, Grid + Hungarian solver
│   ├── metrics.py           # Tracking error, convergence time, safety distance
│   └── controllers/
│       ├── centralized.py   # Global Hungarian + PD tracking + APF safety
│       ├── decentralized.py # Laplacian velocity consensus + Reynolds flocking
│       └── hybrid.py        # Centralized guidance + decentralized APF safety barrier
├── simulator/               # Lightweight 2D/3D numerical physics engine & visualizer
│   ├── engine.py            # Multi-drone integration step with wireless emulation
│   └── visualizer.py        # Static figure & trajectory renderers
├── sitl/                    # ArduPilot Software-In-The-Loop & 3D Gazebo Integration
│   ├── flight_prep.py       # Robust arming & takeoff verification (handles slow EKF convergence)
│   ├── start_and_climb.py   # Headless or Gazebo automated arm, climb to 5m, hold V-formation
│   ├── interactive_flight_console.py # Pilot console with WASD controls & Square Patrol
│   ├── mavlink_swarm_adapter.py      # Translates swarm targets to MAVLink SET_POSITION_TARGET
│   ├── common_frame.py      # GPS WGS84 to NED datum transforms
│   ├── launch_gazebo.sh     # Starts Gazebo Harmonic 3D world with 3 Cinewhoop drones
│   └── swarm_params.parm    # ArduPilot SITL parameter tuning
├── tests/                   # 73 unit tests
└── docs/                    # Architectural reports, parity analyses, troubleshooting guides
```

---

## 3. Critical Debugging Breakthroughs (Oct 2026)

Three major bugs previously prevented the drones from flying in 3D Gazebo. All three were diagnosed and fixed (a fourth change, aligning the spawn layout with the V slots and commanding one common heading, is described in `docs/SITL_GAZEBO_TROUBLESHOOTING.md`):

### 1. Clock Drift & EKF Failure (`<lock_step>1</lock_step>`)
- **Problem:** When `<lock_step>0</lock_step>` was set, ArduPilot SITL ran at 1.0x real-time while Gazebo GUI ran at ~0.3x Real-Time Factor (RTF) on this machine. Because clocks desynchronized, ArduPilot rejected simulated GPS and IMU sensor data (`Arm: Need Position Estimate`).
- **Fix:** Restored `<lock_step>1</lock_step>` in `simulator/gazebo/models/cinewhoop_{1,2,3}/model.sdf`. Both Gazebo and ArduPilot now share a unified clock. EKF converges after ~45s of sim-time (~90–100s of wall-clock time).

### 2. Unverified Takeoff & Premature Timeouts
- **Problem:** Scripts used a short 15–25s timeout and sent `NAV_TAKEOFF` even when the autopilot was still disarmed, printing "Takeoff commanded" while the drones stayed on the runway.
- **Fix:** Built `sitl/flight_prep.py`:
  - `arm_with_retry()`: Repeatedly sets GUIDED and requests ARM up to 180s, logging status every 10s until the vehicle MAVLink heartbeat confirms `MAV_MODE_FLAG_SAFETY_ARMED`.
  - `takeoff_and_verify()`: Sends `NAV_TAKEOFF` and verifies that relative altitude climbs above 0.5m.

### 3. V-Formation Collapse (Staggered `--home` Bug)
- **Problem:** Gazebo's JSON plugin already sends absolute world positions for each drone (`(0,0)`, `(-2.9,-3.0)`, `(3.1,-3.0)`). Previous scripts passed staggered `--home` coordinates (spaced 5m apart in longitude). ArduPilot added the offsets twice, causing wing drones to shift 5m and 10m west, collapsing the formation.
- **Fix:** In Gazebo mode, all SITL instances now share **one single world origin home** (`-35.363261, 149.165230, 584, 0`). Staggered homes are preserved only for standalone `--model quad` mode.

---

## 4. How to Run the System

### Mode A: Full 3D Gazebo Simulation (3 Cinewhoop Multirotors)
1. **Terminal 1** (Launch 3D Gazebo world on `DISPLAY=:1`):
   ```bash
   ./sitl/launch_gazebo.sh
   ```
2. **Terminal 2** (Arm, climb to 5.0m, and hold V-formation):
   ```bash
   python3 sitl/start_and_climb.py
   ```
   *(Note: Allow ~90–100 seconds wall-clock time for the EKF to converge at ~0.3x RTF).*

3. **Or Terminal 2 with Interactive Pilot Console**:
   ```bash
   python3 sitl/interactive_flight_console.py --gazebo
   ```
   - Supports WASD manual keyboard control of the swarm centroid.
   - Supports `p` to trigger an automated Square Patrol mission.
   - Supports `l` for synchronized landing.

### Mode B: Fast Standalone Headless Simulation (No Gazebo GUI)
Runs 3 ArduPilot SITL instances at 1.0x Real-Time Factor:
```bash
python3 sitl/interactive_flight_console.py --standalone
```
Arming takes <10 seconds.

### Running Unit Tests
```bash
PYTHONPATH=. pytest
```

---

## 5. Active Roadmap & Suggested Next Tasks

If branching into a new conversation or task, here are the most impactful next objectives:

1. **In-Flight Formation Morphing in Gazebo**:
   - Extend `sitl/interactive_flight_console.py` and `sitl/start_and_climb.py` to trigger live transitions (e.g. V-Formation $\to$ Circle $\to$ Line $\to$ Grid) while airborne in Gazebo, leveraging Hungarian assignment to minimize trajectory intersections.
2. **Gazebo Headless Decoupled Mode (`gz sim -s`)**:
   - Run the Gazebo physics server headless (`gz sim -s -r ...`) and attach a separate GUI client only when needed, bringing Real-Time Factor (RTF) from 0.3x closer to 1.0x to cut EKF wait times.
3. **Fleet Scaling (3 Drones $\to$ 5+ Drones)**:
   - Add models 4 and 5 to `cinewhoop_3drones.sdf` and expand launch ports in `mavlink_swarm_adapter.py`.
4. **Collision Avoidance Stress Testing**:
   - Test APF (Artificial Potential Field) repulsive forces in Gazebo under simulated GPS noise or comms packet loss (Gilbert-Elliott burst outages).

---

## 6. Key File Reference
- Controller logic: `swarm_core/controllers/hybrid.py`, `swarm_core/controllers/centralized.py`
- Formations & Hungarian: `swarm_core/formations.py`
- MAVLink adapter: `sitl/mavlink_swarm_adapter.py`
- Arming & takeoff verification: `sitl/flight_prep.py`
- Gazebo world: `simulator/gazebo/worlds/cinewhoop_3drones.sdf`
- Troubleshooting details: `docs/SITL_GAZEBO_TROUBLESHOOTING.md`
