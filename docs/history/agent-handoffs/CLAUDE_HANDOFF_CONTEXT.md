# FYP Swarm Drones Project - Comprehensive Technical Handoff for Claude

## 1. Project Overview & Architecture
- **Objective:** Final Year Project (FYP) demonstrating an autonomous 5-drone swarm capable of flight in both **2D SITL** (ArduPilot quad physics) and **3D Gazebo** (Gazebo Harmonic with ArduCopter JSON backend).
- **Vehicles:** 5 Cinewhoop / Iris Quadrotors (X-frame, `FRAME_CLASS=1`, `FRAME_TYPE=1`), SysIDs 1 through 5.
- **Flight Stack:** ArduCopter 4.3+ SITL compiled binary (`/home/drone/ardupilot/build/sitl/bin/arducopter`).
- **Telemetry & GCS:** MAVLink multiplexed over UDP ports:
  - Broadcast to QGroundControl: `udp:127.0.0.1:14550`
  - Individual Drone Control Ports:
    - Drone 1: `udp:127.0.0.1:14552`
    - Drone 2: `udp:127.0.0.1:14562`
    - Drone 3: `udp:127.0.0.1:14572`
    - Drone 4: `udp:127.0.0.1:14582`
    - Drone 5: `udp:127.0.0.1:14592`
  - Gazebo JSON FDM In/Out Ports: 9002/9003, 9012/9013, 9022/9023, 9032/9033, 9042/9043.
- **Control Strategy:** 10 Hz `SET_POSITION_TARGET_LOCAL_NED` stream in `GUIDED` mode referencing a shared flat-earth WGS84 datum origin (`-35.363261, 149.165230, 584m`).
- **Formations Supported:** Flying-V (Chevron), Line Abreast, Column (In-Trail), Diamond/Star, Pentagon/Ring.

---

## 2. Chronological Engineering History: "How We Got Here"
To prevent repeating past mistakes, here is the full engineering timeline and resolved bugs:

### Phase 1: Scaling from 2 Drones to 5 Drones (2D SITL)
- **Challenge:** Initial prototype only flew 2 drones. Needed to scale to 5 drones for the final defense demo.
- **Solution:** 
  - Allocated instances `-I 0` through `-I 4` in `start_swarm_5drones_2d.sh`.
  - Configured MAVProxy instances to forward individual telemetry to UDP ports 14552, 14562, 14572, 14582, 14592, while forwarding all 5 streams to UDP 14550 so QGroundControl displays Vehicles 1 through 5 on a single radar screen.
  - Built `common_frame.py` (Flat-Earth WGS84 projection) to solve the problem of drones spawning at different runway GPS coordinates having mismatched local NED origins.

### Phase 2: Bridging to 3D Gazebo Harmonic & Port Collisions
- **Symptom:** In Gazebo, only Drones 1 and 2 moved; Drones 3, 4, and 5 stayed inert on the ground.
- **Root Cause 1:** In `start_swarm_5drones_3d.sh`, Drones 3, 4, and 5 were missing the `-f gazebo-iris --model JSON` arguments in `sim_vehicle.py`, causing them to silently fall back to standalone 2D physics.
- **Root Cause 2:** In `simulator/gazebo/models/cinewhoop_*/model.sdf`, the ArduPilotPlugin defaulted `fdm_port_out` to port 9003 for all instances if omitted. This caused motor PWM commands from Drones 2–5 to collide on Drone 1's socket.
- **Solution:** Injected explicit `<fdm_port_out>` tags (9003, 9013, 9023, 9033, 9043) into all model SDFs, matching the inbound ports (9002, 9012, 9022, 9032, 9042).

### Phase 3: Gazebo Runway Collision Clipping & Inverted Spawn
- **Symptom:** Drones violently flipped upside-down immediately upon spawning in Gazebo, triggering `Accels inconsistent` and refusing to arm.
- **Root Cause:** Drones were spawned at `Z=0.05m` in `cinewhoop_5drones.sdf`. Their collision meshes clipped through the runway's infinite ground plane collision geometry, producing infinite repulsive contact forces.
- **Solution:** Elevated spawn heights to `Z=0.20m` in `cinewhoop_5drones.sdf`, allowing gravity to gently settle the landing gear onto the tarmac.

### Phase 4: Physics Step Size vs. ArduPilot Gyro Loop Rate Limit
- **Symptom:** SITL threw: `Arm: Gyro 0 rate 501Hz < loop rate*1.8 720Hz` and refused to arm.
- **Root Cause:** In an attempt to reduce CPU load, `max_step_size` in `cinewhoop_5drones.sdf` was set to `0.002` (500 Hz). ArduCopter SITL enforces an internal safety check requiring gyro telemetry $\ge 720\text{ Hz}$.
- **Solution:** Reverted `max_step_size` to `0.001` (1000 Hz) to permanently satisfy the 720 Hz gyro rate limit.

### Phase 5: Transition to Self-Healing Swarm (Decentralized Fault-Tolerance)
- **Challenge:** The original leader-follower architecture had a Single Point of Failure (SPOF): if Drone 1 was lost, the remaining 4 drones had no reference origin and risked drifting.
- **Solution:** Created `sitl/self_healing_swarm_console.py` as an isolated standalone script (keeping `interactive_flight_console_5drones.py` untouched):
  - 10 Hz background heartbeat monitor.
  - Dynamic Leader Election: If Drone 1 drops out, Drone 2 is dynamically elected Apex Leader; if Drone 2 drops, Drone 3 is elected.
  - Self-Healing Topologies: Formations automatically recalculate offsets for 5, 4, 3, 2, or 1 active drone.
  - Live failure injection (`f <id>`) and revival with auto re-arm & liftoff (`r <id>`).

### Phase 6: The `arm_with_retry` Keyword Bug
- **Symptom:** Async climb worker crashed with:
  `TypeError: arm_with_retry() got an unexpected keyword argument 'max_attempts'`
- **Root Cause:** In `flight_prep.py`, the function signature is:
  `def arm_with_retry(conn, label: str, timeout: float = 180.0, force_after: float = 90.0) -> bool:`
  The script passed `max_attempts=6`.
- **Solution:** Corrected to pass `timeout=120.0, force_after=60.0`.

---

## 3. Key Codebase Files & Locations
- **Physical Repository Path:** `/home/drone/.gemini/antigravity/scratch/swarm_drones_fyp/`
- **Convenience Symlink Path:** `/home/drone/swarm_drones_fyp` (points directly to the physical repository above; they are the exact same files and git repository).

All project files are located in this directory:
1. **`sitl/self_healing_swarm_console.py`**:
   The resilient flight controller with Dynamic Leader Election, adaptive topologies, failure injection, and liftoff.
2. **`sitl/interactive_flight_console_5drones.py`**:
   The original stable baseline 5-drone controller (kept intact as fallback).
3. **`sitl/start_all_swarm.sh`** (symlinked at repo root as `start_all.sh`):
   Master single-command orchestrator that launches QGroundControl, 5-drone SITL, Gazebo (if `--gazebo`), and the console in xterm.
4. **`sitl/start_swarm_5drones_3d.sh`**:
   Spawns 5 ArduCopter instances linked to Gazebo JSON ports via `sim_vehicle.py`.
5. **`sitl/start_swarm_5drones_2d.sh`**:
   Spawns 5 standalone ArduCopter quad physics instances.
6. **`sitl/launch_gazebo_5drones.sh`**:
   Boots Gazebo Harmonic (`gz sim -v4 -r cinewhoop_5drones.sdf`).
7. **`sitl/flight_prep.py`**:
   Arming and takeoff verification routines (`arm_with_retry`, `takeoff_and_verify`).
8. **`sitl/stop_all.sh`**:
   Sweeps and terminates all lingering SITL, Gazebo, and controller processes.

---

## 4. The Current Technical Problems to Solve

### Problem 1: Gazebo 3D Real-Time Factor (RTF) & EKF Arming Rejection
- **Symptom:** In Gazebo 3D mode, commanding takeoff can trigger:
  `!! [ERROR] Drone X failed to arm for liftoff. Check EKF status.`
- **Root Cause:** Running 5 complete multirotor rigid-body models + physics plugins in Gazebo Harmonic drops the Real-Time Factor down to **~17.6%** (~5.7× slower than real time).
- ArduPilot's EKF3 requires ~15–20 seconds of **simulation time** to settle gyro drift and achieve GPS lock before permitting motor arming in `GUIDED` mode.
- In wall-clock time, 20 simulation seconds at 17.6% speed equals **~110–120 real seconds**.
- Short timeouts expire when only ~4.4 simulation seconds have passed, cutting off the EKF before it converges.
- **Question for Claude:** How can the Python controller best query and verify ArduPilot's internal EKF origin and GPS lock status via MAVLink (`EKF_STATUS_REPORT` or `SYS_STATUS`) before attempting arming, rather than relying on fixed wall-clock sleep timers?

### Problem 2: QGroundControl Unexpected Closure & Process Lifecycle
- **Symptom:** QGroundControl window sometimes disappears or fails to stay open when started via bash scripts.
- **Root Causes Identified:**
  1. **Workspace Server Restarts:** Cloud environment restarts terminate background GUI tasks.
  2. **`start_all.sh` cleanup collision:** `stop_all.sh` initially ran `pkill -9 QGroundControl`. Due to sudden SIGKILL, QGC's lockfile hadn't cleared when a new instance launched, causing immediate exit.
  3. **Display / Platform conflict:** The QGC script checked for Wayland on virtual X11 display (`DISPLAY=:1`).
- **Fixes Applied:**
  - Forced `QT_QPA_PLATFORM=xcb` in `launch_qgroundcontrol.sh`.
  - Modified `start_all_swarm.sh` to preserve QGroundControl if already running.
- **Question for Claude:** How to make the process supervisor pattern in bash completely robust against background disown and subshell exit issues?

---

## 5. Current State Summary
- **2D SITL Mode:** 100% stable, fully verified, runs at 100% real-time speed. Formation morphing, dynamic leader failover, and revival work smoothly.
- **3D Gazebo Mode:** Physics and port links are working, all 5 models are placed on the runway, but requires careful handling of the 17.6% RTF simulation warmup period before issuing takeoff commands.
