# FYP Autonomous Swarm Drones - Master Running Log & Handover

**Project:** Autonomous Swarm Drones FYP  
**Repository:** `radahnnn/autonomous-swarm-drones-fyp`  
**Date:** October 6–7, 2026  
**Status:** Ready for Supervisor Demo  

---

## 1. Executive Summary & Achievements

Today's progress successfully covered both physical drone hardware inspection and complete SITL multi-drone swarm simulation for the demonstration.

### Key Milestones Achieved:
1. **Physical Drone Bench Diagnostics & Parameter Backup:**
   * Hardware verified: **Matek Systems H743-SLIM V3** (Dual ICM42688P IMUs, DPS368 Baro).
   * Firmware: **ArduCopter v4.6.2** (Git hash `31656264`).
   * Complete parameter backup extracted: **1,283 parameters** saved to `hardware/params/matek_h743_quad_20261006.param`.
   * MicroSD logging status verified (slot located on underside between FC and ESC; logs require armed state).
2. **3-Drone SITL Swarm Simulation in QGroundControl:**
   * Fixed MAVProxy Wayland/X11 hanging issue by daemonizing MAVProxy.
   * Successfully running 3 ArduCopter SITL instances simultaneously with distinct SysIDs:
     * **Drone 1 (Apex Leader, SysID 1):** UDP `14550` & `14552`
     * **Drone 2 (Left Wing, SysID 2):** UDP `14550`, `14560`, `14562`
     * **Drone 3 (Right Wing, SysID 3):** UDP `14550`, `14570`, `14572`
   * Pre-configured `QGroundControl.ini` to display all 3 vehicles on startup over Canberra airfield.
3. **Multi-Drone Coordinated Formation Flight:**
   * **Simultaneous Parallel Arming & Liftoff:** Implemented threaded simultaneous arming and takeoff via `ThreadPoolExecutor` in `interactive_flight_console.py`.
   * **6 Dynamic Swarm Formations:**
     * `v`: Flying-V (Chevron)
     * `l`: Line Abreast (Side-by-side)
     * `k`: Column In-Trail (Single-file)
     * `t`: Triangle (Delta)
     * `o`: Circle / Ring (Radial 120°)
     * `e`: Echelon (Diagonal)
   * **Smooth Formation Morphing:** Drones transition between formations smoothly over 2.5s without collisions or jerks.
   * **Autonomous QGC Wingman Daemon (`qgc_swarm_wingman.py`):**
     * Drone 1 is directly controlled via QGroundControl's GUI ("Go to location").
     * Drones 2 & 3 autonomously track Drone 1 at 10 Hz, following it in formation without requiring terminal input.

---

## 2. Architecture & Port Mapping

```
+-------------------------------------------------------------------------+
|                          QGroundControl GUI                             |
|         Listens on UDP 14550 (All) | 14560 (Drone 2) | 14570 (Drone 3)   |
+-------------------------------------------------------------------------+
                                   ▲
                                   │ MAVLink Telemetry
                                   ▼
+-------------------------------------------------------------------------+
|                  ArduPilot SITL Swarm (3 Quadrotors)                    |
|                                                                         |
|  [Drone 1: Apex Leader]  --> Telemetry Out: UDP 14550, Console: 14552    |
|  [Drone 2: Left Wing]    --> Telemetry Out: UDP 14550/14560, Cons: 14562|
|  [Drone 3: Right Wing]   --> Telemetry Out: UDP 14550/14570, Cons: 14572|
+-------------------------------------------------------------------------+
                                   ▲
                                   │ 10 Hz Formation Guidance Stream
                                   ▼
+-------------------------------------------------------------------------+
|       Swarm Guidance Controller / QGC Autonomous Wingman Daemon         |
|  - Tracks Leader (GPS/NED)                                              |
|  - Calculates Formation Offsets                                         |
|  - Sends SET_POSITION_TARGET_LOCAL_NED to Followers                     |
+-------------------------------------------------------------------------+
```

---

## 3. Demo Quickstart Guide (For Tomorrow Morning)

### Method A: Full GUI Control via QGroundControl (No Terminal Needed!)
1. Open QGroundControl:
   ```bash
   bash /home/drone/Documents/gemini\ work/launch_qgroundcontrol.sh
   ```
2. Start the simulation swarm and wingman daemon:
   ```bash
   bash /home/drone/Documents/gemini\ work/start_swarm.sh
   python3 /home/drone/Documents/gemini\ work/qgc_swarm_wingman.py &
   ```
3. In QGroundControl:
   * Right-click anywhere on the map &rarr; Select **`Go to location`**.
   * Slide the confirmation slider.
   * **Result:** Vehicle 1 flies to that target, and Vehicles 2 & 3 fly right along with it in V-formation!

---

### Method B: Interactive Keyboard Flight Console (Formation Morphing Demo)
1. In a terminal window, run:
   ```bash
   source ~/venv-ardupilot/bin/activate
   cd ~/swarm_drones_fyp
   python3 sitl/interactive_flight_console.py
   ```
2. **Key Commands to Impress Your Supervisor:**
   * **`w` / `s` / `a` / `d`**: Move the whole formation North / South / West / East.
   * **`l`**: Morph into a **Line Formation** (drones line up side-by-side).
   * **`t`**: Morph into a **Triangle / Delta Formation**.
   * **`k`**: Morph into a **Column Formation** (single file).
   * **`v`**: Morph back to the **Flying-V**.
   * **`p`**: Autonomous **Square Patrol** (swarm flies a 4-corner box).
   * **`q`**: Land and disarm all drones simultaneously.

---

## 4. File Manifest

* `hardware/`:
  * `README.md`: Complete hardware diagnosis and blackbox instructions.
  * `params/matek_h743_quad_20261006.param`: Flight controller parameter backup.
  * `scripts/read_drone_info.py`: Hardware MAVLink inspection script.
  * `scripts/export_parameters.py`: Parameter dumper.
* `sitl/`:
  * `interactive_flight_console.py`: Master console with 6 formations & lockstep flight.
  * `qgc_swarm_wingman.py`: Daemon for QGC "Go to location" follower mode.
  * `start_swarm_daemon.sh`: Clean headless SITL launcher for 3 drones.
  * `verify_swarm.py`: Automated telemetry verifier script.
  * `launch_swarm_console.sh`: Desktop GUI launcher for the flight console.
* `docs/`:
  * `RUNNING_LOG.md`: This comprehensive handover document.
