# Autonomous Swarm Drones FYP: Session Handoff Brief — Oct 5, 2026

> **Purpose:** Give this to any new AI session to instantly restore full context. Read every section before doing anything.

---

## 1. Project Identity

- **Project:** Autonomous Swarm Drones: Coordinated Controlled Movement (Final Year Project, BEE-60, Electrical Engineering)
- **GitHub:** [radahnnn/autonomous-swarm-drones-fyp](https://github.com/radahnnn/autonomous-swarm-drones-fyp.git)
- **Local repo path:** `/home/drone/swarm_drones_fyp/`
- **Branch:** `master`
- **ArduPilot version:** ArduCopter V4.8.0-dev (master branch build)
- **Environment:** Ubuntu Linux, Python 3.12, pymavlink, ROS 2 Jazzy, Gazebo Harmonic
- **venv:** `~/venv-ardupilot/` — always activate before running anything

---

## 2. Repository Structure

```
swarm_drones_fyp/
├── swarm_core/              # Algorithms (Hungarian, Laplacian, APF, formations)
│   ├── config.py
│   ├── drone.py             # 3D kinematics, rotor drag, lag models
│   ├── graph.py             # Adjacency, Laplacian, Fiedler eigenvalue
│   ├── network.py           # Packet drop, Gilbert-Elliott bursts, latency
│   ├── formations.py        # Line, V-Formation, Circle, Grid + Hungarian solver
│   ├── metrics.py           # Tracking error, convergence, safety distance
│   └── controllers/
│       ├── centralized.py   # Hungarian + PD tracking + APF
│       ├── decentralized.py # Laplacian consensus + Reynolds flocking
│       └── hybrid.py        # Centralized guidance + decentralized APF fallback
├── simulator/               # 2D/3D numerical physics + visualizer
│   ├── engine.py
│   └── visualizer.py
├── sitl/                    # ArduPilot SITL + Gazebo integration
│   ├── launch_swarm_2d.sh          <- NEW (created this session): unified 2D launcher
│   ├── launch_drone1.sh            # Individual drone launchers (kept for reference)
│   ├── launch_drone2.sh
│   ├── launch_drone3.sh            <- FIXED this session: port was 14570, now 14572
│   ├── interactive_flight_console.py  <- FIXED this session (see Section 4)
│   ├── swarm_3_drones.py           # Older script — DO NOT use for demos (see Section 5)
│   ├── flight_prep.py              # arm_with_retry(), takeoff_and_verify()
│   ├── mavlink_swarm_adapter.py
│   ├── common_frame.py             # WGS84 to NED transforms
│   ├── swarm_params.parm           # ArduPilot SITL tuning params
│   └── autonomous_wingman.py
├── tests/                   # 10 unit tests (all passing)
└── docs/
```

---

## 3. Test Suite Status

**Only 10 tests exist in the local repo** (NOT 73 — the 73 number was from an older checkpoint, do not trust it).

Run with:
```bash
cd ~/swarm_drones_fyp
PYTHONPATH=. pytest tests/ -v
```

| Test | Status | What it covers |
|------|--------|---------------|
| `test_formation_shapes` | PASS | Line, V, Circle, Grid geometry |
| `test_hungarian_assignment` | PASS | Optimal drone-slot assignment |
| `test_graph_laplacian` | PASS | Adjacency, Laplacian, Fiedler value > 0 |
| `test_disconnected_graph` | PASS | Value = 0 when graph splits |
| `test_stale_and_sequence_rejection` | PASS | Hybrid controller drops stale/dup packets |
| `test_asymmetric_hysteresis_and_dwell_time` | PASS | Centralized->Fallback->Recovery state machine |
| `test_smooth_controller_blending` | PASS | Alpha ramp 1.0->0.0 over ramp_duration |
| `test_centralized_simulation` | PASS | 5 drones -> LINE, error < 0.25m, no collision |
| `test_decentralized_simulation` | PASS | 5 drones -> V-shape via Laplacian |
| `test_hybrid_fallback` | PASS | Coordinator drop -> fallback, min dist > 0.6m |

**NOT tested (gaps):**
- `sitl/` scripts (no unit tests for MAVLink layer)
- `swarm_core/network.py` (packet drop, bursts)
- `swarm_core/metrics.py` directly
- `swarm_core/drone.py` directly
- End-to-end Gazebo integration

---

## 4. Critical Fixes Made This Session (Oct 5, 2026)

### Fix A: Drone 3 Port Mismatch (`launch_drone3.sh`)
- **Bug:** `launch_drone3.sh` had `--out=udp:127.0.0.1:14570` but `swarm_3_drones.py` expected port **14572**
- **Fix:** Changed to `--out=udp:127.0.0.1:14572`

### Fix B: `interactive_flight_console.py` — Wrong Connection Method
- **Bug:** Console used `tcp:127.0.0.1:5760/5770/5780`. When `sim_vehicle.py` is running, MAVProxy occupies those TCP ports — a second TCP client fails or gets blocked.
- **Fix:** Console now connects via **UDP** `udpin:127.0.0.1:14552/14562/14572`
- Drone configs changed from `{"port": 5760}` (TCP) to `{"port": 14552, "udp": True}`

### Fix C: `interactive_flight_console.py` — SITL Detection
- **Bug:** Detection checked `tcp:127.0.0.1:5760` — wrong after Fix B
- **Fix:** Detection first checks `udpin:127.0.0.1:14552`, falls back to TCP 5760 for legacy

### Fix D: `connect_all()` improved
- Better error messages pointing to `launch_swarm_2d.sh`
- 20 retry attempts (was 15), 3s timeout (was 2s)
- Logs SYSID on successful connection

### Fix E: New file `sitl/launch_swarm_2d.sh` (CREATED THIS SESSION)
- Unified launcher for 2D ArduPilot demo
- Kills existing SITL, launches all 3 via `sim_vehicle.py` in separate `xterm` windows
- Each window has `--map` showing MAVProxy 2D map with **arrowhead drone icon**
- Staggered homes so drones spawn at correct V-formation positions
- All forward to `udp:127.0.0.1:14550` for optional QGroundControl unified view

---

## 5. Why `swarm_3_drones.py` Is Broken for Demos (DO NOT USE)

| Problem | Detail |
|---------|--------|
| Only controls 2 of 3 drones | Sends SET_POSITION_TARGET to Drones 2 & 3 only. Drone 1 sits on ground. |
| Not simultaneous | Followers track leader with 100ms polling lag |
| No keyboard control | Cannot move all 3 together interactively |

**Use `interactive_flight_console.py` instead.**

---

## 6. How to Run the 2D Demo (Correct Procedure)

### Step 1 — Terminal 1: Launch all 3 SITL drones with map windows
```bash
cd ~/swarm_drones_fyp
bash sitl/launch_swarm_2d.sh
```
- Opens 3 xterm windows, each with a MAVProxy 2D map showing a drone arrowhead
- Wait ~60s for: `APM: EKF2 IMU0 using GPS` in each window

### Step 2 — Terminal 2: Run swarm controller (after GPS lock)
```bash
source ~/venv-ardupilot/bin/activate
cd ~/swarm_drones_fyp
python3 sitl/interactive_flight_console.py --standalone
```
- Connects to all 3 via UDP
- Arms, takes off to 5m, locks V-formation
- Gives keyboard control console

### Step 3 — Keyboard controls (all 3 move simultaneously):

| Key | Action |
|-----|--------|
| `w` | North +6m |
| `s` | South -6m |
| `d` | East +5m |
| `a` | West -5m |
| `u` | Climb +2m |
| `j` | Descend -2m |
| `p` | Auto square patrol (8x6m loop) |
| `c` | Return to center |
| `Space` | Print live telemetry |
| `q` | Land & exit |

---

## 7. MAVLink Port Reference (CRITICAL — memorize this)

| Drone | SITL Instance | MAVProxy TCP | UDP to console | UDP to QGC |
|-------|--------------|--------------|----------------|-----------|
| Drone 1 (Apex) | -I 0 | 5760 | **14552** | 14550 |
| Drone 2 (Left Wing) | -I 1 | 5770 | **14562** | 14550 |
| Drone 3 (Right Wing) | -I 2 | 5780 | **14572** | 14550 |

> **Rule:** Always use UDP ports (14552/14562/14572) for Python scripts.
> TCP ports (5760/5770/5780) are for MAVProxy's own console — do not compete with them.

---

## 8. Known Issues / Things Still Not Done

1. **`interactive_flight_console.py` UDP fix not yet tested end-to-end** — this session ran out of context before re-running. **VERIFY THIS FIRST next session.**

2. **All 3 arrowheads in one window** — Each drone currently has its own MAVProxy map. To see all 3 in one window: open QGroundControl and connect to `udp:14550` (all 3 broadcast there).

3. **Formation morphing not implemented** — V->Circle->Line->Grid mid-flight is the top roadmap priority.

4. **EKF wait ~60-90s** — On the demo laptop this may be 90-120s. Do NOT kill SITL early.

5. **`sitl/test_telemetry.py` broken** — requires `pymavlink` not in test env. Run `pytest tests/` not `pytest` from root.

---

## 9. Recommended Next Steps (Priority Order)

1. **VERIFY UDP fix** — Run `launch_swarm_2d.sh` then `interactive_flight_console.py --standalone`. Confirm all 3 connect and WASD moves all 3 arrowheads.

2. **Formation Morphing** — Add key `f` to `interactive_flight_console.py` cycling V->Circle->Line->Grid using `swarm_core/formations.py` Hungarian assignment.

3. **Headless Gazebo** (`gz sim -s`) — Run physics server headless to bring RTF from 0.3x to ~1.0x, cutting EKF wait from 90s to ~30s.

4. **APF Stress Testing** — Enable `swarm_core/network.py` packet drop during flight, measure minimum separation, generate graphs for FYP report.

5. **Fleet scaling** — 3->5 drones (low priority).

---

## 10. Hardware Context

- **Development machine:** User's main PC (adequate performance)
- **Demo machine:** Weaker laptop — EKF convergence slower (up to 120s). Do NOT kill SITL early.
- **Gazebo 3D mode:** Needs `DISPLAY=:1` or physical monitor

---

## 11. Previous Session Debugging Breakthroughs (still relevant)

### Clock Sync (`lock_step=1`)
Gazebo SDF must have `<lock_step>1</lock_step>` — prevents clock desync and EKF GPS rejection.

### Arming Reliability (`flight_prep.py`)
- `arm_with_retry()`: retries arm up to 180s, checks `MAV_MODE_FLAG_SAFETY_ARMED`
- `takeoff_and_verify()`: verifies altitude > 0.5m before declaring success

### V-Formation Collapse Fix
- **Gazebo mode:** ALL 3 share one home `-35.363261,149.165230,584,0`
- **Standalone 2D:** staggered homes (different longitudes) to space drones on spawn

---

*Generated: Oct 5, 2026 | Saved at: `/home/drone/swarm_drones_fyp/HANDOFF_OCT5_2026.md`*
