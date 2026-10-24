# Swarm Drones FYP: Handoff Addendum (Oct 5, 2026, later session)

> **Read `HANDOFF_OCT5_2026.md` first, then this file.** This addendum records what happened after that brief was written: verification of the 2D demo, a launcher scare, and a new formation-morphing patch. Items are marked **VERIFIED** (observed by the user on their machine), **TESTED OFFLINE** (run in an AI sandbox only), or **UNVERIFIED**.

---

## 1. Summary

- The 2D 3-drone ArduPilot SITL demo now **works end to end** (VERIFIED).
- A formation-morphing feature (`f` key) was written and unit-tested offline, but **has not been applied or flown yet** (UNVERIFIED in flight).
- The earlier launcher files were pushed to GitHub (commit `4ac9fe1`, "WIP: 2D launcher, UDP console fix, drone3 port fix, handoff brief").

---

## 2. The `--maplevel` Error (resolved by relaunch, root cause not proven)

**Symptom:** Running `bash sitl/launch_swarm_2d.sh` opened 3 black xterms all showing `no such option: --maplevel`.

**Findings:**
- The pushed `launch_swarm_2d.sh` contains **no** `--maplevel` flag and passes `bash -n`. Its flags: `-v ArduCopter -I N -N --auto-sysid --custom-location --out=... --add-param-file --map`, run inside `xterm -hold`.
- Running drone 1 by hand with those flags and `--map` worked: params loaded, EKF3 initialised, GPS detected, origin set.
- Versions: ArduPilot checkout commit `26c7363f64` (Sep 29, 2026), MAVProxy 1.8.75. No version mismatch found.
- After `pkill`, `git pull` and relaunching the script, all 3 windows started correctly and arrowhead map windows appeared.

**Likely cause (UNVERIFIED):** the user had run an older local version of the launcher that was later replaced. If `--maplevel` reappears, run one `sim_vehicle.py` command by hand (see Section 5) and capture the first 40 lines of output.

---

## 3. Verification Results (VERIFIED by the user)

Console output from `python3 sitl/interactive_flight_console.py --standalone`:

- Detected active SITL on UDP 14552/14562/14572, skipped SITL launch.
- Connected all three: Apex (SYSID=1, port 14552), Left Wing (SYSID=2, port 14562), Right Wing (SYSID=3, port 14572).
- GPS lock confirmed, all 3 armed after 0s, all 3 took off.
- Hover altitudes at the check: 4.90m, 4.87m, 3.95m (Drone 2 was probably still climbing; not rechecked).
- Frame origins: Apex E=-0.01m, Left Wing E=+4.99m, Right Wing E=-4.56m (about 5m apart).
- Telemetry: `D0-D1=4.97m | D0-D2=4.56m | Span=9.53m`. The ~0.4m asymmetry likely comes from the staggered home longitudes (Drone 3 uses `149.165180`); cosmetic.
- Pressed `w` twice: setpoints stacked North=6m then 12m, and the user confirmed **all 3 drones moved together**.

**Why all 3 arrowheads look co-located:** each xterm has its own MAVProxy map that centers on its own drone. They are really ~5m apart. For one shared view, connect QGroundControl to `udp:14550`.

**Not yet tested by the user:** keys `a`, `d`, `u`, `j`, `c`, `p`, and the `q` exit.

The "3 simultaneous drones error" the user mentioned earlier was never described in detail. It may have been the `--maplevel` issue or something that did not recur. Ask the user.

---

## 4. Formation Morphing Design (PLANNED / NEXT STEP)

Proposed and designed below for implementation cleanly on top of commit `4ac9fe1`.

### Changes
1. `swarm_core/formations.py`: new `plan_formation_transition(current_offsets, formation_type, spacing, path_samples=51)`. Uses Hungarian assignment (`assign_optimal_slots`) and returns `(assigned_offsets, min_separation)` measured along straight-line paths.
2. `sitl/interactive_flight_console.py`:
   - Adds `sys.path.insert` for the repo root and imports `FormationType`, `plan_formation_transition`.
   - Class constants: `FORMATION_SPACING = 5.0`, `MORPH_DURATION = 4.0`, `FORMATION_CYCLE = [V_SHAPE, CIRCLE, LINE, GRID]`.
   - New state: `formation_idx`, `offsets_from`, `offsets_to`, `morph_t0`.
   - New `_offsets_at(now)` (smoothstep blend; caller must hold `self.lock`) and `morph_to_next()`.
   - `send_formation_setpoints()` now reads the blended offsets instead of the fixed `self.v_offsets`.
   - New command `f` / `9`; help text and invalid-command message updated.
3. `tests/test_formation_morph.py`: 5 new tests (every slot assigned once, min separation > 1.5m, full cycle stays safe).

### Offline results (spacing 5.0, start = console's initial V)
Closest approach en route: Circle 4.24m, Line 4.33m, Grid 3.54m, back to V 4.83m (cycle starting from the initial V; values printed by `morph_to_next()` vary slightly depending on mid-morph presses). Full test suite passed in the sandbox with `pymavlink`, `numpy`, `scipy` installed.

### Design notes and caveats
- Offsets are in the console's convention: formation x = North, y = East, z unchanged (2D morph, altitude is separate).
- `morph_to_next()` plans from the **commanded** (interpolated) offsets, not from measured positions. This avoids stale telemetry but ignores tracking lag.
- Circle/Line/Grid offsets are centered on the formation point; the original V had the apex at the point. So a morph can shift the apex by ~2m.
- Hungarian matching assigns whichever slot is nearest, so "Apex / Left Wing / Right Wing" labels stop meaning positions after a morph.
- Yaw stays fixed (`FORMATION_YAW_RAD = 0.0`).

### To implement and test (next task)
1. Add `plan_formation_transition` to `swarm_core/formations.py`.
2. Add the `f` key handler and blending state machine to `sitl/interactive_flight_console.py`.
3. Add `tests/test_formation_morph.py`.
4. Run:
```bash
cd ~/swarm_drones_fyp
PYTHONPATH=. pytest tests/test_formation_morph.py -q
python3 sitl/interactive_flight_console.py --standalone
# in console: press f, watch maps, press Space for spacing readings
```

---

## 5. Useful Manual SITL Command (for debugging launcher problems)

```bash
source ~/venv-ardupilot/bin/activate
pkill -9 -f arducopter; pkill -9 -f sim_vehicle
cd ~/ardupilot/ArduCopter
python3 ../Tools/autotest/sim_vehicle.py -v ArduCopter -I 0 -N --auto-sysid \
  --custom-location=-35.363261,149.165230,584,0 --out=udp:127.0.0.1:14552 --map 2>&1 | head -40
```

---

## 6. Open Issues and Observations

1. **`q` does not land the drones.** Help text says "Land & exit" but the code only sets `running = False` and exits. Candidate fix: set mode LAND on each connection before exiting. Not changed.
2. **Possibly stale telemetry in `print_status()`.** It calls `recv_match(type="GLOBAL_POSITION_INT", blocking=False)` once per drone, which returns the oldest buffered message rather than the latest. Readings looked plausible, but this has not been investigated. Candidate fix: drain the queue and keep the last message (as the climb monitor already does).
3. **Test count discrepancy.** The previous brief says only 10 tests exist locally. The GitHub copy has 9 test files (`test_config_and_safety`, `test_flight_prep`, `test_formations`, `test_graph`, `test_hybrid_features`, `test_simulation`, `test_simulation_sitl_parity`, `test_sitl_adapter`, plus the new morph file) and the sandbox run collected far more than 10. Have the user run `PYTHONPATH=. pytest tests/ -q --co | tail -1` locally to confirm the real count and fix the notes.
4. Drone 2's hover altitude (3.95m at the check) was not rechecked.
5. All unchanged items from the first brief still hold: EKF wait of 60-120s on the demo laptop, `lock_step=1` for Gazebo, Gazebo mode uses one shared home, `sitl/test_telemetry.py` is broken (run `pytest tests/`, not from repo root), `swarm_3_drones.py` is not for demos.

---

## 7. Next Steps (priority order)

1. Apply the morph patch and fly it; report smoothness and measured spacing (Section 4).
2. Test the remaining keys (`a`, `d`, `u`, `j`, `c`, `p`) and fix `q` to land.
3. Headless Gazebo (`gz sim -s`) to improve real-time factor and cut EKF wait.
4. APF stress testing: enable `swarm_core/network.py` packet drop in flight, measure minimum separation, make graphs for the FYP report. The morph transition's `min_separation` output is a ready-made data point.
5. Add unit tests for `network.py`, `metrics.py`, `drone.py`, and the `sitl/` MAVLink layer.
6. Scale from 3 to 5 drones (low priority).

---

## 8. Working Notes for the Next AI Session

- The user runs commands on their own machine and pastes output; an AI sandbox cannot run their SITL or see their display.
- The repo (`github.com/radahnnn/autonomous-swarm-drones-fyp`) is public and can be cloned for reading code. Unpushed local changes are invisible, so ask the user to push first.
- The user prefers step-by-step guidance, one step at a time, with output pasted back after each step.
- Previous AI coding tools struggled when given long multi-part prompts, so keep tasks small.
