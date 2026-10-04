# SITL + Gazebo: drones "fly" in the terminal but stay on the runway

Symptom: `sitl/start_and_climb.py` (or the interactive consoles) print "Takeoff commanded" and sit in
"Monitoring climb", but the Cinewhoops never leave the runway in Gazebo (z stays ~0.05 m).

## Root cause (found 4 Oct 2026, debugged live over SSH)
1. ArduCopter will not arm in GUIDED until the EKF has a position estimate. ArduPilot's log showed
   `Arm: Need Position Estimate` / `Arm: Accels inconsistent` and motor outputs stuck at 1000.
2. The flight scripts sent the arm command for only 15-25 s, never checked the result, and printed
   "Takeoff commanded" anyway, so a vehicle that never armed looked like it was flying.
3. The Gazebo models had `<lock_step>0</lock_step>`. Without lock-step ArduPilot's clock ran ahead of
   Gazebo's (about 190 s of SITL time after about 100 s of wall time), so the EKF never fused GPS.
   With `<lock_step>1</lock_step>` both share one clock and the EKF starts using GPS after about 45 s of
   simulated time.

## Fix
- `simulator/gazebo/models/cinewhoop_{1,2,3}/model.sdf`: `lock_step` back to 1.
- New `sitl/flight_prep.py`: `arm_with_retry()` retries GUIDED + arm until the heartbeat reports ARMED
  (up to 180 s, forced arm only as a last resort) and `takeoff_and_verify()` confirms the vehicle climbs.
  `start_and_climb.py`, `interactive_flight_console.py` and `interactive_swarm_flight.py` use them and now
  stop with a clear error instead of pretending to fly. Tests: `tests/test_flight_prep.py`.

## Second bug: one wing drone ends up behind/next to another (V-formation broken)
Symptom: after takeoff the wings are not at the V slots; e.g. both wings end up on the same side, about 1 m apart.

Cause: the scripts started the three SITL instances with staggered `--home` points (5 m apart in longitude).
In Gazebo (JSON) mode the plugin sends each drone's *absolute* world position and SITL adds it to `--home`, so
every drone's real position was shifted west by 5 m per drone index (drone 2: -5 m, drone 3: -10 m) relative
to where the controller thought it was.

Fix: in Gazebo mode all instances share ONE home (the world origin, `-35.363261,149.165230`).
`start_and_climb.py` and `interactive_flight_console.py` (Gazebo branch) do this; the standalone
`--model quad` mode keeps staggered homes, which is correct there. Verified in Gazebo: apex (0, 0),
wings (-2.9, -3.0) and (+3.1, -3.0) in (East, North). Test: `test_gazebo_mode_uses_one_shared_home`.

## Expect a wait
Gazebo with a GUI on this machine runs at about 0.3x real time (`real_time_factor` in
`gz topic -e -t /world/cinewhoop_3drones/stats -n 1`). With lock-step on, SITL runs at the same speed, so
the first arm takes roughly 100 s of wall-clock time. The script prints "waiting to arm" every 10 s.
Running the Gazebo server headless (`gz sim -s`) with a separate GUI is a possible way to speed it up (untested).

## Useful checks
```bash
ss -lunp | grep -E ":(9002|9012|9022) "                       # Gazebo plugin listening for SITL
gz topic -e -t /world/cinewhoop_3drones/dynamic_pose/info -n 1 | grep -A12 'cinewhoop_1"' | grep "z:"
# ArduPilot dataflash logs (cwd of arducopter, logs/*.BIN): look for 'Arm:' MSG lines, XKF4.SS, RCOU
```
