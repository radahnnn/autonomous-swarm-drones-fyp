# Comparison with existing swarm systems — draft (2026-11-04)

Status: DRAFT for the report (user-approved "Report/viva" item). External facts are taken **only** from
`docs/reports/RESEARCH_BENCHMARK_2026-10-30.md` and keep its labels: **[V]** source read, **[S]** search snippet /
bibliographic record only, **[U]** unverified. Verify every [S]/[U] item against the primary source before the report is
submitted. "Ours" reflects the code and evidence on 2026-11-04 (`docs/PROJECT_STATE.md`).

## Key message

**Positioning error, not the control law, sets how close a swarm can fly.** Systems with millimetre positioning (motion
capture) fly sub-metre separations; GPS-positioned outdoor systems need metres, and drone shows reach dense formations
only with RTK (centimetre) positioning and pre-computed trajectories. Our formation spacing is therefore derived from the
position error: in simulation the slots are spread ×1.4 (≥ 4.2 m between neighbours) and the planner's clearance grows
with the assumed error σ = 1 m; on hardware the spacing will be set from the GPS error measured in the 5-minute hover logs
(hardware plan, PROJECT_STATE section 8).

## Table

| | **Ours (BEE-60)** | Crazyswarm (ICRA 2017) | Skybrush | Vásárhelyi et al. 2018 | EGO-Swarm (RA-L 2021) / Zhou et al. 2022 |
|---|---|---|---|---|---|
| Evidence level | SITL 2D (10 drones) and Gazebo (6); **no flight yet** | indoor hardware [V] | outdoor hardware (shows) [V] | outdoor hardware, 30 drones [S] | indoor hardware (EGO: 3 real), forest hardware (Zhou: 10) [V]/[S] |
| Positioning | simulated GPS with assumed error (0.5 m wander, 120 ms lag); EKF3 | motion capture [V] | GNSS, RTK optional (1-10 cm) [V] | GPS [S] | onboard visual-inertial odometry, no external positioning [V] |
| Typical separation | 2.5 m evaluation gate; ≥ 4.2 m slot spacing (×1.4 for GPS error) | sub-metre on nano-quads (exact figures not read) | not reported in docs | not read — **do not quote** [U] | 0.2 m obstacle margin in simulation (inter-agent figure not read) [V] |
| Control architecture | centralized, decentralized, hybrid in one codebase, switchable in flight; decentralized = leader-follower consensus on a formation reference over a 1-hop ring | central PC, broadcast radio [V] | central ground station, pre-computed show trajectories [V] | decentralized flocking model, 11 tuned parameters [S] | fully decentralized trajectory optimisation, broadcast over an unreliable network [V] |
| Inter-drone safety | control-barrier-function filter (exact QP), re-applied to the final target; flyaway guard from autopilot EKF messages | planning with a downwash model [V] | pre-show verification, geofences [V] | emergent from the flocking terms [S] | soft barrier cost inside the optimiser [V] |
| Formation change | coordinator-planned slot assignment (Hungarian) with a synchronized prepare/ack/start/HOLD protocol | discrete + continuous planner (downwash-aware paper) [V] | authored in Studio [V] | emergent | emergent / replanned |
| Comm-loss behaviour | decentralized/hybrid keep the mission (elected leader carries it); centralized holds then lands | not reported | fleet-level land/RTH/halt, secondary radio [V] | decentralized by design [S] | asynchronous replanning tolerates loss [V] |
| Formal guarantee | CBF invariance under single-integrator assumptions only; consensus convergence for a connected graph | planner-level, under assumptions [V] | verification, no proof stated [V] | none | none (benchmarked) [V] |

## How to use this in the report

- Lead with the key message and the positioning row; it explains why our spacing is metres, not centimetres, and why
  the hardware plan measures GPS error first.
- Our distinctive point is **breadth with one safety layer**: three control architectures, in-flight switching and a
  shared barrier filter, evaluated with strict whole-fleet grading and simulator truth, not flight evidence.
- Be explicit that every comparator except the simulations has a higher evidence level (they flew).
