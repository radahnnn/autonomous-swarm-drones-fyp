# Swarm Drones FYP: Running Log and Plan

**Project:** Swarm Drones Coordinated Controlled Movement (BEE-60, Dept. of Electrical Engineering)
**Supervisor:** Dr. Abdul Ghafoor
**Log started:** 30 Sep 2026
**Target:** software complete by **31 Dec 2026** (proposal runs to May 2027; we are finishing early on purpose)

How this file works: it is a running record of what we did, what went wrong, how we fixed it, and what comes next. It sits next to the Codex files (`CLAUDE.md` and `FYP_SETUP_LOG.md`) and does not replace them. Entries marked *(from Codex handoff)* were not re-verified by Claude.

---

## 1. Project in one paragraph

Build the control software that lets a group of drones fly in formations (line, V, circle, grid), switch between them, and avoid collisions. It has three modes: **centralized** (one planner tells every drone what to do), **decentralized** (each drone reacts to its neighbours using consensus and flocking), and **hybrid** (a combination that can switch between the two). Target size is 5-10 drones over limited-range, possibly unreliable wireless links. Validation is mainly in simulation, with real drones as a bonus.

## 2. Scope from the approved proposal

- Mathematical models: swarm as a graph (drones = nodes, links = edges).
- Centralized control: global trajectory and formation planning.
- Decentralized control: consensus algorithms and flocking (boids: separation, alignment, cohesion).
- Hybrid framework with switching between the two.
- Formations: line, V, circle, grid, plus dynamic reconfiguration.
- Collision avoidance and inter-drone spacing.
- Swarm size 5-10 drones, wireless links that may be limited or unreliable.
- Distributed onboard controllers plus a central unit (ground station or leader drone).
- Extension of the earlier BEE-59 swarm robot project.
- Domains ticked: AI, Communication, Embedded Systems, Computer Networks.

## 3. Hardware facts

- Department has **10 drones**; the team currently has **2**.
- Flight controller: Matek **H743**-SLIM V3 (a teammate once wrote "H543", but the PDF and photos say H743; confirm on the physical board).
- Free ports per the port-mapping PDF: UART4, UART6, UART7, UART8, I2C1, I2C2, CAN. The PDF was written from photos, so nothing is verified in software.
- Team is trying to link the drones to an **ESP32** (hardware teammates own this).
- **Firmware is unknown.** The PDF mentions both Betaflight and ArduPilot. Betaflight cannot be commanded by our swarm code; ArduPilot can. This needs a teammate to check with the drone on USB, props off.
- **GPS is unknown.** Formation flight on real drones needs a position source.
- Do not flash, arm, or connect to a physical aircraft until firmware and wiring are confirmed with the hardware teammates.

## 4. Software state *(from Codex handoff, not re-verified)*

- Ubuntu 24.04.5, Ryzen 5 3500X (6 cores), 15 GiB RAM, GTX 1660 Super.
- ROS 2 Jazzy installed. ArduPilot workspace at `~/ardu_ws`, PX4 workspace at `~/ros2_px4_ws`.
- ArduPilot SITL + MAVProxy runs; the Micro ROS Agent connects; `/ap/*` topics were seen from one **disarmed** drone.
- PX4 SITL with Gazebo x500 ran earlier (kept only as a backup).
- The current ArduPilot simulation has **no 3D Gazebo model** yet.
- Fresh-terminal setup and restart commands are in `CLAUDE.md`.

## 5. Decisions made

| Date | Decision | Reason |
|---|---|---|
| 30 Sep | Target completion **Dec 31**, not May | User request; proposal allows it |
| 30 Sep | **ArduPilot** is the autopilot (PX4 = backup only) | Real drones likely run ArduPilot; less split effort |
| 30 Sep | Algorithms first in a **fast Python dot simulator**, then validate on ArduPilot SITL | Fast, repeatable experiments; easy debugging |
| 30 Sep | ArduPilot SITL starts **now** (2 drones → 5) | Already installed, light on the machine |
| 30 Sep | Gazebo is a **visual layer**, added around week 3, never a dependency | Heavy on the GPU; would slow experiments |
| 30 Sep | Control ArduPilot via **MAVLink (pymavlink/MAVSDK)**; ROS 2 DDS multi-vehicle work postponed | Simpler for a beginner; proposal doesn't require ROS 2 |
| 30 Sep | Real drones and ESP32 are a **bonus**, not a blocker | Hardware timeline is outside our control |
| 30 Sep | Cybersecurity is **out of scope** (stretch only) | Not in the approved proposal |

## 6. Open questions

- [ ] What does "hybrid" mean for this project? (Suggested: central planner picks formation and trajectory, drones handle spacing and collision avoidance locally, flocking fallback if the link or leader is lost.) **Owner: user with supervisor.**
- [ ] What firmware do the two drones run? **Owner: hardware teammates.**
- [ ] Do the drones have GPS or another position source? **Owner: hardware teammates.**
- [ ] Is there a safe, permitted place to fly? **Owner: team.**
- [ ] What are the real mid-term and final deadlines? **Owner: user.**
- [ ] What did BEE-59 deliver, and can we reuse it? **Owner: user.**
- [ ] Which ESP32 role: simple MAVLink relay (laptop runs the logic) or onboard logic per drone? **Owner: team.**
- [ ] What citation style does the department require? (Likely IEEE.)
- [ ] Message format between drones (draft: drone ID, position, velocity, formation, timestamp, rate).

## 7. Plan (1 Oct to 31 Dec 2026)

**Goal:** 5 simulated ArduPilot drones flying line, V, circle and grid with centralized, decentralized and hybrid control, tested under network problems, with a report and demo video.

### Phase 0: Ground rules (1-7 Oct)
- [ ] Define "hybrid" with the supervisor.
- [ ] Get real deadlines.
- [ ] Ask teammates: firmware, GPS.
- [ ] Set up project folder and Git repo.
- [ ] Draft the shared message format.

### Phase 1: Foundations (1-21 Oct)
- [ ] Track A: Python swarm library (drone object, graph model, consensus, line formation), plus a network emulator (delay, loss, range).
- [ ] Track B: launch 2 ArduPilot SITL drones (unique system IDs, separate ports); script to connect, take off, and read positions; shared coordinate frame; one drone holds an offset from the other.
- [ ] Track C: ArduPilot + Gazebo with 1 drone, then 2 (visual only).
- **Done when:** 5 dots form a line and 2 SITL drones hold a fixed offset.

### Phase 2: Formations and safety (22 Oct-11 Nov)
- [ ] V, circle and grid formations; formation switching.
- [ ] Collision avoidance and spacing (boids-style separation first).
- [ ] Port controller to 2 SITL drones.
- [ ] Start collecting metrics.
- **Done when:** dot swarm forms all four shapes with no collisions, and 2 SITL drones switch formation.

### Phase 3: Three control modes (12 Nov-2 Dec)
- [ ] Decentralized (consensus + flocking).
- [ ] Centralized planner.
- [ ] Hybrid switching (link loss, leader loss).
- [ ] Scale SITL to 3, then 5 drones.
- [ ] Apply the network emulator to SITL runs.
- **Done when:** all three modes work on 5 SITL drones and hybrid switches when the link is cut.

### Phase 4: Experiments (3-16 Dec)
- [ ] Delay, loss and range sweeps for each mode.
- [ ] 5 → 10 drone scaling study in the dot simulator.
- [ ] Save all plots and tables.
- [ ] Gazebo with 5 drones (if the machine copes).
- [ ] 2-drone ESP32 test (if hardware is ready).

### Phase 5: Freeze and finish (17-31 Dec)
- [ ] **Code freeze 17 Dec.**
- [ ] Demo video.
- [ ] Report and slides (drafts start during Phase 3).

### Metrics to record
Formation error, time to converge, minimum inter-drone distance, behaviour during formation switches, robustness to delay/loss/range, scaling from 5 to 10 drones.

### Decision gates

| Date | Gate | If missed |
|---|---|---|
| 7 Oct | Hybrid defined; firmware known | Proceed on assumptions, flag the risk |
| 21 Oct | 2 SITL drones hold offset | Drop Gazebo, stay headless |
| 11 Nov | All four formations in dot simulator | Cut circle or grid |
| 2 Dec | Hybrid working on SITL | Present hybrid in dot simulator only |
| 17 Dec | Code freeze | No exceptions |

## 8. Literature and tools

**Recent papers found (abstracts only, not yet read in depth):**
- RSC: Decentralized Rigid Formation Flocking for Large-Scale Swarms via Hybrid Predictive Control and Online Reconfiguration (arXiv 2606.04248)
- LAEI: Layered Autonomous Edge Intelligence Framework for Robust UAV Swarm Operations (2606.09099)
- Hybrid Centralized Distributed Control for Lifelong MAPF over Wireless Connections (2601.08214)
- Consensus and flocking with transmission and reaction delays (2604.18848)
- Asymptotic consensus and flocking under decaying persistent excitation on rooted digraphs (2608.27263)
- Bio-inspired Integrated Networking and Control for Large-Scale Swarm (2511.16067)

**Classic references to look up and verify before citing:**
- Reynolds (1987), "Flocks, Herds, and Schools" (boids)
- Olfati-Saber (2006), "Flocking for Multi-Agent Dynamic Systems: Algorithms and Theory"
- Olfati-Saber and Murray (2004), "Consensus Problems in Networks of Agents with Switching Topology and Time-Delays"
- Ren and Cao, *Distributed Coordination of Multi-agent Networks*

**Tools:** alphaXiv and SciSpace connectors (available in Claude); Zotero for references (to set up later); ArduPilot docs and forum for SITL and MAVLink.

**To do next with papers:** read Olfati-Saber flocking, RSC, and LAEI; summarize the controller used, how each handles communication limits, and what is reusable for the hybrid design.

## 9. Known issues and fixes

| Problem | Fix | Source |
|---|---|---|
| Micro-XRCE-DDS-Gen build fails on Java 21 (`Unsupported class file major version 65`) | Use Java 17 for the build | Codex handoff |
| MAVProxy not found in PATH | `source ~/venv-ardupilot/bin/activate` | Codex handoff |
| `ros2 topic echo /ap/status` says "message type invalid" | Re-source `/opt/ros/jazzy/setup.bash` and `~/ardu_ws/install/setup.bash` | Codex handoff |
| MAVProxy warning: Matplotlib `Axes3D` import failed | Non-fatal; only affects the map's 3D plot | Codex handoff |
| Build warning: Fast DDS / Fast CDR version conflict with `/usr/local` XRCE agent | Ignore unless runtime errors appear; then check which agent binary is running | Codex handoff |
| Do not run old direct SITL alongside ROS-launched SITL | Run one at a time to avoid port conflicts | Codex handoff |

---

# Daily log

## 30 Sep 2026

### What we did
- Reviewed the Codex handoff (`CLAUDE.md`). Found it well organized on safety and state, but the requirement section was out of date and tooling had run ahead of the project itself.
- Read the approved proposal and extracted the scope (section 2 above).
- Read the H743-SLIM V3 port-mapping PDF (section 3 above).
- Explained the project in plain language and agreed the overall approach.
- Agreed to aim for completion by 31 Dec instead of May.
- Decided the architecture: swarm logic written once in Python, run on a dot simulator, ArduPilot SITL (MAVLink), and later real drones through the ESP32.
- Agreed to start ArduPilot SITL now with 2 drones, scale to 5, and add Gazebo as a visual layer.
- Wrote the Oct-Dec plan with phases, decision gates, and fallbacks.
- Searched alphaXiv for relevant papers and listed the best matches (section 8).
- Explained Zotero; setup postponed.
- Created this log.

### Problems and solutions
- **Problem:** the handoff said the required behaviour was undecided. **Solution:** the proposal defines it (centralized, decentralized, hybrid, four formations, 5-10 drones). The handoff's requirement section is now superseded by section 2 above.
- **Problem:** effort so far was mostly tooling, with no swarm algorithm yet. **Solution:** put the algorithms first in a fast dot simulator, and use ArduPilot as a validation layer.
- **Problem:** Gazebo with several drones is heavy on a GTX 1660 Super and would slow experiments. **Solution:** headless SITL for experiments; Gazebo for visuals only, tried with 1-2 drones first.
- **Problem:** board name inconsistency (H543 vs H743). **Solution:** treat it as H743 per the PDF and photos; confirm on the physical board.
- **Problem:** the PDF's mention of both Betaflight and ArduPilot means the firmware is not actually known. **Solution:** added to open questions; teammate to check with USB, props off.
- **Problem:** "hybrid" is not defined in the proposal. **Solution:** proposed a definition; needs supervisor approval.
- **Problem:** PX4 and ArduPilot both maintained. **Solution:** ArduPilot chosen; PX4 kept as a backup only.

### End-of-day state
- Environment installed per the Codex handoff; nothing was run or changed on the user's machine today.
- No swarm code written yet.
- Nothing armed; no physical aircraft touched.
- Next actions: (1) ask teammates about firmware and GPS, (2) ask supervisor about "hybrid" and deadlines, (3) start the two-SITL-drone setup, one terminal at a time (first check the multi-instance options in the local ArduPilot checkout).
