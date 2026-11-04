# Theory chapter — draft (2026-11-04)

Status: DRAFT for the report and viva (user-approved "Report/viva" item). Every statement below is tied to the code that
implements it; parameters are the values in the code on 2026-11-04 (see `docs/PARAMETERS.md`). Where the theory needs an
assumption the real system does not meet, the gap is stated.

## 1. Communication graph model

The swarm of *N* drones (N = 10 in 2D SITL, 6 in Gazebo) is modelled as an undirected graph G = (V, E). Vertex *i* is
drone *i*; an edge (i, j) means *i* and *j* exchange state directly over the peer link (UDP datagrams, HMAC-authenticated,
`sitl/peer_udp.py`).

- **Topology.** `AdaptiveMembership.neighbors_for` (`sitl/adaptive_membership.py`) connects each active drone to the
  previous and next active drone in ID order: a **ring** (a path of two for N = 2). A ring on N vertices is connected
  and has diameter ⌊N/2⌋ (5 hops for N = 10).
- **Self-repair.** When a drone stops sending (timeout), the survivors drop it and close the ring over the gap, so the
  graph stays connected as long as at least two drones remain (an isolated drone lands: `peer_isolation_failsafe`).
- **Imperfect links.** Messages are delayed (mean 40 ms ± 15 ms in `--realistic`), lost (5 %, bursty Gilbert-Elliott)
  and can be reordered; each message carries its sample time, so receivers know its age (clocks are synchronised
  against the operator clock; `sitl/console_clock.py`).

The graph is therefore *time-varying* and *lossy*, but it is connected at any instant in which every ring link has
delivered a fresh message. This is the property the consensus result below needs.

## 2. Decentralized formation: consensus on a shared reference

Implemented in `sitl/formation_consensus.py`, used by `agent_mission._decentralized_goal`.

Each drone *i* keeps two internal states, both shared with its neighbours *N(i)*:
- r_i ∈ ℝ², its estimate of the **formation reference** (the point the formation slots are measured from);
- v_i ∈ ℝ², its estimate of the **leader's goal velocity**.

One drone, the elected leader L (lowest active ID, `AdaptiveMembership.leader_id`), anchors the reference to the
mission goal g(t) (from its mission intent or route): r_L = g(t), v_L = ġ(t). Followers never read the operator's goal.
They run, at the agent's 10 Hz loop,

    v̇_i = −k_v ( v_i − mean_{j∈N(i)} v_j )                       (k_v = VELOCITY_GAIN = 3.0 s⁻¹)
    ṙ_i =  v_i − k ( r_i − mean_{j∈N(i)} r̂_j )                   (k   = GAIN = 3.0 s⁻¹)

where r̂_j = r_j + v_j·age_j is neighbour *j*'s reference extrapolated to the present. Drone *i* then flies

    target_i = r_i + s_i + c_i ,   c_i = sat_{1 m}( 0.5 · mean_{j∈N(i)} [ (p_j − s_j) − (p_i − s_i) ] )

with s_i its formation slot and p the measured (GPS-based) positions: c_i is a bounded correction towards the slot that
the neighbours' actual positions imply. Degraded or landing neighbours are excluded from c_i (live finding 2026-11-04:
a leader flyaway otherwise pulled its neighbour along).

**Convergence (why it works).** Stack the followers' errors e_i = r_i − r_L. With the leader pinned, the follower
dynamics are ė = −k (L_f + B) e + (velocity-estimate error), where L_f is the Laplacian of the follower subgraph and B
is diagonal with B_ii = 1/|N(i)| for followers adjacent to the leader. The matrix L_f + B is positive definite if and
only if **every follower is connected to the leader through the graph** (the leader is a root of a spanning tree; Ren &
Beard 2008, *Distributed Consensus in Multi-vehicle Cooperative Control*, Ch. 3). The ring of §1 satisfies this whenever
it is connected. Then:
- the velocity estimates converge, v_i → v_L, at rate set by k_v;
- with v_i = v_L the reference errors decay exponentially, e(t) → 0, at a rate set by the smallest eigenvalue of
  k(L_f + B). For a 10-drone ring this mode is slow (0.147 s⁻¹ at k = 3, i.e. a ~7 s time constant; computed from the matrix), which is why the gain was raised from 0.8;
- for a **constant-velocity** leader the steady-state error is **zero** (the velocity estimate supplies the feed-forward).

Discrete-time stability needs Δt·k < 1 (here 0.1 × 3 = 0.3). Two implementation errors were found by the unit tests
before any flight and are worth stating in the viva: (i) feeding the neighbours' *full* reference rate forward (instead
of a separate leader-velocity estimate) never settled, because followers kept exciting each other; (ii) comparing a
one-tick-old own reference with neighbours' references extrapolated to *now* biased every hop by v·Δt, accumulating
along the ring.

**Measured (UNIT, `tests/test_formation_consensus.py`).** 10-drone ring: static goal → exact; constant 0.3 m/s leader →
exact, also with 30 % packet loss; leader change mid-run → continuous; turning leader (6 m circle at 0.3 m/s) → lag up
to ~0.8 m at 5 hops (the velocity estimate lags a rotating velocity: a second-order effect the law does not remove).

**Measured (SITL, 10 drones, single runs).** Operator link cut 90 s mid-patrol: the reference moved ~13 m with the leader;
followers' references trailed the leader goal by 0.4 m (1 hop) to 2.3 m (5 hops) at the patrol corners.

**What remains centralized.** Formation *changes* (which slot each drone takes, the synchronized start) are planned by
the coordinator (`sitl/formation_transition.py`, `sitl/transition_protocol.py`). The accurate claim is therefore
"decentralized formation keeping and goal propagation; centrally planned formation changes".

## 3. Safety filter: why drones stay outside each other's keep-out zone

Implemented in `sitl/cbf_filter.py`; applied to the guidance output and again to the final transmitted target
(`IndependentAgentNode._final_barrier`).

For each neighbour *j* define the barrier h_ij = ‖p_i − p_j‖² − D², with D the safety distance (2.8 m, above the 2.5 m
evaluation gate; +2 m for a neighbour whose estimate is degraded). The safe set is h_ij ≥ 0. If the drone is treated as a
**single integrator** ṗ_i = u_i and the neighbour keeps its current velocity v_j, then

    ḣ_ij = 2 (p_i − p_j)·(u_i − v_j),

and enforcing ḣ_ij ≥ −α h_ij (α = 3.0 s⁻¹) makes h_ij ≥ 0 **forward invariant** (Ames et al. 2017, IEEE TAC 62(8)):
if the pair starts outside D it cannot cross D, and if it starts inside, h recovers exponentially. The condition is one
half-plane per neighbour,

    (p_i − p_j)·u_i ≥ (p_i − p_j)·v_j − α h_ij / 2 .

The filter chooses the velocity closest to the nominal one subject to all half-planes and the speed limit ‖u‖ ≤ u_max:
a 2-D quadratic program over a disc, solved **exactly** by enumerating the candidate optima (projections onto lines,
line-line and line-circle intersections, the disc point nearest the nominal). The filtered velocity is converted back to
a position setpoint for ArduPilot.

**Assumptions the real system does not meet** (and how the project covers them):
- the drone is not a single integrator: ArduPilot's position loop lags (→ D = 2.8 m is above the 2.5 m gate);
- positions are estimates with GPS error (→ formation spacing scaled ×1.4, planner clearance + σ, degraded-peer margin);
- neighbours' velocities are not constant and arrive late (→ extrapolation by message age);
- the filter runs at 10 Hz on sampled data, not continuously;
- if constraints are infeasible (surrounded) the guarantee does not apply; the filter reports it and holds.
The measured near-misses under `--realistic` came from autopilot estimation events (flyaways), where the drone does not
follow its commanded target at all; no command-level filter can prevent those, which is why a separate flyaway guard
(`sitl/estimate_health.py`) isolates such a drone.

## 4. Slot-tracking control loop

Each agent closes the outer loop; ArduPilot closes the inner loops (`sitl/decentralized_guidance.LocalDroneAgent.command`,
`IndependentAgentNode._run_guidance`):

1. **Reference:** goal g (centralized/hybrid) or consensus reference r_i (decentralized), plus slot s_i (from the
   current formation or, during a transition, a minimum-jerk interpolation between start and target slots).
2. **Slew limit:** the commanded position moves towards g + s_i at most 0.45 m/s, so a new goal never asks the
   autopilot for a multi-metre step (a velocity-derived look-ahead was tried first and caused drift after morphs).
3. **Obstacle repulsion, estimate-health freeze, geofence clamp,** then the **safety filter again** on the final target.
4. **MAVLink** SET_POSITION_TARGET_LOCAL_NED (position + fixed yaw north) to ArduPilot GUIDED at 10 Hz; ArduPilot's
   cascaded position → velocity → attitude → rate controllers do the rest.

Loop properties worth stating: the outer loop is a reference generator, not a dynamic controller, so its stability rests
on ArduPilot's tuned inner loops plus the slew limit; tracking error under `--realistic` is typically < 1 m (p95 ~1 m)
with occasional autopilot flyaways of 7-18 m, which dominate the safety statistics.

## References (to verify and format for the report)

- W. Ren, R. W. Beard, *Distributed Consensus in Multi-vehicle Cooperative Control*, Springer, 2008.
- R. Olfati-Saber, R. M. Murray, "Consensus problems in networks of agents with switching topology and time-delays",
  IEEE TAC 49(9), 2004.
- A. D. Ames, X. Xu, J. W. Grizzle, P. Tabuada, "Control barrier function based quadratic programs for safety critical
  systems", IEEE TAC 62(8), 2017.
- L. Wang, A. D. Ames, M. Egerstedt, "Safety barrier certificates for collisions-free multirobot systems", IEEE T-RO
  33(3), 2017.
