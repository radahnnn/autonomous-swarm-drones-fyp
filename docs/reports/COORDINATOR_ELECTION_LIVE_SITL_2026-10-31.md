# Acting-coordinator failover — live 10-drone SITL (2026-10-31)

**Evidence level: SITL** — 10 ArduPilot SITL instances, standalone 2D, one host, loopback UDP; `--control-mode hybrid --agent-runtime independent --adaptive-swarm --transition-protocol`, patrol square 12 m at 0.3 m/s, swarm switched to **centralized** mode before the failure. Commits under test: `cb278e5` (run 1) and `c977a04` (run 2). Raw logs (not in Git, checksummed): `runs/independent/election-live-20261031/`. Run 2 report: `docs/evidence/2026-10-30/election_run2_run_report.json`.

## What was built (`sitl/coordinator_election.py`, agent + peer-link wiring)
A **term-numbered, lease-based takeover, not a full Raft vote**:
- If the operator's mission stream is silent for 3 s, the drone the swarm already agrees is its leader (lowest live ID from adaptive membership) becomes the *acting coordinator*: it increments the **term** and republishes the last mission — including its route — to peers every second over a new HMAC-authenticated `coordinator` peer message.
- Followers follow the best claim (higher term, then lower ID), ignore acting messages while the operator is fresh, and **the operator always wins on return**. The acting role expires after 5 minutes. There is no vote round, so under a partition each side may elect its own acting coordinator; the term/ID rule resolves it when the partition heals.
- Centralized and hybrid drones see a coordinator again (`coordinator_connected` stays true), so **centralized survives the operator's death** instead of holding and landing.
- Found while tracing the code: the agent's cached mission intent expired after 60 s during any long autonomous period, which would have caused an "invalid mission intent" landing. Lease extensions now refresh the cached copy as well (regression-tested over 200 s).

## Live results (run 2)
| Event | Result |
|---|---|
| Operator stream paused (t+76.9 s), swarm in centralized mode, patrolling | D1 became acting coordinator at **t+79.6 s (2.7 s later)**, term 1; all 10 drones reported `acting:0`; all 10 stayed airborne and centralized |
| Patrol under the acting coordinator (31 s) | centroid kept moving: 9.2 m net displacement along the route |
| Brief hold before takeover | the 10 centralized drones logged "coordinator lost; holding position" once (their 2.5 s coordinator timeout fires slightly before the 3 s election timeout) and resumed when acting messages arrived |
| D1's agent killed (manually, ~t+112 s) | D2 became acting coordinator at **t+114.3 s**, term 2; the swarm followed `acting:1` |
| Operator stream restored (t+121.9 s) | D2 stepped down at **t+122.5 s**; all drones reported `operator` again |
| Landing | 9 of 9 live agents landed on command; the console rescue-landed D1's vehicle |
| Run report | **FAIL** (see below): min gap 0.101 m, 1 violation episode |

## Findings
1. **False takeover at startup (run 1, fixed in `c977a04`):** D1 had been blocked inside the arm/takeoff helper after `launch`, so it had not read the operator's messages and woke up believing the operator was silent. The election now restarts its silence timers after such pauses (explicitly after the helper and for any gap > 2 s between ticks), as the membership code already did.
2. **The run-report FAIL is not caused by the election.** Among the nine surviving drones (D2…D10) the minimum gap was **3.90 m through the whole acting-coordinator phase and fell to 0.10 m only after D1 was dropped from the membership at t+114 s**. When a member is lost the formation is redistributed across the survivors and they fly straight to their new slots; in the SITL position-setpoint path the separation guard can only shift a setpoint by ≤ 0.25 m, so it cannot stop crossing paths. This is a pre-existing weakness (membership change while flying a moving formation), now reproduced with 10 drones, centralized mode and an active route. It is the motivation for the control-barrier safety filter (next feature).
3. The driver's kill pattern missed once and I killed D1's agent by PID manually (logged); timing of that kill is therefore approximate (~t+112 s).

## Limits
- The operator was "lost" by pausing its intent stream, not by killing the console process. One run per scenario; no network impairment; no partition.
- The takeover used `elected_leader_id` from adaptive membership; without `--adaptive-swarm` failover is disabled.
- The acting coordinator republishes the last mission only. It does not accept new commands, run formation transitions or mode switches, or take over clock synchronization.
- Not exercised: partition/heal with two acting coordinators, takeover during a formation transition, the 5-minute expiry.
