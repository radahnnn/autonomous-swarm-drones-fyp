# Software roadmap: decentralized swarm toward realistic validation

## Purpose

Move the current five-vehicle decentralized SITL prototype toward a credible software-only
demonstration of decentralized coordination. No physical hardware or flight claim is included.
The operator may still provide a mission goal and an emergency land/stop command; decentralization
means the ground computer does not continuously decide each aircraft's movement or depend on one
vehicle to control the others.

## Historical starting point checked on 2026-10-21

At the roadmap's initial inspection, the experimental branch was `work/decentralized-sitl-20261020`
at `717d62a` and its committed revision matched the GitHub branch. That describes the baseline only:
the current working tree has substantial uncommitted implementation, test, evidence, and documentation
changes, so do not interpret this as a statement that the checkout is currently clean. The baseline
already contained local guidance logic, a one-hop peer
state bus with delay/loss/staleness/link-fault simulation, a process-per-agent controller option,
five-aircraft standalone SITL evidence, and 185-test evidence documented in
`reports/HYBRID_HARDENING_2026-10-20.md`.

The earlier controller-process runtime had material central dependencies: its host owned all
MAVLink links and telemetry, prepared topology and slots, distributed goals, and sent setpoints.
The new `independent` runtime moves each vehicle's MAVLink connection, telemetry reader, local
guidance loop, and setpoint sender into that vehicle's own process. Those processes exchange
one-hop peer state directly over UDP loopback. A separate operator console broadcasts the same
mission intent to each node and only reads status. The host still starts SITL and agent processes,
provisions each node's roster/slot, and provides the local operating system/network; this is a
single-computer process architecture, not multiple computers or an onboard deployment.

## Milestones

### 0. Define the claim and interfaces — completed by this document

- Keep the operator as the source of mission intent and emergency override.
- Require each aircraft agent to estimate its own state, select its own target, and compute its
  own flight-controller setpoint.
- Require neighbor information to arrive directly over agent-to-agent links; no host relay may
  update or refresh another agent's data.
- Allow common static configuration to be provisioned at startup, but do not let a runtime leader
  assign every drone's actions. Formation role/slot assignment must become deterministic from
  shared configuration or a distributed agreement method.
- Make the central mission console optional after a goal is accepted. Losing it must not stop
  local controllers or peer coordination. Emergency-stop behavior remains explicitly testable.

### 1. Separate mission intent from per-aircraft control

Create a small versioned mission-intent message (goal, formation, sequence, issue time, expiry,
coordinate frame, units). A command UI publishes the same intent to every agent. Each agent derives
its own slot/target locally. First demonstrate this with five independent processes on one host;
then terminate the command UI after acceptance and confirm that the swarm continues safely.

Acceptance: no code in the command UI calculates or sends a per-drone movement target in
decentralized mode; replayed, expired, invalid-frame, and malformed intents are rejected; an agent
with no valid intent enters a documented safe hold/land policy.

### 2. Give every agent its own flight-controller and peer transports

Move the MAVLink connection, telemetry parser, local controller, setpoint sender, and watchdog
inside the per-aircraft agent runtime. Replace `PeerStateBus` as the runtime relay with direct
agent-to-agent UDP messages. Keep deterministic fault injection as a test transport, behind the
same message interface. Add peer discovery or an explicit peer roster, message authentication,
sequence/replay checks, bounded queues, and clock-skew-safe message age handling.

Acceptance: each agent can run and be stopped independently; packet capture/logs show that peer
messages travel sender-to-recipient without a central state relay; loss of one agent does not
silently corrupt freshness for others; a dead agent affects only its own aircraft and its peers'
local policies. Transport-session restart/rejoin has a local UDP integration test. A full agent
process restart now uses a conservative policy: if its flight controller reports an armed, airborne
vehicle, the restarted agent commands LAND, announces that state to peers, and does not resume the
old mission. Unit tests and five live five-SITL trials cover detection, peer propagation, and
all-five landing/disarm. The trials recorded 16.92–17.98 seconds to fleet landing confirmation and
3.83–3.93 m minimum sampled separation; the sample is too small to estimate general reliability.
See `runs/independent/agent_restart_batch_summary_20261021.json` for metrics and definitions.
The direct UDP impairment scheduler now caps its pending queue at 64 datagrams per peer and 512
per agent process, evicts the oldest enqueued datagram under pressure, and reports `dropped_queue`.
A regression test verifies both limits and newest-packet retention. This bounds simulator-side
memory use; it does not model physical radio buffers or replace multi-host testing.

### 3. Prove coordinator independence and network behavior

Use scripted scenarios with fixed seeds and preserved raw logs: kill the UI after goal acceptance,
kill one agent, isolate one directed peer link, create a network partition, inject delay/jitter,
random and burst loss, reorder and duplicate messages, and restore the network. Include joining and
rejoining agents. Record each agent's local inputs, decisions, message age, setpoints, and safety
state. Compare against the current centralized baseline using identical initial states and seeds.

Acceptance: the swarm keeps the accepted goal or reaches the documented safe state without the
UI; no agent uses stale peer data beyond its configured limit; all runs report minimum separation,
tracking error, recovery time, and failure counts, with uncertainty intervals over multiple seeds.
Failed trials remain in the evidence set.

Current status: the same-host five-agent runtime has passed repeated operator-loss, agent-restart,
peer-partition, random-loss/reorder/duplicate, and correlated-burst scenarios. A combined
correlated-burst plus operator-loss batch produced five passing trials, while a preserved pre-fix
seed-178 failure exposed a clock-domain error in cached mission expiry. The corrected seed passed
the identical scenario and the full 257-test suite. A clean post-fix batch of seeds 178–182 then
passed 5/5, including convergence and autonomous landing/disarm of every vehicle after operator
loss. See
`reports/DECENTRALIZED_INDEPENDENT_SITL_2026-10-21.md`. A genuine two-computer LAN run, external
traffic capture, and measured host clock drift/step trials remain open.

### 4. Raise simulation realism in layers

1. Retain fast pure-software Monte Carlo for algorithm development and held-out-seed evaluation.
2. Run five independent ArduPilot SITL vehicles with one agent process per vehicle and actual
   MAVLink links. Keep the console limited to operator intent, display, and logging.
3. Run the same architecture with the project's Gazebo/ArduPilot model so vehicle dynamics,
   sensor update rates, estimator behavior, and 3D motion are in the loop. Record achieved real-time
   factor; do not silently treat slow simulation time as wall-clock time.
4. Add declared sensor imperfections (GPS/position noise, bias, update jitter/dropout), radio
   impairment, CPU/load stress, startup skew, clock skew, and agent restarts. Calibrate parameters
   only from available SITL or measured data; label assumptions where physical measurements do not
   exist.

Acceptance: the same scenarios run reproducibly in fast simulation and SITL; Gazebo results are
reported separately; sensor/network profiles and random seeds are preserved with each run. Do not
claim physical-airframe fidelity without measured mass, thrust, inertia, sensor, and flight-log
data.

### 5. Prepare, but do not claim, a future hardware boundary

Document the flight-controller interface and radio/CAN/UART choices as replaceable transports. A
software-in-the-loop agent must not assume that the MCP2551 alone supplies a CAN controller or that
the flight controller already accepts the swarm's protocol. Bench and flight integration are
separate future gates and are outside this software-only roadmap.

## Work order

Implement milestone 1 first, then milestone 2. Do not begin large-scale algorithm tuning until the
control and communication ownership boundaries are real in the runtime. Keep centralized mode and
the demo path intact. At each milestone, preserve a source revision, run focused tests and the full
software suite when appropriate, run the five-vehicle SITL scenario, and save raw evidence plus a
plain-language report. Do not merge or publish claims that exceed the evidence.

## Progress on 2026-10-21

Milestone 1 is implemented in the independent runtime: the operator broadcasts a versioned,
expiring common goal directly to each node; nodes cache a valid intent between updates and locally
combine it with a provisioned formation slot. No per-drone movement target is generated by the
operator. If an intent expires, the node commands LAND.

Milestone 2 has a meaningful first implementation: each of five node processes owns its own TCP
MAVLink connection to one SITL vehicle, reads that vehicle's telemetry, runs guidance, sends its
own position targets, and sends direct UDP peer state to its one-hop roster. An independent
five-drone standalone SITL run succeeded with a 2 m north command, 10% injected peer loss, 50 ms
mean peer latency, and mission-stream pause/resume. The fleet reached about 5 m altitude, moved
about 2 m north, kept the accepted goal during a 5-second operator-intent interruption, and landed
on operator command. Minimum observed horizontal separation was 4.136 m. Across agents, final
cumulative counters recorded 4,360 peer sends, 460 intentional sender-side loss drops, and 4,349
peer receives; no replay or invalid packets were recorded. The host peer bus is not used by this
runtime. Raw evidence: `runs/independent/20261021T134539.157325Z/`.

The first live attempt is also preserved at `runs/independent/20261021T134330.833134Z/`. It caught
a watchdog interaction: synchronous arming/takeoff checks drained MAVLink while the loop was
blocked, after which a stale-heartbeat check incorrectly commanded LAND. The heartbeat anchor is
now refreshed after that checked command sequence, and the repeat run stayed airborne and moved.

A separate seeded scenario killed the operator process while the five agents and ArduPilot SITL
continued in separate process sessions. All agents continued on the accepted mission, then
independently requested LAND when its 30-second lease expired. A corrected repeat (seed 52) confirmed
all five transitioned to `landed`, disarmed, and reached ground altitude within 42.98 seconds, with
no emergency LAND fallback. Minimum time-aligned horizontal separation was 4.179 m. Configuration:
10% injected peer loss and 50 ms mean latency. Evidence:
`runs/independent/20261021T161122.758370Z/`. The earlier seed-50 observer had no armed field and only
confirms reaching ground, not disarm. After accepted intent expiry moved to a local monotonic
deadline, seed 89 repeated the coordinator-loss trial on the updated code. All five agents explicitly
reported landed, disarmed, and on the ground 42.99 seconds after operator loss; no emergency fallback
was used and minimum time-aligned separation was 4.20 m. Evidence:
`runs/independent/20261021T162235.986244Z/`. Harness false-negative attempts are also preserved.
After adding launch preflight and a five-second in-flight startup grace, seed 91 repeated the
operator-loss scenario successfully: all five agents landed and disarmed 43.03 seconds after
operator loss, without emergency fallback; minimum time-aligned separation was 4.199 m. The failed
seed-90 regression that exposed staggered-takeoff timing is preserved separately at
`runs/independent/20261021T163346.001631Z/`; seed-91 evidence is at
`runs/independent/20261021T163704.820929Z/`. Two later repeats (seeds 92 and 93) also passed using
fresh latest-status confirmation and no emergency fallback. There are now five confirmed
operator-loss runs (seeds 52, 89, 91, 92, 93). Their lease-expiry landing times ranged from 42.98 to
43.08 seconds; sampled minimum separation ranged from 4.126 to 4.200 m. See
`runs/independent/operator_loss_batch_summary_20261021.json` for the data and method. An older
folder that stopped with all agents armed near ground is preserved but excluded from successful
counts. These five same-host seeds are not a reliability estimate. See the detailed
[independent SITL report](reports/DECENTRALIZED_INDEPENDENT_SITL_2026-10-21.md).

The direct UDP runtime now supports a directed link cut using `--peer-block 1:2`. In one live
five-drone trial, only drone 1's messages to drone 2 were blocked; reverse and other peer links
continued. The swarm completed a 2 m north move and all five agents were confirmed disarmed on the
ground before shutdown. The selected sender counted 406 blocked packets, and minimum observed
horizontal separation was 4.171 m. Evidence: `runs/independent/20261021T141447.196880Z/`.

The independent agent's mission lease now uses wall-clock time only to validate an incoming,
expiring intent, then tracks its remaining lifetime with a local monotonic deadline. Before mission
operation, the operator now estimates each agent's wall-clock offset with NTP-style UDP probes and
distributes an offset map used for mission/command expiry, status timestamps, and peer packet age.
The external-agent integration test deliberately skewed five fake agent clocks by up to six seconds;
the console recovered the offsets and received acknowledgement from all five. It then simulated a
0.4-second post-start offset change and verified the scheduled 60-second re-estimation path (sped up
in the test). This is still one-host simulated-clock evidence; real multi-host asymmetry and host
clock-step behavior remain open. A focused four-timestamp test also injects strongly asymmetric
one-way delays and checks that offset error stays within half the measured network RTT, the
worst-case bound when both clocks remain stable during a probe. This verifies the estimator's
arithmetic; it is not a measurement of LAN delay asymmetry. Clock probes now also compare wall-clock
elapsed time with monotonic elapsed time at the operator, and the agent reports its monotonic probe
processing duration. Samples are discarded if either clock changes by more than 5 ms during a
probe. Focused regressions cover operator and agent clock steps; actual host clock services and
multi-computer drift remain untested.

These were one-run results on localhost SITL. Later sections record Gazebo, network-fault,
agent-restart, and simulated clock-offset evidence. Multi-host routing, real clock-step behavior,
full identity, and hardware validation remain open.

On 2026-10-21, two more live standalone SITL runs exercised a software-agent process failure. In
both, the operator killed drone 3's independent controller while all five simulated aircraft were
airborne. Four agents remained active. The operator's emergency MAVLink connection then selected
LAND for drone 3 and confirmed it disarmed at ground level; all five vehicles were eventually
confirmed landed before SITL shutdown. One run also used 5% peer loss, 50 ms mean delay, and a
two-way cut between drones 1 and 2; the second used 2% peer loss and 25 ms mean delay. The closest
time-aligned horizontal separation observed was about 4.16 m in both runs. Evidence is saved in
`runs/independent/20261021T141935.340047Z/` and
`runs/independent/20261021T142220.566537Z/`.

These runs validate the test/operator emergency rescue path, not autonomous fault recovery. The
failed aircraft's SITL flight controller remained alive and was reachable over TCP; the operator
computer took over its MAVLink connection and landed it. A failed onboard companion or lost
flight-controller link still needs an independently configured and tested flight-controller
failsafe. The simulator shutdown logic was adjusted so it waits for active agents, then immediately
attempts emergency landing for any dead agent instead of waiting for a stale status to time out.

An additional SITL-only profile enables ArduPilot's GCS failsafe to select LAND after five seconds
without the per-aircraft agent heartbeat. In one five-drone run, drone 3's agent process was killed
while airborne; an independent MAVLink observer saw ArduCopter switch to LAND and later confirm
disarmed at 0.008 m relative altitude without an operator rescue command to that vehicle. The normal
parameter file is untouched; the generated profile and observation are under
`runs/independent/20261021T142748.189010Z/`. This validates one simulated autopilot response to
companion-process loss, but does not simulate autopilot power loss, sensor failure, or a physical
radio link. ArduPilot's [GCS failsafe documentation](https://ardupilot.org/copter/docs/gcs-failsafe.html)
describes its heartbeat timeout and configurable mode action.

Dynamic `link-block` and `link-restore` fault-injection commands were then added. In a five-drone
run, both directions between D1 and D2 were cut and then restored while airborne. Each direction
counted 72 blocked datagrams over the run, and post-restoration peer traffic resumed. The fleet
landed with a minimum time-aligned horizontal separation of about 4.20 m. Evidence:
`runs/independent/20261021T143211.872887Z/`. This confirms transport restoration, not distributed
partition resolution: the operator/test harness chose when to restore the links.

The independent runtime has now also completed one live run in the five-Cinewhoop Gazebo Harmonic
world. All five EKFs became ready after about 206 seconds; the vehicles armed, took off to roughly
5 m, moved on a 2 m north mission, and landed/disarmed. With 2% injected peer loss and 25 ms mean
latency, the minimum time-aligned horizontal separation was about 4.173 m. A ground-level
telemetry-timeout LAND command occurred before launch while one vehicle was disarmed; it later
armed and flew. No in-flight telemetry-timeout event was observed. Evidence:
`runs/independent/20261021T143853.501763Z/`. An earlier sandbox-only attempt
(`runs/independent/20261021T143620.815628Z/`) is retained as a failed environment check: OpenGL
could not initialize, so no flight was attempted. This is one Gazebo run and does not establish
repeatability or measured physical sensor fidelity.

The independent runtime now sends separate peer-liveness heartbeats every 0.5 seconds, including
during blocking EKF, arming and takeoff checks. Position samples still expire after 0.5 seconds and
alone feed guidance. While airborne, a link becomes stale after 1.5 seconds without a heartbeat;
if still missing after a further 2-second grace period, that agent commands ArduCopter LAND and
records the missing peer IDs and reason. Grounded agents learn which configured peers are
reachable without starting the failsafe timer. Link recovery never triggers automatic takeoff.
This remains a per-agent fail-safe, not distributed consensus: only agents whose own required links
time out respond, while others may continue their mission.

Three five-drone standalone SITL partition trials have passed: one interactive run and two scripted
seeds (67, 68) with 2% peer loss and 25 ms mean latency. Cutting both D2↔D4 directions led both
endpoints to land; the other three stayed airborne until operator LAND, then all five disarmed.
The scripted failsafe status appeared 3.71 and 3.74 seconds after cuts; touchdown took about 16.96
seconds. A first automated run exposed false triggers because position packets stop during blocking
takeoff checks; it was preserved and fixed by adding the independent heartbeat channel. These are
initial trials, not reliability statistics. Raw runs:
`runs/independent/20261021T144908.936448Z/`, `runs/independent/20261021T145411.084432Z/`,
`runs/independent/20261021T145947.252933Z/`, and
`runs/independent/20261021T150113.677325Z/`.

## Current conclusion

The project has moved beyond the host-emulated decentralized prototype: independent SITL node
processes now own per-drone MAVLink and direct peer-state transport, while the operator broadcasts
only shared intent. Static peer and operator routes, shared-key HMAC, source-endpoint validation for
peer datagrams, and an external-agent console are implemented, but operation has only been tested on
one PC; topology/slots are fixed and there is no per-drone cryptographic identity. Operator-process
loss and directed peer cut/restoration have repeated same-host SITL evidence; a replacement operator
has one live takeover trial. Agent-process loss has two SITL runs with operator-assisted rescue and
one run using the SITL flight-controller failsafe; the independent runtime has three completed Gazebo runs.
A full-agent restart policy is now implemented and unit-tested: an armed, airborne vehicle is
commanded to LAND, and peer agents propagate that landing decision. Five live five-SITL trials used
`restart 3`; D3's restarted node entered its restart failsafe, the other four entered their peer
landing failsafe, and all five reported disarmed at ground level. The trials passed with seeds
105–109; the latest evidence is in `runs/independent/20261021T172005.327854Z/`. The earlier
direct UDP regression test confirms a restarted sender's reset sequence is accepted and a delayed
datagram from its retired session is
rejected; transport rejoin and flight-agent recovery are distinct checks. Five seeds remain far too
few to estimate reliability; expand the matrix and report uncertainty before making a reliability
claim.
A conservative peer-timeout LAND policy is implemented, unit-tested, and validated in nine
standalone SITL partition runs (one interactive and eight scripted seeds across random loss,
latency, burst loss, duplication, and reordering). A run-local sensor-and-wind stress profile also completed one
five-drone mission in standalone SITL and two in Gazebo 3D. One Gazebo run now records 3,090
world-statistics samples; airborne motion ran at 0.992× integrated real-time factor even though
the full sample window including EKF startup averaged 0.331×. The recorder and evidence are saved
with that run. Stress settings remain assumptions, not calibrated aircraft models. Next, test the
external-agent path on two computers, validate clock behavior under real asymmetric delay and host
clock steps, expand the airborne agent-restart LAND trial matrix and report uncertainty, establish
per-drone identity, and measure tracking/separation uncertainty across repeated stress runs. See the live-run details in
[`DECENTRALIZED_INDEPENDENT_SITL_2026-10-21.md`](reports/DECENTRALIZED_INDEPENDENT_SITL_2026-10-21.md).

The external-operator path has now also been exercised against five real ArduPilot SITL agent
processes on one host using distinct loopback addresses. Seed 122 met the added movement gate
(1.51 m leader progress toward a 2 m north goal), observed a directed peer cut and restoration,
kept all agents nominal after restoration, and confirmed all five landed/disarmed. This remains
same-host evidence, not the outstanding two-computer test. See
`runs/independent/external-sitl-20261021T173901.719377Z/`.

Since that first external-agent run, the harness has also validated operator-process loss: seed 135
killed the separate operator and confirmed all five agents chose LAND when their accepted mission
lease expired, then landed and disarmed in 42.99 seconds without an operator fallback. Seed 136
verified the swarm-wide direct-peer preflight before launch; all configured one-hop heartbeat links
were live, the 2 m goal and directed cut/restore scenario passed, and all five landed/disarmed.
Transport delay samples are now recorded per agent, and malformed non-finite fault-injection rates
are rejected. Evidence and limits are detailed in
[`DECENTRALIZED_INDEPENDENT_SITL_2026-10-21.md`](reports/DECENTRALIZED_INDEPENDENT_SITL_2026-10-21.md).
Both runs still use same-host loopback addresses. The outstanding test is five independent SITL
agents across at least two computers over the actual LAN, followed by measured asymmetry/clock
stress. No radio or physical flight has been validated.

Seed 137 added the missing-peer negative preflight case: cutting D1→D2 before launch caused launch
to be blocked, kept all five SITL vehicles grounded, and restoring the peer link allowed preflight
and the full mission to proceed. All five later landed/disarmed. Evidence is in
`runs/independent/external-sitl-20261021T180346.638316Z/`. This is stronger same-host software-gate
evidence, but does not close the separate-host LAN, radio, or physical-flight gaps.

After adding peer source-endpoint validation, seed 139 repeated the same five-agent scenario. The
negative preflight gate, restored launch, 1.542 m leader progress toward the 2 m goal, directed
cut/restore, and all-five landing/disarm passed with 2% injected peer loss, 25 ms mean delay, and
5 ms jitter. A unit test rejects a correctly signed packet received from an unrostered source. Raw
evidence is in `runs/independent/external-sitl-20261021T181141.293642Z/`; it remains same-host
loopback evidence, not multi-computer or radio proof.

The staged evidence gates for actual radios and supervised aircraft testing are documented in
[`PHYSICAL_NETWORK_VALIDATION_PLAN.md`](PHYSICAL_NETWORK_VALIDATION_PLAN.md). A later
[five-agent host/VM SITL run](reports/VM_SPLIT_HOST_SITL_2026-10-25.md) crossed a virtual
network boundary on one physical PC; repeated split-host faults and a separate computer remain.

Seed 141 repeated the operator-process-loss case on the source-validated transport. All five agents
reported mission-lease expiry, requested LAND, and confirmed landed/disarmed within 42.93 seconds;
the peer cut/restore check also passed, with 1.564 m leader progress toward the 2 m goal. Evidence
is in `runs/independent/external-sitl-20261021T181541.466947Z/`. This confirms the updated behavior
on one host only.

Seed 143 also repeated the airborne D3 agent-restart trial on the source-validated peer transport.
The restarted agent commanded LAND, the other four agents entered peer-landing failsafe, and all
five disarmed at ground level within 17.83 seconds of restart. Evidence is in
`runs/independent/20261021T181849.530309Z/`. The
remaining evidence gap is separate-host LAN testing; these trials still use one computer.

The operator command channel now retries launch and LAND with the same sequence until every agent
acknowledges receipt, and agents re-acknowledge duplicates without repeating the action. Focused
tests cover retry and idempotency. Seed 152 exercised this change in five independent SITL processes
on one host: all five acknowledged launch and LAND on the first send, the leader moved 1.531 m
toward the 2 m goal, the directed peer-link cut/restore passed, and all five landed/disarmed. The
raw run is `runs/independent/external-sitl-20261021T182636.809231Z/`. This does not test a lost
acknowledgment in flight or close the two-computer LAN gap.

Seed 154 adds one operator-replacement SITL trial: a new console attached to five airborne agents,
restored their existing 2 m north goal, and landed all five after a directed peer cut/restore. The
agents now scope command and mission sequence counters to a random operator epoch and reject delayed
messages from retired epochs. This is still one-host loopback evidence; repeated takeover, conflicting
intent under live conditions, and two-computer LAN validation remain open. See the detailed run in
[`DECENTRALIZED_INDEPENDENT_SITL_2026-10-21.md`](reports/DECENTRALIZED_INDEPENDENT_SITL_2026-10-21.md).

A two-host SITL layout is now explicitly prepared: split agents across A and B so required peer edges
cross the LAN, with SITL/operator on A. The dedicated launcher passed a five-instance startup and
cleanup smoke through a loopback alias. Seed 155 also passed the full five-agent external-operator
scenario using the planned A/B address grouping on one PC; this rehearses the roster but does not
prove LAN routing. No second computer was involved. The prepared commands and roster are not counted
as LAN validation; that evidence still needs to be collected.

Seed 156 added a time-aligned all-agent separation gate to the external-agent harness. In the
split-loopback scenario, 22 airborne samples aligned all five status streams within 250 ms; minimum
horizontal spacing was 4.217 m. Full unit tests passed afterward. The real two-computer LAN test is
still open.

Seed 157 tested the split topology's cross-group negative preflight: cutting D1→D3 before launch
blocked takeoff for all five vehicles; restoring it passed preflight and the full mission completed.
All five later landed/disarmed; 21 aligned samples measured 4.153 m minimum horizontal separation.
This is loopback rehearsal only, not proof of LAN routing.

## Follow-up verification on 2026-10-22

The operator and agents now compare monotonic and wall-clock intervals during clock probes and
discard samples when either wall clock changes by more than 5 ms mid-probe. Strongly asymmetric
one-way-delay arithmetic is covered by a focused test. Five clock tests, the five-endpoint localhost
operator/agent integration test, and all 247 project tests passed after this change. The evidence
still does not include a real two-computer LAN run or real host-clock-service behavior.

The external-agent validator now reports time-aligned mission tracking and pairwise formation
error in the common global NED frame, excluding samples after a LAND command and including a
separate subset where the whole fleet is within 0.5 m of the target altitude. Back-processing seed
157 found 1.210 m fleet horizontal RMSE over 11 mission-command samples for the 2 m north goal,
with 0.046 m formation-offset RMSE. Ten samples were in the altitude band, where horizontal RMSE
was 1.114 m; the scenario did not wait for convergence, so this is diagnostic evidence rather than
a steady-state performance estimate. The full suite now passes 248 tests after adding the metric
regression. The derived values are in
`runs/independent/external-sitl-20261021T185051.169880Z/tracking_metrics_postprocess.json`.

Seed 158 added a fleet-wide convergence gate: all five agents had to remain within 0.5 m
horizontal/vertical target tolerances and preserve the V formation for three time-aligned dwell
samples before the peer-link fault was injected. The gate passed; all five then recovered to nominal
after link restoration and landed/disarmed. Across the approach, formation-offset RMSE was 0.040 m
and fleet position RMSE was 1.048 m; the final dwell confirms convergence after the approach error.
Minimum time-aligned separation was 4.200 m. The full suite passed with local socket access. This is
still one-computer split-loopback SITL; the next gate is running the prepared split topology across
two actual computers on the LAN. Evidence: `runs/independent/external-sitl-20261021T190625.340962Z/`.

Seed 159 extends the five-agent external SITL validator with recorded duplicate and reorder
impairments. With 2% random loss, 25 ms mean delay, 5 ms jitter, 1% duplication, and 5% reorder
probability (150 ms added-delay scale), all agents completed the convergence dwell, directed link
cut/restore, and safe landing; duplicate and stale/replay counters confirm those cases occurred.
The complete test suite passed. This strengthens same-host network-fault evidence, but remains
software-injected loopback and does not replace two-host LAN or radio testing. Raw evidence:
`runs/independent/external-sitl-20261021T191012.649071Z/`.

Seed 160 combines the fault modes: with packet loss, delay/jitter, duplication, and reordering active,
the operator was terminated during flight. All five agents independently detected mission-lease
expiry, requested LAND, and were confirmed landed/disarmed within 42.99 s; peer cut/restore and
mission convergence also passed first. This remains a same-host software fault-injection result,
not a host-crash, LAN, radio, or aircraft test. Evidence:
`runs/independent/external-sitl-20261021T191227.507122Z/`.

Seeds 160 and 165–168 expand that combined operator-loss case to five comparable trials. Every run
confirmed that all five were airborne before the operator was killed, all five independently
requested mission-expiry LAND, and all five landed/disarmed. The maximum observed completion time
was 42.99 s; minimum aligned separation across the batch was 4.147 m. The 5/5 outcome remains an
initial same-host sample (95% Wilson interval 56.6%–100%), not proof of LAN, radio, or physical
reliability. Aggregate: `runs/independent/impaired_operator_loss_batch_summary_20261023.json`.

Seed 161 repeated the seed-159 impaired-network mission and passed: all five converged, recovered
from the directed peer-link cut, and landed/disarmed. The run recorded 36 duplicate and 150 reordered
deliveries, 105 replay/stale-order rejections, and 4.148 m minimum aligned separation. Two seeds now
pass this profile, which improves same-host repeatability evidence but is still too small for a
reliability estimate. Evidence: `runs/independent/external-sitl-20261022T185705.877030Z/`.

Seeds 159 and 161–164 now provide a five-run batch for that exact profile. All five passed mission
convergence, directed link recovery, and landing/disarm; the worst sampled separation was 4.060 m.
The observed 5/5 pass fraction has a wide 95% Wilson interval of 56.6%–100%, so it is reported as
initial same-host repeatability evidence rather than a reliability guarantee. The profile-checked
machine-readable aggregate is `runs/independent/impaired_network_batch_summary_20261023.json`.

Seed 169 adds a configurable correlated-loss trial. A two-state process with 2% burst-entry and
75% burst-continuation probabilities produced an 8.0% empirical aggregate drop fraction; delay,
jitter, duplication, and reordering were also active. All five converged, passed directed-link
recovery, maintained 4.153 m minimum aligned separation, and landed/disarmed. This is a single
same-host seed and needs repetition before it can support a repeatability statement. Evidence:
`runs/independent/external-sitl-20261022T191755.623060Z/`.

Seeds 169–173 now provide five trials of that correlated-loss profile. All five passed convergence,
directed-link recovery, and landing/disarm. The aggregate empirical loss fraction was 8.29%
(1,463 of 17,640 attempted peer datagrams), and the worst sampled separation was 3.982 m. The 5/5
result remains initial same-host repeatability evidence with a wide 56.6%–100% 95% Wilson interval.
Aggregate: `runs/independent/correlated_burst_loss_batch_summary_20261023.json`.
