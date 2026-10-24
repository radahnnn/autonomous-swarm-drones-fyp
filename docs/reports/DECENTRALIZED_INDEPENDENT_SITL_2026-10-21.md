# Independent-agent five-drone SITL run — 2026-10-21

> **How to read this report:** this is a chronological lab record. Its early “current limits” and
> “next validation” sections are snapshots from the first pilot runs, not the present project
> status; later sections add evidence and supersede those snapshots. The latest same-host and
> two-host preparation evidence is near the end. As of the latest recorded run, real two-computer
> LAN routing, radio behavior, and physical flight remain unverified.

## What this run checks

This is the first live run of the separate `--agent-runtime independent` software path. Each of
five agent processes connects to one ArduPilot SITL vehicle over its own MAVLink TCP endpoint,
reads that vehicle's telemetry, computes its own guidance target, and sends its own setpoints.
Agents exchange peer state directly over UDP loopback. The operator console broadcasts the same
mission intent to all agents and displays their status; it does not calculate or send per-drone
movement targets.

This is still a single Ubuntu computer. The console starts SITL and agent processes, provisions the
fixed peer roster and formation slots, and provides the operating-system network. UDP loopback is
not a radio model. **This particular pilot run** makes no claim about Gazebo, multiple computers,
physical flight, hardware communications, collision safety, or reliability across repeated trials.

## Run configuration

- Five ArduPilot standalone SITL quad vehicles, TCP ports 5760, 5770, 5780, 5790, and 5800.
- Decentralized control, independent process per vehicle, V formation.
- 2 m north mission at 5 m target altitude.
- Direct peer UDP with 10% configured application-level sender loss, 50 ms mean latency, seed 44.
- Mission-intent stream paused for about 5 seconds after the move command, then resumed.
- Operator LAND command after the swarm reached the goal.

Command used:

```bash
PYTHONPATH=. python3 -m sitl.swarm_console --drones 5 --standalone \
  --control-mode decentralized --agent-runtime independent \
  --peer-loss 0.1 --peer-latency-ms 50 --peer-seed 44
```

## Observed result

All five agents passed EKF readiness, armed, reached approximately 5 m altitude, responded to the
shared north command while preserving their V offsets, continued toward and held the accepted goal
during the intent-stream pause, and landed after the operator command. The session ended with all
five nodes reporting not airborne. Minimum observed horizontal separation was approximately 4.136
m, calculated from the timestamped status-position samples. The fleet moved approximately 2 m
north overall; individual displacement differs slightly due to tracking and measurement timing.

Final per-agent cumulative UDP counters totaled 4,360 peer datagrams sent, 4,349 received, and 460
intentional sender-side loss drops. The independent runtime uses no `PeerStateBus` relay. No invalid
or replayed packet was reported. These packet totals span each agent's cumulative counter at its
final status sample; they are not a network-throughput benchmark.

Raw events:

```text
runs/independent/20261021T134539.157325Z/events.jsonl
```

## Operator-process-loss validation

A second integration scenario launched the five ArduCopter SITL processes as siblings of the
operator, then killed only the operator after the agents accepted and began a 2 m north mission.
The seed-50 run continued toward the accepted goal, then the agents entered their mission-expiry
LAND behavior. They reached ground with no emergency fallback, 43.03 seconds after operator loss;
minimum sampled horizontal separation was approximately 4.212 m. The older observer did not record
armed state, so this run does not prove disarm and is not counted as a confirmed landing.
Configuration: 10% injected peer loss and 50 ms mean latency. Evidence:
`runs/independent/20261021T140630.833617Z/`.

A corrected repeat (seed 52, same loss/latency profile) requires each node to report `landed`,
`armed=false`, and ground altitude before it declares completion. All five agents met this gate
within 42.98 seconds after operator loss, without emergency fallback; minimum time-aligned horizontal
separation was 4.179 m. Evidence:
`runs/independent/20261021T161122.758370Z/operator_loss_summary.json` and
`operator_loss_status.jsonl`.

After changing accepted mission expiry to use a local monotonic deadline, seed 89 repeated the
operator-process-loss test against the updated agent code. All five agents reached the 2 m north
goal under 10% injected peer loss and 50 ms mean latency. The operator process was then terminated;
each agent expired its last accepted intent, requested LAND, and reported `landed`, `armed=false`,
`airborne=false` at ground altitude. This completed 42.99 seconds after operator loss with no
emergency fallback. Minimum time-aligned horizontal separation across airborne status samples was
4.20 m. All agent and SITL PIDs recorded in the evidence manifest were absent after cleanup. This
is a second strictly confirmed local coordinator-loss trial, not a reliability estimate. Evidence:

```text
runs/independent/20261021T162235.986244Z/operator_loss_summary.json
runs/independent/20261021T162235.986244Z/operator_loss_status.jsonl
runs/independent/20261021T162235.986244Z/console.log
```

## Launch preflight and takeoff startup regression

The peer watchdog was tightened so it also monitors configured peers that had never been heard
before launch, and launch was gated on a valid mission plus fresh configured peer heartbeats. The
first live regression (seed 90) exposed a startup timing issue: the node processes run synchronous
ArduPilot arm/takeoff checks, so one agent entered peer-loss LAND while another was still completing
its takeoff. The swarm then failed to move. This was a failed coordinator-loss trial and is retained
as evidence, not counted as a successful mission:

```text
runs/independent/20261021T163346.001631Z/console.log
runs/independent/20261021T163346.001631Z/events.jsonl
```

The watchdog now grants a five-second startup grace after entering flight-time monitoring, followed
by its existing two-second missing-peer debounce. Ground preflight still requires fresh neighbor
heartbeats before arming. Focused tests cover never-seen peers and the grace timer. A live repeat
(seed 91) completed the normal mission, then lost the operator. All five agents reported landed,
disarmed, and on the ground 43.03 seconds after operator loss without emergency fallback. Minimum
time-aligned horizontal separation was 4.199 m. The recorded agent and SITL processes were absent
after cleanup. Evidence:

```text
runs/independent/20261021T163704.820929Z/operator_loss_summary.json
runs/independent/20261021T163704.820929Z/operator_loss_status.jsonl
runs/independent/20261021T163704.820929Z/console.log
```

This confirms the startup grace fixed this seed's failure and the operator-loss scenario completed;
it does not establish a reliability rate.

Two later repeats (seeds 92 and 93) also completed with 10% injected peer loss and 50 ms mean
latency. All five nodes reported current `landed`, `armed=false`, `airborne=false` status after the
operator was killed, and neither run used the emergency fallback. The harness now retains only
fresh latest landing confirmations and removes a vehicle from the confirmation set if a newer status
contradicts it. The trials completed 43.08 and 43.03 seconds after operator loss. Using nearest
airborne position samples within 0.4 seconds, their minimum sampled horizontal separations were
4.126 and 4.164 m. Aggregate results and exact metric definitions for the five confirmed seeds
(52, 89, 91, 92, 93) are in
[`operator_loss_batch_summary_20261021.json`](../../runs/independent/operator_loss_batch_summary_20261021.json).

One older folder, `runs/independent/20261021T160844.987834Z/`, is explicitly excluded: its legacy
summary says `all_landed`, but the final statuses show all five still armed near ground with
`command=landing:mission_expired`; it did not prove disarm. The separate seed-90 regression remains
preserved as a failed trial.

## Peer transport restart and rejoin

Reviewing the restart path found that each process reset its peer sequence counters to zero, while
receivers retained the previous counters indefinitely. A restarted sender would therefore have its
new packets rejected as replays. Peer datagrams gained a random per-process session ID so a receiver
can accept a new session, reset sequence tracking, retire the old session, and reject delayed
packets from that retired session. The loopback UDP test
`test_direct_udp_transport_accepts_rejoining_agent_and_rejects_retired_session` covers that
transport behavior.

The protocol is now version 3 and also carries an explicit `landing` state. If a full agent process
restarts while its flight controller reports the vehicle armed and airborne, it has lost its local
mission/controller history, so it commands LAND instead of resuming. It broadcasts the landing
state and other airborne agents also land. After touchdown and disarm, the node records
`landed_after_process_restart`; a later relaunch requires an explicit operator command and normal
preflight. Unit tests cover restart detection and peer propagation. The independent console has a
`restart <id>` command for end-to-end fault injection. Five standalone five-vehicle SITL trials used
`restart 3` while the fleet was airborne. D3's restarted process detected the armed vehicle and
entered `agent_restart_failsafe`; its LAND flag propagated across the one-hop peer graph, and the
other four nodes independently entered `peer_landing_failsafe`. All five later reported disarmed at
ground level; D3 reported `landed_after_process_restart`. Five behavioral trials passed, using seeds
105–109; each run directory contains `agent_restart_summary.json` and raw event logs. The aggregate
and metric definitions are in
[`agent_restart_batch_summary_20261021.json`](../../runs/independent/agent_restart_batch_summary_20261021.json).
Across these runs, fleet landing confirmation occurred 16.92–17.98 seconds after the restart event;
the minimum sampled horizontal separation ranged from 3.83 to 3.93 m. Those are five same-host SITL
samples, not a general reliability estimate or a continuous collision bound. The earlier attempts at
`20261021T165422.632176Z/` and `20261021T165557.417281Z/` remain preserved as observer
false-negative/incomplete behavioral trials. A separate seed-108 attempt at
`20261021T171751.927006Z/` failed before agent startup because the sandbox denied localhost sockets;
the rerun with socket access passed at `20261021T171843.179183Z/`.
This is a conservative landing policy, not automatic mission recovery, and a random session ID is
not a cryptographic drone identity. Hardware validation remains future work.

## Operator/agent clock-offset handling

The external-agent path previously compared operator and agent wall-clock timestamps as if the
clocks were identical. It now performs five NTP-style UDP probes per agent before enabling mission
operation, selects the lowest-delay valid estimate with a 250 ms maximum round trip, and reports
half-RTT as the offset uncertainty bound. The operator distributes the host-offset map and waits for
all agents to acknowledge it. Agents use the map when checking mission and command expiry and when
validating direct peer packet age; the console normalizes agent status timestamps into the operator
clock domain. It periodically rebroadcasts the map so a restarted agent can load the current epoch.

The five-fake-agent integration test deliberately offset agent clocks by −6, −3, 0, +3, and +6
seconds. The console recovered all five offsets within 10 ms and every agent acknowledged the map.
This verifies the code path and the offset arithmetic on localhost, not real inter-host network
asymmetry. While active, the console now remeasures and distributes offsets every 60 seconds. The
integration test also simulates a 0.4-second post-start clock change and verifies a second sync
replaces the offsets. Failed refreshes are logged and retried after another minute while the last
acknowledged map remains active. Hosts should still keep NTP or another clock service active; a
second-computer SITL test with asymmetric delay and actual host clock changes remains open.

The estimator now rejects a sample if operator wall time disagrees with monotonic elapsed time by
more than 5 ms, or if the agent's wall-clock receive/send interval disagrees with its monotonic
processing duration by more than 5 ms. Focused tests cover strongly asymmetric path-delay arithmetic
and simulated clock steps at each endpoint. These checks prevent a detected in-probe clock step
from entering the offset map; they do not measure real LAN asymmetry or replace host time
synchronization. Five focused clock tests and the five-endpoint localhost command/status integration
test passed on 2026-10-21. A subsequent full-suite run on 2026-10-22 passed all 248 collected
tests. This is still not a second-computer validation.

The external-agent SITL validator now computes sampled tracking error against each agent's shared
mission goal plus its deterministic slot in the common global NED frame. It reports horizontal,
vertical, 3D, and pairwise formation-offset error, and separates samples where every vehicle is
within 0.5 m of its altitude target. A retrospective analysis of seed 157 is saved at
`runs/independent/external-sitl-20261021T185051.169880Z/tracking_metrics_postprocess.json`. For
the 2 m north goal, 11 mission-command samples gave 1.210 m fleet horizontal RMSE (1.936 m 95th
percentile) and 0.046 m pairwise formation-offset RMSE. Commanded landing samples are excluded.
Ten samples were within the 0.5 m altitude band; their horizontal RMSE was 1.114 m. This run's
validation gate required leader progress of at least 1.5 m and then exercised link faults and
landing; it did not wait for mission convergence. These metrics expose a short tracking sample,
not a steady-state or repeat-run performance claim.

The first observer attempt (`20261021T135300.978136Z`) ended the SITL processes with the operator.
Two later attempts (`20261021T160340.089697Z` and `20261021T160615.436307Z`) did not reach the
operator-kill point because the harness rejected the actual leader position of 2.26 m for its 2 m
goal. Their normal shutdowns landed the vehicles, but they are not coordinator-loss trials. The
harness now uses a target tolerance and strict disarmed landing confirmation. Earlier attempts
remain preserved with their raw logs.

## Failed first attempt and correction

The first attempt is retained at:

```text
runs/independent/20261021T134330.833134Z/events.jsonl
```

It caught a watchdog interaction. Arming and takeoff helpers synchronously consumed MAVLink
messages while the agent loop was busy; immediately after the helpers returned, the normal stale
heartbeat check interpreted that expected pause as a telemetry failure and commanded LAND. The
node now refreshes the watchdog anchor after its checked arm/takeoff sequence. The repeat run
remained airborne, moved, and landed on command. This first failure remains part of the evidence.

## Initial pilot limits and next validation (historical snapshot)

The following status was accurate immediately after the first pilot and the seed-52 operator-loss
repeat. It was superseded by the later dated sections below; it is retained to show how the test
plan and evidence developed.

- At that point, the five nodes and operator ran on one host. Operator loss had one confirmed
  trial; agent-process loss had two operator-assisted rescue trials; no multi-host partition had
  been tested.
- The fixed peer roster and V slots are provisioned at startup. Agents do not discover neighbors,
  negotiate slots, or elect a leader.
- Peer messages and mission commands are unauthenticated, unencrypted localhost UDP. The
  impairment layer injects loss and delay at the application; it does not model radio contention,
  interference, bandwidth, antenna behavior, or packet corruption.
- The early pilot comprised a small number of seeded SITL runs, not enough to establish confidence
  intervals or broad robustness.
- The initial pilot did not include Gazebo or physical-aircraft evidence. A later Gazebo section
  records subsequent simulation evidence; no physical-aircraft result is claimed anywhere in this
  report.

At that point the next steps were to test flight-controller failsafe behavior without operator
rescue, expand partition tests, collect per-agent decision/setpoint traces, and validate in Gazebo.
Later sections document progress on the failsafe, partition, and Gazebo items. Multi-host LAN
validation, paired comparison against the centralized baseline, richer decision/setpoint analysis,
and physical validation remain open.

## Directed peer-link fault

The independent UDP transport now accepts a directed edge cut using `--peer-block 1:2` (IDs are
1-indexed at the console). Sender 1 drops only its datagrams to receiver 2; reverse traffic and
other configured peer links continue. The run used 10% random loss, 50 ms mean latency, seed 52,
then flew the same 2 m north mission and landed on operator command. The blocked sender recorded
406 directed drops, while its other peer traffic continued. All five vehicles reported disarmed,
grounded (`|down| < 0.02 m`), and `landed`; minimum observed horizontal separation was 4.171 m.
Raw events: `runs/independent/20261021T141447.196880Z/events.jsonl`.

An earlier directed-link run (`20261021T141216.957643Z`) is retained but not counted as a pass: its
shutdown code treated low altitude as landed even though the vehicles were still armed at about
0.35 m. Shutdown now requires a fresh status reporting disarmed, `landed`, and ground position. If
that confirmation does not arrive, it leaves the SITL/agent processes running instead of killing
them. The corrected repeat above passed that stronger check.

## Independent-agent process loss

Two additional live five-vehicle standalone SITL runs exercised the console's `fail` and emergency
`rescue` behavior while the swarm was airborne. In each run, drone 3's agent process was killed;
its simulated flight controller stayed active on its own TCP MAVLink endpoint. The other four
agents continued running. An emergency connection opened by the operator console selected LAND
for drone 3 and verified both disarm and ground-level altitude. In the second run, the operator
then requested fleet landing without manually issuing `rescue`; shutdown landed the active agents
first, then rescued the failed agent before removing SITL. All five aircraft were confirmed landed
and the simulator processes were cleaned up.

First run settings: 5% peer packet loss, 50 ms mean peer delay, seed 61, plus a two-way peer cut
between drones 1 and 2. Minimum time-aligned horizontal separation in the airborne status samples
was approximately 4.160 m. Evidence:

```text
runs/independent/20261021T141935.340047Z/events.jsonl
```

Second run settings: 2% peer packet loss, 25 ms mean peer delay, seed 62. Minimum time-aligned
horizontal separation was approximately 4.155 m. Evidence:

```text
runs/independent/20261021T142220.566537Z/events.jsonl
```

The first run caught a shutdown delay: an agent with a killed process cannot publish fresh status,
so requiring every agent's latest status to stay fresh made shutdown wait its full 45-second window.
Shutdown now waits for all still-running agents, then attempts the emergency landing for each dead
agent before deciding whether it is safe to terminate SITL. The second run exercised and passed
that updated path.

This is controlled SITL fault injection, not a complete decentralized fault-tolerance result. The
operator process provided the rescue MAVLink connection and explicitly chose LAND in both trials.
Autonomous onboard failsafe behavior is documented separately below; flight-controller link or
power loss remains untested.

## ArduPilot GCS failsafe after agent loss

Added `--sitl-gcs-failsafe`, an opt-in independent-runtime option that writes a run-local parameter
profile under the evidence directory. It enables `FS_GCS_ENABLE=5` (LAND), sets a five-second
`FS_GCS_TIMEOUT`, and leaves `FS_OPTIONS=0`; the normal shared SITL and hardware files are not
modified. Each independent agent sends a MAVLink GCS heartbeat, including during EKF startup,
arming, and takeoff checks. The exact failsafe behavior follows ArduPilot's [GCS failsafe
documentation](https://ardupilot.org/copter/docs/gcs-failsafe.html).

In the five-drone run at `runs/independent/20261021T142748.189010Z/`, drone 3's process was killed
while airborne. No rescue command was sent to drone 3 before observing it. An independent MAVLink
observer saw its flight controller in custom mode 9 (LAND) after the agent disappeared; a later
observation confirmed disarmed at 0.008 m relative altitude. The other four agents were then landed
through the operator console. The run-local parameter file and summary are
`independent_failsafe.parm` and `failsafe_observation.json` in that directory.

This is one local SITL run, not proof of radio-range behavior or physical failsafe correctness. It
tests loss of the agent's MAVLink heartbeat while the simulated flight controller and its TCP
endpoint remain alive. Sensor faults, FC power loss, real telemetry radios, and real airframe
response are outside this result.

## Dynamic peer partition and restoration

The independent console now supports `link-block <sender> <recipient>` and
`link-restore <sender> <recipient>`. Each node applies a directed edge change locally; blocking
D1-to-D2 leaves D2-to-D1 intact. These commands are fault-injection controls for SITL, not mission
control. Repeated link changes are sequence checked, and the sender reports the number of changes
and blocked packets in its status metrics.

In run `runs/independent/20261021T143211.872887Z/`, five vehicles took off, accepted a 2 m north
goal, then both D1-to-D2 and D2-to-D1 were cut. Each direction counted 72 dropped datagrams by the
end of the run, with two state changes (blocked then restored). After both edges were restored,
the peers resumed sending and receiving. All five vehicles completed landing and were confirmed
disarmed; minimum time-aligned horizontal separation in the airborne status samples was 4.198 m.
Configuration: 2% random packet loss, 25 ms mean peer delay, seed 64. Full event trace:

```text
runs/independent/20261021T143211.872887Z/events.jsonl
```

This demonstrates link fault injection and transport recovery on loopback. It does not yet test a
distributed partition-resolution policy: agents do not elect a new leader or negotiate new
formation slots, and the operator/test harness explicitly restores the links. A later live run
tested a local fail-safe for sustained peer loss, documented next.

## Autonomous response to a sustained peer partition

Each independent agent now sends a peer-liveness heartbeat every 0.5 seconds, including during
blocking EKF, arming and takeoff checks. Position samples remain separate and expire after 0.5
seconds; a heartbeat cannot make an old position fresh. In flight, the watchdog treats a peer as
stale after 1.5 seconds without a heartbeat and commands LAND if the peer is still missing for a
further 2-second grace period. Preflight absence is recorded without aging the failsafe timer, and
a landed vehicle does not automatically take off when links return. This is a local conservative
response, not swarm-wide consensus or automatic topology repair.

The live five-drone run at `runs/independent/20261021T144908.936448Z/` started in the standalone
ArduPilot SITL, with 2% injected peer loss, 25 ms mean latency, seed 66. After all vehicles reached
about 5 m, the operator cut both peer directions between D2 and D4. Their own status records show
`safety_state=peer_link_failsafe`; D2 records missing D4 and D4 records missing D2. Both vehicles
landed and disarmed without an operator LAND command to either one, while D1, D3 and D5 remained
airborne. The operator then commanded the remaining three to land; the session ended with all five
confirmed grounded and disarmed (`session_end.landed=true`).

Raw trace:

```text
runs/independent/20261021T144908.936448Z/events.jsonl
```

The first implementation used fresh position packets as proof of peer health. A failed automated
trial exposed a false timeout during a neighbor's blocking takeoff sequence. That trial is retained
at `runs/independent/20261021T145411.084432Z/`; it was safely landed, and it is not counted as a
successful policy trial. The runtime was corrected to send distinct liveness heartbeats while
continuing to reject stale positions.

The correction passed two scripted five-drone standalone SITL trials with seeds 67 and 68, 2%
packet loss, 25 ms mean peer latency, and a two-way D2↔D4 cut. In both runs, D2 and D4 independently
entered `peer_link_failsafe`, landed, and disarmed. The other three remained airborne until the
operator then commanded fleet LAND, and all five were confirmed landed/disarmed. First failsafe
status appeared 3.71 s and 3.74 s after the link-cut events; endpoint touchdown took about 16.96 s
in each trial. The initial interactive seed-66 run also passed. Scripted evidence and summaries:

```text
runs/independent/20261021T145947.252933Z/partition_validation.json
runs/independent/20261021T150113.677325Z/partition_validation.json
runs/independent/peer_partition_trials.json
```

This initial three-run check was later extended with four additional scripted seeds, for seven
successful trials total including the interactive run; this is still not a reliability estimate.
The other agents continued their
mission while the two edge endpoints landed. It does not provide distributed agreement, automatic
rejoining, partition healing, radio propagation, or repeated multi-condition statistics. The
timeout and behavior still need larger seeded trials and review against a clearly specified swarm
safety policy.

## Five-drone Gazebo 3D validation

After granting this Codex run access to the host's GPU/display devices, the independent-agent
runtime also completed one live run in the project's Gazebo Harmonic five-Cinewhoop world. The
initial sandboxed attempt is retained at `runs/independent/20261021T143620.815628Z/`; Gazebo's
OpenGL context could not initialize in that restricted run, so no agent became ready and no flight
was attempted. The successful unsandboxed retry reached EKF-ready for all five vehicles after 206
seconds. All five armed and took off to approximately 5 m, accepted a 2 m north mission, moved as a
swarm, and were confirmed landed and disarmed before Gazebo was closed. Configuration: 2% injected
peer loss, 25 ms mean peer latency, seed 65, and the run-local GCS failsafe profile enabled. The
minimum time-aligned horizontal separation was approximately 4.173 m. Evidence:

```text
runs/independent/20261021T143853.501763Z/events.jsonl
runs/independent/20261021T143853.501763Z/independent_failsafe.parm
```

One telemetry-timeout LAND command appeared while a vehicle was still on the ground and disarmed,
before the operator launch; the vehicle subsequently armed and flew the mission. No in-flight
telemetry-timeout LAND was recorded. This is one 3D run, not broad Gazebo reliability evidence;
sensor fidelity, real-time factor, and repeat-seed uncertainty still need reporting.

## Follow-up: loss-profile matrix and sensor stress runs

The directed D2↔D4 partition validation was extended to eight scripted seeds. Seeds 67–68 used 2%
random loss with 25 ms mean latency, seeds 69–70 used 5% loss with 50 ms mean latency, seed 71
added burst loss to the 2% / 25 ms profile, and seed 75 added duplication and reordering to that
profile. All eight scripted trials had the two partition endpoints
autonomously enter the local peer-link failsafe and land; the other three stayed airborne until
operator LAND. Detection took 3.71–3.75 seconds and touchdown about 16.96 seconds. Seed 76 also
used distinct loopback bind and peer addresses (127.0.0.2 through 127.0.0.6) and HMAC authentication;
all five agents completed the partition trial with zero authentication rejections. Seed 77 reran the
default local transport after the operator-channel changes; both partition endpoints again landed,
touchdown was 16.97 seconds, and all five disarmed. With the earlier interactive run, this is nine
successful trials, not a statistically useful reliability
estimate. In seed 75 the five agents emitted 172 duplicated datagrams and 791 delayed datagrams;
they rejected 468 replayed/out-of-order datagrams. Both partition endpoints still landed locally
and the other three remained airborne until operator LAND. The trial matrix is preserved at
`runs/independent/peer_partition_trials.json`.

The independent runtime also gained `--sitl-sensor-profile stress`. This builds a run-local
ArduPilot parameter file and leaves the shared defaults unchanged. The declared settings are GPS
horizontal wander 0.50 m, vertical GPS sine-noise amplitude 0.30 m, GPS lag 250 ms, GPS accuracy
setting 0.60, accelerometer noise setting 0.60, and wind 2.0 m/s from 180 degrees with 0.50
turbulence. These values are stress assumptions, not calibrated measurements of the Matek drone.
ArduPilot's [SITL parameter reference](https://ardupilot.org/dev/docs/sitl-parameters.html)
documents simulator controls; `SIM_GPS1_NOISE` here is a vertical sine-wave error, not horizontal
white noise.

One five-drone standalone SITL run and one five-drone Gazebo Harmonic run completed with this
profile, each also using 2% peer loss and 25 ms mean latency. In both, all five agents became
EKF-ready, armed, held roughly 5 m altitude, followed a 2 m northward shared intent, and landed
and disarmed. Minimum time-aligned horizontal separation was 3.97 m in standalone SITL and 3.99 m
in Gazebo. The Gazebo EKF gate took about 211 wall-clock seconds. All recorded airborne statuses
reported nominal safety. Raw evidence and exact generated profiles:

```text
runs/independent/20261021T151239.716950Z/events.jsonl
runs/independent/20261021T151239.716950Z/independent_profile.parm
runs/independent/20261021T151552.260273Z/events.jsonl
runs/independent/20261021T151552.260273Z/independent_profile.parm
```

These are one run per simulator path. A follow-up five-drone Gazebo run (seed 74, same peer and
sensor stress settings) captured the world-statistics stream. The agents became EKF-ready after
211 seconds wall time, flew the 2 m mission at about 5 m altitude, and landed/disarmed. Minimum
time-aligned horizontal separation was 3.946 m. Across 3,090 samples, the integrated simulation
rate was 0.331× wall time over the sampled world window, which includes slow EKF startup. During
the airborne interval, Gazebo advanced 49.81 simulation seconds in 50.19 wall seconds: 0.992×
integrated RTF, with 1.000× median instantaneous RTF. Thus flight was close to real-time while
estimator startup was substantially slower on this PC. Evidence:

```text
runs/independent/20261021T152633.911550Z/events.jsonl
runs/independent/20261021T152633.911550Z/independent_profile.parm
runs/independent/20261021T152633.911550Z/gazebo_world_stats.jsonl
runs/independent/20261021T152633.911550Z/gazebo_rtf_summary.json
```

`experiments/record_gazebo_rtf.py` now captures raw Gazebo statistics with a wall-clock duration
limit and can align an airborne-only window to the run events. Its five-second live smoke check
collected 47 samples and finalized normally. The measured RTF remains specific to this PC and
these runs. Repeat-seed performance, sensor calibration against real measurements, radio
behavior, and physical-aircraft safety remain open.

## Bounded impairment queue regression

The direct UDP delay/duplicate scheduler now retains at most 64 pending datagrams for any one peer
and 512 total per agent process. When either limit is reached, it discards the oldest enqueued
datagram and records the event in `dropped_queue`, keeping newer state available instead of
allowing an unbounded delayed backlog. The focused regression test filled a per-peer queue and the
global queue and verified the caps, eviction order, and counter. The complete Python test suite
passed with 227 tests collected. This verifies the software queue bound only; no long-delay
five-drone flight was used to claim radio behavior or flight safety under queue saturation.

## External operator with five real SITL agents

The new `experiments/validate_external_agent_sitl.py` check starts the ArduPilot SITL vehicles,
five independent agents, and external operator separately. It uses five distinct loopback
addresses, HMAC-authenticated UDP, 2% injected peer loss, and 25 ms mean application delay. The
initial seed-121 smoke run started and landed all vehicles but moved the leader only 0.49 m toward
the 2 m goal; it exposed that the first acceptance gate checked intent acceptance without requiring
mission progress. That run is retained as exploratory evidence and is not counted as a movement
pass.

The tightened seed-122 run required the leader to travel at least 1.5 m toward the goal before
testing the link fault. It achieved 1.51 m progress, observed the D1-to-D2 directed cut at the
sender (12 blocked datagrams) and subsequent restoration, saw all five agents remain airborne
with nominal safety after restoration, then confirmed all five were landed and disarmed. No HMAC,
invalid-packet, or replay rejection occurred. Summary and process logs are in
`runs/independent/external-sitl-20261021T173901.719377Z/`. Because all five addresses were on
127.0.0.x, this verifies process separation and the external-operator code path on one computer;
LAN routing, real clock asymmetry, and physical radios remain unverified.

### Explicit peer-delay jitter run

After exposing `--peer-jitter-ms` through the independent agent, console, and external-agent
validation harness, seed 133 was run with 2% application-level loss, 25 ms mean injected delay,
and 8 ms standard deviation. The five real SITL agents accepted the shared 2 m north goal; the
leader progressed 1.575 m; the D1-to-D2 directed link cut was recorded (10 blocked datagrams) and
restored; all agents remained nominal after restoration; and all five were confirmed landed and
disarmed. The run recorded no authentication, invalid-packet, or replay rejections. It also recorded
expected random loss and stale-state rejection under the injected delay. Full summary and raw logs:
`runs/independent/external-sitl-20261021T174508.987605Z/`.

This confirms that an explicit jitter setting reaches all five agent transports and that this
particular same-host SITL scenario completed. It still uses loopback networking and synthetic
impairments, so it is not evidence for a real radio, separate computers, or physical flight.

The transport now reports the count, mean, standard deviation, minimum, and maximum of its
scheduled one-hop delay samples. This exposes the impairment actually drawn by each agent instead
of reporting only requested configuration. A subsequent seed-134 run used the same 2% loss, 25 ms
mean, and 8 ms standard deviation. It recorded 395–795 delay samples per agent; observed per-agent
means ranged 24.32–25.15 ms and standard deviations 7.55–8.21 ms. The leader progressed 1.542 m,
the directed cut and restoration passed, all agents remained nominal after restoration, and all
five landed/disarmed. No authentication, malformed-packet, or replay rejects occurred. The run
ended with zero agent processes and normal SITL cleanup. Evidence:
`runs/independent/external-sitl-20261021T174902.634712Z/`.

### External operator process loss with independent agents

The external-agent harness gained an operator-loss mode. It runs the five agents as separate
processes with their own ArduPilot SITL MAVLink connections, runs the operator separately, accepts
a shared mission, and verifies leader progress and a directed peer-link cut/restore. It then kills
only the operator process and receives authenticated status directly from the surviving agents.
In seed 135 (2% peer loss, 25 ms mean delay, 8 ms jitter), all five agents reported
`landing:mission_expired`; all five subsequently reported landed, disarmed, and at ground altitude
within 42.99 seconds. No operator LAND fallback was sent. The leader moved 1.509 m toward the 2 m
goal before operator loss. The peer-link cut and restoration passed, and no authentication,
invalid-packet, or replay rejections were recorded. The raw status-after-loss stream is
`runs/independent/external-sitl-20261021T175401.693794Z/operator_loss_status.jsonl`; the summary,
agent logs, roster, and console transcript are in the same folder.

The preceding attempt, `runs/independent/external-sitl-20261021T175311.824957Z/`, failed before
agent startup because a randomly generated binary HMAC key ended in a whitespace byte and the
loader stripped it, reducing its length below 32 bytes. The loader now preserves raw key bytes
exactly, and a regression uses both leading and trailing whitespace bytes. Seed 135 is the passing
repeat after that fix.

This is stronger same-host coordinator-loss evidence for the external-agent architecture than
stopping the operator thread while retaining its host process. It is still loopback SITL: it does
not prove separate-host network loss, a physical radio, or aircraft failsafe configuration.

### Swarm-wide peer-connectivity preflight

Agent status now reports `peer_live_ids` separately from configured `peer_ids`, plus whether the
accepted mission lease is active. The external operator's `preflight` command checks fresh agent
status, grounded/disarmed state, live telemetry, active mission, the current clock-sync epoch,
nominal safety state, and all configured direct-peer heartbeats. `launch` repeats that check and
sends no launch command if any agent is not ready, preventing a partial swarm takeoff after a
missing LAN route or peer process.

Seed 136 exercised the new gate against five independent ArduPilot SITL agents with 2% injected
peer loss, 25 ms mean delay, and 8 ms jitter. Every configured one-hop neighbor was live at
preflight; the operator recorded a pass, the leader progressed 1.52 m toward the 2 m goal, the
directed cut/restore passed, and all vehicles landed/disarmed. The complete operator event log is
under `runs/independent/external-sitl-20261021T180020.779637Z/operator-evidence/`; the run summary
and per-agent logs are in the parent directory. A unit regression also verifies that a missing
required peer blocks the launch command. This remains same-host loopback SITL evidence; the actual
two-computer route remains to be tested.

Seed 137 exercised the negative path in the live external-agent harness. It cut the directed D1→D2
peer link before launch; D2 reported the required D1 heartbeat missing, launch preflight failed, and
all five vehicles remained grounded and disarmed. After the edge was restored, preflight passed and
the test continued through the 2 m mission, another directed cut/restore, and landing/disarming of
all five vehicles. The summary records the negative-gate checks as passed; operator and per-agent
logs are preserved at `runs/independent/external-sitl-20261021T180346.638316Z/`. This remains a
single same-host, loopback-address SITL validation with injected packet loss/delay. It validates the
software launch gate under a missing peer heartbeat, not a real LAN route, radio, or aircraft.

### Source-endpoint validation follow-up

Peer datagrams now retain and validate the UDP source address against the advertised endpoint for
the claimed sender. An authenticated packet from another source endpoint is rejected and counted as
`rejected_source`. A unit regression verifies this with a correctly signed packet sent from an
unrostered local socket. The five-agent live scenario was repeated in seed 139 with 2% injected
loss, 25 ms mean delay, and 5 ms jitter. The pre-launch D1→D2 cut blocked launch with all five
grounded; restoring the link passed preflight. The leader advanced 1.542 m toward the 2 m goal,
the later directed cut/restore completed, and all five vehicles landed and disarmed. The run recorded
zero unexpected-source rejections during nominal traffic. Evidence:
`runs/independent/external-sitl-20261021T181141.293642Z/`. This is still one-host loopback SITL
evidence; it does not validate actual LAN source routing, NAT behavior, radios, or aircraft.

Seed 141 repeated the external-agent coordinator-loss trial with source-endpoint validation active.
Five agents accepted the shared 2 m goal, the leader progressed 1.564 m, and the injected directed
peer cut/restore completed with all agents nominal afterward. The operator process was killed after
the mission and fault check. All five agents independently reported `landing:mission_expired`, then
were confirmed landed/disarmed within 42.93 seconds; no operator fallback was sent. The run used
2% peer loss, 25 ms mean delay, and 5 ms jitter, with zero unexpected-source rejections. Evidence is
in `runs/independent/external-sitl-20261021T181541.466947Z/`. This remains one-host loopback SITL
evidence.

### Agent-restart propagation after source validation

Seed 143 repeated the airborne process-restart scenario after peer source-endpoint checks were
added. The restarted D3 agent found its vehicle armed and airborne, entered
`landing_after_process_restart`, and sent a peer landing request. The other four agents entered
`peer_landing_failsafe`; all five vehicles were confirmed landed and disarmed. The run recorded zero
unexpected-source rejections. The event log's first landed/disarmed status for all five arrived
within 17.83 seconds of the restart event. Evidence is in
`runs/independent/20261021T181849.530309Z/`. This is one-host standalone ArduCopter SITL evidence;
it does not validate a separate LAN, radios, or physical flight.

### Operator command delivery acknowledgment

The operator now retries launch and LAND with the same sequence number until every agent
acknowledges receipt or the four-second retry window ends. An agent re-acknowledges a duplicate
sequence without repeating the action. Receipt acknowledgment is distinct from proof of completed
takeoff or landing; the operator still checks fresh agent status for outcomes. Unit tests cover
same-sequence retry and duplicate suppression.

Seed 152 completed the current external-operator path against five separate ArduCopter SITL and
agent processes on one host, with distinct loopback addresses, 2% injected peer loss, 25 ms mean
peer delay, and 5 ms jitter. All five acknowledged launch and LAND on the first transmission. The
leader progressed 1.531 m toward the 2 m north goal. A directed peer link was cut and restored, all
agents returned to nominal status, and all five were confirmed landed/disarmed. The event log
reported five acknowledgments for each command and zero missing acknowledgments; the session ended
with `landed: true`, zero agent processes, and normal SITL cleanup. This live run validates command
receipt and status confirmation under the configured peer impairment, but did not cause an
acknowledgment loss; retry behavior itself is covered by a focused unit test. Evidence:
`runs/independent/external-sitl-20261021T182636.809231Z/`, including the operator event stream at
`operator-evidence/20261021T182639.907750Z/events.jsonl`.

An earlier attempt, `runs/independent/external-sitl-20261021T182606.948440Z/`, stopped before agent
startup because the sandbox denied UDP sockets to child processes. No flight was attempted and no
SITL or agent process remained. The passing seed 152 run was repeated after granting the required
network access.

### Replacement operator takeover while the agents remain airborne

The operator session now has its own random epoch. Agents accept mission, command, and network-fault
messages only from their current epoch, reset those sequence counters after a new clock-sync epoch,
and reject delayed packets from retired epochs. A replacement external console waits for fresh agent
status and recovers the common active mission target before it begins publishing; it refuses takeover
if the agents disagree about the active target, formation, or altitude. This avoids accepting reset
sequence numbers as stale and avoids replacing an in-progress goal with the new console's default.

Seed 154 exercised this path against five standalone ArduCopter SITL instances and five separately
running agent processes on one host, using distinct loopback addresses, 2% injected peer loss, 25 ms
mean peer delay, and 5 ms jitter. All five launched, the leader advanced 1.509 m toward the 2 m north
goal, and a directed peer cut/restore completed. The operator process was then terminated while the
agents remained airborne. A replacement console attached, recorded that it resumed the same
`[2.0, 0.0]` north/east goal, and commanded LAND. All five vehicles landed and disarmed; the run
closed with `landed: true` and no agent or SITL processes left behind. The operator replacement and
completion evidence is in
`runs/independent/external-sitl-20261021T183534.476514Z/`, with the replacement session events at
`operator-evidence/20261021T183640.846969Z/events.jsonl`.

This is one same-host loopback takeover trial, not evidence of robust failover, separate-host routing,
radio behavior, or physical flight. Conflicting-mission refusal is unit-tested but has not been
injected into a live SITL swarm.

### Prepared two-computer SITL launcher

`experiments/launch_multihost_sitl.py` starts only the requested number of standalone ArduCopter
instances, prints their remote TCP MAVLink endpoints, records a PID manifest and per-instance logs,
and cleans up only its own child processes. It is intended for a split two-computer topology: five
SITL instances and agents D1/D2 plus the operator on Computer A, agents D3/D4/D5 on Computer B. This
places the direct V-topology links D1↔D3 and D2↔D4 across the LAN while the remote agents connect to
SITL over TCP.

A five-instance five-second smoke run connected to the advertised loopback alias `127.0.0.2` at
ports 5760, 5770, 5780, 5790, and 5800, then exited with all SITL children cleaned up. Evidence is in
`runs/independent/multihost-sitl-20261021T184308.547509Z/`. An earlier sandboxed attempt at
`runs/independent/multihost-sitl-20261021T184142.519044Z/` failed before sockets opened because the
ArduCopter child was denied socket creation; no simulation or flight occurred in that attempt. The
successful alias smoke is local loopback evidence, not LAN proof. The two-host roster and exact
commands are in `docs/PEER_NETWORK_SETUP.md`.

The five-agent validator now accepts `--topology split-loopback` to use the exact two-host split
from the roster: D1/D2 at `127.0.0.1`, D3/D4/D5 at `127.0.0.2`, with expected cross-group peer edges
D1↔D3 and D2↔D4. Seed 155 passed: all five launched, D1 advanced 1.620 m toward the 2 m goal, the
directed link cut/restore completed, all configured neighbors were live in final status, and all
five landed/disarmed. Summary and raw logs are in
`runs/independent/external-sitl-20261021T184527.241236Z/`; it records zero surviving agent or SITL
processes. This checks the grouped roster and process configuration with real ArduPilot SITL, but
the two addresses still route locally on one PC. It is not separate-computer LAN evidence.

Seed 156 repeated the split-loopback run after adding a time-aligned separation gate to the
validator. The harness aligned all five airborne agent status streams within 250 ms for 22 samples
and measured a minimum horizontal separation of 4.217 m (maximum selected timestamp offset 46.7 ms).
The 2 m mission, directed cut/restore, all-five landing/disarm, and process cleanup passed. Summary:
`runs/independent/external-sitl-20261021T184822.843234Z/external_agent_sitl_summary.json`. This
remains local loopback evidence; the metric does not establish radio separation or flight safety.

Seed 157 exercised the missing-cross-group-link gate before takeoff. With D1→D3 cut, the recipient
reported the missing required peer and launch preflight failed while all five vehicles remained
grounded/disarmed. After restoring that edge, preflight passed and the five-drone mission proceeded;
the later directed cut/restore also passed. All five landed/disarmed. The validator aligned 21
airborne samples within 250 ms and measured 4.153 m minimum horizontal separation (maximum alignment
offset 5.0 ms). Evidence is in
`runs/independent/external-sitl-20261021T185051.169880Z/`. This is still one-computer loopback, not
LAN routing evidence.

## Follow-up software verification — 2026-10-22

The clock-probe path now uses monotonic elapsed time at the operator and monotonic processing time
at each agent to reject samples when either wall clock jumps by more than 5 ms during a probe. The
four-timestamp estimator also has regression coverage for strongly asymmetric one-way delays and
the half-round-trip offset-error bound. Five focused clock tests, the five-endpoint localhost
operator/agent integration test, and the complete project suite (247 tests) passed after these
changes. This verifies code and loopback behavior; the actual two-computer LAN test, real host-clock
service behavior, radios, and physical flight remain open.

### Five-agent convergence-gated mission — seed 158

The validator now requires the complete five-agent fleet to be airborne, not landing, time-aligned
within 250 ms, and inside 0.5 m horizontal and vertical tolerances of the shared 2 m north goal,
5 m altitude, and deterministic V-formation slots for three contiguous samples. Samples must be
at least 0.5 s apart and no more than 1.75 s apart. Seed 158 passed this gate after 13 aligned
samples; the three-sample dwell completed with each agent 0.245–0.283 m horizontally and at most
0.001 m vertically from its target. It then passed the directed peer-link cut and restore, all
agents returned to nominal status, and all five landed/disarmed.

Across all 14 mission-command samples, fleet horizontal RMSE was 1.048 m (p95 1.891 m), while
pairwise formation-offset RMSE was 0.040 m. This shows that the drones held their relative V shape
closely while the fleet took time to reach the commanded point; the convergence dwell confirms
that it ultimately reached the configured tolerance. Time-aligned minimum airborne separation was
4.200 m over 24 samples. These are deterministic local ArduCopter SITL results with simulated
2% peer loss, 25 ms mean peer delay, and 5 ms jitter, using five separate agent processes on one
computer and split loopback addresses. They are not evidence of actual LAN routing, radio behavior,
or flight safety. Run artifacts are in
`runs/independent/external-sitl-20261021T190625.340962Z/`.

The full project test suite passed after the convergence gate and its regression test were added.
Running it inside the restricted command sandbox first caused socket-dependent tests to fail at
socket creation (`PermissionError: Operation not permitted`); rerunning with local socket access
allowed passed the suite. This is an execution-environment restriction, not a failure of the tested
UDP code. No second computer was available, so cross-host LAN validation remains the next evidence
gate.

### Five-agent mission with duplicate and reordered packets — seed 159

The external five-agent SITL validator now exposes and records peer burst loss, packet duplication,
packet reordering, and reordering delay in addition to random loss, mean delay, and jitter. Seed 159
used 2% independent packet loss, 25 ms mean delay, 5 ms jitter, 1% duplication, and 5% reordering
with a 150 ms added-delay scale. The run produced duplicate packets and deliberately delayed packets
on every agent (1–6 duplicates and 21–48 reordered delays per agent); stale/replay checks rejected
older/duplicate deliveries. All five agents reached the convergence gate (14 aligned samples, three
within-tolerance dwell samples), passed the directed peer cut/restore, returned to nominal status,
and landed/disarmed. Minimum time-aligned separation was 4.060 m over 25 samples.

During the 15 aligned mission-command samples, fleet horizontal RMSE was 1.107 m and formation-offset
RMSE was 0.088 m. These approach-window errors are not steady-state estimates; the separate
convergence dwell confirms the target was reached. This remains same-host split-loopback SITL with
network impairments injected in software, not a real LAN, radio, or flight test. The exact config,
per-agent transport counters, logs, and events are in
`runs/independent/external-sitl-20261021T191012.649071Z/`. The complete project test suite passed
after this validator change.

### Operator loss during impaired peer traffic — seed 160

Seed 160 combined the network fault cases with operator loss. It used 2% random peer loss, 25 ms
mean delay, 5 ms jitter, 1% duplication, and 5% reordering with a 150 ms added-delay scale. After
all five agents converged (13 aligned samples and a three-sample dwell), passed a directed link
cut/restore, and returned to nominal, the operator process was killed while the vehicles were
airborne. Each of the five independently detected mission-lease expiry, requested LAND, and was
confirmed landed and disarmed within 42.99 seconds. The run also recorded duplicate, delayed, stale,
and replay-rejected traffic on every agent. Minimum aligned airborne separation was 4.147 m across
15 samples.

This is evidence that the software's operator-loss response worked in combination with these
injected peer-network impairments in same-host split-loopback ArduCopter SITL. It does not test a
real host failure, LAN, radio, or physical flight, and one seed is not a reliability estimate. Raw
summary and status logs are in `runs/independent/external-sitl-20261021T191227.507122Z/`.

Seeds 160 and 165–168 now form a comparable five-run batch of this combined scenario. All five
runs first passed mission convergence and directed peer-link recovery. The operator process was
then killed while every vehicle was airborne; in every run, all five agents independently reported
mission-lease-expiry LAND and were confirmed landed/disarmed. The slowest completed within 42.99 s
of operator loss. Across the batch, the impaired transports generated 318 duplicates and 1,592
reordered deliveries, while receivers logged 1,169 replay/order rejections. The minimum aligned
separation across the five trials was 4.147 m.

The observed result is 5/5, with the same wide 95% Wilson interval of 56.6%–100%. It is an initial
repeatability result for one same-host injected-network profile, not a general coordinator-loss
reliability claim. The profile-checked machine-readable result is
`runs/independent/impaired_operator_loss_batch_summary_20261023.json`.

### Repeat impaired-network mission — seed 161

Seed 161 repeated the seed-159 profile with 2% random peer loss, 25 ms mean delay, 5 ms jitter,
1% duplication, and 5% reordering with a 150 ms added-delay scale. All five agents completed the
convergence gate (13 aligned samples and a three-sample dwell), detected and recovered from the
directed peer-link cut, returned to nominal, and landed/disarmed. The recorded transports generated
36 duplicates and 150 reordered deliveries in total; receivers rejected 105 replayed or superseded
packets. Minimum time-aligned separation was 4.148 m.

Together, seeds 159 and 161 provide two passing repetitions of this exact same-host impairment
profile. This is useful repeatability evidence but is not a statistical reliability claim and does
not represent a real LAN or radio. Evidence is in
`runs/independent/external-sitl-20261022T185705.877030Z/`.

### Five-seed impaired-network batch

Seeds 159 and 161–164 form a comparable five-run batch under the same profile: split-loopback,
2% random peer loss, 25 ms mean delay, 5 ms jitter, 1% duplication, and 5% reordering with a
150 ms added-delay scale. All five runs passed convergence, directed link cut/restore, and confirmed
five-vehicle landing/disarm. Across the batch, the network layer generated 160 duplicates and 831
reordered deliveries, while receivers recorded 537 replay/order rejections. The lowest time-aligned
separation in any run was 4.060 m.

The observed pass fraction is 5/5, but the 95% Wilson interval is still wide at 56.6%–100%; this is
an initial repeatability sample, not a general reliability estimate. The batch tool refuses to mix
different impairment profiles, operator-loss modes, incomplete files, or operator-restart modes.
The machine-readable result is
`runs/independent/impaired_network_batch_summary_20261023.json`; its source utility is
`experiments/summarize_external_sitl_runs.py`.

### Configurable correlated burst loss — seed 169

The direct peer transport and external validator now expose and record the two burst transition
probabilities instead of hiding fixed values. Seed 169 used a 2% chance of entering the bad state
and a 75% chance of remaining there for the next attempted datagram, which gives an expected burst
length of four attempted datagrams after entry and a long-run bad-state share of about 7.4%. The
measured aggregate drop fraction was 8.0% (281 burst drops versus 3,234 datagrams accepted for the
delay queue). The run also retained 25 ms mean delay, 5 ms jitter, 1% duplication, and 5% reordering.

All five agents passed convergence and the three-sample dwell, recovered from the explicit directed
link cut, returned to nominal, and landed/disarmed. Minimum time-aligned separation was 4.153 m.
This is one same-host correlated-loss trial, not a repeatability or real-radio result. Evidence:
`runs/independent/external-sitl-20261022T191755.623060Z/`.

Seeds 169–173 extend the same correlated-loss profile to five trials. All five passed mission
convergence, directed-link recovery, and confirmed landing/disarm. Across 17,640 attempted peer
datagrams, 1,463 were dropped by the burst process, for an 8.29% empirical aggregate loss fraction;
the individual runs ranged from 7.56% to 9.35%. The batch also generated 151 duplicates and 826
reordered deliveries, with 552 replay/order rejections. The minimum time-aligned separation across
the five trials was 3.982 m.

The observed 5/5 pass fraction again has a 56.6%–100% 95% Wilson interval. This supports initial
repeatability for the specified correlated-loss model on one computer, not a general network or
flight reliability claim. Machine-readable aggregate:
`runs/independent/correlated_burst_loss_batch_summary_20261023.json`.

### Correlated burst loss combined with operator loss — seeds 174–178

Five trials combined the correlated-loss profile with operator removal: split-loopback addresses,
2% probability of entering a loss burst, 75% probability of continuing it, 25 ms mean delay,
5 ms jitter, 1% duplication, and 5% reordering with a 150 ms added-delay scale. Seeds 174–177
passed. The first seed-178 attempt is deliberately preserved as a failed trial at
`runs/independent/external-sitl-20261024T061152.356759Z/`. Three agent processes terminated after
operator loss because cached mission guidance compared operator-domain expiry timestamps against
the agents' raw wall clocks. Only two agents requested mission-expiry LAND before the 75-second
observer timeout. This was a software safety defect, not a network-harness failure.

The independent node now evaluates cached intent in the same clock domain used when accepting its
monotonic mission lease. A defensive boundary also converts any remaining invalid-intent exception
into a local LAND request instead of allowing the agent process to terminate. A regression test
covers both the clock-domain correction and safe exception path. The complete suite then passed
257 tests. Repeating seed 178 with the same profile passed: all five agents were airborne when the
operator was killed, all five independently requested mission-expiry LAND, and all five landed and
disarmed in 42.83 seconds. Corrected evidence is in
`runs/independent/external-sitl-20261024T061616.543861Z/`.

The first profile-checked aggregate of five passing trials (174–178, with corrected seed 178) is
retained at `runs/independent/correlated_burst_operator_loss_batch_summary_20261024.json`. Because
four of those trials preceded the correction, a second clean batch repeated the updated code with
seeds 178–182. All five post-fix runs passed convergence, directed-link recovery, operator removal,
independent mission-expiry LAND, and confirmed five-vehicle disarm. The slowest fleet landing
confirmation was 42.98 seconds and the lowest sampled time-aligned horizontal separation was
4.155 m. Across 31,841 attempted peer datagrams, the burst process dropped 2,349 (7.38%); the
transports also generated 279 duplicates and 1,485 reordered deliveries, while receivers rejected
1,062 replayed or superseded packets.

The post-fix observed pass fraction is 5/5, with a still-wide 56.6%–100% Wilson 95% interval. It is
repeatability evidence for this exact same-host injected-network scenario. It does not establish
real-LAN, radio, or physical-flight reliability. The failed pre-fix seed-178 attempt remains part
of the engineering record. Post-fix aggregate:
`runs/independent/correlated_burst_operator_loss_postfix_batch_summary_20261024.json`.
