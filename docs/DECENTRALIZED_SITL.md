# Decentralized and hybrid SITL stages

This branch adds a separate experimental control mode to the unified SITL console:

```bash
python3 -m sitl.swarm_console --drones 5 --standalone --control-mode decentralized
```

Add `--auto-launch` to take off after all five SITL instances become ready. The default
`--control-mode centralized` remains unchanged.

An experimental hybrid mode is available:

```bash
python3 -m sitl.swarm_console --drones 5 --standalone --control-mode hybrid --auto-launch
```

In flight, `coord off` drops central waypoint heartbeats; each local controller blends into a
peer-based fallback that holds its last assigned slot. `coord on` restores heartbeats, then the
per-agent supervisor waits for healthy packets and its dwell period before blending back. This
hybrid path has the specific standalone validation evidence listed below. It remains experimental;
those runs do not establish broad reliability or physical-flight readiness.

In the pure Python `SwarmSimulation`, decentralized mode now defaults to 3.0 m formation spacing
to provide a buffer above the 2.5 m project minimum. Centralized and hybrid defaults are unchanged;
an explicit `set_formation(..., spacing=...)` still overrides the mode default.

## What “decentralized” means in this stage

The console acts as a SITL transport harness. It keeps the MAVLink links and relays telemetry,
but creates one `LocalDroneAgent` for each simulated vehicle. Each local controller receives:

- its own measured horizontal position and a velocity estimate from its own telemetry history;
- only the states of peers in its one-hop neighbor list;
- relative formation offsets for those same peers;
- the pilot's shared high-level goal and its own configured slot.

Each agent computes its own bounded short-horizon position target through the existing
`DecentralizedController`. Agent acceleration and target velocity are bounded. Stale peer telemetry
is not relabeled as current by the host relay, peer influence fades with message age, and packets
that arrive out of sequence are rejected. The harness sends each resulting target to its SITL vehicle. The
one-hop graph is preconfigured by the SITL harness: links connect nearby formation slots (within
5 m), with minimum-spanning-tree links added where needed to keep the graph connected. After a
simulated failure, the harness updates the active peer graph while retaining the failed vehicle's
slot. Each controller receives only its own state and one-hop peer states.

The software still runs on one computer. In the default `local` agent runtime, the harness
centrally prepares slot configuration and peer links, broadcasts the mission goal, and
`PeerStateBus` emulates one-hop messages with configurable delay, loss, stale-data expiry, and
directed link blocking. In `--agent-runtime process`, each controller runs in a separate OS
process and sends peer-state datagrams directly to its configured neighbors over localhost UDP; the
host no longer relays peer state, but still owns all MAVLink links, supplies each agent's own
telemetry, and provisions slots and neighbor rosters. Process mode can inject configured packet
loss, latency, jitter, and burst loss at each sender; these are software impairments, not a physical
radio model. Neither runtime runs an agent on each flight controller, proves a drone-to-drone radio
link, nor establishes physical-flight safety. Treat process runtime as a closer software boundary
for decentralized control logic, not as a completed onboard architecture.

## Stage 1 limits

- Select the formation while the drones are on the ground; mid-flight morphs are disabled.
- A failed drone is commanded to land and its slot is reserved while it descends. The remaining
  agents hold their current slots and rebuild their one-hop graph. An operator can request `r <id>`;
  after that, the console checks EKF,
  rearms and lifts the vehicle, then tracks its reserved slot while other agents retain
  theirs. This is a staged SITL recovery path, not an autonomous hardware recovery claim.
- Optional `--auto-recover` makes one recovery attempt after a heartbeat-timeout fault only after
  the heartbeat is stable for 3 seconds and the aircraft is confirmed disarmed. Operator-injected
  failures are never automatically relaunched.
- Hybrid mode remains a host-emulated prototype. The PC still supplies central waypoints, and
  `coord off/on` simulates coordinator loss and recovery; it does not isolate a real coordinator
  network or flight computer.
- The control output is horizontal. The console still distributes the common altitude goal.
- Peer topology and relative slots are provisioned by the SITL harness; agents do not discover
  neighbors or agree on a mission goal through peer consensus.
- The potential-field separation term is not a hard collision constraint. A run must be assessed
  against the project's measured spacing and tracking criteria; no safety guarantee is implied.
- Gazebo and physical aircraft are not validated by the implementation itself.

## Validation completed on 2026-10-20

- Five-drone standalone SITL: 4 m north move, 10% random one-hop loss, 100 ms mean peer latency,
  one directed peer link blocked, and Drone 3 injected as failed. The remaining four held their
  slots; all five landed/disarmed. No watchdog timeout. Summary: p95 settled slot error 0.277 m,
  minimum measured separation 4.139 m. Raw record: `runs/console/20261020T174629.974360Z/`.
- Ten paired software-simulation trials used identical initial states and seeds within each pair,
  the SITL-fitted dynamics profile, 0.08 m GPS noise, 10% packet loss, 50 ms mean channel latency,
  and the same moving V mission. Repeated at 2.5 m and 3.0 m spacing. Results are in
  [the spacing and comparison report](reports/DECENTRALIZED_SPACING_TUNING_2026-10-20.md).
- Automated suite: 167 tests passed.

Updated implementation checks on 2026-10-20:

- The full pytest suite passes after adding stale-source rejection, out-of-order packet rejection,
  acceleration bounding, hybrid fallback/recovery, and reserved-slot fault handling.
- Five-drone standalone hybrid SITL: 2 m north command, coordinator link cut/restored, Drone 3
  fail/rejoin, then all vehicles landed. All recorded criteria passed: minimum separation 4.036 m,
  p95 settled slot error 0.443 m, 0 watchdog timeouts. All five agents returned to centralized
  mode after the coordinator link returned. Evidence:
  `runs/console/20261020T181535.817439Z/`.
- Five-drone standalone decentralized SITL: 10% peer packet loss, 50 ms mean latency, 2 m north
  command, Drone 3 fail/rejoin, then all vehicles landed. All recorded criteria passed: minimum
  separation 4.110 m, p95 settled slot error 0.718 m, 0 watchdog timeouts. Evidence:
  `runs/console/20261020T181830.290662Z/`.
- The first hybrid fail/rejoin attempt failed the spacing criterion because healthy drones were
  moved into the faulted drone's space while it was landing. That run is preserved at
  `runs/console/20261020T181034.919544Z/`. The slot-reservation fix was then added and the repeat
  run passed; the failed run remains part of the audit trail.

With 2.5 m target spacing, neither controller consistently maintained the 2.5 m measured minimum.
At 3.0 m target spacing, both controllers maintained at least 2.5 m in all 10 trials. Decentralized
tracking error remained higher than centralized. This is a tested software-simulation setting, not
a collision guarantee or a physical-flight result. The earlier comparison report used starting
positions wider than the target formation and is superseded by the corrected matched-start report.

## Current improvement work and next build sequence

1. Validate the 3.0 m decentralized default under more formations, GPS-noise levels, and missions;
   retain failed runs and report confidence intervals.
2. Repeat hybrid coordinator loss/recovery and staged rejoin over many seeds and with burst loss,
   directed peer cuts, longer delays, and different formations; current evidence is one run per
   mode and is not enough for a reliability claim.
3. Expand measured trials across loss, delay, stale data, directed peer cuts, and formations; report
   confidence intervals and compare tracking error with centralized control.
4. Add collision-aware path planning before enabling mid-flight formation changes or unsupervised
   rejoining on physical aircraft.
5. Define a versioned peer-message protocol and mission-goal handoff, including IDs, coordinate
   frame, units, timestamps, sequence numbers, and stale-message behavior.
6. Only after software validation, package the local agent as an onboard process per vehicle and
   test a real peer transport and approved flight-controller interface on the bench, followed by
   controlled flight tests. The onboard runtime and hardware communication path do not exist yet.

The new paths now pass the recorded tests and the two listed standalone SITL runs. This is evidence
for these specific simulated conditions only; do not treat it as multi-run reliability, Gazebo
validation, hardware validation, or a safety guarantee.

## Follow-up hardening

See [the hardening report](reports/HYBRID_HARDENING_2026-10-20.md). Mission updates now stop
during a reserved-slot recovery, and hybrid outage handling freezes altitude as well as horizontal
goals/slots. Rejoin uses a conservative corridor check. `--agent-runtime process` adds separate
controller workers using the [versioned IPC protocol](AGENT_PROTOCOL.md). This partially implements
the process/protocol steps above; independent onboard flight-controller ownership and real radio
transport are still future work. Existing live results above describe their original source state.

## Independent MAVLink-owner runtime — added 2026-10-21

The earlier `local` and `process` runtimes above still use a host-owned MAVLink harness. A newer,
separate experimental path gives every SITL vehicle an independent agent process that owns that
vehicle's MAVLink connection and setpoint loop:

```bash
PYTHONPATH=. python3 -m sitl.swarm_console --drones 5 --standalone \
  --control-mode decentralized --agent-runtime independent \
  --peer-loss 0.1 --peer-latency-ms 50 --peer-seed 44
```

The operator broadcasts common versioned mission intent directly to each node. Each node combines
the goal with its provisioned slot and exchanges state with its one-hop peers over direct UDP
loopback; no host process relays peer state or computes per-drone movement targets. Nodes cache the
last valid mission between broadcasts. The current 30-second intent lease expires into a LAND
request. See [the protocol notes](AGENT_PROTOCOL.md) for local ports and message behavior.

One live five-drone standalone SITL run passed: all vehicles took off to about 5 m, followed a 2 m
north goal, held the goal while mission updates were paused for about 5 seconds, then landed on
operator command. Peer traffic used 10% configured application-level loss and 50 ms mean delay;
minimum observed horizontal separation was about 4.136 m. Raw record:
`runs/independent/20261021T134539.157325Z/`. A first failed run is preserved at
`runs/independent/20261021T134330.833134Z/`; it exposed and led to a fix for a false heartbeat
timeout immediately after synchronous takeoff checks. Full details are in
[the independent SITL report](reports/DECENTRALIZED_INDEPENDENT_SITL_2026-10-21.md).

A corrected operator-process-kill trial passed with explicit disarm confirmation. SITL stayed alive
after the operator was killed in flight; all five agents continued their accepted mission, then
entered LAND when the 30-second intent lease expired. All five reported `landed`, disarmed, and were
at ground altitude within 42.98 seconds after operator loss; no emergency fallback was sent.
Configuration: 10% peer loss, 50 ms mean latency, seed 52, minimum time-aligned horizontal
separation 4.179 m. Evidence: `runs/independent/20261021T161122.758370Z/`. After the agent's
accepted-mission expiry was changed to a monotonic deadline, seed 89 repeated this coordinator-loss
scenario against the updated code: all five agents reported landed and disarmed 42.99 seconds after
operator loss, without fallback; minimum time-aligned separation was 4.20 m. Evidence:
`runs/independent/20261021T162235.986244Z/`. These were the first two confirmed local SITL trials,
not a reliability or safety guarantee. The earlier seed-50 run reached ground but omitted armed state, so
it is not used as disarm evidence. A later regression run (seed 90) exposed a false peer-timeout
LAND during staggered takeoff and failed to complete the mission; it is preserved at
`runs/independent/20261021T163346.001631Z/`. A five-second flight-monitor startup grace was added,
while preflight continues to require a valid mission and fresh required peers. The seed-91 repeat
then completed operator-loss landing: all five landed and disarmed 43.03 seconds after operator
loss, with no emergency fallback and 4.199 m minimum time-aligned separation. Evidence:
`runs/independent/20261021T163704.820929Z/`. Two further strict repeats (seeds 92 and 93) landed
and disarmed without fallback. Five confirmed operator-loss runs now exist (seeds 52, 89, 91, 92,
93); the failed seed 90 remains visible and excluded. The current-status gate also excludes an older
run that reported vehicles on the ground while they were still armed. Aggregate metrics and method:
`runs/independent/operator_loss_batch_summary_20261021.json`.

The direct UDP path also passed one directed-link-fault trial with `--peer-block 1:2`, 10% random
loss, 50 ms mean latency, and seed 52. Only drone 1 to drone 2 peer datagrams were blocked; the
reverse edge and other links remained active. The swarm completed a 2 m north move, then all five
vehicles were confirmed disarmed and grounded before SITL shutdown. The selected sender recorded
406 blocked datagrams; minimum observed horizontal separation was 4.171 m. Evidence:
`runs/independent/20261021T141447.196880Z/`.

This is a process boundary on one host, not a multi-computer or onboard deployment. The operator
kill tests demonstrate expiry-to-LAND in three local trials. Fixed startup slots/rosters,
unauthenticated localhost UDP, no actual second-computer routing test, and no hardware remain
limitations. Startup estimates clock offsets through five NTP-style probes and requires every node
to acknowledge the map; offsets are remeasured every 60 seconds while the console is active. Five
fake endpoints with clocks skewed by up to six seconds and a simulated post-start 0.4-second clock
change passed this path. A second-computer run with real network asymmetry and host clock changes
remains untested. A restarted airborne agent now commands LAND and announces the request to peers
rather than resuming a mission from lost process state. Five live five-SITL restart trials succeeded:
the restarted node landed, its landing request propagated through the peer graph, and all five
disarmed at ground level. See `runs/independent/20261021T165833.817892Z/`,
`runs/independent/20261021T170858.201957Z/`, `runs/independent/20261021T171445.160289Z/`,
`runs/independent/20261021T171843.179183Z/`, and
`runs/independent/20261021T172005.327854Z/`. See
`runs/independent/agent_restart_batch_summary_20261021.json` for the five-trial metrics and method.
Five seeds on one host are not a general reliability estimate. The prior experiment summaries in
this document describe their original host-owned runtime and remain useful as separate baseline
evidence.

## Software-only realism checks added on 2026-10-21

The independent agent now sends MAVLink GCS heartbeats. Use
`--sitl-gcs-failsafe` to generate a run-local parameter profile that tells ArduCopter SITL to
switch to LAND after five seconds without an agent heartbeat. A live five-drone agent-kill trial
observed the failed drone's flight controller switch to LAND and disarm at ground level without
operator rescue. The option is independent-runtime SITL only; it does not alter shared or hardware
parameter files. See `runs/independent/20261021T142748.189010Z/`.

While airborne, use `link-block <sender> <recipient>` and `link-restore <sender> <recipient>` to
inject and restore one directed peer edge for network testing. A live trial cut both directions
between D1 and D2, restored them, and confirmed peer messages resumed; all aircraft landed with a
minimum observed separation of 4.198 m. Evidence: `runs/independent/20261021T143211.872887Z/`.

Agents send liveness heartbeats separately from position samples, so a heartbeat cannot refresh an
old position used for guidance. If a required peer has no heartbeat for 1.5 seconds and remains
missing for a further 2-second grace period, the affected airborne agent commands its own LAND.
Nine five-drone standalone SITL partition trials passed: one interactive run and eight scripted seeds
(67–71, 75–77). Seeds 67–68 used 2% loss and 25 ms mean latency; 69–70 used 5% loss and 50 ms mean
latency; seed 71 enabled burst loss with the 2% / 25 ms profile. Cutting both D2↔D4 directions
caused D2 and D4 to land autonomously, while the other three remained airborne until operator
LAND. All five then disarmed. Scripted failsafe status appeared 3.71–3.75 seconds after the cuts;
endpoint touchdown took about 16.96 seconds. Evidence:
`runs/independent/20261021T144908.936448Z/`,
`runs/independent/20261021T145947.252933Z/`, and
`runs/independent/20261021T150113.677325Z/`,
`runs/independent/20261021T150539.157149Z/`,
`runs/independent/20261021T150705.602130Z/`, and
`runs/independent/20261021T150843.755075Z/`. A further run combined the partition with 5% duplicate
injection and 25% reordering at seed 75; it logged 172 duplicated datagrams, 791 reorder delays,
and 468 rejected replays. Both endpoints still landed autonomously and the other three remained
airborne until operator LAND. Evidence: `runs/independent/20261021T153950.467547Z/` and
`runs/independent/peer_partition_trials.json`. Seed 77 reran the default local transport after the
operator-channel changes; both isolated endpoints landed, touchdown was 16.97 seconds after the cut,
and all five disarmed. Evidence: `runs/independent/20261021T155633.517618Z/`. These are local
fail-safe checks, not distributed consensus, automatic graph repair, or a physical-radio result;
the nine runs are not a reliability estimate.

## Independent-agent Gazebo 3D result

One five-drone run completed with the independent agents in the Gazebo Harmonic
`cinewhoop_5drones.sdf` world. EKF readiness took 206 seconds. All five vehicles armed, climbed to
about 5 m, followed a 2 m north mission, and landed/disarmed. Peer settings were 2% injected loss,
25 ms mean latency, seed 65. Minimum time-aligned horizontal separation was about 4.173 m. Evidence
is in `runs/independent/20261021T143853.501763Z/`. A telemetry-timeout LAND was observed while one
vehicle was on the ground before operator launch; that aircraft later armed and flew, and no
in-flight timeout LAND was recorded. The earlier sandboxed attempt is preserved at
`runs/independent/20261021T143620.815628Z/` and did not proceed to flight because the OpenGL context
failed to initialize.

The project suite now passes 224 tests after adding restart detection, peer landing propagation,
stable landed-status coverage, and clock-offset handling. The peer protocol is version 3: in addition to session/replay
handling, state packets carry a landing flag. A node that restarts while its vehicle is armed and
airborne commands LAND; other airborne nodes propagate that landing request. Five standalone SITL
trials used the console's `restart 3` command while airborne (seeds 105–109).
The restarted D3 agent reported `landed_after_process_restart`; the other four entered
`peer_landing_failsafe`; all five reported disarmed at ground level. Evidence:
`runs/independent/20261021T165833.817892Z/`,
`runs/independent/20261021T170858.201957Z/`, and
`runs/independent/20261021T171445.160289Z/`,
`runs/independent/20261021T171843.179183Z/`, and
`runs/independent/20261021T172005.327854Z/`. Two earlier observer attempts remain preserved but are
not counted as successful repetitions. The sandbox-only socket failure for seed 108 is preserved
at `runs/independent/20261021T171751.927006Z/` and excluded from behavioral results. Current results
include local 2D, three
independent-agent 3D Gazebo runs, and nine standalone SITL trials of the local peer-timeout LAND
policy; they do not establish real radio behavior, multi-computer operation, calibrated sensor
fidelity, broad reliability, distributed partition agreement, or physical-flight safety.

## Static peer addresses and packet authentication

The independent UDP peer transport now accepts `--peer-roster` with a per-agent bind address,
advertised IPv4 address, and port. `--peer-auth-key-file` enables HMAC-SHA256 verification for
peer state and heartbeat datagrams; invalid signatures are counted as `rejected_auth`. Localhost
and unauthenticated behavior remain the defaults for development.

Seed 76 completed a five-drone standalone SITL partition trial with peer endpoints at distinct
loopback addresses `127.0.0.2` through `127.0.0.6`, all using UDP port 20000, with a shared
test-only HMAC key, 2% injected peer loss, and 25 ms mean latency. Both isolated endpoints landed
autonomously, all five disarmed, and no invalid authentication packets were observed. A wrong-key
rejection and matching-key acceptance are also covered by transport tests. Evidence:

```text
runs/independent/20261021T154647.684138Z/events.jsonl
runs/independent/configs/loopback-five-agent-roster.json
runs/independent/peer_partition_trials.json
```

This is still a one-PC loopback test. The operator channel can now use rostered remote addresses and
external-agent mode, but has not been verified across computers. The shared HMAC key does not
provide per-drone identity. See
[`PEER_NETWORK_SETUP.md`](PEER_NETWORK_SETUP.md) for the roster format, security limits, and work
needed before attempting a two-computer run.

## Declared sensor and wind stress profile (2026-10-21)

The independent runtime accepts `--sitl-sensor-profile stress`. It creates a run-local ArduPilot
parameter file and leaves `sitl/swarm_params.parm` unchanged. The stress settings used here were
GPS horizontal wander 0.50 m, vertical GPS sine-noise amplitude 0.30 m, GPS lag 250 ms, GPS
accuracy setting 0.60, accelerometer noise setting 0.60, and wind 2.0 m/s from 180 degrees with
0.50 turbulence. These are declared software stress assumptions, not measurements or calibrated
models of the physical Matek aircraft. ArduPilot's [SITL simulation parameter reference](https://ardupilot.org/dev/docs/sitl-parameters.html)
describes these simulator controls; in particular, `SIM_GPS1_NOISE` is not horizontal white noise.

The profile completed a five-aircraft standalone SITL mission and a five-aircraft Gazebo Harmonic
3D mission. In both runs all vehicles became EKF-ready, armed, climbed to about 5 m, accepted a
2 m northward shared mission intent, and were confirmed landed/disarmed. Minimum time-aligned
horizontal separation was 3.97 m in standalone SITL and 3.99 m in Gazebo. EKF readiness took
about 211 seconds in Gazebo; all five vehicles remained `safety=nominal` in the recorded runs.
Both runs also injected 2% peer packet loss and 25 ms mean peer latency. Evidence:

```text
runs/independent/20261021T151239.716950Z/events.jsonl
runs/independent/20261021T151239.716950Z/independent_profile.parm
runs/independent/20261021T151552.260273Z/events.jsonl
runs/independent/20261021T151552.260273Z/independent_profile.parm
```

This is one stress run per simulator path. A follow-up five-drone Gazebo stress run captured
3,090 world-statistics samples. During the interval when vehicles were airborne, Gazebo advanced
49.81 simulation seconds in 50.19 wall-clock seconds (integrated RTF 0.992; median reported RTF
1.000). The entire sample window, which includes the slow EKF startup, averaged 0.331 integrated
RTF. The five agents became ready after 211 wall-clock seconds, then completed a 2 m mission and
landed; minimum observed horizontal separation was 3.946 m. Evidence:

```text
runs/independent/20261021T152633.911550Z/events.jsonl
runs/independent/20261021T152633.911550Z/gazebo_world_stats.jsonl
runs/independent/20261021T152633.911550Z/gazebo_rtf_summary.json
```

The RTF recorder is `experiments/record_gazebo_rtf.py`; it stores raw Gazebo statistics and can
align an airborne-only RTF window with agent events. These runs do not prove physical sensor
fidelity, broad repeatability, or flight safety.
