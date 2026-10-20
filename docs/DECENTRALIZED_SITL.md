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

The software still runs on one computer. The harness centrally prepares slot configuration and
peer links, and broadcasts the mission goal. Peer state passes through `PeerStateBus`, which
emulates one-hop messages with configurable delay, loss, stale-data expiry, and directed link
blocking. The harness still observes all MAVLink telemetry and runs all agents in one process.
This does not run code on each flight controller, prove a drone-to-drone radio link, or establish
physical-flight safety. Treat it as an emulation of local per-drone control logic in SITL, with a
tested input boundary between agents.

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
