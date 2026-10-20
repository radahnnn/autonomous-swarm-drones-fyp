# Decentralized SITL stage 1

This branch adds a separate experimental control mode to the unified SITL console:

```bash
python3 -m sitl.swarm_console --drones 5 --standalone --control-mode decentralized
```

Add `--auto-launch` to take off after all five SITL instances become ready. The default
`--control-mode centralized` remains unchanged.

## What “decentralized” means in this stage

The console acts as a SITL transport harness. It keeps the MAVLink links and relays telemetry,
but creates one `LocalDroneAgent` for each simulated vehicle. Each local controller receives:

- its own measured horizontal position and a velocity estimate from its own telemetry history;
- only the states of peers in its one-hop neighbor list;
- relative formation offsets for those same peers;
- the pilot's shared high-level goal and its own configured slot.

Each agent computes its own bounded short-horizon position target through the existing
`DecentralizedController`. The harness sends each resulting target to its SITL vehicle. The
one-hop graph is preconfigured by the SITL harness: links connect nearby formation slots (within
5 m), with minimum-spanning-tree links added where needed to keep the graph connected. After a
simulated failure, the harness updates the active peer graph. Each controller receives only its
own state and one-hop peer states.

The software still runs on one computer. The harness centrally prepares slot configuration and
peer links, and broadcasts the mission goal. Peer state passes through `PeerStateBus`, which
emulates one-hop messages with configurable delay, loss, stale-data expiry, and directed link
blocking. The harness still observes all MAVLink telemetry and runs all agents in one process.
This does not run code on each flight controller, prove a drone-to-drone radio link, or establish
physical-flight safety. Treat it as an emulation of local per-drone control logic in SITL, with a
tested input boundary between agents.

## Stage 1 limits

- Select the formation while the drones are on the ground; mid-flight morphs are disabled.
- A failed drone is commanded to land. The remaining agents keep their existing slots and rebuild
  their one-hop graph. Automatic revive and rejoin are not included in this stage.
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
  the SITL-fitted dynamics profile, 10% packet loss, 50 ms mean channel latency, and the same
  moving V mission. Results and confidence intervals are in
  [the comparison report](reports/DECENTRALIZED_5DRONE_COMPARISON_2026-10-20.md).
- Automated suite: 166 tests passed.

The paired comparison found higher decentralized tracking error and lower separation than the
centralized controller. Neither mode had a simulated collision, but both had a minimum separation
below the project's 2.5 m target in this software experiment. One successful SITL run does not
prove general robustness; repeat across communication conditions and missions.

## Next build sequence

1. Tune the decentralized separation/formation objective against the 2.5 m spacing target and
   rerun paired trials; record failures and confidence intervals.
2. Expand SITL runs across loss, delay, stale data, directed peer cuts, and loss of a vehicle.
3. Add collision-aware formation changes before enabling mid-flight morph commands.
4. Define a versioned peer-message protocol and mission-goal handoff, including IDs, coordinate
   frame, units, timestamps, sequence numbers, and stale-message behavior.
5. Only after software validation, package the local agent as an onboard process per vehicle and
   test a real peer transport and approved flight-controller interface on the bench, followed by
   controlled flight tests. The onboard runtime and hardware communication path do not exist yet.

Do not describe this stage as a validated hybrid system. Hybrid mode will combine a proven
mission-level coordinator with these separately validated local controllers later.
