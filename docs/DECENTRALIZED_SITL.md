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
peer links, and broadcasts the mission goal. This does not yet run code on each flight controller,
prove a drone-to-drone radio link, or establish physical-flight safety. Treat it as an emulation
of local per-drone control logic in SITL.

## Stage 1 limits

- Select the formation while the drones are on the ground; mid-flight morphs are disabled.
- A failed drone is commanded to land. The remaining agents keep their existing slots and rebuild
  their one-hop graph. Automatic revive and rejoin are not included in this stage.
- The control output is horizontal. The console still distributes the common altitude goal.
- The potential-field separation term is not a hard collision constraint. A run must be assessed
  against the project's measured spacing and tracking criteria; no safety guarantee is implied.
- Gazebo and physical aircraft are not validated by the implementation itself.

## Build sequence

1. Validate launch, position tracking, movement and landing in standalone SITL with three drones.
2. Compare centralized and decentralized using the same initial positions, mission commands,
   safety limits and randomized conditions. Record all failures, not only successful runs.
3. Add delayed and missing one-hop peer data tests and verify the remaining swarm after one lost
   link or vehicle.
4. Add collision-aware formation changes before enabling mid-flight morph commands.
5. Only after those stages, move the local agent runtime to separate onboard processes and test
   the actual inter-drone message transport.

Do not describe this stage as a validated hybrid system. Hybrid mode will combine a proven
mission-level coordinator with these separately validated local controllers later.
