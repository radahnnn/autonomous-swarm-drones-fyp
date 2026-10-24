# Adaptive decentralized build — 24 October 2026

## Purpose

This opt-in build extends the ten-agent independent SITL architecture without replacing the
previous validated fixed-topology checkpoint.

## Implemented software

- **Dynamic discovery:** every independent node sends authenticated liveness and state packets
  to the configured fleet roster. Discovery traffic is separate from the neighbour set used by
  the guidance controller.
- **Topology repair:** each node derives a predecessor/successor ring from its current live-ID
  view. Removing a stale member therefore creates a new one-hop route without an operator-issued
  topology update.
- **Leader election:** the lowest live ID in a node's current component view is the elected
  leader. This currently provides a deterministic role/status result; the operator still supplies
  the high-level mission lease.
- **Formation redistribution:** active IDs, in sorted order, receive a freshly generated formation
  for the active fleet size. The controller's existing bounded target and separation behaviour
  moves agents toward those slots.
- **Obstacle response:** repeatable `--obstacle N,E,R` options define circular keep-out zones in
  the shared simulated frame. Each agent applies its own bounded repulsion using its local
  position. These are declared virtual obstacles, not readings from a physical sensor.
- **Existing safety retained:** mission expiry, stale vehicle telemetry, invalid mission data and
  complete local isolation still select LAND. A peer's individual landing is state information
  and does not itself become a fleet-wide LAND command.
- **Visual operator dashboard:** the localhost dashboard presents a North/East map, mission
  target, one-hop peer graph, active leader, compact per-agent health, movement/altitude controls
  and directed link block/restore controls. Centralized, decentralized and hybrid profiles are
  visible together. A profile change is locked in flight; on the ground it provides the correct
  relaunch command because those architectures use different process layouts.

## Architecture boundary

All nodes are separate OS processes and use direct UDP peer datagrams, but they still run on one
laptop. Full-fleet discovery is used to repair the sparse guidance ring. This is a practical
software prototype, not proof of radio-range neighbour discovery, consensus under arbitrary
network partitions, or onboard execution.

The same node executable already accepts either a SITL TCP/UDP endpoint or a serial MAVLink device
with an explicit system ID and baud rate. A physical bench step would connect one node to the
Matek over a connectorized UART harness, props removed, verify heartbeat/telemetry and mode
commands, and only then progress through restrained and controlled flight tests. That bench and
flight evidence has not been produced.

## Validation state

Two recorded ten-drone standalone SITL flights completed on 24 October:

- `runs/independent/20261024T141134.905213Z`: all ten agents acknowledged launch, became airborne,
  and remained in nominal safety state. Blocking D1 -> D2 caused D2 to remove D1, repair its ring
  neighbours, and elect D2 from its local nine-member view. Restoring the link returned D2 to the
  ten-member view, D1 leadership and its D1/D3 neighbours. All ten acknowledged LAND and the run
  ended with `landed=true`.
- `runs/independent/20261024T141743.367331Z`: repeated launch/landing after dashboard changes. All
  ten agents remained nominal with a ten-member view and D1 leader, acknowledged both commands,
  and confirmed landing.

The focused dashboard/membership/external-agent/decentralized suite passed 93 tests. The broad
project suite also passed with only the invariant that forbids an intentionally live SITL process
excluded. The dashboard's grounded profile responses and in-flight HTTP 409 architecture lock were
checked against the live server.

The declared circular obstacle produced bounded avoidance response during the first run, but that
run did not establish general path completion around obstacles. The current repulsive behavior can
slow or stall near a symmetric obstacle; a waypoint or tangent path planner remains necessary
before claiming complete obstacle navigation. These results remain single-host SITL evidence and
do not establish multi-computer, radio-network or physical-flight behavior.
