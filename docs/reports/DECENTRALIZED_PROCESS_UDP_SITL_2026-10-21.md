# Decentralized process runtime: five-drone SITL check

Date: 2026-10-21
Branch: `work/decentralized-sitl-20261020`
Source revision at run: `717d62a` plus the uncommitted working-tree changes described here.
Raw run: [`runs/console/20261021T133508.325565Z/`](../../runs/console/20261021T133508.325565Z/)

## Scenario

- Five ArduPilot standalone quad SITL vehicles, each connected over its own TCP MAVLink port.
- One Python controller process per simulated vehicle.
- Peer state sent directly between configured neighboring processes over localhost UDP.
- Peer impairment: 10% application-level independent packet loss, 50 ms mean latency, seed 44.
- Operator sequence: launch all five, hold, command `w 2`, inspect status, then land all five.
- No physical hardware, Gazebo physics, or radio link was involved.

## Results

The run completed and all five vehicles reported disarmed after landing. All recorded run criteria
passed: 5/5 connected and EKF-ready, all five were observed airborne, no unexpected watchdog
timeouts, minimum measured horizontal separation 4.170 m (criterion 2.5 m), p95 settled slot error
0.290 m (criterion 1.0 m), and maximum heartbeat age 0.984 s (criterion 3.5 s). The 2 m north
command was recorded at 67.8 s; status showed the formation had moved north while holding about
5 m altitude.

The final direct-peer UDP counters recorded 1,392 queued messages, 1,385 sent, 1,382 received,
160 intentionally dropped, and zero stale, replayed, or invalid packets. The final status showed
each vehicle had one or two fresh direct peers. The host `PeerStateBus` recorded zero transmissions
for this process-runtime run.

Full run summary: [`summary.json`](../../runs/console/20261021T133508.325565Z/summary.json). The
per-tick telemetry and command log is `events.jsonl` in the same directory.

## What this validates

This is evidence that five separate controller processes can exchange peer-state datagrams
directly over localhost, apply configured software loss/latency, receive common high-level goals,
compute local slot targets, command ArduPilot SITL, and land as a swarm in this one scenario.

## Limits and next work

The console still owns every MAVLink connection, supplies each agent's own telemetry, creates the
mission intent each guidance tick, and provisions formation slots and neighbor rosters. The links
are local-machine UDP and impairment is injected in the application transport. This run does not
test coordinator-process removal, peer discovery, network partitions, authentication, Gazebo, a
physical radio, physical flight, or general reliability. It is one five-drone validation run, not
a reliability estimate. Repeat across seeded conditions and add coordinator/agent outage tests
before making broader claims.
