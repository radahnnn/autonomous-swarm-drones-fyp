# Experimental controller process protocol, version 1

Run the five-drone console with `--control-mode hybrid --agent-runtime process`
(or `--control-mode decentralized --agent-runtime process`). The default remains `local`.
Each drone's controller runs in a separate Python process. The console still owns MAVLink,
the shared frame, mission distribution, telemetry estimates, topology, and simulated peer transport.
This is host process isolation, not onboard deployment or independent radio communication.

Transport: newline-delimited JSON on private parent/child pipes, one request and response in flight
per agent. Version is `1`, frame is `shared_local_north_east_metres`, drone IDs are zero-based,
sequence numbers strictly increase. Position/goal/velocity use north/east metres and metres/second;
`sample_time` and `now` are seconds in the same host wall-clock domain. `dt` is seconds.
Each request contains only the recipient's own telemetry, its assigned goal, relative neighbor
slot offsets and received one-hop peer messages; it never includes the whole fleet state.
Peer entries carry ID, position, velocity, age and confidence. Altitude remains managed by the
console, and hybrid altitude/slots freeze with the last received mission during coordinator loss.

A request with an unsupported version/frame, wrong recipient, replayed sequence, own telemetry
older than 1 s or in the future, nonfinite positions or stale peer state is rejected. Missing or
invalid replies cause an agent fault and a LAND request for that simulated aircraft. Recovery
from a failed controller process requires a session restart. Response deadline is 2 s; this is a
prototype with sequential request/response handling and no real-time scheduling guarantee.

The IPC channel is private and trusted. It does not implement network authentication, clock
synchronization, discovery, encryption or radio framing. Do not expose this worker on a network.
These are requirements for a future transport, alongside independent MAVLink ownership per agent.
