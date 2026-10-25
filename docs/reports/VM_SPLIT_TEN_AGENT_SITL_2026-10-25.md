# Ten-agent SITL across the Ubuntu host and VM — 2026-10-25

After the five-agent host/VM test and its bounded timestamp-tolerance fix,
the same experimental branch was run with ten independent agents. Five agents,
all ten standalone ArduCopter SITL instances, and the operator ran on the
Ubuntu desktop (`192.168.100.1`). Five agents ran in the Ubuntu Server VM
(`192.168.100.2`). The static roster placed D1–D5 on the host and D6–D10 in
the VM. It sent direct peer UDP across the virtual network boundary, operator
commands/status across that boundary, and remote MAVLink TCP to the five VM
agents. The shared HMAC test key was kept outside Git.

All ten agents connected to their own SITL endpoint and became EKF-ready.
The operator's ten-agent peer-link preflight passed with all required links
live, including the cross-host D4↔D6 and D5↔D7 links. All ten took off to
5 m, accepted a 2 m north mission, moved north while retaining their V
formation, and later reported `landed`, `armed=false`, and `airborne=false`.
No agent reported a peer source-address or HMAC authentication rejection.
Every final status still listed its required peer links as live.

Time alignment within 250 ms yielded 34 full-fleet airborne samples. The
minimum **sampled** horizontal separation was **4.094 m**; the maximum
selected timestamp difference was 96 ms. For the 2 m mission, horizontal
tracking RMSE was 1.028 m and p95 was 1.774 m across the whole movement,
including its transient. Pairwise formation-offset RMSE was 0.036 m. Final
horizontal slot errors after landing were at most 0.280 m. These results
describe one run and sampled telemetry; they do not bound continuous spacing.

The operator event stream, roster, all ten agent logs, and all ten SITL logs
are preserved in [`evidence/VM_SPLIT_TEN_2026-10-25/`](evidence/VM_SPLIT_TEN_2026-10-25/).
After all ten were confirmed disarmed, only this temporary fleet's agents and
SITL instances were stopped. The VM remained running.

This advances the project beyond ten agents on one OS: ten independent agents
were exercised across two OS instances and a host-only virtual network. Both
instances still share one physical computer, and this single clean run has no
injected network impairment or restart fault. Repeated seeded runs, controlled
loss/delay and clock skew, link-fault and coordinator-loss trials, a separate
physical computer, actual radios, and physical flight remain distinct gates.
