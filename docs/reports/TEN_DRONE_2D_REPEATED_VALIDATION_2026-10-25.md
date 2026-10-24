# Repeated ten-drone 2D comparison — 2026-10-25

## What was run

The pure-Python 2D comparison was extended from 10 to 30 paired seeds per
mode and scenario: 270 trials total, each with ten drones and a 30 s mission.
The modes were centralized, decentralized, and hybrid. Scenarios were nominal
impaired peer networking, two repeated coordinator outages, and the same
outages with bursty packet loss. The predefined gates were a sampled minimum
pair separation of 2.5 m and steady p95 formation error no greater than 1 m.
The simulator uses its `assumed_baseline` dynamics; this is not a hardware,
radio, or live ArduPilot test.

`experiments/summarize_three_mode_repeats.py` adds a per-cell pass-rate Wilson
interval, worst observed clearance margin, failure list, and a separate
0.25 m advisory clearance buffer. The buffer is an analysis aid, not a
retroactive change to the predefined 2.5 m gate.

| Scenario | Mode | Passes | Worst pair distance | Margin over 2.5 m | Runs below 2.75 m buffer | Mean steady p95 error |
| --- | --- | ---: | ---: | ---: | ---: | ---: |
| Nominal impaired | Centralized | 30/30 | 2.604 m | 0.104 m | 14/30 | 0.125 m |
| Nominal impaired | Decentralized | 30/30 | 2.603 m | 0.103 m | 14/30 | 0.178 m |
| Nominal impaired | Hybrid | 30/30 | 2.603 m | 0.103 m | 14/30 | 0.125 m |
| Repeated outage | Centralized | 30/30 | 2.604 m | 0.104 m | 15/30 | 0.125 m |
| Repeated outage | Decentralized | 30/30 | 2.603 m | 0.103 m | 14/30 | 0.278 m |
| Repeated outage | Hybrid | 30/30 | 2.603 m | 0.103 m | 15/30 | 0.125 m |
| Burst repeated outage | Centralized | 30/30 | 2.538 m | 0.038 m | 27/30 | 0.125 m |
| Burst repeated outage | Decentralized | 30/30 | 2.529 m | 0.029 m | 22/30 | 0.208 m |
| Burst repeated outage | Hybrid | 30/30 | **2.502 m** | **0.002 m** | **29/30** | 0.125 m |

Each 30/30 observed pass rate has a two-sided 95% Wilson interval of
approximately 88.6–100%. That does not establish a 100% real-world success
rate. The minimum is sampled at the simulator's 0.05 s step, not a
continuous-time guarantee; the 2 mm worst margin and frequent advisory-buffer
shortfalls are a clear reason to improve clearance before making stronger
safety claims.

The exact raw trials, conditions, Git revision, and software environment are
in `runs/comparison/three_modes_10drones_20261025_30seed.json`; the derived
summary is in `runs/comparison/three_modes_10drones_20261025_30seed_summary.json`.
The comparison code had no tracked changes during the run; its dirty-tree flag
records an untracked analysis script added while it was running.

## Live SITL boundary

The earlier [ten-drone 2D/SITL report](TEN_DRONE_2D_VALIDATION_2026-10-24.md)
contains one successful matched standalone ArduPilot run per mode and separate
fault tests. Those runs remain the live evidence. A fresh independent-agent
rerun attempted for this report could not obtain MAVLink heartbeats while a
separate ten-drone desktop SITL/console session was already using the fixed
SITL instance ports. Port contention is the likely cause; the environment
failure was not isolated further. It was stopped before flight and is **not** counted as a
controller failure or a successful repeat. The existing desktop session was
left untouched. Fresh live repeats require its owner to close that session
first, or an isolated host with distinct SITL ports.

## Reproduce the numerical result

```bash
python3 -m experiments.compare_three_modes --drones 10 --trials 30 \
  --output runs/comparison/three_modes_10drones_20261025_30seed.json
python3 -m experiments.summarize_three_mode_repeats \
  runs/comparison/three_modes_10drones_20261025_30seed.json \
  --output runs/comparison/three_modes_10drones_20261025_30seed_summary.json
python3 -m pytest -q
```

Next, target a material clearance reserve under burst loss and repeat the
ten-drone standalone SITL fault cases after freeing the fixed instance ports.
Independent computers, physical radios, and real aircraft remain untested.
