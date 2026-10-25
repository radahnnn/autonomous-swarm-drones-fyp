# Software acceptance matrix

Status: proposed engineering gates, 25 October 2026. These are measurable
completion criteria for the software objectives in the FYP proposal. The team
and supervisor should approve the thresholds before they become formal claims.
The existing 2.5 m sampled separation and 1 m steady tracking-error gates are
preserved; a 0.25 m clearance reserve is an additional advisory target.

## One definition for each measure

| Measure | Definition | Gate or target |
| --- | --- | --- |
| Pair separation | Smallest Euclidean distance between vehicle centers, over every pair and every logged sample, including formation transitions | Existing gate: at least 2.5 m. Advisory target: at least 2.75 m. State sample period and never describe a sampled pass as a continuous-time guarantee. |
| Steady formation error | For each vehicle and sample after settling, distance to its assigned slot; report the pooled 95th percentile, not a final snapshot or mean | Existing numerical gate: at most 1 m. Record the settling interval. |
| Mission completion | Every intended vehicle reaches its target within the declared tolerance, then lands and reports disarmed | All intended vehicles; no silent timeout. |
| Fault response | Detection time, mode transition, stale-state rejection, landing/hold action, and recovery time for each injected fault | Set explicit deadlines from the configured lease and watchdog before each run. No automatic re-arming after a fault. |
| Reproducibility | Code revision, parameter profile, seed, per-vehicle event and telemetry logs, network conditions, pass/fail result | Required for every counted run, including failures. |

Do not compare a pooled steady p95 from one experiment with a final snapshot,
mean error, or 3D error from another as though they were the same measure. Use
the same initial layout, target trajectory, formation change, distance gate,
settling interval, and seed in each paired mode comparison.

## Evidence gates

| Proposal objective | Minimum software evidence | Current state |
| --- | --- | --- |
| Mathematical model | Dynamics, measurement, communication, and control assumptions with units and parameter provenance; fit and held-out error | Partial. Corrected SITL fits exist; physical airframe parameters do not. |
| Centralized control | Repeatable 5- and 10-vehicle missions with assignments, movement, formation changes, logs, and all metrics above | Partial. Live SITL missions exist; repeated matched live runs remain. |
| Decentralized control | Each agent owns its vehicle link and local decision, receives peer messages directly, and survives peer loss/restart under logged tests | Partial. Same-host fault tests and one five-agent host/VM SITL mission passed; repeated split-host faults and radio validation remain. |
| Hybrid switching | Coordinator outage and restoration trigger logged fallback/recovery without unsafe setpoint jumps or stale commands | Partial. Same-host SITL evidence exists; repeated live and separate-host evidence remains. |
| Formations and reconfiguration | Line, V, circle/ring, and grid tests with slot assignment, movement, morphing, separation, and steady error | Partial. Formation generators exist; Phase 2 V-to-line morph currently fails the 2.5 m separation gate. |
| Scaling and reliability | Paired seeds for 5 and 10 vehicles; nominal, delay/loss, burst loss, coordinator loss, peer loss, restart, and stale packets | Partial. Thirty-seed 10-vehicle point-mass results pass the 2.5 m gate with a thin margin; live SITL has one matched run per mode. |
| Prototype interface | Onboard agent protocol and a verified flight-controller command path, first on a propellers-off bench | Open. No end-to-end ESP32/CAN/Matek command has been confirmed. |

## Promotion rule

Keep pure numerical, ArduPilot SITL, Gazebo, separate-host network, propellers-off
bench, and flight results in distinct evidence classes. A result moves to the
next class only when that class has its own complete run log and gate evaluation.
Do not promote a Phase 2 controller because a different ten-drone controller
passed. Preserve failing seeds and the configuration that produced them.

The Ubuntu Server VM provided a first **separate-OS-instance network gate**:
five independent agents ran across the desktop and VM, with direct UDP peers
and remote TCP MAVLink. See the
[split-host report](reports/VM_SPLIT_HOST_SITL_2026-10-25.md). It is one
physical PC and one clean mission, so repeated faults, ten agents across the
boundary, separate hardware, physical radio, and CAN remain open.

See [ten-drone live validation](reports/TEN_DRONE_2D_VALIDATION_2026-10-24.md),
[repeated comparison](reports/TEN_DRONE_2D_REPEATED_VALIDATION_2026-10-25.md),
and the separate Phase 2 branch's
[spacing diagnosis](https://github.com/radahnnn/autonomous-swarm-drones-fyp/blob/work/corrected-phase2-20261020/docs/reports/PHASE2_S4_BRIDGE_2026-10-25.md).
