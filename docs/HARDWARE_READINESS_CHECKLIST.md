# Hardware-readiness checklist (document only)

**Status: nothing on this list is done for flight.** Everything the software has shown is SIMULATION / SITL
(`docs/PROJECT_STATE.md` section 4). This page says what must be true, and what evidence would show it, before
anyone flies the swarm stack on a real aircraft. It is not approval to fly and does not change `hardware/`.

How to use: a gate is passed only when its evidence exists in the repo (a report under `hardware/reports/` or
`docs/evidence/`) with a date and an evidence label (BENCH / PHYSICAL). Take gates in order; stop at the first
unmet one. Anything marked *propellers off* must be done with the propellers physically removed.

## Gate 0 - People, place, rules (before any power-on outdoors)
| # | Requirement | Evidence |
|---|---|---|
| 0.1 | A named remote pilot with a manual RC override within reach for every flight | Flight log with pilot name |
| 0.2 | The legal basis for flying at the test site is established with the **local aviation authority and the university** (registration, permission, insurance). Jurisdiction rules were **not** researched here; JARUS SORA 2.5 is a reference framework for the risk argument, not a licence | Written permission on file |
| 0.3 | A test site clear of people, with a defined exclusion zone larger than the geofence plus the stopping distance; weather limits written down (wind, rain, visibility) | Site sheet |
| 0.4 | Written abort criteria and who calls them (loss of link, GPS degradation, any drone leaving the fence, battery limit) | Checklist signed per flight |
| 0.5 | Remote ID / identification requirement checked for the site. Not implemented in this project | Note in site sheet |

## Gate 1 - Each aircraft on the bench (propellers off)
| # | Requirement | Evidence |
|---|---|---|
| 1.1 | Per-aircraft inventory: frame, motors, ESC protocol, FC model and firmware, GPS, battery; the team reports 10 department drones and has 2, none inventoried yet | Inventory table |
| 1.2 | FC parameters exported and diffed against `hardware/params/`; pre-arm checks pass with no overrides (RC found, accelerometer 3D calibration, compass, GPS fix) | Param file + pre-arm log |
| 1.3 | Motor order and direction verified for the frame type; failsafes configured: RC loss, battery low/critical, GCS loss, **EKF failsafe**, **ArduPilot fence enabled** (the software fence in `sitl/geofence.py` is not a substitute) | Param diff + screenshot/log |
| 1.4 | Arming, disarm and kill switch tested on RC with propellers off | Bench log |

## Gate 2 - Companion link (propellers off)
| # | Requirement | Evidence |
|---|---|---|
| 2.1 | The ESP32 (or companion) role and its link to the FC are decided and demonstrated: a **connectorized UART/MAVLink path first**; CAN only as a separate experiment. Today only USB enumeration of the ESP32 is verified | `hardware/reports/` bench report |
| 2.2 | Heartbeat both ways, setpoint accepted in GUIDED, loss of the companion link makes the FC fall back to its own failsafe (not hover forever) | Bench log with timestamps |
| 2.3 | The agent runs against a real FC's telemetry with the flight battery disconnected / props off, using the same `IndependentAgentNode` path as SITL | Run folder under `runs/` |
| 2.4 | Peer links (Wi-Fi/UDP) tested with the real radios: HMAC key provisioned per aircraft, measured latency and loss, clock-sync uncertainty read from the console against `docs/reports/...` budget (`sitl/clock_budget.py`) | `docs/PHYSICAL_NETWORK_VALIDATION_PLAN.md` filled in |

## Gate 3 - First flight, one aircraft, tethered or low and slow
| # | Requirement | Evidence |
|---|---|---|
| 3.1 | Manual hover and RC-triggered failsafes proven (RTL/Land) before any software command | Flight log |
| 3.2 | One aircraft GUIDED hover under the agent, geofence and ceiling set small, pilot ready to switch to manual | Flight log + video |
| 3.3 | GPS accuracy measured at the site (plain GNSS is metres, not centimetres). The 2.5 m separation gate and the 2.8 m barrier distance were chosen in simulation and must be re-derived from measured position error and vehicle size (stress results: `docs/reports/STRESS_AND_SENSITIVITY_2026-10-31.md`) | Measurement + decision note |

## Gate 4 - Two aircraft
| # | Requirement | Evidence |
|---|---|---|
| 4.1 | Altitude layers on (`--altitude-layers`) for the first multi-aircraft flights | Flight log |
| 4.2 | Separation held with pilots able to take over each aircraft; coordinator-loss and peer-loss behaviour tested on the ground first with props off | Run reports |

Do not scale beyond what the previous gate demonstrated. Ten aircraft is a simulation claim until Gates 0-4 are
repeated with real vehicles.

## What the software does and does not provide
- Provides: software geofence and ceiling (`sitl/geofence.py`), separation guard and barrier filter, authenticated
  peer links, lease/failover policies, run reports, replay.
- Does **not** provide: a certified failsafe, detect-and-avoid for other aircraft or people, wind/gust rejection beyond
  what the autopilot does, battery management, or RF-interference handling.
