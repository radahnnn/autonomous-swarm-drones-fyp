# Experimental live hybrid guidance (14 October 2026)

`python3 -m sitl.swarm_console --drones 5 --standalone --control-mode hybrid` selects the
experimental host-side adapter in `sitl/live_hybrid.py`. The default console mode remains
`centralized`. In hybrid mode, `coord off` suppresses coordinator heartbeats and `coord on`
restores them. The adapter uses the research `HybridController` and
`DecentralizedController`, current MAVLink positions, and other vehicles' telemetry relayed
inside the same laptop process. It sends bounded short-horizon GUIDED position setpoints.
This demonstrates controller state transitions; it does not demonstrate onboard or radio
distributed fallback, independent peer communications, or collision avoidance guarantees.

The five-drone standalone run at `runs/console/20261014T140738.839270Z/` connected with
SysIDs 1–5, passed EKF checks, flew, failed and revived drone 1, and confirmed landing.
Its summary was **fail**: minimum observed separation **2.298 m** against 2.5 m, and p95
settled slot error **2.797 m** against 1.0 m. At about 225 seconds, drone 5 had drifted
roughly 26 m from its assigned slot before recovering. A prior hybrid run separately logged
all five controllers entering fallback after `coord off` and returning after `coord on`; that
run also failed mission criteria. The rejoin staging and climb-column reservation need further
controller tuning and repeated runs. Do not use hybrid mode as the presentation's validated
five-drone flight demonstration.

The five-drone Gazebo world rendered on the NVIDIA GPU and all SITL links connected. The
console's EKF position readiness gate remained unsatisfied after 120 seconds, so no 3D
takeoff was attempted. Evidence: `runs/console/20261014T134345.938470Z/`.

The run folders are ignored by Git; retain the local copies if the evidence must be archived.
