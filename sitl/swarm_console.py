#!/usr/bin/env python3
"""
Swarm Flight Console entry point: `python3 -m sitl.swarm_console --drones 10 --standalone ...` (or `--gazebo`).

Every drone runs its own independent agent process with its own MAVLink connection
(`sitl/independent_agent_node.py`); the operator console is `sitl/independent_swarm_console.py` and SITL start-up is
`sitl/sitl_launcher.py`. This module only parses and validates the command line and applies `--realistic`.

The older host-side runtimes ("local" and "process", with `mavlink_swarm_adapter.py`, `mode_supervisor.py`,
`agent_process.py`) were retired on 2026-11-04 by the user's decision (PROJECT_STATE section 8, item 5) and archived,
not deleted, under `archive/legacy_runtimes/` (git history keeps them too).
"""
from __future__ import annotations

import argparse
import math
import os
from pathlib import Path

from sitl.fleet import MAX_DRONES
from sitl.formation_library import STYLES


def apply_realistic(args) -> None:
    """--realistic: the realistic SITL profile plus a degraded radio link (explicit peer settings are kept)."""
    if not getattr(args, "realistic", False):
        return
    from sitl.independent_swarm_console import REALISTIC_PEER_LINK
    if args.sitl_sensor_profile not in (None, "realistic", "realistic-ekf"):
        raise ValueError("--realistic sets its own SITL profile (realistic or realistic-ekf); drop --sitl-sensor-profile")
    args.sitl_sensor_profile = args.sitl_sensor_profile or "realistic"
    if getattr(args, "formation_scale", None) is None:
        args.formation_scale = 1.4
    if not getattr(args, "position_sigma", 0.0):
        args.position_sigma = 1.0
    defaults = {"peer_loss": 0.0, "peer_latency_ms": 30.0, "peer_jitter_ms": None, "peer_burst_loss": False}
    for name, value in REALISTIC_PEER_LINK.items():
        if getattr(args, name) == defaults[name]:
            setattr(args, name, value)


def resolve_sitl_sensor_profile(requested: str | None, agent_runtime: str,
                                external_agents: bool = False) -> str:
    """Use the project-matched compass setup for independent SITL by default."""
    if requested is not None:
        return requested
    if agent_runtime == "independent" and not external_agents:
        return "matek-single-compass"
    return "baseline"


def main():
    parser = argparse.ArgumentParser(description="Swarm Flight Console (ArduPilot SITL, standalone or Gazebo)")
    parser.add_argument("-n", "--drones", type=int, default=5, help=f"swarm size, 1-{MAX_DRONES} (default 5)")
    parser.add_argument("--gazebo", action="store_true", help="force Gazebo 3D mode (--model JSON)")
    parser.add_argument("--standalone", action="store_true", help="force standalone SITL (--model quad)")
    parser.add_argument("--restart", action="store_true", help="kill existing SITL processes and start fresh")
    parser.add_argument("--ekf-timeout", type=float, default=None, help="seconds to wait for the EKF (default 300, 900 in Gazebo)")
    parser.add_argument("--auto-launch", action="store_true", help="take off and form up immediately")
    parser.add_argument("--control-mode", choices=("decentralized", "hybrid"), default="hybrid",
                        help="start mode; the swarm can switch to centralized (and back) in flight with 'mode <name>'")
    parser.add_argument("--peer-loss", type=float, default=0.0,
                        help="decentralized SITL only: independent one-hop packet loss probability (0-1)")
    parser.add_argument("--peer-latency-ms", type=float, default=30.0,
                        help="decentralized SITL only: mean one-hop peer-message latency in milliseconds")
    parser.add_argument("--peer-jitter-ms", type=float, default=None,
                        help="independent runtime only: standard deviation of one-hop peer delay in milliseconds (default 5 when mean delay is nonzero)")
    parser.add_argument("--peer-seed", type=int, default=None,
                        help="decentralized SITL only: reproducible peer-channel random seed")
    parser.add_argument("--peer-burst-loss", action="store_true",
                        help="decentralized/hybrid SITL only: add bursty Gilbert-Elliott packet loss")
    parser.add_argument("--peer-duplicate-rate", type=float, default=0.0,
                        help="independent runtime only: duplicate probability per peer datagram (0-1)")
    parser.add_argument("--peer-reorder-rate", type=float, default=0.0,
                        help="independent runtime only: probability of extra delay that can reorder peer datagrams (0-1)")
    parser.add_argument("--peer-reorder-delay-ms", type=float, default=150.0,
                        help="independent runtime only: variable extra delay scale for reordered datagrams")
    parser.add_argument("--peer-roster", type=Path, default=None,
                        help="independent runtime only: shared JSON file with per-agent peer bind/advertised addresses")
    parser.add_argument("--peer-auth-key-file", type=Path, default=None,
                        help="independent runtime only: shared HMAC key file for direct peer datagrams")
    parser.add_argument("--external-agents", action="store_true",
                        help="independent runtime only: run operator console without launching SITL or agent processes")
    parser.add_argument("--operator-bind-host", default="127.0.0.1",
                        help="independent runtime only: local interface for agent status UDP sockets (use 0.0.0.0 for remote agents)")
    parser.add_argument("--agent-runtime", choices=("independent",), default="independent",
                        help="run experimental controllers in processes, or give every drone its own MAVLink connection")
    parser.add_argument("--formation", choices=tuple(STYLES), default="v",
                        help="starting formation for independent agent runtime")
    parser.add_argument("--auto-recover", action="store_true",
                        help="after heartbeat loss, rejoin only when the link is stable 3s and disarm is confirmed")
    parser.add_argument("--peer-block", type=str, default=None,
                        help="decentralized SITL only: directed link(s) to drop, e.g. 2:4 or 1:2,2:1")
    parser.add_argument("--sitl-gcs-failsafe", action="store_true",
                        help="independent runtime only: enable ArduPilot SITL LAND after 5s without that agent's GCS heartbeat")
    parser.add_argument("--sitl-sensor-profile", choices=("baseline", "stress", "matek-single-compass", "realistic", "realistic-gps", "realistic-wind",
                                 "realistic-vibration", "realistic-battery", "realistic-novibration", "realistic-ekf"),
                        default=None,
                        help="independent runtime only: run-local SITL profile (defaults to matek-single-compass for independent agents)")
    parser.add_argument("--safety-filter", choices=("on", "off"), default="on",
                        help="independent runtime: control-barrier filter on the commanded setpoint (default on)")
    parser.add_argument("--safety-distance", type=float, default=2.8, metavar="M",
                        help="independent runtime: barrier distance of the safety filter")
    parser.add_argument("--geofence-radius", type=float, default=None, metavar="M",
                        help="independent runtime: circular software fence around the local origin")
    parser.add_argument("--geofence-box", default=None, metavar="N0,E0,N1,E1",
                        help="independent runtime: rectangular software fence (north/east metres)")
    parser.add_argument("--geofence-max-alt", type=float, default=None, metavar="M",
                        help="independent runtime: maximum altitude")
    parser.add_argument("--geofence-margin", type=float, default=2.0, metavar="M",
                        help="commanded targets and operator moves stay this far inside the fence")
    parser.add_argument("--altitude-layers", type=int, default=1, choices=range(1, 6), metavar="K",
                        help="independent runtime: K vertical layers (drone ID mod K) for extra vertical separation")
    parser.add_argument("--layer-spacing", type=float, default=1.5, metavar="M",
                        help="metres between altitude layers")
    parser.add_argument("--time-scale", type=float, default=1.0, metavar="R",
                        help="independent runtime: simulator speed relative to the wall clock (Gazebo about 0.15); "
                             "slows setpoint/transition speeds and stretches vehicle-response timeouts")
    parser.add_argument("--sensed-obstacle", action="append", default=[], metavar="N,E,R",
                        help="independent runtime: a true obstacle of the simulated world that each drone finds with "
                             "its range sensor (use --sensed-obstacle=N,E,R); repeatable")
    parser.add_argument("--sense-range", type=float, default=8.0, metavar="M",
                        help="independent runtime: simulated obstacle sensor range (m)")
    parser.add_argument("--position-sigma", type=float, default=0.0, metavar="M",
                        help="independent runtime: expected position error (m); planner clearance and transition "
                             "tolerances grow with it (--realistic sets 1.0)")
    parser.add_argument("--formation-scale", type=float, default=None, metavar="K",
                        help="spread every formation by K (1.0-3.0; --realistic uses 1.4); also sets the start layout")
    parser.add_argument("--realistic", action="store_true",
                        help="independent runtime: realistic disturbances (GPS error and lag, wind and gusts, battery sag, "
                             "vibration, sensor noise) plus a lossy, delayed, bursty radio link; values are assumptions")
    parser.add_argument("--gate-metric", choices=("horizontal", "3d", "ellipsoid"), default="horizontal",
                        help="separation measure for the run report: horizontal (project gate), 3d (sphere) "
                             "or ellipsoid (needs --vertical-radius; horizontal-equivalent gap)")
    parser.add_argument("--vertical-radius", type=float, default=None, metavar="M",
                        help="vertical semi-axis of the ellipsoid keep-out (downwash-aware gate)")
    parser.add_argument("--transition-protocol", action="store_true",
                        help="independent runtime (opt-in): synchronized formation transitions with "
                             "prepare/ack/common start, local supervision and HOLD on abort "
                             "(docs/TRANSITION_PROTOCOL_SPEC.md)")
    parser.add_argument("--adaptive-swarm", action="store_true",
                        help="independent runtime: dynamic membership, topology repair, leader election and slot redistribution")
    parser.add_argument("--obstacle", action="append", default=[], metavar="N,E,R",
                        help="independent runtime: circular north,east,radius keep-out zone; repeatable")
    parser.add_argument("--dashboard-port", type=int, default=0,
                        help="independent runtime: serve the simple browser controller on this localhost port")
    parser.add_argument("--evidence-dir", type=Path, default=None, help="parent folder for per-run JSONL and summary (default runs/console)")
    args = parser.parse_args()
    try:
        apply_realistic(args)
        if args.formation_scale is not None:
            from sitl.formation_library import FORMATION_SCALE_ENV, formation_scale
            os.environ[FORMATION_SCALE_ENV] = str(args.formation_scale)     # inherited by every agent process
            formation_scale()                                                 # validate now
    except ValueError as exc:
        parser.error(str(exc))
    args.sitl_sensor_profile = resolve_sitl_sensor_profile(
        args.sitl_sensor_profile, args.agent_runtime, args.external_agents,
    )
    if not 1 <= args.drones <= MAX_DRONES:
        parser.error(f"--drones must be between 1 and {MAX_DRONES}")
    if (not math.isfinite(args.peer_loss) or not 0.0 <= args.peer_loss <= 1.0 or
            not math.isfinite(args.peer_latency_ms) or args.peer_latency_ms < 0):
        parser.error("--peer-loss must be 0..1 and --peer-latency-ms finite and non-negative")
    if (args.peer_jitter_ms is not None and
            (not math.isfinite(args.peer_jitter_ms) or args.peer_jitter_ms < 0)):
        parser.error("--peer-jitter-ms must be finite and non-negative")
    if args.peer_jitter_ms is not None and args.agent_runtime != "independent":
        parser.error("--peer-jitter-ms requires --agent-runtime independent")
    if (not math.isfinite(args.peer_duplicate_rate) or
            not 0.0 <= args.peer_duplicate_rate <= 1.0 or
            not math.isfinite(args.peer_reorder_rate) or
            not 0.0 <= args.peer_reorder_rate <= 1.0 or
            not math.isfinite(args.peer_reorder_delay_ms) or
            args.peer_reorder_delay_ms < 0):
        parser.error("peer duplicate/reorder rates must be finite 0..1 and extra delay finite/non-negative")
    if (args.peer_duplicate_rate or args.peer_reorder_rate) and args.agent_runtime != "independent":
        parser.error("peer duplicate/reorder injection currently requires --agent-runtime independent")
    if (args.peer_roster is not None or args.peer_auth_key_file is not None) and args.agent_runtime != "independent":
        parser.error("peer roster and authentication options require --agent-runtime independent")
    if args.agent_runtime in ("process", "independent") and args.control_mode == "centralized":
        parser.error(f"--agent-runtime {args.agent_runtime} requires decentralized or hybrid control")
    if args.agent_runtime == "independent":
        if not 0 <= args.dashboard_port <= 65535:
            parser.error("--dashboard-port must be 0 (disabled) or a valid TCP port")
        if args.control_mode not in ("decentralized", "hybrid"):
            parser.error("--agent-runtime independent supports decentralized or hybrid control")
        if args.auto_recover:
            parser.error("--auto-recover is not implemented for independent UDP nodes yet")
        for raw_obstacle in args.obstacle:
            try:
                values = tuple(float(value) for value in raw_obstacle.split(","))
            except (TypeError, ValueError):
                parser.error("--obstacle must be north,east,radius, for example 5,0,2")
            if (len(values) != 3 or not all(math.isfinite(value) for value in values)
                    or values[2] <= 0):
                parser.error("--obstacle must contain finite north,east and positive radius")
        if args.external_agents and (args.peer_roster is None or args.peer_auth_key_file is None):
            parser.error("--external-agents requires --peer-roster and --peer-auth-key-file")
        if args.external_agents and (args.peer_loss or args.peer_latency_ms != 30.0 or
                                     args.peer_burst_loss or args.peer_seed is not None or
                                     args.peer_block or args.peer_duplicate_rate or
                                     args.peer_reorder_rate or args.peer_jitter_ms is not None or
                                     args.sitl_gcs_failsafe or
                                     args.sitl_sensor_profile != "baseline"):
            parser.error("network fault injection is configured on external agents, not the operator console")
        if args.peer_block:
            try:
                links = [tuple(int(value) for value in link.split(":"))
                         for link in args.peer_block.split(",")]
            except (TypeError, ValueError):
                parser.error("--peer-block must contain directed links like 2:4,3:5 (1-indexed IDs)")
            if any(len(link) != 2 or not (1 <= link[0] <= args.drones and
                                           1 <= link[1] <= args.drones) or link[0] == link[1]
                   for link in links):
                parser.error("--peer-block IDs must be different valid drone IDs")
        from sitl.independent_swarm_console import run_independent_console
        run_independent_console(args)
        return
    parser.error("only the independent runtime is supported (legacy runtimes: archive/legacy_runtimes/)")


if __name__ == "__main__":
    main()
