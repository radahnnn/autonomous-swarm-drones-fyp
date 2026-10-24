"""Exercise the external-operator path against five real ArduPilot SITL agents.

SITL vehicles, per-aircraft agent processes, and the operator console are launched as
separate process groups. Agents use distinct loopback addresses to exercise the same
static-roster routing code used by the documented multi-host setup.
"""

import argparse
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import pty
import socket
import statistics
import subprocess
import sys
import time

import numpy as np

from experiments.validate_independent_operator_loss import (
    ConsolePty,
    start_standalone_sitl,
    terminate_pids,
)
from sitl.fleet import build_fleet
from sitl.formation_library import slots_for
from sitl.operator_udp import decode_message


ROOT = Path(__file__).resolve().parents[1]


def read_events(path: Path) -> list[dict]:
    events = []
    if not path.exists():
        return events
    for line in path.read_text(encoding="utf-8").splitlines():
        try:
            events.append(json.loads(line))
        except json.JSONDecodeError:
            continue
    return events


def peer_impairment_cli_args(args) -> list[str]:
    """Build the identical deterministic peer-network profile for each agent process."""
    result = [
        "--peer-loss", str(args.peer_loss),
        "--peer-latency-ms", str(args.peer_latency_ms),
        "--peer-jitter-ms", str(args.peer_jitter_ms),
        "--peer-duplicate-rate", str(args.peer_duplicate_rate),
        "--peer-reorder-rate", str(args.peer_reorder_rate),
        "--peer-reorder-delay-ms", str(args.peer_reorder_delay_ms),
        "--peer-burst-start-rate", str(args.peer_burst_start_rate),
        "--peer-burst-continue-rate", str(args.peer_burst_continue_rate),
    ]
    if args.peer_burst_loss:
        result.append("--peer-burst-loss")
    return result


def latest_statuses(events: list[dict]) -> dict[int, dict]:
    latest = {}
    for event in events:
        if event.get("kind") == "agent_status":
            latest[int(event["drone_id"])] = event
    return latest


def minimum_time_aligned_separation(events: list[dict], count: int = 5,
                                    max_time_delta_s: float = 0.25) -> dict:
    """Measure minimum horizontal spacing from near-simultaneous airborne statuses."""
    if not math.isfinite(max_time_delta_s) or max_time_delta_s <= 0:
        raise ValueError("max_time_delta_s must be finite and positive")
    series = {index: [] for index in range(count)}
    for event in events:
        if event.get("kind") != "agent_status" or event.get("airborne") is not True:
            continue
        try:
            drone_id = int(event["drone_id"])
            timestamp = float(event["timestamp"])
            position = event["position"]
            north, east = float(position[0]), float(position[1])
        except (KeyError, TypeError, ValueError, IndexError):
            continue
        if (drone_id not in series or
                not all(math.isfinite(value) for value in (timestamp, north, east))):
            continue
        series[drone_id].append((timestamp, north, east))

    aligned_minima = []
    max_alignment_error = 0.0
    for anchor_time, _, _ in series.get(0, []):
        selected = []
        alignment_errors = []
        for drone_id in range(count):
            if not series[drone_id]:
                break
            nearest = min(series[drone_id], key=lambda row: abs(row[0] - anchor_time))
            delta = abs(nearest[0] - anchor_time)
            if delta > max_time_delta_s:
                break
            selected.append(nearest)
            alignment_errors.append(delta)
        if len(selected) != count:
            continue
        pair_distances = [
            math.hypot(first[1] - second[1], first[2] - second[2])
            for index, first in enumerate(selected)
            for second in selected[index + 1:]
        ]
        if pair_distances:
            aligned_minima.append(min(pair_distances))
            max_alignment_error = max(max_alignment_error, *alignment_errors)

    return {
        "minimum_horizontal_separation_m": min(aligned_minima) if aligned_minima else None,
        "time_aligned_samples": len(aligned_minima),
        "max_alignment_error_s": max_alignment_error if aligned_minima else None,
        "max_time_delta_s": max_time_delta_s,
    }


def formation_tracking_metrics(events: list[dict], count: int = 5,
                               max_time_delta_s: float = 0.25,
                               altitude_tolerance_m: float = 0.5) -> dict:
    """Compare time-aligned global NED positions with each agent's mission plus its V slot.

    Status positions are already converted into the common WGS84 tangent-plane frame by each
    agent. Commanded landing samples are excluded. This is sampled tracking error, not a continuous
    trajectory or collision bound.
    """
    if count < 1:
        raise ValueError("count must be positive")
    if not math.isfinite(max_time_delta_s) or max_time_delta_s <= 0:
        raise ValueError("max_time_delta_s must be finite and positive")
    if not math.isfinite(altitude_tolerance_m) or altitude_tolerance_m <= 0:
        raise ValueError("altitude_tolerance_m must be finite and positive")

    series = {index: {} for index in range(count)}
    for event in events:
        command = event.get("command", "")
        if (event.get("kind") != "agent_status" or event.get("airborne") is not True or
                event.get("mission_active") is not True or
                (isinstance(command, str) and command.startswith("landing"))):
            continue
        try:
            drone_id = int(event["drone_id"])
            timestamp = float(event["timestamp"])
            position = np.asarray(event["position"], dtype=float)
            intent = event["mission_intent"]
            goal = np.asarray(intent["goal_ne"], dtype=float)
            altitude = float(intent["target_alt_m"])
            formation = str(intent["formation"])
        except (KeyError, TypeError, ValueError, IndexError):
            continue
        if (drone_id not in series or position.shape != (3,) or goal.shape != (2,) or
                not np.isfinite(position).all() or not np.isfinite(goal).all() or
                not math.isfinite(timestamp) or not math.isfinite(altitude)):
            continue
        mission_key = (tuple(goal.tolist()), formation, altitude)
        series[drone_id].setdefault(mission_key, []).append(
            (timestamp, position.tolist()),
        )

    mission_keys = sorted(
        set.intersection(*(set(agent_rows) for agent_rows in series.values()))
        if count else set(), key=repr,
    )
    mission_results = []
    for mission_key in mission_keys:
        goal_tuple, formation, target_altitude = mission_key
        goal = np.asarray(goal_tuple, dtype=float)
        slots = slots_for(formation, count)
        if slots.shape != (count, 3):
            continue
        anchors = series[0][mission_key]
        per_agent_errors = {index: [] for index in range(count)}
        steady_altitude_errors = {index: [] for index in range(count)}
        formation_errors = []
        steady_formation_errors = []
        max_alignment_error = 0.0
        steady_max_alignment_error = 0.0
        for anchor_time, _ in anchors:
            selected = []
            alignment_errors = []
            for drone_id in range(count):
                samples = series[drone_id][mission_key]
                nearest = min(samples, key=lambda row: abs(row[0] - anchor_time))
                delta = abs(nearest[0] - anchor_time)
                if delta > max_time_delta_s:
                    break
                selected.append(np.asarray(nearest[1], dtype=float))
                alignment_errors.append(delta)
            if len(selected) != count:
                continue

            sample_errors = []
            for drone_id, position in enumerate(selected):
                target = np.array([
                    goal[0] + slots[drone_id, 0],
                    goal[1] + slots[drone_id, 1],
                    -target_altitude + slots[drone_id, 2],
                ])
                error = position - target
                row = {
                    "horizontal_m": float(np.linalg.norm(error[:2])),
                    "vertical_m": float(error[2]),
                    "three_dimensional_m": float(np.linalg.norm(error)),
                }
                per_agent_errors[drone_id].append(row)
                sample_errors.append(row)
            for first in range(count):
                for second in range(first + 1, count):
                    actual_delta = selected[second][:2] - selected[first][:2]
                    expected_delta = slots[second, :2] - slots[first, :2]
                    formation_errors.append(
                        float(np.linalg.norm(actual_delta - expected_delta))
                    )
            max_alignment_error = max(max_alignment_error, *alignment_errors)
            if all(abs(row["vertical_m"]) <= altitude_tolerance_m
                   for row in sample_errors):
                for drone_id, row in enumerate(sample_errors):
                    steady_altitude_errors[drone_id].append(row)
                steady_formation_errors.extend(
                    float(np.linalg.norm(
                        (selected[second][:2] - selected[first][:2]) -
                        (slots[second, :2] - slots[first, :2])
                    ))
                    for first in range(count)
                    for second in range(first + 1, count)
                )
                steady_max_alignment_error = max(
                    steady_max_alignment_error, *alignment_errors,
                )

        sample_count = len(per_agent_errors[0])
        if sample_count == 0:
            continue

        def summarize(values: list[float]) -> dict:
            ordered = sorted(values)
            p95_index = min(len(ordered) - 1, math.ceil(0.95 * len(ordered)) - 1)
            return {
                "mean_m": statistics.fmean(ordered),
                "rmse_m": math.sqrt(statistics.fmean(value * value for value in ordered)),
                "p95_m": ordered[p95_index],
                "max_m": ordered[-1],
            }

        all_horizontal = [
            row["horizontal_m"] for rows in per_agent_errors.values() for row in rows
        ]
        all_vertical = [
            row["vertical_m"] for rows in per_agent_errors.values() for row in rows
        ]
        all_3d = [
            row["three_dimensional_m"] for rows in per_agent_errors.values() for row in rows
        ]
        steady_sample_count = len(steady_altitude_errors[0])
        steady_horizontal = [
            row["horizontal_m"]
            for rows in steady_altitude_errors.values() for row in rows
        ]
        steady_vertical = [
            row["vertical_m"]
            for rows in steady_altitude_errors.values() for row in rows
        ]
        steady_3d = [
            row["three_dimensional_m"]
            for rows in steady_altitude_errors.values() for row in rows
        ]
        mission_results.append({
            "goal_ne_m": goal.tolist(),
            "formation": formation,
            "target_altitude_m": target_altitude,
            "aligned_samples": sample_count,
            "max_alignment_error_s": max_alignment_error,
            "fleet_horizontal_error": summarize(all_horizontal),
            "fleet_vertical_signed_error": summarize(all_vertical),
            "fleet_3d_error": summarize(all_3d),
            "per_agent": {
                str(index + 1): {
                    "horizontal_error": summarize([
                        row["horizontal_m"] for row in rows
                    ]),
                    "vertical_signed_error": summarize([
                        row["vertical_m"] for row in rows
                    ]),
                    "three_dimensional_error": summarize([
                        row["three_dimensional_m"] for row in rows
                    ]),
                }
                for index, rows in per_agent_errors.items()
            },
            "formation_pairwise_offset_error": summarize(formation_errors),
            "within_altitude_band": {
                "tolerance_m": altitude_tolerance_m,
                "aligned_samples": steady_sample_count,
                "max_alignment_error_s": (steady_max_alignment_error
                                          if steady_sample_count else None),
                "fleet_horizontal_error": summarize(steady_horizontal)
                                          if steady_horizontal else None,
                "fleet_vertical_signed_error": summarize(steady_vertical)
                                                 if steady_vertical else None,
                "fleet_3d_error": summarize(steady_3d) if steady_3d else None,
                "per_agent_horizontal_error": {
                    str(index + 1): summarize([
                        row["horizontal_m"] for row in rows
                    ])
                    for index, rows in steady_altitude_errors.items() if rows
                },
                "formation_pairwise_offset_error": (
                    summarize(steady_formation_errors)
                    if steady_formation_errors else None
                ),
            },
        })
    return {
        "coordinate_frame": "shared_global_ned_from_common_wgs84_datum",
        "metric_method": "time-aligned agent positions versus shared mission goal plus deterministic configured slot",
        "max_time_delta_s": max_time_delta_s,
        "missions": mission_results,
    }


def mission_convergence_state(events: list[dict], count: int = 5,
                              goal_ne: tuple[float, float] | None = None,
                              max_time_delta_s: float = 0.25,
                              horizontal_tolerance_m: float = 0.5,
                              vertical_tolerance_m: float = 0.5,
                              dwell_samples: int = 3,
                              min_sample_interval_s: float = 0.5,
                              max_sample_gap_s: float = 1.75) -> dict:
    """Check that every airborne agent holds its mission-plus-slot target for a dwell window."""
    if count < 1 or dwell_samples < 1:
        raise ValueError("count and dwell_samples must be positive")
    for name, value in (("max_time_delta_s", max_time_delta_s),
                        ("horizontal_tolerance_m", horizontal_tolerance_m),
                        ("vertical_tolerance_m", vertical_tolerance_m),
                        ("min_sample_interval_s", min_sample_interval_s),
                        ("max_sample_gap_s", max_sample_gap_s)):
        if not math.isfinite(value) or value <= 0:
            raise ValueError(f"{name} must be finite and positive")
    if goal_ne is not None:
        goal_ne = tuple(float(value) for value in goal_ne)
        if len(goal_ne) != 2 or not all(math.isfinite(value) for value in goal_ne):
            raise ValueError("goal_ne must contain two finite coordinates")

    series = {index: {} for index in range(count)}
    for event in events:
        command = event.get("command", "")
        if (event.get("kind") != "agent_status" or event.get("airborne") is not True or
                event.get("mission_active") is not True or
                (isinstance(command, str) and command.startswith("landing"))):
            continue
        try:
            drone_id = int(event["drone_id"])
            timestamp = float(event["timestamp"])
            position = np.asarray(event["position"], dtype=float)
            intent = event["mission_intent"]
            goal = tuple(float(value) for value in intent["goal_ne"])
            formation = str(intent["formation"])
            target_altitude = float(intent["target_alt_m"])
        except (KeyError, TypeError, ValueError, IndexError):
            continue
        if (drone_id not in series or position.shape != (3,) or len(goal) != 2 or
                not np.isfinite(position).all() or not math.isfinite(timestamp) or
                not all(math.isfinite(value) for value in goal) or
                not math.isfinite(target_altitude) or
                (goal_ne is not None and goal != goal_ne)):
            continue
        mission_key = (goal, formation, target_altitude)
        series[drone_id].setdefault(mission_key, []).append(
            (timestamp, position.tolist()),
        )

    common_missions = set.intersection(*(set(rows) for rows in series.values()))
    if goal_ne is not None:
        common_missions = {key for key in common_missions if key[0] == goal_ne}
    if not common_missions:
        return {
            "converged": False, "reason": "no_common_active_mission_status",
            "goal_ne_m": list(goal_ne) if goal_ne is not None else None,
            "aligned_samples": 0, "current_dwell_samples": 0,
            "required_dwell_samples": dwell_samples,
        }

    mission_key = max(
        common_missions,
        key=lambda key: max(row[0] for row in series[0][key]),
    )
    goal, formation, target_altitude = mission_key
    slots = slots_for(formation, count)
    aligned = []
    last_anchor_time = None
    for anchor_time, _ in sorted(series[0][mission_key]):
        if (last_anchor_time is not None and
                anchor_time - last_anchor_time < min_sample_interval_s):
            continue
        last_anchor_time = anchor_time
        selected = []
        alignment_errors = []
        for drone_id in range(count):
            samples = series[drone_id][mission_key]
            nearest = min(samples, key=lambda row: abs(row[0] - anchor_time))
            delta = abs(nearest[0] - anchor_time)
            if delta > max_time_delta_s:
                break
            selected.append(np.asarray(nearest[1], dtype=float))
            alignment_errors.append(delta)
        if len(selected) != count:
            continue
        errors = {}
        for drone_id, position in enumerate(selected):
            target = np.array([
                goal[0] + slots[drone_id, 0],
                goal[1] + slots[drone_id, 1],
                -target_altitude + slots[drone_id, 2],
            ])
            error = position - target
            errors[str(drone_id + 1)] = {
                "horizontal_error_m": float(np.linalg.norm(error[:2])),
                "vertical_error_m": float(error[2]),
            }
        within_tolerance = all(
            row["horizontal_error_m"] <= horizontal_tolerance_m and
            abs(row["vertical_error_m"]) <= vertical_tolerance_m
            for row in errors.values()
        )
        aligned.append({
            "timestamp": anchor_time,
            "max_alignment_error_s": max(alignment_errors),
            "within_tolerance": within_tolerance,
            "per_agent_errors": errors,
        })

    current_streak = 0
    previous_time = None
    best_streak = 0
    for sample in aligned:
        if (previous_time is not None and
                sample["timestamp"] - previous_time > max_sample_gap_s):
            current_streak = 0
        if sample["within_tolerance"]:
            current_streak += 1
        else:
            current_streak = 0
        best_streak = max(best_streak, current_streak)
        previous_time = sample["timestamp"]
    latest = aligned[-1] if aligned else None
    current_streak = current_streak if latest and latest["within_tolerance"] else 0
    return {
        "converged": current_streak >= dwell_samples,
        "goal_ne_m": list(goal),
        "formation": formation,
        "target_altitude_m": target_altitude,
        "horizontal_tolerance_m": horizontal_tolerance_m,
        "vertical_tolerance_m": vertical_tolerance_m,
        "required_dwell_samples": dwell_samples,
        "min_sample_interval_s": min_sample_interval_s,
        "max_sample_gap_s": max_sample_gap_s,
        "max_time_delta_s": max_time_delta_s,
        "aligned_samples": len(aligned),
        "best_dwell_samples": best_streak,
        "current_dwell_samples": current_streak,
        "max_alignment_error_s": (latest["max_alignment_error_s"] if latest else None),
        "latest_timestamp": latest["timestamp"] if latest else None,
        "latest_per_agent_errors": latest["per_agent_errors"] if latest else {},
        "reason": "all_agents_within_tolerance_for_dwell"
                  if current_streak >= dwell_samples else "mission_not_yet_converged",
    }


def landed(status: dict) -> bool:
    position = status.get("position")
    try:
        down = float(position[2])
    except (TypeError, ValueError, IndexError):
        return False
    return bool(status.get("command") == "landed" and status.get("armed") is False
                and status.get("airborne") is False and down > -0.5)


def observe_operator_loss(operator: subprocess.Popen, terminal: ConsolePty,
                          rows: list[dict], auth_key: bytes, run_dir: Path,
                          timeout_s: float) -> dict[int, dict]:
    """Kill only the operator, then confirm every agent lease expires into LAND."""
    operator.kill()
    operator.wait(timeout=5.0)
    terminal.close()
    sockets = []
    try:
        for row in rows:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.bind((row["status_host"], int(row["status_port"])))
            sock.setblocking(False)
            sockets.append(sock)

        status_path = run_dir / "operator_loss_status.jsonl"
        latest = {}
        airborne_seen = set()
        expiry_land_seen = set()
        landed_seen = set()
        started = time.monotonic()
        with status_path.open("w", encoding="utf-8") as out:
            while time.monotonic() - started < timeout_s:
                for drone_id, sock in enumerate(sockets):
                    while True:
                        try:
                            raw, _ = sock.recvfrom(8192)
                        except BlockingIOError:
                            break
                        status = decode_message(raw, auth_key)
                        if (status is None or status.get("type") != "status" or
                                status.get("drone_id") != drone_id):
                            continue
                        latest[drone_id] = status
                        if status.get("airborne") is True:
                            airborne_seen.add(drone_id)
                        if status.get("command") == "landing:mission_expired":
                            expiry_land_seen.add(drone_id)
                        if drone_id in airborne_seen and landed(status):
                            landed_seen.add(drone_id)
                        out.write(json.dumps({
                            "utc": datetime.now(timezone.utc).isoformat(),
                            "kind": "agent_status_after_operator_loss", **status,
                        }, allow_nan=False, separators=(",", ":")) + "\n")
                        out.flush()
                if (len(airborne_seen) == len(rows) and len(expiry_land_seen) == len(rows)
                        and len(landed_seen) == len(rows)):
                    break
                time.sleep(0.05)
            result = {
                "operator_process_killed": operator.poll() is not None,
                "all_agents_were_airborne": len(airborne_seen) == len(rows),
                "all_agents_requested_mission_expiry_land": len(expiry_land_seen) == len(rows),
                "all_agents_landed_disarmed": len(landed_seen) == len(rows),
                "airborne_ids": sorted(airborne_seen),
                "mission_expiry_land_ids": sorted(expiry_land_seen),
                "landed_ids": sorted(landed_seen),
                "seconds_after_operator_loss": round(time.monotonic() - started, 2),
                "status_log": str(status_path),
            }
            out.write(json.dumps({
                "utc": datetime.now(timezone.utc).isoformat(),
                "kind": "operator_loss_result", **result,
            }, separators=(",", ":")) + "\n")
            out.flush()
        return latest, result
    finally:
        for sock in sockets:
            sock.close()


def wait_for_events(terminal: ConsolePty, event_path: Path, predicate,
                    timeout_s: float, description: str) -> list[dict]:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        events = read_events(event_path)
        if predicate(events):
            return events
        if terminal.process.poll() is not None:
            raise RuntimeError(f"external operator exited before {description}")
        terminal.read(0.15)
        time.sleep(0.05)
    raise TimeoutError(f"timed out waiting for {description}")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seed", type=int, default=121)
    parser.add_argument("--timeout", type=float, default=180.0)
    parser.add_argument("--convergence-timeout", type=float, default=90.0,
                        help="seconds allowed for all agents to settle at the shared goal")
    parser.add_argument("--horizontal-tolerance-m", type=float, default=0.5)
    parser.add_argument("--altitude-tolerance-m", type=float, default=0.5)
    parser.add_argument("--convergence-dwell-samples", type=int, default=3)
    parser.add_argument("--topology", choices=("distinct-loopback", "split-loopback"),
                        default="distinct-loopback",
                        help="use one loopback alias per agent or emulate the two-host ID split with .1/.2 aliases")
    parser.add_argument("--peer-loss", type=float, default=0.02)
    parser.add_argument("--peer-latency-ms", type=float, default=25.0)
    parser.add_argument("--peer-jitter-ms", type=float, default=5.0,
                        help="standard deviation of one-hop peer delay in milliseconds")
    parser.add_argument("--peer-burst-loss", action="store_true",
                        help="use bursty loss instead of independent random loss")
    parser.add_argument("--peer-burst-start-rate", type=float, default=0.04,
                        help="probability of entering a packet-loss burst")
    parser.add_argument("--peer-burst-continue-rate", type=float, default=0.65,
                        help="probability that a packet-loss burst continues")
    parser.add_argument("--peer-duplicate-rate", type=float, default=0.0,
                        help="probability of duplicating each peer datagram")
    parser.add_argument("--peer-reorder-rate", type=float, default=0.0,
                        help="probability of adding variable delay that can reorder peer packets")
    parser.add_argument("--peer-reorder-delay-ms", type=float, default=150.0,
                        help="extra delay scale for packets selected for reordering")
    parser.add_argument("--operator-loss", action="store_true",
                        help="kill the external operator after the mission/fault test and verify autonomous lease-expiry landing")
    parser.add_argument("--operator-restart", action="store_true",
                        help="replace the operator during flight and verify it resumes the agents' shared mission")
    parser.add_argument("--operator-loss-timeout", type=float, default=75.0,
                        help="maximum seconds to wait for all agents to LAND after operator loss")
    parser.add_argument("--test-launch-blocked-peer", default=None,
                        help="before launch, cut one required directed peer link (1-based sender:recipient) and verify the swarm-wide gate blocks takeoff")
    args = parser.parse_args()
    if args.operator_loss and args.operator_restart:
        parser.error("--operator-loss and --operator-restart are mutually exclusive")
    if (not math.isfinite(args.timeout) or args.timeout <= 0 or
            not math.isfinite(args.convergence_timeout) or args.convergence_timeout <= 0 or
            not math.isfinite(args.horizontal_tolerance_m) or args.horizontal_tolerance_m <= 0 or
            not math.isfinite(args.altitude_tolerance_m) or args.altitude_tolerance_m <= 0 or
            args.convergence_dwell_samples < 1 or
            not math.isfinite(args.operator_loss_timeout) or args.operator_loss_timeout <= 0 or
            not math.isfinite(args.peer_loss) or not 0.0 <= args.peer_loss <= 1.0 or
            not math.isfinite(args.peer_latency_ms) or args.peer_latency_ms < 0 or
            not math.isfinite(args.peer_jitter_ms) or args.peer_jitter_ms < 0 or
            not math.isfinite(args.peer_duplicate_rate) or
            not 0.0 <= args.peer_duplicate_rate <= 1.0 or
            not math.isfinite(args.peer_burst_start_rate) or
            not 0.0 <= args.peer_burst_start_rate <= 1.0 or
            not math.isfinite(args.peer_burst_continue_rate) or
            not 0.0 <= args.peer_burst_continue_rate <= 1.0 or
            not math.isfinite(args.peer_reorder_rate) or
            not 0.0 <= args.peer_reorder_rate <= 1.0 or
            not math.isfinite(args.peer_reorder_delay_ms) or
            args.peer_reorder_delay_ms < 0):
        parser.error("timeout must be positive; loss, latency, and jitter must be finite and non-negative (loss <= 1)")
    if args.test_launch_blocked_peer is not None:
        try:
            block_sender, block_recipient = (
                int(value) for value in args.test_launch_blocked_peer.split(":")
            )
        except (TypeError, ValueError):
            parser.error("--test-launch-blocked-peer must be a directed edge like 1:2")
        if (not 1 <= block_sender <= 5 or not 1 <= block_recipient <= 5 or
                block_sender == block_recipient):
            parser.error("--test-launch-blocked-peer IDs must be different values from 1 to 5")

    run_dir = ROOT / "runs" / "independent" / (
        "external-sitl-" + datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S.%fZ")
    )
    run_dir.mkdir(parents=True, exist_ok=False)
    roster_path = run_dir / "peer_roster.json"
    key_path = run_dir / "operator-peer.key"
    key_path.write_bytes(os.urandom(32))
    key_path.chmod(0o600)

    base = 30000 + (os.getpid() % 10000)
    agents = []
    agent_logs = []
    sitl_processes = []
    operator = None
    terminal = None
    final_status_override = None
    operator_loss_statuses = None
    operator_loss_result = None
    result = {
        "kind": "external_agent_sitl_validation",
        "seed": args.seed,
        "peer_loss": args.peer_loss,
        "peer_latency_ms": args.peer_latency_ms,
        "peer_jitter_ms": args.peer_jitter_ms,
        "peer_burst_loss": args.peer_burst_loss,
        "peer_burst_start_rate": args.peer_burst_start_rate,
        "peer_burst_continue_rate": args.peer_burst_continue_rate,
        "peer_duplicate_rate": args.peer_duplicate_rate,
        "peer_reorder_rate": args.peer_reorder_rate,
        "peer_reorder_delay_ms": args.peer_reorder_delay_ms,
        "topology": args.topology,
        "started_utc": datetime.now(timezone.utc).isoformat(),
        "passed": False,
    }

    try:
        rows = []
        for agent_id in range(5):
            if args.topology == "split-loopback":
                # Emulate the documented split: D1/D2 on host A, D3/D4/D5 on B.
                # Linux loopback routing still means this is not a real LAN test.
                address = "127.0.0.1" if agent_id < 2 else "127.0.0.2"
            else:
                address = f"127.0.0.{agent_id + 2}"
            rows.append({
                "id": agent_id,
                "bind_host": address,
                "host": address,
                "port": base + agent_id,
                "command_bind_host": address,
                "command_host": address,
                "command_port": base + 100 + agent_id,
                "status_host": "127.0.0.1",
                "status_port": base + 200 + agent_id,
            })
        roster_path.write_text(json.dumps({"version": 1, "agents": rows}, indent=2) + "\n",
                               encoding="utf-8")
        result["agent_address_groups"] = {
            "host_a_ids": [0, 1],
            "host_b_ids": [2, 3, 4],
            "expected_cross_host_peer_edges": [[0, 2], [1, 3]],
        } if args.topology == "split-loopback" else None

        sitl_processes = start_standalone_sitl(ROOT, 5)
        result["sitl_pids"] = [process.pid for process in sitl_processes]
        for spec in build_fleet(5):
            command = [
                sys.executable, "-u", "-m", "sitl.independent_agent_node",
                "--id", str(spec.index), "--count", "5",
                "--endpoint", f"tcp:127.0.0.1:{spec.tcp_port}",
                "--formation", "v", "--peer-roster", str(roster_path),
                "--peer-auth-key-file", str(key_path),
                *peer_impairment_cli_args(args),
                "--peer-seed", str(args.seed + spec.index),
            ]
            log = (run_dir / f"agent-{spec.index + 1}.log").open("w", encoding="utf-8")
            agent_logs.append(log)
            agents.append(subprocess.Popen(
                command, cwd=ROOT, env={**os.environ, "PYTHONPATH": str(ROOT)},
                stdout=log, stderr=subprocess.STDOUT, start_new_session=True,
            ))

        operator_command = [
            sys.executable, "-u", "-m", "sitl.swarm_console",
            "--drones", "5", "--standalone", "--control-mode", "decentralized",
            "--agent-runtime", "independent", "--external-agents",
            "--operator-bind-host", "127.0.0.1", "--peer-roster", str(roster_path),
            "--peer-auth-key-file", str(key_path),
            "--evidence-dir", str(run_dir / "operator-evidence"),
        ]
        master, slave = pty.openpty()
        operator = subprocess.Popen(
            operator_command, cwd=ROOT, stdin=slave, stdout=slave, stderr=slave,
            env={**os.environ, "PYTHONPATH": str(ROOT)}, start_new_session=True,
        )
        os.close(slave)
        terminal = ConsolePty(operator, master, run_dir / "operator-console.log")
        terminal.wait_for(lambda text: "Independent Swarm > " in text, 420.0,
                          "all five real SITL agents becoming ready")
        match = None
        for line in terminal.text.splitlines():
            if ">> Independent-node evidence:" in line:
                match = line.split(">> Independent-node evidence:", 1)[1].strip()
                break
        if not match:
            raise RuntimeError("operator did not report its evidence directory")
        event_path = Path(match) / "events.jsonl"
        result["operator_run_dir"] = match
        result["agent_pids"] = [process.pid for process in agents]
        result["operator_pid"] = operator.pid
        (run_dir / "processes.json").write_text(json.dumps({
            "sitl_pids": result["sitl_pids"], "agent_pids": result["agent_pids"],
            "operator_pid": operator.pid,
        }, indent=2) + "\n", encoding="utf-8")

        if args.test_launch_blocked_peer is not None:
            sender, recipient = block_sender - 1, block_recipient - 1
            result["launch_preflight_fault_test"] = {
                "sender": sender, "recipient": recipient,
            }
            terminal.send(f"link-block {block_sender} {block_recipient}")
            events = wait_for_events(
                terminal, event_path,
                lambda rows: any(row.get("kind") == "peer_link_fault"
                                 and row.get("sender") == sender
                                 and row.get("recipient") == recipient
                                 and row.get("blocked") is True for row in rows),
                5.0, "preflight test directed peer cut",
            )
            events = wait_for_events(
                terminal, event_path,
                lambda rows: (
                    (state := latest_statuses(rows).get(recipient)) is not None
                    and sender not in state.get("peer_live_ids", [])
                ),
                8.0, "the recipient reporting the missing peer heartbeat",
            )
            terminal.send("launch")
            events = wait_for_events(
                terminal, event_path,
                lambda rows: any(
                    row.get("kind") == "launch_preflight" and row.get("passed") is False
                    and any(agent.get("drone_id") == recipient and
                            sender in agent.get("missing_peer_ids", [])
                            for agent in row.get("agents", []))
                    for row in rows
                ),
                8.0, "the operator blocking launch for the missing peer",
            )
            states = latest_statuses(events)
            result["launch_preflight_fault_test"]["launch_blocked"] = True
            result["launch_preflight_fault_test"]["all_vehicles_stayed_grounded"] = all(
                states.get(index, {}).get("airborne") is False
                and states.get(index, {}).get("armed") is False for index in range(5)
            )
            if not result["launch_preflight_fault_test"]["all_vehicles_stayed_grounded"]:
                raise RuntimeError("a vehicle armed or left the ground despite failed swarm preflight")

            terminal.send(f"link-restore {block_sender} {block_recipient}")
            events = wait_for_events(
                terminal, event_path,
                lambda rows: any(row.get("kind") == "peer_link_fault"
                                 and row.get("sender") == sender
                                 and row.get("recipient") == recipient
                                 and row.get("blocked") is False for row in rows),
                5.0, "preflight test directed peer restoration",
            )
            events = wait_for_events(
                terminal, event_path,
                lambda rows: all(
                    (state := latest_statuses(rows).get(index)) is not None
                    and set(state.get("peer_ids", [])).issubset(
                        set(state.get("peer_live_ids", []))
                    ) for index in range(5)
                ),
                8.0, "all configured peer heartbeats returning",
            )
            terminal.send("preflight")
            events = wait_for_events(
                terminal, event_path,
                lambda rows: any(row.get("kind") == "network_preflight"
                                 and row.get("passed") is True for row in rows),
                5.0, "swarm preflight passing after peer restoration",
            )
            result["launch_preflight_fault_test"]["restored_preflight_passed"] = True

        terminal.send("launch")
        events = wait_for_events(
            terminal, event_path,
            lambda rows: len(latest_statuses(rows)) == 5 and all(
                latest_statuses(rows)[index].get("airborne") is True
                and latest_statuses(rows)[index].get("armed") is True
                for index in range(5)
            ), 180.0, "all five SITL vehicles airborne",
        )
        result["all_five_airborne"] = True
        launch_states = latest_statuses(events)
        leader_north_before = float(launch_states[0]["position"][0])
        result["leader_north_before_move_m"] = leader_north_before

        terminal.send("w 2")
        events = wait_for_events(
            terminal, event_path,
            lambda rows: any(row.get("kind") == "mission_intent"
                             and row.get("goal_ne") == [2.0, 0.0] for row in rows),
            10.0, "the two-metre shared north mission intent",
        )
        result["shared_north_goal_accepted"] = True
        events = wait_for_events(
            terminal, event_path,
            lambda rows: (
                (state := latest_statuses(rows).get(0)) is not None
                and state.get("airborne") is True
                and state.get("armed") is True
                and isinstance(state.get("position"), list)
                and float(state["position"][0]) >= leader_north_before + 1.5
            ), 30.0, "the leader flying at least 1.5 m toward the 2 m north goal",
        )
        leader_after_move = latest_statuses(events)[0]
        result["leader_north_after_move_m"] = float(leader_after_move["position"][0])
        result["leader_north_progress_m"] = (
            result["leader_north_after_move_m"] - leader_north_before
        )

        result["mission_convergence_config"] = {
            "horizontal_tolerance_m": args.horizontal_tolerance_m,
            "altitude_tolerance_m": args.altitude_tolerance_m,
            "dwell_samples": args.convergence_dwell_samples,
            "min_sample_interval_s": 0.5,
            "max_sample_gap_s": 1.75,
            "max_time_delta_s": 0.25,
            "timeout_s": args.convergence_timeout,
        }
        events = wait_for_events(
            terminal, event_path,
            lambda rows: mission_convergence_state(
                rows, count=5, goal_ne=(2.0, 0.0),
                horizontal_tolerance_m=args.horizontal_tolerance_m,
                vertical_tolerance_m=args.altitude_tolerance_m,
                dwell_samples=args.convergence_dwell_samples,
            )["converged"],
            args.convergence_timeout,
            "all five agents holding their mission-plus-slot targets for the dwell window",
        )
        result["mission_convergence"] = mission_convergence_state(
            events, count=5, goal_ne=(2.0, 0.0),
            horizontal_tolerance_m=args.horizontal_tolerance_m,
            vertical_tolerance_m=args.altitude_tolerance_m,
            dwell_samples=args.convergence_dwell_samples,
        )

        terminal.send("link-block 1 2")
        events = wait_for_events(
            terminal, event_path,
            lambda rows: any(row.get("kind") == "peer_link_fault"
                             and row.get("sender") == 0 and row.get("recipient") == 1
                             and row.get("blocked") is True for row in rows),
            5.0, "directed D1-to-D2 peer cut",
        )
        events = wait_for_events(
            terminal, event_path,
            lambda rows: latest_statuses(rows).get(0, {}).get("peer_metrics", {}).get(
                "dropped_blocked", 0) > 0,
            5.0, "the sender recording blocked peer datagrams",
        )
        result["directed_cut_observed"] = True
        terminal.send("link-restore 1 2")
        events = wait_for_events(
            terminal, event_path,
            lambda rows: any(row.get("kind") == "peer_link_fault"
                             and row.get("sender") == 0 and row.get("recipient") == 1
                             and row.get("blocked") is False for row in rows),
            5.0, "directed D1-to-D2 peer restoration",
        )
        result["directed_link_restored"] = True
        events = wait_for_events(
            terminal, event_path,
            lambda rows: all(
                latest_statuses(rows).get(index, {}).get("safety_state") == "nominal"
                and latest_statuses(rows).get(index, {}).get("airborne") is True
                for index in range(5)
            ), 8.0, "all agents remaining airborne with nominal safety after link restoration",
        )
        result["all_agents_nominal_after_restore"] = True

        if args.operator_restart:
            prior_operator_pid = operator.pid
            operator.kill()
            operator.wait(timeout=5.0)
            terminal.close()
            terminal = None
            master, slave = pty.openpty()
            operator = subprocess.Popen(
                operator_command, cwd=ROOT, stdin=slave, stdout=slave, stderr=slave,
                env={**os.environ, "PYTHONPATH": str(ROOT)}, start_new_session=True,
            )
            os.close(slave)
            terminal = ConsolePty(
                operator, master, run_dir / "operator-console-replacement.log",
            )
            terminal.wait_for(lambda text: "Independent Swarm > " in text, 25.0,
                              "replacement operator attaching to live agents")
            replacement_dir = None
            for line in terminal.text.splitlines():
                if ">> Independent-node evidence:" in line:
                    replacement_dir = line.split(">> Independent-node evidence:", 1)[1].strip()
                    break
            if replacement_dir is None:
                raise RuntimeError("replacement operator did not report its evidence directory")
            replacement_event_path = Path(replacement_dir) / "events.jsonl"
            replacement_events = wait_for_events(
                terminal, replacement_event_path,
                lambda rows: any(
                    row.get("kind") == "operator_takeover_mission"
                    and row.get("resumed") is True
                    and row.get("goal_ne") == [2.0, 0.0]
                    for row in rows
                ),
                15.0, "replacement operator restoring the shared north goal",
            )
            result["operator_restart"] = {
                "old_operator_pid": prior_operator_pid,
                "replacement_operator_pid": operator.pid,
                "replacement_run_dir": replacement_dir,
                "resumed_goal_ne": [2.0, 0.0],
                "takeover_recorded": True,
            }
            event_path = replacement_event_path
            terminal.send("q")
            deadline = time.monotonic() + args.timeout
            while time.monotonic() < deadline and operator.poll() is None:
                terminal.read(0.2)
            if operator.poll() is None:
                raise TimeoutError("replacement operator did not confirm landing and exit")
        elif args.operator_loss:
            operator_loss_statuses, operator_loss_result = observe_operator_loss(
                operator, terminal, rows, key_path.read_bytes(), run_dir,
                args.operator_loss_timeout,
            )
            terminal = None
            final_status_override = operator_loss_statuses
            result["operator_loss"] = operator_loss_result
            if not (operator_loss_result["operator_process_killed"] and
                    operator_loss_result["all_agents_were_airborne"] and
                    operator_loss_result["all_agents_requested_mission_expiry_land"] and
                    operator_loss_result["all_agents_landed_disarmed"]):
                raise RuntimeError(
                    f"external agents did not autonomously land after operator loss: "
                    f"{operator_loss_result}"
                )
        else:
            terminal.send("q")
            deadline = time.monotonic() + args.timeout
            while time.monotonic() < deadline and operator.poll() is None:
                terminal.read(0.2)
            if operator.poll() is None:
                raise TimeoutError("external operator did not confirm landing and exit")
        events = read_events(event_path)
        states = latest_statuses(events)
        if final_status_override is not None:
            states = final_status_override
        result["final_statuses"] = {str(index + 1): states.get(index, {})
                                     for index in range(5)}
        result["all_five_landed_disarmed"] = all(
            landed(states.get(index, {})) for index in range(5)
        )
        result["peer_metrics"] = {
            str(index + 1): states.get(index, {}).get("peer_metrics", {})
            for index in range(5)
        }
        result["separation_metrics"] = minimum_time_aligned_separation(events, count=5)
        result["tracking_metrics"] = formation_tracking_metrics(events, count=5)
        result["session_end"] = next(
            (row for row in reversed(events) if row.get("kind") == "session_end"), None,
        )
        result["passed"] = bool(
            result["all_five_airborne"] and result["shared_north_goal_accepted"]
            and result["leader_north_progress_m"] >= 1.5
            and result["directed_cut_observed"] and result["directed_link_restored"]
            and result["all_agents_nominal_after_restore"]
            and result["separation_metrics"]["time_aligned_samples"] > 0
            and result["separation_metrics"]["minimum_horizontal_separation_m"] is not None
            and (not args.operator_loss or (
                result["operator_loss"]["operator_process_killed"]
                and result["operator_loss"]["all_agents_were_airborne"]
                and result["operator_loss"]["all_agents_requested_mission_expiry_land"]
            ))
            and (not args.operator_restart or (
                result.get("operator_restart", {}).get("takeover_recorded") is True
                and result.get("operator_restart", {}).get("resumed_goal_ne") == [2.0, 0.0]
            ))
            and (args.test_launch_blocked_peer is None or (
                result.get("launch_preflight_fault_test", {}).get("launch_blocked") is True
                and result.get("launch_preflight_fault_test", {}).get(
                    "all_vehicles_stayed_grounded"
                ) is True
                and result.get("launch_preflight_fault_test", {}).get(
                    "restored_preflight_passed"
                ) is True
            ))
            and result["all_five_landed_disarmed"]
        )
        result["completed_utc"] = datetime.now(timezone.utc).isoformat()
        (run_dir / "external_agent_sitl_summary.json").write_text(
            json.dumps(result, indent=2) + "\n", encoding="utf-8",
        )
        print(json.dumps({"passed": result["passed"], "run_dir": str(run_dir),
                          "all_five_landed_disarmed": result["all_five_landed_disarmed"],
                          "peer_metrics": result["peer_metrics"]}))
        if not result["passed"]:
            raise RuntimeError("external-agent SITL validation did not meet all acceptance checks")
    except Exception as exc:
        result["error"] = f"{type(exc).__name__}: {exc}"
        result["completed_utc"] = datetime.now(timezone.utc).isoformat()
        (run_dir / "external_agent_sitl_summary.json").write_text(
            json.dumps(result, indent=2) + "\n", encoding="utf-8",
        )
        raise
    finally:
        if terminal is not None:
            if operator is not None and operator.poll() is None:
                try:
                    terminal.send("q")
                    operator.wait(timeout=60.0)
                except (OSError, subprocess.TimeoutExpired):
                    operator.terminate()
                    try:
                        operator.wait(timeout=3.0)
                    except subprocess.TimeoutExpired:
                        operator.kill()
            terminal.close()
        elif operator is not None and operator.poll() is None:
            operator.terminate()
            operator.wait(timeout=3.0)
        preserve_processes = bool(
            args.operator_loss and operator is not None and operator.poll() is not None and
            (operator_loss_result is None or
             not operator_loss_result.get("all_agents_landed_disarmed", False))
        )
        if not preserve_processes:
            terminate_pids([process.pid for process in agents])
        for log in agent_logs:
            log.close()
        if not preserve_processes:
            terminate_pids([process.pid for process in sitl_processes])


if __name__ == "__main__":
    main()
