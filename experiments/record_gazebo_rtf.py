#!/usr/bin/env python3
"""Capture Gazebo world statistics as raw JSONL plus a compact RTF summary."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import select
import signal
import statistics
import subprocess
import time


def _seconds(value: dict) -> float:
    return float(value.get("sec", 0)) + float(value.get("nsec", 0)) / 1_000_000_000


def _airborne_window(samples: list[dict], events_path: Path, started_utc: str) -> dict | None:
    if not events_path.exists():
        return None
    start = datetime.fromisoformat(started_utc)
    airborne_times = []
    for line in events_path.read_text(encoding="utf-8").splitlines():
        try:
            row = json.loads(line)
            if row.get("kind") == "agent_status" and row.get("airborne"):
                airborne_times.append(datetime.fromisoformat(row["utc"]))
        except (json.JSONDecodeError, KeyError, ValueError):
            continue
    if not airborne_times:
        return None
    begin = (min(airborne_times) - start).total_seconds()
    end = (max(airborne_times) - start).total_seconds()
    flight_samples = [row for row in samples if begin <= row["captureWallElapsedSeconds"] <= end]
    if len(flight_samples) < 2:
        return None
    sim_elapsed = _seconds(flight_samples[-1]["simTime"]) - _seconds(flight_samples[0]["simTime"])
    wall_elapsed = (flight_samples[-1]["captureWallElapsedSeconds"] -
                    flight_samples[0]["captureWallElapsedSeconds"])
    factors = [float(row["realTimeFactor"]) for row in flight_samples]
    return {
        "wall_seconds": round(wall_elapsed, 3),
        "sim_seconds": round(sim_elapsed, 3),
        "integrated_rtf": sim_elapsed / wall_elapsed if wall_elapsed > 0 else None,
        "median_reported_rtf": statistics.median(factors),
        "p10_reported_rtf": sorted(factors)[int(0.10 * (len(factors) - 1))],
        "p90_reported_rtf": sorted(factors)[int(0.90 * (len(factors) - 1))],
        "sample_count": len(flight_samples),
    }


def capture(topic: str, duration: float, output_dir: Path,
            events_path: Path | None = None) -> dict:
    output_dir.mkdir(parents=True, exist_ok=True)
    raw_path = output_dir / "gazebo_world_stats.jsonl"
    summary_path = output_dir / "gazebo_rtf_summary.json"
    command = ["gz", "topic", "-e", "-t", topic, "--json-output"]
    started = time.monotonic()
    started_utc = datetime.now(timezone.utc).isoformat()
    samples = []
    stopped_by_recorder = False
    old_handlers = {}
    stop_requested = False

    def request_stop(_signum, _frame):
        nonlocal stop_requested
        stop_requested = True

    for signum in (signal.SIGINT, signal.SIGTERM):
        old_handlers[signum] = signal.signal(signum, request_stop)
    with raw_path.open("w", encoding="utf-8") as raw:
        process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                   bufsize=0)
        buffer = b""
        try:
            assert process.stdout is not None
            fd = process.stdout.fileno()
            while not stop_requested and time.monotonic() - started < duration:
                if process.poll() is not None:
                    break
                readable, _, _ = select.select([fd], [], [], 0.25)
                if not readable:
                    continue
                chunk = os.read(fd, 65536)
                if not chunk:
                    break
                buffer += chunk
                while b"\n" in buffer:
                    line, buffer = buffer.split(b"\n", 1)
                    try:
                        row = json.loads(line)
                    except (json.JSONDecodeError, UnicodeDecodeError):
                        continue
                    factor = float(row.get("realTimeFactor", "nan"))
                    sim_time = _seconds(row.get("simTime", {}))
                    real_time = _seconds(row.get("realTime", {}))
                    if not (math.isfinite(factor) and math.isfinite(sim_time) and math.isfinite(real_time)):
                        continue
                    row["captureWallElapsedSeconds"] = time.monotonic() - started
                    raw.write(json.dumps(row, separators=(",", ":")) + "\n")
                    raw.flush()
                    samples.append(row)
            if buffer.strip():
                try:
                    row = json.loads(buffer)
                except (json.JSONDecodeError, UnicodeDecodeError):
                    row = None
                if row is not None:
                    row["captureWallElapsedSeconds"] = time.monotonic() - started
                    raw.write(json.dumps(row, separators=(",", ":")) + "\n")
                    samples.append(row)
        finally:
            if process.poll() is None:
                stopped_by_recorder = True
                process.terminate()
                try:
                    process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    process.kill()
            return_code = process.wait()
            stderr = process.stderr.read().decode("utf-8", errors="replace") if process.stderr else ""
            for signum, handler in old_handlers.items():
                signal.signal(signum, handler)

    if not samples:
        raise RuntimeError(f"Gazebo stats topic produced no valid samples: {stderr.strip()}")
    factors = [float(item["realTimeFactor"]) for item in samples]
    elapsed_wall = time.monotonic() - started
    sim_elapsed = _seconds(samples[-1]["simTime"]) - _seconds(samples[0]["simTime"])
    sample_wall_elapsed = (samples[-1]["captureWallElapsedSeconds"] -
                           samples[0]["captureWallElapsedSeconds"])
    stats = {
        "topic": topic,
        "started_utc": started_utc,
        "capture_wall_seconds": round(elapsed_wall, 3),
        "sample_count": len(samples),
        "return_code": return_code,
        "stopped_by_recorder": stopped_by_recorder,
        "rtf_mean": statistics.fmean(factors),
        "rtf_median": statistics.median(factors),
        "rtf_min": min(factors),
        "rtf_max": max(factors),
        "sim_time_advanced_seconds": round(sim_elapsed, 3),
        "world_sample_wall_seconds": round(sample_wall_elapsed, 3),
        "integrated_rtf_over_samples": (sim_elapsed / sample_wall_elapsed
                                         if sample_wall_elapsed > 0 else None),
        "raw_samples": raw_path.name,
    }
    if events_path is not None:
        stats["airborne_window"] = _airborne_window(samples, events_path, started_utc)
    summary_path.write_text(json.dumps(stats, indent=2) + "\n", encoding="utf-8")
    return stats


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--topic", default="/world/cinewhoop_5drones/stats")
    parser.add_argument("--duration", type=float, default=600.0,
                        help="wall-clock capture limit in seconds; Ctrl-C stops and saves early")
    parser.add_argument("--output-dir", type=Path, required=True,
                        help="run evidence directory for raw samples and summary")
    parser.add_argument("--events-file", type=Path,
                        help="optional run events.jsonl for an airborne-only RTF window")
    args = parser.parse_args()
    if args.duration <= 0:
        parser.error("--duration must be greater than zero")
    try:
        summary = capture(args.topic, args.duration, args.output_dir, args.events_file)
    except (OSError, RuntimeError) as exc:
        parser.exit(1, f"Gazebo RTF capture failed: {exc}\n")
    print(json.dumps(summary, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
