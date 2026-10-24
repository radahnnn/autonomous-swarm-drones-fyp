"""Launch a standalone ArduCopter SITL fleet for remote independent agents.

Run this on the simulator host. ArduPilot SITL's SERIAL0 TCP listeners use the
instance ports listed by sitl.fleet (5760 + 10 * ID); remote agents connect to
the simulator host's LAN address. Ctrl-C terminates only the SITL children
started by this script.
"""

import argparse
from datetime import datetime, timezone
import ipaddress
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from sitl.fleet import build_fleet


def find_arducopter(configured: str | None = None) -> Path:
    candidates = [
        Path(configured) if configured else None,
        ROOT.parent.parent / "dependencies" / "ardupilot" / "build" / "sitl" / "bin" / "arducopter",
        Path.home() / "ardupilot" / "build" / "sitl" / "bin" / "arducopter",
    ]
    binary = next((path for path in candidates if path is not None and path.is_file()), None)
    if binary is None:
        raise FileNotFoundError(
            "Could not locate ArduCopter SITL; set ARDUCOPTER_BIN to the binary path"
        )
    return binary.resolve()


def stop_processes(processes: list[subprocess.Popen]) -> None:
    for process in processes:
        if process.poll() is None:
            process.terminate()
    deadline = time.monotonic() + 5.0
    for process in processes:
        remaining = max(0.0, deadline - time.monotonic())
        try:
            process.wait(timeout=remaining)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=2.0)


def wait_for_tcp_fleet(fleet, host: str, timeout_s: float = 30.0) -> None:
    deadline = time.monotonic() + timeout_s
    pending = {spec.index: spec.tcp_port for spec in fleet}
    while pending and time.monotonic() < deadline:
        for drone_id, port in list(pending.items()):
            try:
                with socket.create_connection((host, port), timeout=0.2):
                    del pending[drone_id]
            except OSError:
                pass
        time.sleep(0.1)
    if pending:
        raise TimeoutError(f"SITL TCP listeners did not open: {pending}")


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--drones", type=int, default=5, choices=range(1, 11))
    parser.add_argument("--advertise-ip", required=True,
                        help="simulator host's reachable IPv4 address for remote agents")
    parser.add_argument("--speedup", type=float, default=1.0)
    parser.add_argument("--duration-s", type=float, default=None,
                        help="optional automatic shutdown after this many seconds")
    parser.add_argument("--arducopter-bin", default=os.environ.get("ARDUCOPTER_BIN"))
    parser.add_argument("--evidence-dir", default=None,
                        help="directory for child logs and process manifest")
    args = parser.parse_args(argv)
    try:
        advertised_ip = str(ipaddress.IPv4Address(args.advertise_ip))
    except ipaddress.AddressValueError as exc:
        parser.error(f"--advertise-ip must be an IPv4 address: {exc}")
    if not (0.1 <= args.speedup <= 10.0):
        parser.error("--speedup must be between 0.1 and 10")
    if args.duration_s is not None and args.duration_s <= 0:
        parser.error("--duration-s must be positive")

    binary = find_arducopter(args.arducopter_bin)
    fleet = build_fleet(args.drones)
    evidence_parent = (Path(args.evidence_dir) if args.evidence_dir
                       else ROOT / "runs" / "independent")
    run_dir = evidence_parent / (
        "multihost-sitl-" + datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S.%fZ")
    )
    run_dir.mkdir(parents=True, exist_ok=False)
    params = ROOT / "sitl" / "swarm_params.parm"
    processes = []
    logs = []
    try:
        for spec in fleet:
            log = (run_dir / f"sitl-{spec.index + 1}.log").open("w", encoding="utf-8")
            logs.append(log)
            command = [
                str(binary), f"-I{spec.inst}", "--sysid", str(spec.sysid),
                "--model", "quad", "--home", spec.home_standalone,
                "--defaults", str(params), "--speedup", str(args.speedup),
            ]
            processes.append(subprocess.Popen(
                command, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT,
                start_new_session=True,
            ))
        wait_for_tcp_fleet(fleet, advertised_ip)
        manifest = {
            "started_utc": datetime.now(timezone.utc).isoformat(),
            "simulator_host_ip": advertised_ip,
            "tcp_bind": "ArduPilot SITL default SERIAL0 listener (all interfaces)",
            "vehicles": [
                {"id": spec.index, "sysid": spec.sysid,
                 "tcp_endpoint": f"tcp:{advertised_ip}:{spec.tcp_port}",
                 "pid": process.pid}
                for spec, process in zip(fleet, processes)
            ],
        }
        (run_dir / "processes.json").write_text(
            json.dumps(manifest, indent=2) + "\n", encoding="utf-8",
        )
        print(f"SITL fleet is ready for remote agents. Evidence: {run_dir}")
        for vehicle in manifest["vehicles"]:
            print(f"D{vehicle['id'] + 1}: SYSID {vehicle['sysid']} at {vehicle['tcp_endpoint']}")
        if args.duration_s is None:
            print("Press Ctrl-C here to stop this SITL fleet.", flush=True)
        else:
            print(f"SITL will stop automatically after {args.duration_s:g} seconds.", flush=True)
        started = time.monotonic()
        while all(process.poll() is None for process in processes):
            if args.duration_s is not None and time.monotonic() - started >= args.duration_s:
                break
            time.sleep(0.5)
        failed = [process.pid for process in processes if process.poll() not in (None, 0)]
        if failed:
            print(f"SITL process(es) exited unexpectedly: {failed}", file=sys.stderr)
            return 1
        return 0
    except KeyboardInterrupt:
        print("\nStopping the SITL processes started by this launcher.")
        return 0
    finally:
        stop_processes(processes)
        for log in logs:
            log.close()


if __name__ == "__main__":
    raise SystemExit(main())
