"""Versioned JSON-lines IPC for one controller per OS process (host simulation only)."""
import json
import select
import subprocess
import sys
from pathlib import Path

import numpy as np

from sitl.decentralized_guidance import LocalDroneAgent
from swarm_core.controllers.hybrid import HybridMode

VERSION = 1
FRAME = "shared_local_north_east_metres"


def plain(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, dict):
        return {str(k): plain(v) for k, v in value.items()}
    if isinstance(value, (tuple, list)):
        return [plain(v) for v in value]
    return value


def validate(message, drone_id, previous):
    if message.get("version") != VERSION or message.get("frame") != FRAME:
        raise ValueError("unsupported protocol version or coordinate frame")
    if message.get("drone_id") != drone_id or message.get("sequence", 0) <= previous:
        raise ValueError("wrong recipient or replayed sequence")
    args = message["args"]
    now, sample = float(args["now"]), float(args["sample_time"])
    if not np.isfinite([now, sample]).all() or not 0 <= now - sample <= 1.0:
        raise ValueError("stale, future or invalid own telemetry")
    for field in ("position", "goal"):
        v = np.asarray(args[field], dtype=float)
        if v.shape != (2,) or not np.isfinite(v).all():
            raise ValueError("invalid " + field)
    if not 0 < float(args["dt"]) <= .25:
        raise ValueError("invalid integration interval")
    seen = set()
    for peer in args["neighbor_states"]:
        peer_id = peer.get("id")
        if not isinstance(peer_id, int) or peer_id < 0 or peer_id == drone_id or peer_id in seen:
            raise ValueError("invalid or duplicate peer ID")
        seen.add(peer_id)
        if not 0 <= float(peer.get("confidence", 1)) <= 1:
            raise ValueError("invalid peer confidence")
        for key in ("position", "velocity"):
            v = np.asarray(peer[key], dtype=float)
            if v.shape != (2,) or not np.isfinite(v).all():
                raise ValueError("invalid peer state")
        if not 0 <= float(peer.get("age", 0)) <= .5:
            raise ValueError("stale peer state")
    args["desired_offsets"] = {int(k): np.asarray(v) for k, v in args["desired_offsets"].items()}
    for offset in args["desired_offsets"].values():
        if offset.shape != (2,) or not np.isfinite(offset).all():
            raise ValueError("invalid relative slot offset")
    return args


class HybridSnapshot:
    def __init__(self):
        self.mode = HybridMode.CENTRALIZED
        self.alpha = 1.0
        self.switch_counts = {}

    def get_drone_mode(self, _):
        return self.mode

    def get_alpha(self, _):
        return self.alpha


class ProcessDroneAgent(LocalDroneAgent):
    """Parent retains a telemetry estimator; all control decisions run in the child."""
    def __init__(self, drone_id, safe_radius=3.0, hybrid=False):
        super().__init__(drone_id, safe_radius, hybrid=False)
        self.hybrid_controller = HybridSnapshot() if hybrid else None
        self.sequence = 0
        self.process = subprocess.Popen(
            [sys.executable, "-u", "-m", "sitl.agent_process", str(drone_id),
             str(safe_radius), str(int(hybrid))],
            cwd=Path(__file__).resolve().parent.parent,
            stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True, bufsize=1,
        )

    def command(self, position, sample_time, goal, neighbor_states, desired_offsets, dt,
                coordinator_connected=True, now=None):
        self.sequence += 1
        args = dict(position=np.asarray(position)[:2], sample_time=sample_time,
                    goal=np.asarray(goal)[:2], neighbor_states=neighbor_states,
                    desired_offsets=desired_offsets, dt=dt,
                    coordinator_connected=coordinator_connected, now=now)
        message = dict(version=VERSION, frame=FRAME, drone_id=self.drone_id,
                       sequence=self.sequence, args=plain(args))
        self.process.stdin.write(json.dumps(message, allow_nan=False) + "\n")
        self.process.stdin.flush()
        if not select.select([self.process.stdout], [], [], 2.0)[0]:
            raise RuntimeError(f"agent {self.drone_id} response timed out")
        response = json.loads(self.process.stdout.readline())
        if response.get("sequence") != self.sequence or "error" in response:
            raise RuntimeError(f"agent protocol error: {response}")
        if self.hybrid_controller is not None:
            self.hybrid_controller.mode = HybridMode(response["mode"])
            self.hybrid_controller.alpha = response["alpha"]
            self.hybrid_controller.switch_counts[self.drone_id] = response["switches"]
        result = np.asarray(response["target"], dtype=float)
        if result.shape != (2,) or not np.isfinite(result).all():
            raise RuntimeError("invalid agent target")
        return result

    def close(self):
        if self.process.poll() is None:
            self.process.stdin.close()
            try:
                self.process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                self.process.terminate()
                self.process.wait(timeout=2)
        self.process.stdout.close()


def main():
    drone_id, radius, hybrid = int(sys.argv[1]), float(sys.argv[2]), bool(int(sys.argv[3]))
    agent = LocalDroneAgent(drone_id, radius, hybrid)
    previous = 0
    for line in sys.stdin:
        request = {}
        try:
            request = json.loads(line)
            args = validate(request, drone_id, previous)
            target = agent.command(**args)
            previous = request["sequence"]
            result = {"sequence": previous, "target": target.tolist()}
            if hybrid:
                h = agent.hybrid_controller
                result.update(mode=h.get_drone_mode(drone_id).value,
                              alpha=h.get_alpha(drone_id), switches=h.switch_counts.get(drone_id, 0))
        except Exception as exc:
            result = {"sequence": request.get("sequence"), "error": str(exc)}
        print(json.dumps(result, allow_nan=False), flush=True)


if __name__ == "__main__":
    main()
