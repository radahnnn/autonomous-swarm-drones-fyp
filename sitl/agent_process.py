"""Versioned JSON-lines IPC for one controller per OS process (host simulation only)."""
import json
import select
import subprocess
import sys
from pathlib import Path

import numpy as np

from sitl.decentralized_guidance import LocalDroneAgent
from sitl.mission_intent import MissionIntent
from sitl.peer_udp import UdpPeerStateTransport
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
    for field in ("position",):
        v = np.asarray(args[field], dtype=float)
        if v.shape != (2,) or not np.isfinite(v).all():
            raise ValueError("invalid " + field)
    if args.get("mission_intent") is not None:
        MissionIntent.from_dict(args["mission_intent"], now)
        slot = np.asarray(args.get("slot_offset"), dtype=float)
        if slot.shape != (2,) or not np.isfinite(slot).all():
            raise ValueError("invalid local formation slot")
    else:
        v = np.asarray(args["goal"], dtype=float)
        if v.shape != (2,) or not np.isfinite(v).all():
            raise ValueError("invalid goal")
    if not 0 < float(args["dt"]) <= .25:
        raise ValueError("invalid integration interval")
    seen = set()
    for peer in args.get("neighbor_states", []):
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
    def __init__(self, drone_id, safe_radius=3.0, hybrid=False, peer_count=1,
                 peer_port_base=20000, peer_loss=0.0, peer_latency_ms=30.0,
                 peer_seed=None, peer_burst_loss=False):
        super().__init__(drone_id, safe_radius, hybrid=False)
        self.hybrid_controller = HybridSnapshot() if hybrid else None
        self.sequence = 0
        self.peer_count = int(peer_count)
        self.peer_port_base = int(peer_port_base)
        self.peer_loss = float(peer_loss)
        self.peer_latency_ms = float(peer_latency_ms)
        self.peer_seed = peer_seed
        self.peer_burst_loss = bool(peer_burst_loss)
        self.last_peer_ids = []
        self.last_peer_metrics = {}
        self.process = subprocess.Popen(
            [sys.executable, "-u", "-m", "sitl.agent_process", str(drone_id),
             str(safe_radius), str(int(hybrid)), str(self.peer_count),
             str(self.peer_port_base), str(self.peer_loss),
             str(self.peer_latency_ms), str(-1 if peer_seed is None else peer_seed),
             str(int(self.peer_burst_loss))],
            cwd=Path(__file__).resolve().parent.parent,
            stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True, bufsize=1,
        )

    def command(self, position, sample_time, goal, neighbor_states, desired_offsets, dt,
                coordinator_connected=True, now=None, mission_intent=None, slot_offset=None,
                peer_ids=None):
        self.sequence += 1
        args = dict(position=np.asarray(position)[:2], sample_time=sample_time,
                    goal=None if goal is None else np.asarray(goal)[:2],
                    mission_intent=mission_intent,
                    slot_offset=None if slot_offset is None else np.asarray(slot_offset)[:2],
                    peer_ids=sorted(set(int(value) for value in (peer_ids or []))),
                    peer_transport=peer_ids is not None,
                    neighbor_states=neighbor_states,
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
        self.last_peer_ids = response.get("peer_ids", [])
        self.last_peer_metrics = response.get("peer_metrics", {})
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
    peer_count = int(sys.argv[4]) if len(sys.argv) > 4 else 1
    peer_port_base = int(sys.argv[5]) if len(sys.argv) > 5 else 20000
    peer_loss = float(sys.argv[6]) if len(sys.argv) > 6 else 0.0
    peer_latency_ms = float(sys.argv[7]) if len(sys.argv) > 7 else 0.0
    peer_seed = int(sys.argv[8]) if len(sys.argv) > 8 else -1
    peer_burst_loss = bool(int(sys.argv[9])) if len(sys.argv) > 9 else False
    agent = LocalDroneAgent(drone_id, radius, hybrid)
    transport = UdpPeerStateTransport(
        drone_id, peer_port_base, peer_count,
        packet_loss_rate=peer_loss,
        latency_mean=peer_latency_ms / 1000.0,
        latency_std=0.005 if peer_latency_ms > 0 else 0.0,
        seed=None if peer_seed < 0 else peer_seed + drone_id * 65537,
        burst_loss=peer_burst_loss,
    ) if peer_count > 1 else None
    previous = 0
    for line in sys.stdin:
        request = {}
        try:
            request = json.loads(line)
            args = validate(request, drone_id, previous)
            peer_transport_enabled = args.pop("peer_transport", False)
            peer_ids = args.pop("peer_ids", [])
            neighbors = args.get("neighbor_states", [])
            if peer_transport_enabled and transport is not None:
                agent.update_local_state(args["position"], args["sample_time"])
                transport.publish(args["position"], agent.velocity, args["sample_time"],
                                  peer_ids, args["now"])
                neighbors = transport.receive(args["now"], peer_ids)
                args["neighbor_states"] = neighbors
            target = agent.command(**args)
            previous = request["sequence"]
            result = {
                "sequence": previous,
                "target": target.tolist(),
                "peer_ids": sorted(int(peer["id"]) for peer in neighbors),
                "peer_metrics": transport.stats() if transport is not None else {},
            }
            if hybrid:
                h = agent.hybrid_controller
                result.update(mode=h.get_drone_mode(drone_id).value,
                              alpha=h.get_alpha(drone_id), switches=h.switch_counts.get(drone_id, 0))
        except Exception as exc:
            result = {"sequence": request.get("sequence"), "error": str(exc)}
        print(json.dumps(result, allow_nan=False), flush=True)
    if transport is not None:
        transport.close()


if __name__ == "__main__":
    main()
