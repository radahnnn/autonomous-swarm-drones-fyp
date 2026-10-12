"""
Fleet layout for the SITL / Gazebo swarm: ports, labels and home locations for N drones.

Conventions (shared by the launch scripts, the Gazebo worlds and the console):
- drone i is ArduPilot instance -I<i>, SYSID i+1
- MAVLink telemetry on UDP 14552 + 10*i when a launcher added `--out=udp:127.0.0.1:<port>`, otherwise on the
  instance's own TCP server at 5760 + 10*i (used when the console starts the SITL binaries itself)
- Gazebo ArduPilotPlugin FDM port 9002 + 10*i
- Gazebo mode: ALL instances share one home (the world origin). The plugin sends absolute
  world positions that SITL adds to --home, so staggered homes would shift the drones.
- Standalone (--model quad) mode: each drone's home is its own V-formation slot, so the swarm
  starts on the ground in the same layout the Gazebo worlds spawn it in.
"""

import math
from dataclasses import dataclass
from typing import List

from sitl.formation_library import slots_for

DATUM_LAT = -35.363261
DATUM_LON = 149.165230
DATUM_ALT = 584.0
MAX_DRONES = 10

_M_PER_DEG_LAT = 111_320.0


@dataclass(frozen=True)
class DroneSpec:
    index: int          # 0-based
    port: int           # MAVLink UDP port
    tcp_port: int       # ArduPilot's built-in SERIAL0 TCP server
    fdm_port: int       # Gazebo FDM (ArduPilotPlugin) port
    home_standalone: str
    home_gazebo: str
    label: str

    @property
    def inst(self) -> int:
        return self.index

    @property
    def sysid(self) -> int:
        return self.index + 1


def _home(n_off: float, e_off: float) -> str:
    lat = DATUM_LAT + n_off / _M_PER_DEG_LAT
    lon = DATUM_LON + e_off / (_M_PER_DEG_LAT * math.cos(math.radians(DATUM_LAT)))
    return f"{lat:.7f},{lon:.7f},{DATUM_ALT:.0f},0"


def build_fleet(n: int) -> List[DroneSpec]:
    """Specs for an n-drone swarm (1..MAX_DRONES)."""
    if not 1 <= n <= MAX_DRONES:
        raise ValueError(f"Swarm size must be between 1 and {MAX_DRONES}, got {n}")
    slots = slots_for("v", n)
    gazebo_home = _home(0.0, 0.0)
    return [
        DroneSpec(
            index=i,
            port=14552 + 10 * i,
            tcp_port=5760 + 10 * i,
            fdm_port=9002 + 10 * i,
            home_standalone=_home(float(slots[i, 0]), float(slots[i, 1])),
            home_gazebo=gazebo_home,
            label=f"Drone {i + 1} (SysID {i + 1})",
        )
        for i in range(n)
    ]
