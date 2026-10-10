"""
SITL Environmental Scenarios, Parameter Verification & Configuration Dumper (Item 23).
- Provides named, switchable environmental scenarios for SITL (wind, turbulence, GPS noise).
- Verifies parameter names against the ArduPilot version in use by reading each value back.
- Dumps the full active vehicle parameter set into experiments/results/ at the start of every run.
"""

import os
import sys
import time
import logging
from pathlib import Path
from typing import Dict, List, Optional, Tuple, Any, Union

logger = logging.getLogger("SITLScenarios")

# Named switchable environmental scenarios
SITL_SCENARIOS: Dict[str, Dict[str, float]] = {
    "calm": {
        "SIM_WIND_SPD": 0.0,
        "SIM_WIND_DIR": 180.0,
        "SIM_WIND_TURB": 0.0,
        "SIM_GPS1_NOISE": 0.0,
        "SIM_GPS1_LAG_MS": 100.0,
        "SIM_GPS1_ACC": 0.30,
    },
    "moderate_wind": {
        "SIM_WIND_SPD": 4.0,       # 4.0 m/s (~8 knots)
        "SIM_WIND_DIR": 90.0,      # East wind
        "SIM_WIND_TURB": 0.15,     # Moderate turbulence
        "SIM_GPS1_NOISE": 0.0,
        "SIM_GPS1_LAG_MS": 100.0,
        "SIM_GPS1_ACC": 0.30,
    },
    "high_wind": {
        "SIM_WIND_SPD": 8.0,       # 8.0 m/s (~16 knots)
        "SIM_WIND_DIR": 45.0,      # North-East crosswind
        "SIM_WIND_TURB": 0.35,     # Strong turbulence
        "SIM_GPS1_NOISE": 0.0,
        "SIM_GPS1_LAG_MS": 100.0,
        "SIM_GPS1_ACC": 0.30,
    },
    "gps_noisy": {
        "SIM_WIND_SPD": 0.0,
        "SIM_WIND_DIR": 180.0,
        "SIM_WIND_TURB": 0.0,
        "SIM_GPS1_NOISE": 1.50,    # 1.50m std dev matching plain u-blox M10Q GPS profile
        "SIM_GPS1_LAG_MS": 150.0,  # Elevated GPS latency
        "SIM_GPS1_ACC": 1.50,
    },
    "harsh_environment": {
        "SIM_WIND_SPD": 6.0,       # 6.0 m/s gusting wind
        "SIM_WIND_DIR": 60.0,
        "SIM_WIND_TURB": 0.25,
        "SIM_GPS1_NOISE": 1.50,    # 1.50m GPS noise
        "SIM_GPS1_LAG_MS": 150.0,
        "SIM_GPS1_ACC": 1.50,
    },
}


def get_available_scenarios() -> List[str]:
    """Returns the list of available named environmental scenarios."""
    return list(SITL_SCENARIOS.keys())


def get_scenario_parameters(scenario_name: str) -> Dict[str, float]:
    """Returns the parameter dictionary for a given scenario."""
    if scenario_name not in SITL_SCENARIOS:
        raise ValueError(f"Unknown scenario '{scenario_name}'. Available: {get_available_scenarios()}")
    return dict(SITL_SCENARIOS[scenario_name])


def verify_and_set_param(
    conn: Any,
    param_name: str,
    param_value: float,
    param_type: int = 9,
    timeout: float = 2.0,
) -> Tuple[bool, Optional[float]]:
    """
    Sets an ArduPilot parameter over MAVLink and reads it back to verify that
    the parameter name is valid and accepted by the running ArduPilot firmware (Item 23).
    """
    if not conn:
        return False, None

    # 1. Dispatch parameter set command
    name_bytes = param_name.encode("utf-8")[:16]
    conn.mav.param_set_send(
        conn.target_system,
        conn.target_component,
        name_bytes,
        float(param_value),
        param_type,
    )

    # 2. Wait and read back acknowledged PARAM_VALUE
    t0 = time.time()
    while time.time() - t0 < timeout:
        msg = conn.recv_match(type="PARAM_VALUE", blocking=False)
        if msg:
            p_id = msg.param_id if isinstance(msg.param_id, str) else msg.param_id.decode("utf-8", errors="ignore")
            p_id = p_id.strip("\x00")
            if p_id == param_name:
                val = float(msg.param_value)
                return True, val
        time.sleep(0.02)

    # 3. Fallback: explicitly request read
    try:
        conn.mav.param_request_read_send(
            conn.target_system,
            conn.target_component,
            name_bytes,
            -1,
        )
        t1 = time.time()
        while time.time() - t1 < timeout:
            msg = conn.recv_match(type="PARAM_VALUE", blocking=False)
            if msg:
                p_id = msg.param_id if isinstance(msg.param_id, str) else msg.param_id.decode("utf-8", errors="ignore")
                p_id = p_id.strip("\x00")
                if p_id == param_name:
                    return True, float(msg.param_value)
            time.sleep(0.02)
    except Exception as e:
        logger.debug("Failed fallback param request read: %s", e)

    return False, None


def apply_scenario_via_mavlink(
    conn: Any,
    scenario_name: str = "calm",
    verify: bool = True,
) -> Dict[str, Optional[float]]:
    """
    Applies the specified named environmental scenario parameters to a connected SITL vehicle
    and verifies that each parameter was acknowledged by reading it back (Item 23).
    """
    params = get_scenario_parameters(scenario_name)
    results = {}
    for p_name, p_val in params.items():
        if verify:
            ok, ack_val = verify_and_set_param(conn, p_name, p_val)
            results[p_name] = ack_val if ok else None
        else:
            conn.mav.param_set_send(
                conn.target_system,
                conn.target_component,
                p_name.encode("utf-8")[:16],
                float(p_val),
                9,
            )
            results[p_name] = p_val
    return results


def parse_parm_file(parm_path: Union[str, Path]) -> Dict[str, float]:
    """Parses an ArduPilot .parm text file into a dictionary of key-value floats."""
    params = {}
    path = Path(parm_path)
    if not path.exists():
        return params

    with open(path, "r") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            if len(parts) >= 2:
                k = parts[0].strip()
                try:
                    v = float(parts[1].strip())
                    params[k] = v
                except ValueError:
                    continue
    return params


def dump_vehicle_parameters(
    output_dir: str = "experiments/results",
    scenario_name: str = "calm",
    base_parm_path: Optional[str] = None,
    live_conn: Optional[Any] = None,
    profile_name: str = "sitl_default_quad",
) -> str:
    """
    Dumps the full vehicle parameter set into experiments/results/ at the start of every run (Item 23).
    Merges swarm_params.parm, active environmental scenario, and verified firmware parameters.
    """
    out_dir = Path(output_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    dump_path = out_dir / "sitl_vehicle_params_dump.parm"

    if base_parm_path is None:
        repo_root = Path(__file__).resolve().parent.parent
        base_parm_path = str(repo_root / "sitl" / "swarm_params.parm")

    # 1. Parse base single source of truth
    params = parse_parm_file(base_parm_path)

    # 2. Merge active environmental scenario overrides
    scenario_params = get_scenario_parameters(scenario_name)
    params.update(scenario_params)

    # 3. If connected to a live vehicle, query live parameters
    live_acknowledged = {}
    if live_conn:
        for p_name in list(params.keys()):
            ok, val = verify_and_set_param(live_conn, p_name, params[p_name], timeout=0.2)
            if ok and val is not None:
                live_acknowledged[p_name] = val
                params[p_name] = val

    # 4. Write output parameter dump with complete provenance metadata
    with open(dump_path, "w") as f:
        f.write("# ==============================================================================\n")
        f.write("# Autonomous Swarm Drones - Active Vehicle Configuration Parameter Dump (Item 23)\n")
        f.write("# ==============================================================================\n")
        f.write(f"# Timestamp        : {time.strftime('%Y-%m-%d %H:%M:%S UTC', time.gmtime())}\n")
        f.write(f"# Configuration    : {profile_name}\n")
        f.write(f"# Active Scenario  : {scenario_name}\n")
        f.write(f"# Source File      : {base_parm_path}\n")
        f.write(f"# Total Parameters : {len(params)}\n")
        f.write("# ==============================================================================\n\n")

        for k in sorted(params.keys()):
            val = params[k]
            # Format nicely as float or int
            val_str = f"{val:.6f}" if isinstance(val, float) and not val.is_integer() else f"{int(val) if val.is_integer() else val}"
            f.write(f"{k:<20} {val_str}\n")

    logger.info("Dumped vehicle parameters (%d params) to: %s", len(params), dump_path)
    return str(dump_path)
