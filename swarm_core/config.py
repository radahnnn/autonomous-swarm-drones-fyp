"""
Swarm Drones FYP Configuration & Simulation Parameters.
Every parameter includes an explicit provenance label:
- "assumed": Engineering assumption / literature baseline. Not yet verified on physical hardware.
- "fitted from SITL": Calibrated against ArduPilot/Gazebo simulation logs.
- "measured on hardware": Directly measured or calibrated on physical 5-inch 6S Matek H743 hardware.
"""

from dataclasses import dataclass
from typing import Dict, Any


@dataclass(frozen=True)
class ParameterProvenance:
    value: Any
    unit: str
    provenance: str
    notes: str


# ==============================================================================
# PHYSICAL DRONE DYNAMICS (5-INCH FPV QUADROTOR BASELINE)
# ==============================================================================
CONFIG_DYNAMICS = {
    # First-order closed-loop attitude/thrust response time constant
    "attitude_tau": ParameterProvenance(
        value=0.992,
        unit="s",
        provenance="fitted from SITL",
        notes="Identified from ArduPilot SITL GUIDED mode 5.0m position step response (RMSE = 0.1536 m vs SITL telemetry). Captures combined position controller loop lag and vehicle dynamics."
    ),
    # Aerodynamic rotor drag coefficient (deceleration per unit speed)
    "drag_coeff": ParameterProvenance(
        value=0.637,
        unit="1/s",
        provenance="fitted from SITL",
        notes="Identified from ArduPilot SITL GUIDED mode 5.0m position step response (RMSE = 0.1536 m vs SITL telemetry). Reflects rotor drag and position braking dynamics in SITL."
    ),
    # Physical collision radius (frame center to prop tip + safety margin)
    "drone_radius": ParameterProvenance(
        value=0.35,
        unit="m",
        provenance="assumed",
        notes="Based on 5-inch prop span on 230mm wheelbase quadrotor (230mm motor-to-motor + 127mm prop / 2 ~ 0.28m, rounded to 0.35m for safety)."
    ),
    # Maximum commanded ground speed
    "max_speed": ParameterProvenance(
        value=3.0,
        unit="m/s",
        provenance="assumed",
        notes="Conservative ground speed safety ceiling for preliminary field experiments."
    ),
    # Maximum commanded acceleration
    "max_accel": ParameterProvenance(
        value=2.5,
        unit="m/s^2",
        provenance="assumed",
        notes="Well within 6S battery thrust-to-weight capability (~0.25 g lateral acceleration)."
    ),
}

# ==============================================================================
# SENSOR & NAVIGATION NOISE
# ==============================================================================
CONFIG_SENSORS = {
    # Standard GPS horizontal position error (plain GPS baseline)
    "gps_noise_std_plain": ParameterProvenance(
        value=1.5,
        unit="m",
        provenance="assumed",
        notes="Typical U-Blox M8N/M9N plain GPS horizontal accuracy (1.0 to 2.5m CEP without RTK corrections)."
    ),
    # RTK-GPS reference noise (optimistic upper bound)
    "gps_noise_std_rtk": ParameterProvenance(
        value=0.04,
        unit="m",
        provenance="assumed",
        notes="Centimeter-level accuracy achievable only with RTK base station / NTRIP corrections."
    ),
    # Shared (common-mode) constellation error fraction
    "gps_common_mode_fraction": ParameterProvenance(
        value=0.60,
        unit="ratio",
        provenance="assumed",
        notes="Fraction of GPS error shared across co-located drones due to identical satellite geometry and atmospheric delays."
    ),
}

# ==============================================================================
# FORMATION & CONTROL PARAMETERS
# ==============================================================================
CONFIG_CONTROL = {
    # Nominal inter-drone spacing in steady-state formations
    "nominal_spacing": ParameterProvenance(
        value=2.5,
        unit="m",
        provenance="assumed",
        notes="Planned inter-drone slot distance providing clearance while fitting in standard 20x20m flight spaces."
    ),
    # Artificial Potential Field (APF) repulsive activation radius
    "apf_safe_radius": ParameterProvenance(
        value=2.5,
        unit="m",
        provenance="assumed",
        notes="Distance threshold below which inter-drone repulsive potential activates."
    ),
    # Hard physical collision threshold (2 * drone_radius)
    "collision_threshold": ParameterProvenance(
        value=0.70,
        unit="m",
        provenance="assumed",
        notes="Double the physical drone radius (0.35m * 2 = 0.70m)."
    ),
    # Centralized controller proportional gain
    "kp_central": ParameterProvenance(
        value=1.8,
        unit="1/s^2",
        provenance="assumed",
        notes="Position tracking gain."
    ),
    # Centralized controller derivative gain
    "kd_central": ParameterProvenance(
        value=2.2,
        unit="1/s",
        provenance="assumed",
        notes="Velocity damping gain."
    ),
}

# ==============================================================================
# WIRELESS NETWORK & HYBRID HYSTERESIS
# ==============================================================================
CONFIG_NETWORK = {
    # Nominal communication latency
    "latency_mean": ParameterProvenance(
        value=0.03,
        unit="s",
        provenance="assumed",
        notes="Typical 2.4 GHz ESP32 broadcast / MAVLink UDP latency."
    ),
    # Central coordinator heartbeat degradation timeout
    "degrade_timeout": ParameterProvenance(
        value=0.50,
        unit="s",
        provenance="assumed",
        notes="Duration of lost heartbeats before triggering decentralized fallback (~10 missed packets at 20 Hz)."
    ),
    # Sliding observation window length for recovery
    "recovery_window_size": ParameterProvenance(
        value=20,
        unit="ticks",
        provenance="assumed",
        notes="Sliding observation window (1.0s at 20 Hz) for evaluating delivery ratio."
    ),
    # Minimum packet delivery ratio within window required for recovery
    "recovery_ratio_threshold": ParameterProvenance(
        value=0.70,
        unit="ratio",
        provenance="assumed",
        notes="Requires at least 70% delivery (e.g. >= 14/20 packets) to re-engage centralized mode."
    ),
    # Minimum dwell time in fallback state
    "min_dwell_time": ParameterProvenance(
        value=2.0,
        unit="s",
        provenance="assumed",
        notes="Asymmetric hysteresis dwell time preventing chattering across transient link edges."
    ),
    # Blended control transition ramp duration
    "ramp_duration": ParameterProvenance(
        value=0.8,
        unit="s",
        provenance="assumed",
        notes="Smoothing duration for alpha(t) transition between 0.0 and 1.0."
    ),
}
