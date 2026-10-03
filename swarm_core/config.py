"""
Swarm Drones FYP Configuration & Simulation Parameters.
Defines explicit configuration profiles with complete parameter provenance:
- "assumed": Engineering assumption / literature baseline. Not yet verified on physical hardware.
- "fitted from SITL": Calibrated against ArduPilot SITL simulation telemetry.
- "measured on hardware": Directly measured or calibrated on physical 5-inch 6S Matek H743 hardware.

Profiles:
1. "assumed_baseline": Fast multirotor attitude model (tau=0.18s, cd=0.20/s) with ideal sensors.
2. "sitl_fitted": Calibrated against ArduPilot GUIDED mode step response (tau=0.992s, cd=0.637/s).
"""

from dataclasses import dataclass, field
from typing import Any, Dict, Optional, Tuple, Union


@dataclass(frozen=True)
class ParameterProvenance:
    name: str
    value: Any
    unit: str
    meaning: str
    provenance: str
    notes: str
    reference: str = ""
    is_hard_boundary: bool = False


@dataclass
class SwarmConfigProfile:
    name: str
    description: str
    parameters: Dict[str, ParameterProvenance] = field(default_factory=dict)

    def get(self, param_name: str, default: Any = None) -> Any:
        if param_name not in self.parameters:
            if default is not None:
                return default
            raise KeyError(f"Parameter '{param_name}' not defined in profile '{self.name}'")
        return self.parameters[param_name].value

    def get_provenance(self, param_name: str) -> ParameterProvenance:
        if param_name not in self.parameters:
            raise KeyError(f"Parameter '{param_name}' not defined in profile '{self.name}'")
        return self.parameters[param_name]

    def to_dict(self) -> Dict[str, Any]:
        return {k: v.value for k, v in self.parameters.items()}


def _build_assumed_baseline_profile() -> SwarmConfigProfile:
    params = {
        # Physical & Aerodynamic Dynamics
        "attitude_tau": ParameterProvenance(
            name="attitude_tau",
            value=0.18,
            unit="s",
            meaning="First-order closed-loop attitude/thrust time-constant lag",
            provenance="assumed",
            notes="Literature baseline assumption for inner attitude rate response on small multirotors.",
            reference="Standard multirotor literature baseline",
        ),
        "drag_coeff": ParameterProvenance(
            name="drag_coeff",
            value=0.20,
            unit="1/s",
            meaning="Translational aerodynamic rotor drag coefficient (deceleration per unit velocity)",
            provenance="assumed",
            notes="Assumed linear blade drag damping coefficient.",
            reference="Literature baseline",
        ),
        "max_speed": ParameterProvenance(
            name="max_speed",
            value=3.0,
            unit="m/s",
            meaning="Maximum commanded ground speed ceiling",
            provenance="assumed",
            notes="Conservative flight space speed boundary.",
            is_hard_boundary=True,
        ),
        "max_accel": ParameterProvenance(
            name="max_accel",
            value=2.5,
            unit="m/s^2",
            meaning="Maximum commanded lateral acceleration",
            provenance="assumed",
            notes="Lateral acceleration limit corresponding to ~15 deg maximum tilt clamp.",
            is_hard_boundary=True,
        ),
        "drone_radius": ParameterProvenance(
            name="drone_radius",
            value=0.35,
            unit="m",
            meaning="Physical airframe collision radius (center of mass to propeller tip + margin)",
            provenance="assumed",
            notes="Based on 5-inch prop span on 230mm wheelbase quadrotor (230mm / 2 + 127mm / 2 ~ 0.28m, rounded to 0.35m).",
            is_hard_boundary=True,
        ),
        # Safety & Spatial Radii
        "collision_threshold": ParameterProvenance(
            name="collision_threshold",
            value=0.70,
            unit="m",
            meaning="Hard physical vehicle-to-vehicle contact distance (2 * drone_radius)",
            provenance="assumed",
            notes="Physical boundary where propeller/frame contact occurs between two quadrotors.",
            is_hard_boundary=True,
        ),
        "apf_activation_dist": ParameterProvenance(
            name="apf_activation_dist",
            value=1.20,
            unit="m",
            meaning="Artificial Potential Field repulsive activation threshold",
            provenance="assumed",
            notes="Distance below which onboard APF repulsion activates to prevent physical proximity.",
            is_hard_boundary=False,
        ),
        "nominal_spacing": ParameterProvenance(
            name="nominal_spacing",
            value=2.50,
            unit="m",
            meaning="Nominal equilibrium inter-drone slot distance in formations",
            provenance="assumed",
            notes="Provides safe formation clearance while fitting in 20x20m flight volumes.",
            is_hard_boundary=False,
        ),
        "comm_radius": ParameterProvenance(
            name="comm_radius",
            value=12.0,
            unit="m",
            meaning="Maximum RF communication range for 1-hop neighbor adjacency in graph",
            provenance="assumed",
            notes="Range within which peer-to-peer telemetry packets can be exchanged.",
            is_hard_boundary=True,
        ),
        # Sensors & Noise
        "gps_noise_std": ParameterProvenance(
            name="gps_noise_std",
            value=0.04,
            unit="m",
            meaning="Standard deviation of horizontal position measurement noise",
            provenance="assumed",
            notes="Low-noise sensor baseline (representing RTK-GPS or ideal motion capture).",
        ),
        "gps_common_mode_fraction": ParameterProvenance(
            name="gps_common_mode_fraction",
            value=0.60,
            unit="ratio",
            meaning="Fraction of GPS error shared across co-located drones due to identical satellite geometry",
            provenance="assumed",
            notes="Common-mode error cancels out in relative inter-drone baseline estimation.",
        ),
        "gps_corr_time": ParameterProvenance(
            name="gps_corr_time",
            value=30.0,
            unit="s",
            meaning="First-order Gauss-Markov correlation time constant for low-frequency GPS position drift",
            provenance="assumed",
            notes="Models time-correlated atmospheric and ephemeris drift rather than unphysical high-frequency white noise.",
        ),
        "gps_vel_noise_std": ParameterProvenance(
            name="gps_vel_noise_std",
            value=0.08,
            unit="m/s",
            meaning="Standard deviation of velocity estimation error modeled separately from position",
            provenance="assumed",
            notes="Reflects GNSS Doppler and IMU fused velocity estimation accuracy (e.g., ArduPilot EKF3).",
        ),
        # Control Gains
        "centralized_kp": ParameterProvenance(
            name="centralized_kp",
            value=1.8,
            unit="1/s^2",
            meaning="Centralized position tracking proportional gain",
            provenance="assumed",
            notes="Tuned for critically damped tracking without overshoot.",
        ),
        "centralized_kd": ParameterProvenance(
            name="centralized_kd",
            value=2.2,
            unit="1/s",
            meaning="Centralized velocity tracking derivative gain",
            provenance="assumed",
            notes="Provides damping against velocity overshoot.",
        ),
        "k_repulse": ParameterProvenance(
            name="k_repulse",
            value=4.0,
            unit="m^3/s^2",
            meaning="Artificial Potential Field inverse-distance repulsive gain",
            provenance="assumed",
            notes="Scales emergency repulsive force when separation drops below apf_activation_dist.",
        ),
        "decentralized_kv": ParameterProvenance(
            name="decentralized_kv",
            value=1.6,
            unit="1/s",
            meaning="Decentralized Laplacian velocity consensus alignment gain",
            provenance="assumed",
            notes="Governs rate of convergence of peer velocity consensus: dot{v} = -kv * L * v.",
        ),
        "decentralized_k_form": ParameterProvenance(
            name="decentralized_k_form",
            value=1.4,
            unit="1/s^2",
            meaning="Decentralized relative displacement formation cohesion gain",
            provenance="assumed",
            notes="Pulls drones toward their nominal formation offsets relative to neighbors.",
        ),
        # Network & Hybrid Switching
        "packet_loss_rate": ParameterProvenance(
            name="packet_loss_rate",
            value=0.0,
            unit="ratio",
            meaning="Default wireless link packet drop probability",
            provenance="assumed",
            notes="Zero loss baseline.",
        ),
        "latency_mean": ParameterProvenance(
            name="latency_mean",
            value=0.02,
            unit="s",
            meaning="Mean one-way packet transmission and queuing latency",
            provenance="assumed",
            notes="20 ms typical latency over UDP Wi-Fi links.",
        ),
        "hybrid_degrade_timeout": ParameterProvenance(
            name="hybrid_degrade_timeout",
            value=0.50,
            unit="s",
            meaning="Duration of coordinator silence before triggering fallback to decentralized mode",
            provenance="assumed",
            notes="Ensures rapid fallback after ~5 dropped heartbeats at 10 Hz.",
        ),
        "hybrid_recovery_window": ParameterProvenance(
            name="hybrid_recovery_window",
            value=20,
            unit="packets",
            meaning="Sliding observation window size for coordinator heartbeat reception ratio",
            provenance="assumed",
            notes="Evaluates delivery ratio over the last 20 potential heartbeat slots (2.0s at 10 Hz).",
        ),
        "hybrid_recovery_ratio": ParameterProvenance(
            name="hybrid_recovery_ratio",
            value=0.70,
            unit="ratio",
            meaning="Minimum reception ratio in sliding window required to recover to centralized control",
            provenance="assumed",
            notes="Requires at least 14 of the last 20 packets (70%) to be received before recovering.",
        ),
        "hybrid_dwell_time": ParameterProvenance(
            name="hybrid_dwell_time",
            value=2.00,
            unit="s",
            meaning="Minimum time locked in decentralized fallback mode before recovery is permitted",
            provenance="assumed",
            notes="Dwell-time lockout prevents rapid chattering and mode flapping on intermittent channels.",
        ),
        "hybrid_ramp_duration": ParameterProvenance(
            name="hybrid_ramp_duration",
            value=0.80,
            unit="s",
            meaning="Duration of linear control blending alpha(t) from 0.0 to 1.0 upon recovery",
            provenance="assumed",
            notes="Continuous blending eliminates control acceleration jumps during mode transitions.",
        ),
    }
    return SwarmConfigProfile(
        name="assumed_baseline",
        description="Literature baseline assumption for multirotor attitude dynamics with low-noise sensors.",
        parameters=params,
    )


def _build_sitl_fitted_profile() -> SwarmConfigProfile:
    # Start from baseline profile and update SITL-calibrated values
    profile = _build_assumed_baseline_profile()
    params = dict(profile.parameters)

    # 1. Closed-loop dynamics identified from ArduPilot SITL GUIDED mode step response
    params["attitude_tau"] = ParameterProvenance(
        name="attitude_tau",
        value=0.992,
        unit="s",
        meaning="SITL-calibrated closed-loop position/translation time-constant lag",
        provenance="fitted from SITL",
        notes="Identified from ArduPilot SITL GUIDED mode 5.0m position step response (RMSE = 0.1536m vs SITL telemetry). Captures combined outer position loop, inner rate PID, and vehicle inertia.",
        reference="experiments/validate_step_response_sitl.py",
    )
    params["drag_coeff"] = ParameterProvenance(
        name="drag_coeff",
        value=0.637,
        unit="1/s",
        meaning="SITL-calibrated translational drag/braking deceleration coefficient",
        provenance="fitted from SITL",
        notes="Identified from ArduPilot SITL GUIDED mode 5.0m position step response (RMSE = 0.1536m vs SITL telemetry). Reflects rotor drag and autopilot position controller braking in SITL.",
        reference="experiments/validate_step_response_sitl.py",
    )

    # 2. Sensor noise calibrated for plain GPS (u-blox M10Q without RTK)
    params["gps_noise_std"] = ParameterProvenance(
        name="gps_noise_std",
        value=1.50,
        unit="m",
        meaning="Standard deviation of horizontal position noise for plain GPS",
        provenance="assumed",
        notes="Empirical CEP accuracy of standard u-blox M10Q GNSS receivers (1.0 - 2.5m error).",
        reference="U-blox M10Q GNSS datasheet & Task E research report",
    )

    # 3. Enhanced safety separation buffer to accommodate GPS noise
    params["apf_activation_dist"] = ParameterProvenance(
        name="apf_activation_dist",
        value=1.50,
        unit="m",
        meaning="Repulsive activation boundary under plain GPS sensor noise",
        provenance="assumed",
        notes="Increased to 1.50m to maintain target clearance under 1.5m GPS noise.",
    )

    return SwarmConfigProfile(
        name="sitl_fitted",
        description="SITL-calibrated closed-loop translation model (tau=0.992s, cd=0.637/s) and plain GPS noise (1.5m).",
        parameters=params,
    )


# Registry of available configuration profiles
PROFILES: Dict[str, SwarmConfigProfile] = {
    "assumed_baseline": _build_assumed_baseline_profile(),
    "sitl_fitted": _build_sitl_fitted_profile(),
}

# Active profile global state
_ACTIVE_PROFILE_NAME: str = "assumed_baseline"


def get_profile(name: str = "assumed_baseline") -> SwarmConfigProfile:
    """Retrieve a configuration profile by name."""
    if name not in PROFILES:
        raise ValueError(f"Unknown configuration profile: '{name}'. Available: {list(PROFILES.keys())}")
    return PROFILES[name]


def get_active_profile() -> SwarmConfigProfile:
    """Return the currently active configuration profile."""
    return PROFILES[_ACTIVE_PROFILE_NAME]


def set_active_profile(name: str) -> SwarmConfigProfile:
    """Set the globally active configuration profile."""
    global _ACTIVE_PROFILE_NAME
    if name not in PROFILES:
        raise ValueError(f"Unknown configuration profile: '{name}'. Available: {list(PROFILES.keys())}")
    _ACTIVE_PROFILE_NAME = name
    return PROFILES[_ACTIVE_PROFILE_NAME]


def build_controllers_from_profile(
    profile: Union[str, SwarmConfigProfile],
    nominal_latency: Optional[float] = None,
) -> Tuple[Any, Any, Any]:
    """
    Constructs (CentralizedController, DecentralizedController, HybridController)
    identically configured from the specified configuration profile.
    Guarantees shared gains, safety distances, and hysteresis thresholds
    across both the pure numerical simulator and the SITL integration adapter.
    """
    if isinstance(profile, str):
        profile = get_profile(profile)

    # Local imports to avoid circular dependencies
    from swarm_core.controllers.centralized import CentralizedController
    from swarm_core.controllers.decentralized import DecentralizedController
    from swarm_core.controllers.hybrid import HybridController

    apf_dist = float(profile.get("apf_activation_dist"))
    k_repulse = float(profile.get("k_repulse"))

    central_ctrl = CentralizedController(
        kp=float(profile.get("centralized_kp")),
        kd=float(profile.get("centralized_kd")),
        k_repulse=k_repulse,
        collision_dist=apf_dist,
    )
    decentral_ctrl = DecentralizedController(
        k_sep=k_repulse,
        k_align=float(profile.get("decentralized_kv")),
        k_form=float(profile.get("decentralized_k_form")),
        safe_radius=apf_dist,
    )
    hybrid_ctrl = HybridController(
        degrade_timeout=float(profile.get("hybrid_degrade_timeout")),
        recovery_ratio_threshold=float(profile.get("hybrid_recovery_ratio")),
        min_dwell_time=float(profile.get("hybrid_dwell_time")),
        ramp_duration=float(profile.get("hybrid_ramp_duration")),
        window_size=int(profile.get("hybrid_recovery_window")),
        central_controller=central_ctrl,
        decentral_controller=decentral_ctrl,
    )

    if nominal_latency is not None:
        hybrid_ctrl.set_nominal_latency(float(nominal_latency))
    elif "latency_mean" in profile.parameters:
        hybrid_ctrl.set_nominal_latency(float(profile.get("latency_mean")))

    return central_ctrl, decentral_ctrl, hybrid_ctrl

