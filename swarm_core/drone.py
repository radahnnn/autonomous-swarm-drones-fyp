"""
Drone State and Kinematics Model.
Represents individual autonomous UAVs in 2D/3D space with acceleration limits.
"""

from typing import Optional
import numpy as np


class Drone:
    """Autonomous drone model with second-order kinematics."""

    def __init__(
        self,
        drone_id: int,
        initial_position: np.ndarray,
        initial_velocity: Optional[np.ndarray] = None,
        max_speed: float = 3.0,
        max_accel: float = 2.5,
        radius: float = 0.35,  # Collision radius in meters
        attitude_tau: float = 0.18,  # First-order attitude / thrust time-constant lag (s)
        drag_coeff: float = 0.20,    # Aerodynamic rotor drag coefficient (1/s)
        measurement_noise_std: float = 0.04,  # Realistic GPS/EKF sensor noise (m)
    ):
        self.id = drone_id
        self.position = np.array(initial_position, dtype=np.float64)
        self.dim = len(self.position)
        
        if initial_velocity is not None:
            self.velocity = np.array(initial_velocity, dtype=np.float64)
        else:
            self.velocity = np.zeros(self.dim, dtype=np.float64)
            
        self.commanded_accel = np.zeros(self.dim, dtype=np.float64)
        self.acceleration = np.zeros(self.dim, dtype=np.float64)  # Actual lagged acceleration
        
        # Physical & dynamics constraints
        self.max_speed = float(max_speed)
        self.max_accel = float(max_accel)
        self.radius = float(radius)
        self.attitude_tau = float(attitude_tau)
        self.drag_coeff = float(drag_coeff)
        self.measurement_noise_std = float(measurement_noise_std)
        self.wind_vector = np.zeros(self.dim, dtype=np.float64)
        
        # Historical trajectory tracking for metrics/plotting
        self.trajectory = [self.position.copy()]
        self.velocity_history = [self.velocity.copy()]

    def set_control_input(self, desired_accel: np.ndarray) -> None:
        """Apply control input (commanded acceleration) subject to saturation limits."""
        accel = np.array(desired_accel, dtype=np.float64)
        norm_a = np.linalg.norm(accel)
        if norm_a > self.max_accel:
            self.commanded_accel = (accel / norm_a) * self.max_accel
        else:
            self.commanded_accel = accel

    def get_measured_position(self, rng: Optional[np.random.Generator] = None) -> np.ndarray:
        """Returns realistic measured position including sensor noise (GPS/EKF)."""
        if self.measurement_noise_std <= 0:
            return self.position.copy()
        if rng is None:
            noise = np.random.normal(0, self.measurement_noise_std, size=self.dim)
        else:
            noise = rng.normal(0, self.measurement_noise_std, size=self.dim)
        return self.position + noise

    def step(self, dt: float) -> None:
        """
        Integrate multirotor physics state over timestep dt:
        1. First-order actuator/attitude lag: a_actual tracks a_cmd with time constant tau.
        2. Aerodynamic rotor drag opposes velocity: -c_d * v.
        3. Net acceleration integrates velocity and position.
        """
        # 1. First-order lag response
        if self.attitude_tau > 1e-4:
            alpha_lag = dt / (self.attitude_tau + dt)
            self.acceleration += alpha_lag * (self.commanded_accel - self.acceleration)
        else:
            self.acceleration = self.commanded_accel.copy()

        # 2. Net acceleration including aerodynamic rotor drag and wind
        a_drag = -self.drag_coeff * self.velocity
        a_net = self.acceleration + a_drag + self.wind_vector

        # 3. v(t+dt) = v(t) + a_net * dt
        self.velocity += a_net * dt
        
        # Speed saturation limit
        speed = np.linalg.norm(self.velocity)
        if speed > self.max_speed:
            self.velocity = (self.velocity / speed) * self.max_speed
            
        # 4. p(t+dt) = p(t) + v(t+dt) * dt
        self.position += self.velocity * dt
        
        # Record trajectory
        self.trajectory.append(self.position.copy())
        self.velocity_history.append(self.velocity.copy())

    def distance_to(self, other: "Drone") -> float:
        """Euclidean distance to another drone."""
        return float(np.linalg.norm(self.position - other.position))

    def is_colliding_with(self, other: "Drone", safety_margin: float = 0.1) -> bool:
        """Check if physical collision boundary is violated."""
        min_allowed_dist = self.radius + other.radius + safety_margin
        return self.distance_to(other) < min_allowed_dist
