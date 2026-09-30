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
        radius: float = 0.35,  # Collision radius in meters (typical for 300-450mm quad)
    ):
        self.id = drone_id
        self.position = np.array(initial_position, dtype=np.float64)
        self.dim = len(self.position)
        
        if initial_velocity is not None:
            self.velocity = np.array(initial_velocity, dtype=np.float64)
        else:
            self.velocity = np.zeros(self.dim, dtype=np.float64)
            
        self.acceleration = np.zeros(self.dim, dtype=np.float64)
        
        # Physical constraints
        self.max_speed = float(max_speed)
        self.max_accel = float(max_accel)
        self.radius = float(radius)
        
        # Historical trajectory tracking for metrics/plotting
        self.trajectory = [self.position.copy()]
        self.velocity_history = [self.velocity.copy()]

    def set_control_input(self, desired_accel: np.ndarray) -> None:
        """Apply control input (commanded acceleration) subject to saturation limits."""
        accel = np.array(desired_accel, dtype=np.float64)
        norm_a = np.linalg.norm(accel)
        if norm_a > self.max_accel:
            self.acceleration = (accel / norm_a) * self.max_accel
        else:
            self.acceleration = accel

    def step(self, dt: float) -> None:
        """Integrate state by dt using semi-implicit Euler integration."""
        # v(t+dt) = v(t) + a(t)*dt
        self.velocity += self.acceleration * dt
        
        # Speed saturation limit
        speed = np.linalg.norm(self.velocity)
        if speed > self.max_speed:
            self.velocity = (self.velocity / speed) * self.max_speed
            
        # p(t+dt) = p(t) + v(t+dt)*dt
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
