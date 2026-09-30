"""
Swarm Formation Geometries and Optimal Slot Assignment.
Supports: Line, V-Formation, Circle, and Grid formations, plus Hungarian matching.
"""

from enum import Enum
from typing import Dict, List, Optional
import numpy as np
from scipy.optimize import linear_sum_assignment


class FormationType(Enum):
    LINE = "line"
    V_SHAPE = "v_shape"
    CIRCLE = "circle"
    GRID = "grid"


class FormationGenerator:
    """Computes nominal relative offsets for swarm formation geometries."""

    @staticmethod
    def generate_line(num_drones: int, spacing: float = 2.0, orientation_rad: float = 0.0) -> np.ndarray:
        """
        Generate centered linear formation along the given orientation angle.
        Slots are spaced evenly along the perpendicular or specified axis.
        """
        # Center the line at (0, 0)
        indices = np.arange(num_drones) - (num_drones - 1) / 2.0
        offsets = np.zeros((num_drones, 2), dtype=np.float64)
        
        # Along Y axis by default, then rotate
        offsets[:, 1] = indices * spacing
        
        # Rotate by orientation_rad
        c, s = np.cos(orientation_rad), np.sin(orientation_rad)
        rot = np.array([[c, -s], [s, c]])
        return (rot @ offsets.T).T

    @staticmethod
    def generate_v_shape(
        num_drones: int,
        spacing: float = 2.0,
        apex_angle_deg: float = 60.0,
        orientation_rad: float = 0.0,
    ) -> np.ndarray:
        """
        Generate V-formation (chevron) with apex leader at slot 0.
        apex_angle_deg is the angle between the two wings (e.g. 60 deg).
        """
        offsets = np.zeros((num_drones, 2), dtype=np.float64)
        half_angle = np.radians(apex_angle_deg / 2.0)
        
        # Leader at (0, 0)
        offsets[0] = [0.0, 0.0]
        
        left_idx = 1
        right_idx = 2
        step = 1
        
        while left_idx < num_drones:
            dist = step * spacing
            # Trailing behind X (forward is +X, trailing is -X)
            dx = -dist * np.cos(half_angle)
            dy = dist * np.sin(half_angle)
            
            offsets[left_idx] = [dx, dy]
            if right_idx < num_drones:
                offsets[right_idx] = [dx, -dy]
                
            step += 1
            left_idx += 2
            right_idx += 2

        # Center the centroid of the formation to (0, 0)
        centroid = np.mean(offsets, axis=0)
        offsets -= centroid

        # Rotate
        c, s = np.cos(orientation_rad), np.sin(orientation_rad)
        rot = np.array([[c, -s], [s, c]])
        return (rot @ offsets.T).T

    @staticmethod
    def generate_circle(num_drones: int, radius: Optional[float] = None, min_spacing: float = 2.0) -> np.ndarray:
        """Generate regular polygonal circle formation centered at (0, 0)."""
        if radius is None:
            # Ensure chord length between adjacent drones >= min_spacing
            # chord = 2 * R * sin(pi / N) => R >= min_spacing / (2 * sin(pi / N))
            radius = max(2.0, min_spacing / (2.0 * np.sin(np.pi / num_drones)))
            
        angles = np.linspace(0, 2 * np.pi, num_drones, endpoint=False)
        offsets = np.zeros((num_drones, 2), dtype=np.float64)
        offsets[:, 0] = radius * np.cos(angles)
        offsets[:, 1] = radius * np.sin(angles)
        return offsets

    @staticmethod
    def generate_grid(num_drones: int, spacing: float = 2.0) -> np.ndarray:
        """Generate 2D grid/matrix formation."""
        # Find closest rectangular shape (cols >= rows)
        cols = int(np.ceil(np.sqrt(num_drones)))
        rows = int(np.ceil(num_drones / cols))
        
        slots = []
        for r in range(rows):
            for c in range(cols):
                if len(slots) < num_drones:
                    slots.append([c * spacing, r * spacing])
                    
        offsets = np.array(slots, dtype=np.float64)
        # Center at origin
        offsets -= np.mean(offsets, axis=0)
        return offsets

    @classmethod
    def get_formation_offsets(
        cls,
        formation_type: FormationType,
        num_drones: int,
        spacing: float = 2.5,
    ) -> np.ndarray:
        """Factory method returning local offsets for the requested formation type."""
        if formation_type == FormationType.LINE:
            return cls.generate_line(num_drones, spacing=spacing)
        elif formation_type == FormationType.V_SHAPE:
            return cls.generate_v_shape(num_drones, spacing=spacing)
        elif formation_type == FormationType.CIRCLE:
            return cls.generate_circle(num_drones, min_spacing=spacing)
        elif formation_type == FormationType.GRID:
            return cls.generate_grid(num_drones, spacing=spacing)
        else:
            raise ValueError(f"Unknown formation type: {formation_type}")


def assign_optimal_slots(current_positions: np.ndarray, target_slots: np.ndarray) -> np.ndarray:
    """
    Solves the Linear Sum Assignment Problem (Hungarian Algorithm) to minimize total travel distance:
        min sum_{i} ||p_i - s_sigma(i)||^2
    Returns an array of shape (num_drones, 2) where row i is the target position assigned to drone i.
    Prevents path crossing and collisions during formation transitions.
    """
    num_drones = len(current_positions)
    cost_matrix = np.zeros((num_drones, num_drones), dtype=np.float64)
    
    for i in range(num_drones):
        for j in range(num_drones):
            cost_matrix[i, j] = np.sum((current_positions[i] - target_slots[j]) ** 2)
            
    row_ind, col_ind = linear_sum_assignment(cost_matrix)
    assigned_targets = np.zeros_like(current_positions)
    for r, c in zip(row_ind, col_ind):
        assigned_targets[r] = target_slots[c]
        
    return assigned_targets
