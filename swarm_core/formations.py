"""
Swarm Formation Geometries and Optimal Slot Assignment.
Supports: Line, V-Formation, Circle, and Grid formations, plus Hungarian matching.
"""

from enum import Enum
from typing import Dict, List, Optional, Tuple
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


def create_world_slots(local_offsets: np.ndarray, centroid: np.ndarray) -> np.ndarray:
    """
    Translates local relative formation offsets to world coordinates given centroid.
    Works for both 2D (Nx2) and 3D (Nx3) offset arrays.
    """
    local = np.asarray(local_offsets, dtype=np.float64)
    cent = np.asarray(centroid, dtype=np.float64)
    dim = min(local.shape[1], cent.shape[0])
    world = np.zeros_like(local)
    world[:, :dim] = local[:, :dim] + cent[:dim]
    if local.shape[1] > dim:
        world[:, dim:] = local[:, dim:]
    return world


def compute_formation_slots(
    formation_type: FormationType,
    num_drones: int,
    centroid: np.ndarray,
    spacing: float = 2.5,
    current_positions: Optional[np.ndarray] = None,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    Unified pipeline for formation slot geometry calculation:
    1. Computes local relative offsets centered at (0, 0).
    2. Translates offsets to world coordinates anchored at centroid.
    3. If current_positions is supplied, computes Hungarian optimal matching
       minimizing total sum of squared travel distances to mitigate path crossings.
       Otherwise, assigned_slots defaults to world_slots in index order.

    Returns:
        (local_offsets, world_slots, assigned_slots)
    """
    local_offsets = FormationGenerator.get_formation_offsets(
        formation_type, num_drones, spacing=spacing
    )
    centroid_arr = np.asarray(centroid, dtype=np.float64)
    world_slots = create_world_slots(local_offsets, centroid_arr[:2])

    if current_positions is not None:
        pos_2d = np.asarray(current_positions, dtype=np.float64)[:, :2]
        assigned_slots = assign_optimal_slots(pos_2d, world_slots)
    else:
        assigned_slots = world_slots.copy()

    return local_offsets, world_slots, assigned_slots


def compute_desired_neighbor_offsets(
    drone_id: int,
    assigned_targets: np.ndarray,
    neighbor_ids: List[int],
    drone_ids: Optional[List[int]] = None,
) -> Dict[int, np.ndarray]:
    """
    Computes desired relative separation vector (p_target_i - p_target_j)
    for each perceived neighbor j. Used by decentralized consensus flocking.
    
    Parameters:
        drone_id: ID of the evaluating drone.
        assigned_targets: (N, 2) array of assigned target coordinates.
        neighbor_ids: List of neighbor drone IDs.
        drone_ids: Ordered list of all drone IDs matching rows of assigned_targets.
                   Defaults to [0, 1, ..., N-1] if None.
    """
    if drone_ids is None:
        id_to_idx = {i: i for i in range(len(assigned_targets))}
    else:
        id_to_idx = {did: idx for idx, did in enumerate(drone_ids)}

    if drone_id not in id_to_idx:
        return {}

    my_idx = id_to_idx[drone_id]
    my_target = assigned_targets[my_idx]

    desired_offsets = {}
    for nid in neighbor_ids:
        if nid in id_to_idx:
            n_idx = id_to_idx[nid]
            desired_offsets[nid] = my_target - assigned_targets[n_idx]

    return desired_offsets

