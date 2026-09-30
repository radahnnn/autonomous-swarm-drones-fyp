"""
Swarm Graph Theory and Network Topology.
Models the swarm as a dynamic proximity graph G = (V, E) and computes Laplacian matrices.
"""

from typing import List, Tuple
import numpy as np
from swarm_core.drone import Drone


class SwarmGraph:
    """Represents inter-drone communication topology and algebraic connectivity."""

    def __init__(self, comm_radius: float = 8.0):
        self.comm_radius = float(comm_radius)

    def compute_adjacency_matrix(self, drones: List[Drone]) -> np.ndarray:
        """
        Compute adjacency matrix A where A_ij = 1 if ||p_i - p_j|| <= R_comm and i != j.
        Supports smooth spatial decaying weights using smooth bumps if desired.
        """
        n = len(drones)
        adj = np.zeros((n, n), dtype=np.float64)
        for i in range(n):
            for j in range(i + 1, n):
                dist = drones[i].distance_to(drones[j])
                if dist <= self.comm_radius:
                    # Spatial adjacency (can be binary or distance-weighted)
                    adj[i, j] = 1.0
                    adj[j, i] = 1.0
        return adj

    def compute_degree_matrix(self, adj_matrix: np.ndarray) -> np.ndarray:
        """Compute diagonal degree matrix D = diag(sum_j A_ij)."""
        degrees = np.sum(adj_matrix, axis=1)
        return np.diag(degrees)

    def compute_laplacian_matrix(self, adj_matrix: np.ndarray) -> np.ndarray:
        """
        Compute standard graph Laplacian L = D - A.
        Key properties:
          - L is symmetric positive semi-definite for undirected graphs.
          - The smallest eigenvalue lambda_1 = 0 (with eigenvector 1_N).
          - lambda_2 > 0 iff the graph is connected (algebraic connectivity).
        """
        deg = self.compute_degree_matrix(adj_matrix)
        return deg - adj_matrix

    def algebraic_connectivity(self, laplacian: np.ndarray) -> float:
        """
        Returns the Fiedler eigenvalue lambda_2(L).
        lambda_2 > 0 indicates a connected communication graph, essential for consensus.
        """
        eigenvalues = np.sort(np.linalg.eigvalsh(laplacian))
        if len(eigenvalues) < 2:
            return 0.0
        return float(max(0.0, eigenvalues[1]))

    def is_connected(self, laplacian: np.ndarray, tol: float = 1e-4) -> bool:
        """Test whether the swarm network graph is currently connected."""
        return self.algebraic_connectivity(laplacian) > tol

    def get_neighbors(self, drone_idx: int, adj_matrix: np.ndarray) -> List[int]:
        """Return list of indices of active communication neighbors for drone_idx."""
        return list(np.where(adj_matrix[drone_idx] > 0)[0])
