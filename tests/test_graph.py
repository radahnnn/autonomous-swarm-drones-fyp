import numpy as np
from swarm_core.drone import Drone
from swarm_core.graph import SwarmGraph


def test_graph_laplacian():
    drones = [
        Drone(drone_id=0, initial_position=[0.0, 0.0]),
        Drone(drone_id=1, initial_position=[2.0, 0.0]),
        Drone(drone_id=2, initial_position=[4.0, 0.0]),
    ]
    graph = SwarmGraph(comm_radius=2.5)
    adj = graph.compute_adjacency_matrix(drones)
    
    # 0 connects to 1 (dist 2.0 <= 2.5)
    # 1 connects to 2 (dist 2.0 <= 2.5)
    # 0 does NOT connect to 2 (dist 4.0 > 2.5)
    assert adj[0, 1] == 1.0
    assert adj[1, 2] == 1.0
    assert adj[0, 2] == 0.0

    lap = graph.compute_laplacian_matrix(adj)
    # Row sums of Laplacian must be zero
    assert np.allclose(np.sum(lap, axis=1), 0.0)

    # Graph is connected line: Fiedler eigenvalue lambda_2 > 0
    fiedler = graph.algebraic_connectivity(lap)
    assert fiedler > 0.0
    assert graph.is_connected(lap)


def test_disconnected_graph():
    drones = [
        Drone(drone_id=0, initial_position=[0.0, 0.0]),
        Drone(drone_id=1, initial_position=[100.0, 100.0]),
    ]
    graph = SwarmGraph(comm_radius=5.0)
    adj = graph.compute_adjacency_matrix(drones)
    lap = graph.compute_laplacian_matrix(adj)
    assert not graph.is_connected(lap)
    assert graph.algebraic_connectivity(lap) == 0.0


if __name__ == "__main__":
    test_graph_laplacian()
    test_disconnected_graph()
    print("Graph tests passed successfully!")
