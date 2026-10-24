import numpy as np

from experiments import validate_step_response_sitl as validation


def test_swarm_core_step_uses_each_recorded_timestamp_interval(monkeypatch):
    intervals = []

    class RecordingDrone:
        def __init__(self, **kwargs):
            self.position = np.zeros(2)
            self.velocity = np.zeros(2)

        def set_control_input(self, _acceleration):
            pass

        def step(self, dt):
            intervals.append(dt)
            self.position[0] += dt

    monkeypatch.setattr(validation, "Drone", RecordingDrone)

    times = np.array([0.0, 0.10, 0.35, 0.45])
    x, _ = validation.simulate_swarm_core_step(times, tau=0.2, drag=0.0)

    # The first recorded interval is deliberately unequal to the later ones;
    # it must not be reused as a fixed timestep.
    np.testing.assert_allclose(intervals, [0.10, 0.25, 0.10])
    assert x.shape == times.shape
