import numpy as np

from experiments import validate_step_response_sitl as validation
from swarm_core.config import get_profile


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


def test_corrected_s1_profile_keeps_historical_fit_separate():
    old = get_profile("sitl_default_quad")
    corrected = get_profile("sitl_corrected_s1")
    assert old.get("attitude_tau") == 0.992
    assert old.get("drag_coeff") == 0.637
    assert corrected.get("attitude_tau") == 0.21
    assert corrected.get("drag_coeff") == 0.01
    assert "lower" in corrected.get_provenance("drag_coeff").notes
