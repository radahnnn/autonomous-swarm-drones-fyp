"""Regression checks for the SITL adapter around the research controller."""

import numpy as np

from sitl.live_hybrid import LiveHybridGuidance


def test_velocity_uses_telemetry_interval_and_ignores_duplicate_samples():
    guidance = LiveHybridGuidance(1)
    args = dict(nominal_targets=[[5.0, 0.0, -5.0]], active=[True],
                fresh=[True], coordinator_enabled=True)
    guidance.step([[0.0, 0.0, -5.0]], sample_times=[10.0], now=100.0, **args)
    guidance.step([[0.0, 0.0, -5.0]], sample_times=[10.0], now=100.1, **args)
    guidance.step([[0.2, 0.0, -5.0]], sample_times=[10.5], now=100.2, **args)
    np.testing.assert_allclose(guidance.velocity[0][0], 0.12, atol=1e-6)
    guidance.step([[0.2, 0.0, -5.0]], sample_times=[10.5], now=100.3, **args)
    np.testing.assert_allclose(guidance.velocity[0][0], 0.12, atol=1e-6)


def test_healthy_coordinator_sends_nominal_position_target():
    guidance = LiveHybridGuidance(1)
    sent, modes, alphas, _ = guidance.step(
        [[0.0, 0.0, -5.0]], [[5.0, 1.0, -5.0]], [True], [True], True,
        sample_times=[10.0], now=100.0)
    np.testing.assert_allclose(sent[0], [5.0, 1.0, -5.0])
    assert modes == ["centralized"]
    assert alphas == [1.0]
