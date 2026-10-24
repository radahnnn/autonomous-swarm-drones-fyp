"""Tests for the identified ArduPilot setpoint-tracking model."""

from pathlib import Path

import numpy as np
import pytest

from swarm_core.autopilot_tracking import (
    AutopilotTrackingModel, AutopilotTrackingParams,
    inverse_sqrt_controller, sqrt_controller,
)
from swarm_core.drone import Drone

DATA = Path(__file__).resolve().parent.parent / "docs" / "reports" / "hw_matched_identification" / "steps" / "step_5m_r1" / "sitl_step_response.csv"


def _run(model, setpoint, seconds, dt=0.05):
    xs, vs, accs = [], [], []
    prev_v = model.v.copy()
    for _ in range(int(seconds / dt)):
        model.step(dt, setpoint)
        xs.append(model.p.copy())
        vs.append(model.v.copy())
        accs.append(np.linalg.norm(model.v - prev_v) / dt)
        prev_v = model.v.copy()
    return np.array(xs), np.array(vs), np.array(accs)


def test_step_reaches_target_without_large_overshoot():
    m = AutopilotTrackingModel([0.0, 0.0])
    xs, vs, _ = _run(m, [5.0, 0.0], 12.0)
    assert xs[-1, 0] == pytest.approx(5.0, abs=0.05)
    assert abs(xs[-1, 1]) < 1e-6
    assert xs[:, 0].max() < 5.0 + 0.25


def test_no_motion_before_the_transport_delay():
    prm = AutopilotTrackingParams()
    m = AutopilotTrackingModel([0.0, 0.0], prm)
    xs, _, _ = _run(m, [5.0, 0.0], prm.td - 0.05)
    assert np.allclose(xs, 0.0)


def test_speed_and_acceleration_limits_hold():
    prm = AutopilotTrackingParams()
    m = AutopilotTrackingModel([0.0, 0.0], prm)
    _, vs, accs = _run(m, [30.0, 0.0], 8.0)
    assert np.linalg.norm(vs, axis=1).max() <= prm.v_max * 1.1
    assert accs.max() <= prm.a_max * 1.25


def test_diagonal_step_stays_on_the_line():
    m = AutopilotTrackingModel([1.0, -2.0])
    xs, _, _ = _run(m, [1.0 + 3.0, -2.0 + 3.0], 12.0)
    rel = xs - np.array([1.0, -2.0])
    assert np.max(np.abs(rel[:, 0] - rel[:, 1])) < 1e-6
    assert rel[-1] == pytest.approx([3.0, 3.0], abs=0.05)


def test_sqrt_controller_is_linear_near_target_and_limited_far_away():
    near = sqrt_controller(np.array([0.1, 0.0]), 1.0, 2.5)
    assert near[0] == pytest.approx(0.1)
    far = np.linalg.norm(sqrt_controller(np.array([100.0, 0.0]), 1.0, 2.5))
    assert far == pytest.approx(np.sqrt(2 * 2.5 * (100.0 - 2.5 / 2)))
    assert np.all(sqrt_controller(np.zeros(2), 1.0, 2.5) == 0.0)


def test_inverse_sqrt_controller_recovers_requested_velocity():
    for velocity in (np.array([0.2, 0.0]), np.array([2.2, -1.1])):
        error = inverse_sqrt_controller(velocity, 0.923, 2.657)
        np.testing.assert_allclose(sqrt_controller(error, 0.923, 2.657), velocity)


def test_phase2_s4_bridge_adds_setpoint_delay_and_changes_dynamics():
    s4 = Drone(0, np.zeros(2), profile="sitl_s4_bridge")
    s1 = Drone(1, np.zeros(2), profile="sitl_corrected_s1")
    for _ in range(4):
        for drone in (s4, s1):
            drone.set_control_input(np.array([1.0, 0.0]))
            drone.step(0.05)
    np.testing.assert_allclose(s4.position, 0.0, atol=1e-12)
    assert s1.position[0] > 0.0
    for _ in range(80):
        s4.set_control_input(np.array([1.0, 0.0]))
        s4.step(0.05)
    assert s4.position[0] > 0.0
    assert np.isfinite(s4.position).all()
    assert len(s4.trajectory) == 85


@pytest.mark.skipif(not DATA.exists(), reason="recorded SITL step not available")
def test_model_reproduces_the_recorded_5m_step():
    raw = np.genfromtxt(DATA, delimiter=",", names=True)
    t = raw["time"] - raw["time"][0]
    x = raw["x"] - raw["x"][0]
    m = AutopilotTrackingModel([0.0, 0.0])
    pred = []
    t_prev = 0.0
    for ti in t:
        dt = ti - t_prev
        if dt > 0:
            m.step(dt, [5.0, 0.0])
        pred.append(m.p[0])
        t_prev = ti
    rmse = float(np.sqrt(np.mean((np.array(pred) - x) ** 2)))
    assert rmse < 0.08
