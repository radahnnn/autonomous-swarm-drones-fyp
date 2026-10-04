"""Tests that the central config is actually what the code runs with."""

import dataclasses

import numpy as np
import pytest

from swarm_core import config
from swarm_core.config import CONFIG_DYNAMICS, PROFILES, SwarmConfig, get_config
from swarm_core.controllers.centralized import CentralizedController
from swarm_core.controllers.decentralized import DecentralizedController
from swarm_core.controllers.hybrid import HybridController
from swarm_core.drone import Drone
from swarm_core.metrics import SwarmMetricsTracker
from simulator.engine import SwarmSimulation


def test_fitted_profile_matches_provenance_table():
    cfg = get_config("fitted_sitl")
    assert cfg.attitude_tau == CONFIG_DYNAMICS["attitude_tau"].value == pytest.approx(0.992)
    assert cfg.drag_coeff == CONFIG_DYNAMICS["drag_coeff"].value == pytest.approx(0.637)


def test_assumed_profile_keeps_original_values():
    cfg = get_config("assumed")
    assert (cfg.attitude_tau, cfg.drag_coeff) == (0.18, 0.20)


def test_unknown_profile_rejected():
    with pytest.raises(ValueError):
        get_config("nope")


def test_config_is_frozen():
    with pytest.raises(dataclasses.FrozenInstanceError):
        PROFILES["fitted_sitl"].attitude_tau = 1.0


def test_drone_defaults_come_from_default_config():
    d = Drone(drone_id=0, initial_position=[0.0, 0.0])
    c = config.DEFAULT_CONFIG
    assert d.attitude_tau == c.attitude_tau
    assert d.drag_coeff == c.drag_coeff
    assert d.radius == c.drone_radius
    assert d.max_speed == c.max_speed and d.max_accel == c.max_accel


def test_drone_explicit_args_override_config():
    d = Drone(drone_id=0, initial_position=[0.0, 0.0], attitude_tau=0.5, drag_coeff=0.1, radius=0.2)
    assert (d.attitude_tau, d.drag_coeff, d.radius) == (0.5, 0.1, 0.2)


def test_safety_radius_is_single_source():
    c = config.DEFAULT_CONFIG
    assert CentralizedController().collision_dist == c.apf_radius
    assert DecentralizedController().safe_radius == c.apf_radius
    assert HybridController().decentral_controller.safe_radius == c.apf_radius


def test_collision_threshold_is_derived_from_radius():
    c = SwarmConfig(drone_radius=0.4)
    assert c.collision_threshold == pytest.approx(0.8)
    assert SwarmMetricsTracker().collision_threshold == pytest.approx(config.DEFAULT_CONFIG.collision_threshold)


def test_hybrid_defaults_come_from_config():
    h, c = HybridController(), config.DEFAULT_CONFIG
    assert h.degrade_timeout == c.degrade_timeout
    assert h.min_dwell_time == c.min_dwell_time
    assert h.window_size == c.recovery_window_size


def test_engine_feedforward_uses_vehicle_drag():
    drones = [Drone(drone_id=i, initial_position=[3.0 * i, 0.0], drag_coeff=0.9) for i in range(3)]
    sim = SwarmSimulation(drones, control_mode="hybrid", seed=1)
    assert sim.drag_coeff == pytest.approx(0.9)
