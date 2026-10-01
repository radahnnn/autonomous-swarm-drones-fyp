"""
Unit tests for SITL CommonCoordinateFrame and MAVLinkSwarmAdapter (Task C Item 3).
Tests:
1. CommonCoordinateFrame round trip precision: converts NED -> GPS -> NED and verifies error < 1 cm within 100m.
2. MAVLinkDroneInterface & MAVLinkSwarmAdapter against a mocked MAVLink connection:
   Verifies telemetry polling, frame transformations, controller execution, and setpoint dispatch.
"""

from unittest.mock import MagicMock
import numpy as np
import pytest

from sitl.common_frame import CommonCoordinateFrame
from sitl.mavlink_swarm_adapter import MAVLinkSwarmAdapter, MAVLinkDroneInterface
from swarm_core.formations import FormationType


def test_common_frame_round_trip():
    """
    Verifies that converting between Global NED coordinates and WGS84 GPS (Lat, Lon, Alt)
    has a round-trip error strictly under 1 cm (0.01 m) within a 100-meter radius.
    """
    frame = CommonCoordinateFrame(datum_lat=-35.3632621, datum_lon=149.1652374, datum_alt=584.0)

    # Grid of test points across 100m boundary: North, East in [-100, 100], Down in [-50, 0]
    test_points = [
        np.array([0.0, 0.0, 0.0]),
        np.array([100.0, 0.0, -10.0]),
        np.array([-100.0, 0.0, -10.0]),
        np.array([0.0, 100.0, -20.0]),
        np.array([0.0, -100.0, -20.0]),
        np.array([70.71, 70.71, -15.0]),   # ~100m radial
        np.array([-70.71, -70.71, -25.0]), # ~100m radial
        np.array([50.0, -50.0, -5.0]),
        np.array([-30.0, 80.0, -35.0]),
    ]

    for pt in test_points:
        north, east, down = pt[0], pt[1], pt[2]
        # 1. Forward transform: NED -> GPS
        lat, lon, rel_alt = frame.global_ned_to_gps(north, east, down)

        # 2. Inverse transform: GPS -> NED
        reconstructed_ned = frame.gps_to_global_ned(lat, lon, rel_alt)

        # 3. Compute Euclidean error
        error_m = float(np.linalg.norm(pt - reconstructed_ned))
        assert error_m < 0.01, f"Round-trip error {error_m*1000:.3f}mm exceeds 1cm threshold at point {pt}"
        # Even stricter check: should be under 0.1 mm due to double precision
        assert error_m < 0.0001, f"Numerical precision degraded: error was {error_m*1000:.4f}mm"


def create_mock_mavlink_msg(msg_type: str, **kwargs):
    """Factory creating a mock pymavlink message object."""
    msg = MagicMock()
    msg.get_type.return_value = msg_type
    for k, v in kwargs.items():
        setattr(msg, k, v)
    return msg


def test_mavlink_adapter_with_mock_connection():
    """
    Tests MAVLinkSwarmAdapter control cycle using mocked MAVLink telemetry and command channels.
    """
    frame = CommonCoordinateFrame(datum_lat=-35.3632621, datum_lon=149.1652374, datum_alt=584.0)
    adapter = MAVLinkSwarmAdapter(control_mode="hybrid", packet_loss=0.0)

    # Setup mock connections for each of the 3 drones
    mock_conns = []
    initial_offsets = [
        np.array([0.0, 0.0, -5.0]),    # Drone 1: (0, 0)
        np.array([-3.0, -3.5, -5.0]),  # Drone 2: Left wing
        np.array([-3.0, 3.5, -5.0]),   # Drone 3: Right wing
    ]

    for i, iface in enumerate(adapter.interfaces):
        mock_conn = MagicMock()
        mock_conn.target_system = iface.sysid
        mock_conn.target_component = 1
        iface.conn = mock_conn
        iface.connected = True

        # Generate GPS coordinates for initial drone position
        lat, lon, rel_alt = frame.global_ned_to_gps(
            initial_offsets[i][0], initial_offsets[i][1], initial_offsets[i][2]
        )

        # Mock incoming telemetry packet queue
        telemetry_queue = [
            create_mock_mavlink_msg(
                "HEARTBEAT",
                base_mode=128,  # Armed
                custom_mode=4,  # GUIDED
            ),
            create_mock_mavlink_msg(
                "GLOBAL_POSITION_INT",
                lat=int(lat * 1e7),
                lon=int(lon * 1e7),
                relative_alt=int(rel_alt * 1000),
                vx=0, vy=0, vz=0,
            ),
            create_mock_mavlink_msg(
                "ATTITUDE",
                yaw=0.0,
            ),
            None,  # End of message queue for this poll
        ]
        mock_conn.recv_match.side_effect = telemetry_queue
        mock_conns.append(mock_conn)

    # Execute 1 control cycle
    target_setpoints = adapter.run_control_cycle(current_time=0.1, dt=0.1)

    # Verify setpoints generated for all 3 drones
    assert len(target_setpoints) == 3
    for tgt in target_setpoints:
        assert tgt.shape == (3,)
        assert np.isclose(tgt[2], -adapter.cruise_alt, atol=0.1)

    # Verify each interface polled telemetry and updated position
    for i, iface in enumerate(adapter.interfaces):
        assert np.allclose(iface.global_ned[:2], initial_offsets[i][:2], atol=0.05)
        # Verify set_position_target_local_ned_send was invoked
        mock_conn = mock_conns[i]
        assert mock_conn.mav.set_position_target_local_ned_send.called
        call_args = mock_conn.mav.set_position_target_local_ned_send.call_args[0]
        # call_args: (time_boot_ms, target_system, target_component, frame, type_mask, x, y, z, ...)
        assert call_args[1] == iface.sysid


def test_mavlink_adapter_formation_morph():
    """
    Verifies that morphing formations updates target setpoint geometry in the mock adapter.
    """
    adapter = MAVLinkSwarmAdapter(control_mode="centralized", packet_loss=0.0)

    for iface in adapter.interfaces:
        iface.conn = MagicMock()
        iface.connected = True
        iface.home_global_ned = np.zeros(3)
        iface.global_ned = np.zeros(3)
        iface.conn.recv_match.return_value = None

    # V-Shape
    adapter.set_formation(FormationType.V_SHAPE)
    v_targets = adapter.run_control_cycle(current_time=0.1, dt=0.1)

    # Line
    adapter.set_formation(FormationType.LINE)
    line_targets = adapter.run_control_cycle(current_time=0.2, dt=0.1)

    # Targets must differ between V-Shape and Line
    assert not np.allclose(v_targets[1], line_targets[1])
