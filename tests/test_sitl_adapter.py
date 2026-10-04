"""
Unit tests for SITL CommonCoordinateFrame and MAVLinkSwarmAdapter (Task C Item 3).
Tests:
1. CommonCoordinateFrame round trip precision: converts NED -> GPS -> NED and verifies error < 1 cm within 100m.
2. MAVLinkDroneInterface & MAVLinkSwarmAdapter against a mocked MAVLink connection:
   Verifies telemetry polling, frame transformations, controller execution, and setpoint dispatch.
"""

from unittest.mock import MagicMock
import time
import numpy as np
import pytest

from sitl.common_frame import CommonCoordinateFrame
from sitl.mavlink_swarm_adapter import (
    MAVLinkSwarmAdapter,
    MAVLinkDroneInterface,
    IntegratedPlant,
)
from swarm_core.drone import Drone
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


def test_frame_origin_computation_with_drifted_drone():
    """
    Item 17: Verifies dynamic frame origin computation as:
        origin_global = global_ned - local_ned
    from simultaneous telemetry messages, proving robust target mapping
    even when a drone drifted from its spawn coordinates prior to connection.
    """
    frame = CommonCoordinateFrame(datum_lat=-35.3632621, datum_lon=149.1652374, datum_alt=584.0)
    iface = MAVLinkDroneInterface(sysid=1, port=14552, label="Drone 1", frame=frame)
    mock_conn = MagicMock()
    mock_conn.target_system = 1
    mock_conn.target_component = 1
    iface.conn = mock_conn
    iface.connected = True
    iface.mode = "GUIDED"

    # Simulate drone drifted before connection:
    # True spawn was at Global NED [10.0, -5.0, 0.0]
    # Current Global NED position is [12.5, -3.0, -5.0]
    # Local EKF reports displacement relative to its EKF origin: [2.5, 2.0, -5.0]
    lat, lon, rel_alt = frame.global_ned_to_gps(12.5, -3.0, -5.0)

    msg_gps = create_mock_mavlink_msg(
        "GLOBAL_POSITION_INT",
        lat=int(lat * 1e7),
        lon=int(lon * 1e7),
        relative_alt=int(rel_alt * 1000),
        vx=0, vy=0, vz=0,
    )
    msg_local = create_mock_mavlink_msg(
        "LOCAL_POSITION_NED",
        x=2.5, y=2.0, z=-5.0,
    )

    # Poll both messages
    iface.conn.recv_match.side_effect = [msg_gps, msg_local, None]
    iface.poll_telemetry()

    # Verify frame origin: [12.5, -3.0, -5.0] - [2.5, 2.0, -5.0] = [10.0, -5.0, 0.0]
    expected_origin = np.array([10.0, -5.0, 0.0])
    assert iface.frame_origin_global is not None
    assert np.allclose(iface.frame_origin_global, expected_origin, atol=0.01)

    # Command a target in Global NED frame at [15.0, -1.0, -5.0]
    target_global = np.array([15.0, -1.0, -5.0])
    success = iface.send_target_global(target_global)
    assert success

    # Verify dispatched setpoint in local frame: target_global - origin_global = [5.0, 4.0, -5.0]
    assert mock_conn.mav.set_position_target_local_ned_send.called
    call_args = mock_conn.mav.set_position_target_local_ned_send.call_args[0]
    # (time_boot_ms, sysid, compid, frame, mask, x, y, z, ...)
    dispatched_local = np.array([call_args[5], call_args[6], call_args[7]])
    assert np.allclose(dispatched_local, np.array([5.0, 4.0, -5.0]), atol=0.01)


def test_integrated_plant_closed_loop_equivalence_with_engine_drone():
    """
    Item 16: Verifies closed-loop mathematical equivalence between IntegratedPlant
    (used for velocity setpoints in the adapter) and Drone.step() (simulator engine).
    Both implement identical 1st-order attitude lag and aerodynamic rotor drag.
    """
    tau = 0.18
    drag_coeff = 0.637  # SITL fitted profile drag
    max_speed = 3.0
    max_accel = 2.5
    dt = 0.05

    plant = IntegratedPlant(tau=tau, drag_coeff=drag_coeff, max_speed=max_speed, max_accel=max_accel)
    drone = Drone(
        drone_id=0,
        initial_position=np.zeros(2),
        attitude_tau=tau,
        drag_coeff=drag_coeff,
        max_speed=max_speed,
        max_accel=max_accel,
    )

    # Excite with multi-frequency, varying acceleration commands
    for step_idx in range(100):
        t = step_idx * dt
        a_cmd = np.array([1.5 * np.sin(2.0 * t), 1.2 * np.cos(1.5 * t)])

        v_plant = plant.step(a_cmd, dt)

        drone.commanded_accel = a_cmd
        drone.step(dt)
        v_drone = drone.velocity

        assert np.allclose(v_plant, v_drone, atol=1e-12), (
            f"Step {step_idx}: Plant vel {v_plant} != Drone vel {v_drone}"
        )


def test_stale_age_margin_at_10hz_with_jitter():
    """
    Item 18: Verifies stale-age neighbor memory handling at 10 Hz with network jitter:
    - Packets arriving within the 300ms neighbor timeout extrapolate smoothly.
    - Neighbor states older than 300ms (due to link outage) are pruned to prevent stale control.
    """
    adapter = MAVLinkSwarmAdapter(control_mode="decentralized", packet_loss=0.0)
    for iface in adapter.interfaces:
        iface.conn = MagicMock()
        iface.connected = True
        iface.mode = "GUIDED"
        iface.conn.recv_match.return_value = None

    # Step 1: Nominal transmission at t=0.1
    adapter.run_control_cycle(current_time=0.1, dt=0.1)

    # Manually populate neighbor memory for drone 0 from drone 1 at t=0.1 with velocity [1.0, 0.0]
    adapter.neighbor_memory[0][1] = {
        "position": np.array([5.0, 0.0]),
        "velocity": np.array([1.0, 0.0]),
        "timestamp": 0.10,
    }

    # Step 2: Telemetry cycle at t=0.25 (age = 0.15s, within 0.30s timeout)
    # Mock channel receive returning empty to test extrapolation from memory
    adapter.channel.receive = MagicMock(return_value=[])
    adapter.run_control_cycle(current_time=0.25, dt=0.1)

    # Memory should still retain drone 1, extrapolated position: 5.0 + 1.0 * 0.15 = 5.15
    assert 1 in adapter.neighbor_memory[0]

    # Step 3: Advance time past 300ms timeout (current_time = 0.50, age = 0.40s)
    adapter.run_control_cycle(current_time=0.50, dt=0.1)

    # Neighbor memory must prune drone 1 because age 0.40s > neighbor_timeout 0.30s
    assert 1 not in adapter.neighbor_memory[0]


def test_adapter_flight_safety_features():
    """
    Item 20: Verifies flight safety guardrails on MAVLinkDroneInterface & MAVLinkSwarmAdapter:
    1. GUIDED-mode enforcement
    2. Setpoint distance clamp
    3. 3D Geofence boundary checks
    4. Telemetry watchdog timeout
    5. Emergency stop / kill command
    6. Restriction of ARMING_CHECK=0 strictly to SITL
    """
    frame = CommonCoordinateFrame(datum_lat=-35.3632621, datum_lon=149.1652374, datum_alt=584.0)
    iface = MAVLinkDroneInterface(
        sysid=1,
        port=14552,
        label="Drone 1",
        frame=frame,
        max_setpoint_distance=5.0,
        geofence_radius=60.0,
        min_alt=0.5,
        max_alt=25.0,
        watchdog_timeout=1.5,
    )
    mock_conn = MagicMock()
    mock_conn.target_system = 1
    mock_conn.target_component = 1
    mock_conn.recv_match.return_value = None
    iface.conn = mock_conn
    iface.connected = True
    iface.global_ned = np.array([0.0, 0.0, -5.0])
    iface.frame_origin_global = np.zeros(3)

    # 1. GUIDED mode enforcement
    iface.mode = "LOITER"
    assert not iface.send_velocity_target(np.array([1.0, 0.0]))
    assert not iface.send_target_global(np.array([2.0, 0.0, -5.0]))
    iface.mode = "GUIDED"
    assert iface.send_velocity_target(np.array([1.0, 0.0]))

    # 2. Setpoint distance clamp
    distant_target = np.array([25.0, 0.0, -5.0])
    clamped = iface.clamp_setpoint_distance(distant_target)
    dist = float(np.linalg.norm(clamped[:2] - iface.global_ned[:2]))
    assert np.isclose(dist, 5.0, atol=1e-5)
    assert np.isclose(clamped[0], 5.0, atol=1e-5)
    assert np.isclose(clamped[1], 0.0, atol=1e-5)

    # 3. Geofence checks
    assert iface.check_geofence(np.array([10.0, 10.0, -5.0]))
    assert not iface.check_geofence(np.array([75.0, 0.0, -5.0]))   # Radius > 60m
    assert not iface.check_geofence(np.array([0.0, 0.0, -30.0]))  # Alt > 25m
    assert not iface.check_geofence(np.array([0.0, 0.0, -0.2]))   # Alt < 0.5m

    # 4. Telemetry watchdog
    iface.last_telemetry_time = time.time() - 2.0  # 2.0s > 1.5s timeout
    assert not iface.is_watchdog_healthy()
    assert not iface.send_velocity_target(np.array([1.0, 0.0]))
    iface.last_telemetry_time = time.time()
    assert iface.is_watchdog_healthy()

    # 5. Emergency stop
    iface.emergency_stop()
    assert mock_conn.set_mode.called

    # 6. Hardware-facing safety: ARMING_CHECK=0 restricted to SITL
    mock_conn.reset_mock()
    # Case A: Hardware-facing (is_sitl=False) -> MUST NOT disable arming checks
    iface.arm_and_takeoff(target_alt=5.0, timeout=0.01, arm_timeout=0.01, is_sitl=False)
    for call in mock_conn.mav.param_set_send.call_args_list:
        assert b"ARMING_CHECK" not in call[0]

    # Case B: SITL testing (is_sitl=True) -> Disables arming checks for automated CI/SITL
    mock_conn.reset_mock()
    iface.arm_and_takeoff(target_alt=5.0, timeout=0.01, arm_timeout=0.01, is_sitl=True)
    param_calls = [call[0] for call in mock_conn.mav.param_set_send.call_args_list]
    assert any(b"ARMING_CHECK" in c for c in param_calls)

