"""Retired 2026-11-04 with the legacy runtimes (user decision, item 5); moved unchanged from tests/test_config_and_safety.py. Not collected."""


def test_optional_pymavlink_behavior(monkeypatch):
    """
    Verify that:
    1. Core imports and SITL adapter classes function without pymavlink installed.
    2. SITL connection fails clearly and safely with ImportError when pymavlink is absent.
    """
    import sitl.mavlink_swarm_adapter as adapter_mod
    from sitl.common_frame import CommonCoordinateFrame

    # Simulate pymavlink absence by setting mavutil = None in adapter module
    monkeypatch.setattr(adapter_mod, "mavutil", None)

    frame = CommonCoordinateFrame()
    iface = adapter_mod.MAVLinkDroneInterface(
        sysid=1, port=14552, label="Drone 1", frame=frame
    )

    # Calling connect() must raise ImportError with clear user instructions
    with pytest.raises(ImportError, match="pymavlink is required to connect to Drone 1"):
        iface.connect()
