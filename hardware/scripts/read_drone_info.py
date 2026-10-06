import sys
import time
from pymavlink import mavutil

PORT = '/dev/ttyACM0'
BAUD = 115200

print(f"Connecting to flight controller on {PORT} at {BAUD} baud...")
try:
    master = mavutil.mavlink_connection(PORT, baud=BAUD)
except Exception as e:
    print(f"Error opening port {PORT}: {e}")
    sys.exit(1)

print("Waiting for MAVLink HEARTBEAT (up to 5 seconds)...")
msg = master.wait_heartbeat(timeout=5)
if not msg:
    print("No heartbeat received within 5 seconds. Checking connection...")
    sys.exit(1)

# Decode vehicle type and autopilot type
try:
    vtype = mavutil.mavlink.enums['MAV_TYPE'][msg.type].name
except KeyError:
    vtype = f"Type ID {msg.type}"

try:
    ap_type = mavutil.mavlink.enums['MAV_AUTOPILOT'][msg.autopilot].name
except KeyError:
    ap_type = f"Autopilot ID {msg.autopilot}"

try:
    armed = "ARMED" if master.motors_armed() else "DISARMED"
except Exception:
    armed = "UNKNOWN"

try:
    flight_mode = master.flight_mode()
except Exception:
    flight_mode = "UNKNOWN"

print("\n" + "="*50)
print("=== FLIGHT CONTROLLER IDENTIFICATION ===")
print("="*50)
print(f"Vehicle Type:     {vtype}")
print(f"Autopilot:        {ap_type}")
print(f"Current State:    {armed}")
print(f"Flight Mode:      {flight_mode}")
print(f"System ID:        {master.target_system}")
print(f"Component ID:     {master.target_component}")

# Request AUTOPILOT_VERSION
print("\nRequesting firmware version & hardware capabilities...")
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
    0,
    mavutil.mavlink.MAVLINK_MSG_ID_AUTOPILOT_VERSION,
    0, 0, 0, 0, 0, 0
)

# Request data streams
master.mav.request_data_stream_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_DATA_STREAM_ALL,
    4, # 4 Hz
    1  # start
)

# Collect messages for 3 seconds
start_time = time.time()
messages = {}
status_texts = []

while time.time() - start_time < 3.5:
    m = master.recv_msg()
    if m:
        m_type = m.get_type()
        if m_type == 'STATUSTEXT':
            txt = m.text.strip()
            if txt and txt not in status_texts:
                status_texts.append(txt)
        else:
            messages[m_type] = m

# Process Autopilot Version
if 'AUTOPILOT_VERSION' in messages:
    apv = messages['AUTOPILOT_VERSION']
    major = (apv.flight_sw_version >> 24) & 0xFF
    minor = (apv.flight_sw_version >> 16) & 0xFF
    patch = (apv.flight_sw_version >> 8) & 0xFF
    version_str = f"{major}.{minor}.{patch}"
    git_hash = bytes(apv.flight_custom_version).hex()[:8]
    print(f"Firmware Version: v{version_str} (Git hash: {git_hash})")
    print(f"Board Version:    {apv.board_version}")
    print(f"Vendor / Product: VID 0x{apv.vendor_id:04x} / PID 0x{apv.product_id:04x}")
else:
    print("Firmware Version: Not returned by AUTOPILOT_VERSION request (older MAVLink or busy)")

# Process System Status
if 'SYS_STATUS' in messages:
    sys_st = messages['SYS_STATUS']
    v_batt = sys_st.voltage_battery / 1000.0 if sys_st.voltage_battery != 65535 else 0.0
    c_batt = sys_st.current_battery / 100.0 if sys_st.current_battery != -1 else 0.0
    cpu_load = sys_st.load / 10.0
    print("\n" + "="*50)
    print("=== SYSTEM HEALTH & POWER ===")
    print("="*50)
    print(f"CPU Load:         {cpu_load:.1f}%")
    print(f"Battery Voltage:  {v_batt:.2f} V (USB power only if ~0V or ~5V)" if v_batt > 0 else "Battery Voltage:  No main battery connected (USB bus power)")
    if c_batt > 0:
        print(f"Battery Current:  {c_batt:.2f} A")
    print(f"Remaining Batt:   {sys_st.battery_remaining}%" if sys_st.battery_remaining != -1 else "Battery Level:    N/A")

    # Sensor status
    sensors = sys_st.onboard_control_sensors_enabled
    health = sys_st.onboard_control_sensors_health
    
    sensor_map = {
        mavutil.mavlink.MAV_SYS_STATUS_SENSOR_3D_GYRO: "3D Gyroscope",
        mavutil.mavlink.MAV_SYS_STATUS_SENSOR_3D_ACCEL: "3D Accelerometer",
        mavutil.mavlink.MAV_SYS_STATUS_SENSOR_3D_MAG: "3D Magnetometer / Compass",
        mavutil.mavlink.MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE: "Barometer (Pressure)",
        mavutil.mavlink.MAV_SYS_STATUS_SENSOR_GPS: "GPS Module",
        mavutil.mavlink.MAV_SYS_STATUS_SENSOR_RC_RECEIVER: "RC Receiver",
    }
    print("\nOnboard Sensors Status:")
    for flag, name in sensor_map.items():
        enabled = bool(sensors & flag)
        healthy = bool(health & flag)
        status_label = "HEALTHY" if (enabled and healthy) else ("UNHEALTHY/CALIBRATING" if enabled else "DISABLED / NOT DETECTED")
        print(f"  - {name:<26}: {status_label}")

# Process Attitude
if 'ATTITUDE' in messages:
    att = messages['ATTITUDE']
    import math
    roll_deg = math.degrees(att.roll)
    pitch_deg = math.degrees(att.pitch)
    yaw_deg = math.degrees(att.yaw)
    print("\n" + "="*50)
    print("=== LIVE ORIENTATION ===")
    print("="*50)
    print(f"Roll:  {roll_deg:+.1f}°")
    print(f"Pitch: {pitch_deg:+.1f}°")
    print(f"Yaw:   {yaw_deg:+.1f}°")

# Process GPS
if 'GPS_RAW_INT' in messages:
    gps = messages['GPS_RAW_INT']
    fix_types = {0: "No GPS", 1: "No Fix", 2: "2D Fix", 3: "3D Fix", 4: "DGPS/SBAS", 5: "RTK Float", 6: "RTK Fixed"}
    fix_desc = fix_types.get(gps.fix_type, f"Fix {gps.fix_type}")
    print("\n" + "="*50)
    print("=== GPS STATUS ===")
    print("="*50)
    print(f"GPS Fix:          {fix_desc}")
    print(f"Satellites in view: {gps.satellites_visible}")
    if gps.fix_type >= 2:
        print(f"Latitude:         {gps.lat / 1e7:.6f}°")
        print(f"Longitude:        {gps.lon / 1e7:.6f}°")
        print(f"Altitude:         {gps.alt / 1000.0:.1f} m")

# Print recent status messages / pre-arm warnings
if status_texts:
    print("\n" + "="*50)
    print("=== ARDUPILOT SYSTEM MESSAGES & PRE-ARM WARNINGS ===")
    print("="*50)
    for txt in status_texts[-10:]:
        print(f"  * {txt}")

print("\n" + "="*50)
print("Finished reading flight controller information successfully.")
print("="*50)
master.close()
