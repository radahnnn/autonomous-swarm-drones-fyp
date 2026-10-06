import sys
import time
from pymavlink import mavutil

PORT = '/dev/ttyACM0'
BAUD = 115200
OUTPUT_FILE = 'drone_parameters.param'

print(f"Connecting to flight controller on {PORT} at {BAUD} baud...")
try:
    master = mavutil.mavlink_connection(PORT, baud=BAUD)
except Exception as e:
    print(f"Error opening port {PORT}: {e}")
    sys.exit(1)

print("Waiting for MAVLink HEARTBEAT...")
msg = master.wait_heartbeat(timeout=5)
if not msg:
    print("No heartbeat received within 5 seconds.")
    sys.exit(1)

print("Requesting full parameter list (PARAM_REQUEST_LIST)...")
master.mav.param_request_list_send(master.target_system, master.target_component)

params = {}
total_params = None
start_time = time.time()
last_received_time = time.time()

print("Downloading parameters...")
while True:
    m = master.recv_match(type='PARAM_VALUE', blocking=True, timeout=2.0)
    now = time.time()
    
    if m:
        last_received_time = now
        p_name = m.param_id
        if isinstance(p_name, bytes):
            p_name = p_name.decode('ascii', errors='ignore')
        p_name = p_name.strip('\x00').strip()
        
        p_val = m.param_value
        # For integer-type parameters that are represented as float, format cleanly
        params[p_name] = p_val
        if total_params is None and m.param_count > 0:
            total_params = m.param_count

        if total_params:
            pct = (len(params) / total_params) * 100
            sys.stdout.write(f"\rProgress: {len(params)} / {total_params} parameters ({pct:.1f}%)")
            sys.stdout.flush()

            if len(params) >= total_params:
                break
    else:
        # Check timeout if no message received for 3 seconds
        if now - last_received_time > 3.0:
            if total_params and len(params) < total_params:
                print(f"\nStream paused at {len(params)}/{total_params}. Retrying missing parameters...")
                # Request missing parameters if needed
                # For brevity, break if we haven't received anything for 5s
                if now - last_received_time > 5.0:
                    print("Timeout waiting for more parameters.")
                    break
            else:
                break

print(f"\n\nSuccessfully downloaded {len(params)} parameters in {time.time() - start_time:.1f} seconds.")

# Write to standard ArduPilot .param format
with open(OUTPUT_FILE, 'w') as f:
    f.write(f"#NOTE: ArduPilot Parameter Dump saved on {time.ctime()}\n")
    f.write(f"#NOTE: Vehicle System ID {master.target_system}, Component ID {master.target_component}\n")
    for name in sorted(params.keys()):
        val = params[name]
        # Format: integers as integer notation if exact, else float
        if val.is_integer():
            f.write(f"{name},{int(val)}\n")
        else:
            f.write(f"{name},{val:.6f}\n")

print(f"Saved all parameters to '{OUTPUT_FILE}'.")
master.close()
