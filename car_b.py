"""
CAR B - V2V SCRIPT (Socket Client)
===================================
Connects to Car A's socket server, exchanges position data, receives warnings.
"""

import time
import math
from v2v_metrics import MetricsCollector
import socket
import json
import threading
from beamngpy import BeamNGpy, Vehicle

# ==========================================
# CONFIG
# ==========================================
V2V_PORT = 5555
V2V_HOST = '127.0.0.1'
BNG_HOME = r"X:\Steam\steamapps\common\BeamNG.drive"

# Global state
car_b = None
warning_received = False
emergency_stop = False
car_a_pos = None
run_id = 1
car_b_status = "IDLE"

pass_detection_active = False
min_dist_recorded = float('inf')
resumed = False

metrics = MetricsCollector(car_id="CAR_B", csv_filename="results/v2v_results_car_b.csv")

# ==========================================
# V2V RECEIVER THREAD
# ==========================================
def v2v_listener(sock):
    global warning_received, emergency_stop, car_a_pos, car_b, run_id, car_b_status
    global pass_detection_active, min_dist_recorded, resumed

    while True:
        try:
            data = sock.recv(2048).decode()
            if data:
                for msg_str in data.split('}{'):
                    try:
                        if not msg_str.startswith('{'):
                            msg_str = '{' + msg_str
                        if not msg_str.endswith('}'):
                            msg_str = msg_str + '}'
                        msg = json.loads(msg_str)

                        if 'type' not in msg:
                            continue

                        if msg['type'] == 'START_RUN':
                            run_id = msg.get('run', 1)
                            target_vid = msg.get('target_vid', None)
                            warning_received = False
                            emergency_stop = False
                            metrics.start_run(run_id)

                            base_speed = msg.get('base_speed', 12.5)
                            car_b.queue_lua_command("controller.setShiftMode('arcade')")
                            car_b.control(throttle=0, brake=0, parkingbrake=0)
                            time.sleep(0.2)
                            car_b.ai_set_mode('chase')
                            car_b.ai_drive_in_lane(True)
                            car_b.queue_lua_command('ai.driveInLane("on")')
                            car_b.ai_set_speed(base_speed, mode='limit')
                            car_b.ai_set_aggression(0.3)

                            car_b_status = "DRIVING"
                            print(f"[+] Run #{run_id} started. Chasing {target_vid} at {base_speed*3.6:.0f} km/h")

                        elif msg['type'] == 'POSITION':
                            car_a_pos = msg['pos']
                            metrics.log_message_received()
                            metrics.log_position_update()
                            if 'send_ts' in msg:
                                latency_ms = (time.time() - msg['send_ts']) * 1000
                                if latency_ms >= 0:
                                    metrics._latency_samples.append(latency_ms)

                        elif msg['type'] == 'WARNING':
                            try:
                                car_b.sensors.poll()
                            except:
                                pass
                            vel_b = car_b.state.get('vel', [0,0,0])
                            speed_b = math.sqrt(vel_b[0]**2 + vel_b[1]**2 + vel_b[2]**2)
                            warn_dist = msg.get('distance', None)

                            send_ts = msg.get('send_ts', None)
                            if send_ts is not None:
                                latency_ms = (time.time() - send_ts) * 1000
                                if latency_ms >= 0:
                                    metrics._latency_samples.append(latency_ms)
                            metrics.log_warning_received(
                                distance=warn_dist,
                                my_speed=speed_b,
                                send_timestamp=None
                            )

                            car_b.ai_set_speed(5.6, mode='limit')
                            metrics.log_brake_applied(car_b.state['pos'], speed_b)
                            warning_received = True

                            car_b_status = "WARNING"
                            print(f"[!] WARNING from Car A — {warn_dist:.1f}m — slowing to 20 km/h")

                        elif msg['type'] == 'SLOW':
                            target_speed = msg.get('target_speed', 5.6)
                            dist = msg.get('distance', 0)
                            car_b.ai_set_speed(target_speed, mode='limit')
                            car_b_status = "SLOWING"

                        elif msg['type'] == 'YIELDING':
                            # Car A is yielding to us! We maintain safe passing speed.
                            dist = msg.get('distance', 0)
                            car_b_status = "DRIVING"
                            print("    -> Car A is yielding! We have right-of-way.")

                            # Switch from 'chase' to 'random' to keep driving past Car A
                            car_b.ai_set_mode('random')
                            car_b.ai_drive_in_lane(True)
                            car_b.queue_lua_command('ai.driveInLane("on")')
                            car_b.ai_set_speed(5.6, mode='limit')  # Maintain 20 km/h passing speed

                            # Activate pass detection in the main loop instead of a background thread
                            # (Tkinter is not thread-safe and will crash if updated from here)
                            pass_detection_active = True
                            min_dist_recorded = float('inf')
                            resumed = False

                        elif msg['type'] == 'PROCEED':
                            # Car A is proceeding -- WE must yield and stop!
                            car_b_status = "STOPPED"
                            print("    -> Car A is proceeding. We YIELD and stop.")

                            # Stop in our lane
                            car_b.ai_set_mode('stop')
                            car_b.ai_drive_in_lane(True)
                            car_b.queue_lua_command('ai.driveInLane("on")')

                            # Activate pass detection so we resume after Car A passes
                            pass_detection_active = True
                            min_dist_recorded = float('inf')
                            resumed = False

                    except json.JSONDecodeError:
                        pass

        except Exception as e:
            print(f"[!] Connection error: {e}")
            break

# ==========================================
# PHASE 1: CONNECT TO BEAMNG
# ==========================================
print("[-] Connecting to BeamNG...")
bng = BeamNGpy('localhost', 64256, home=BNG_HOME, binary='BeamNG.drive.exe')
import beamngpy.logging as bng_logging

try:
    bng.open(launch=False)
    print("[+] Connected to BeamNG!")
except Exception as e:
    print("[!] ERROR: Could not connect to BeamNG.")
    print("    Make sure you have run car_a.py first to launch the simulator.")
    exit()

# ==========================================
# PHASE 2: IDENTIFY CAR B
# ==========================================
print("[-] Scanning vehicles...")
current_vehicles = bng.vehicles.get_current()
vehicle_list = list(current_vehicles.values())

print(f"\nVehicles found ({len(vehicle_list)}):")
print("  (Car A should already be Blue — skip that one)")
for i, veh in enumerate(vehicle_list, 1):
    veh.connect(bng)
    try:
        veh.set_color((1, 0, 1, 1))  # Flash PINK
        time.sleep(0.3)
    except:
        pass
    print(f"  [{i}] {veh.vid}  (now PINK in-game)")

pick = input("Select CAR B (number) > ").strip()
try:
    car_b = vehicle_list[int(pick) - 1]
    car_b.set_color((1, 0, 0, 1))  # Red
    car_b.queue_lua_command("controller.setShiftMode('arcade')")
    print(f"  -> CAR B = {car_b.vid} (Red, Arcade)")
except (ValueError, IndexError):
    print("[!] Invalid selection!")
    exit()

# ==========================================
# PHASE 3: CONNECT TO CAR A
# ==========================================
print(f"[-] Connecting to Car A at {V2V_HOST}:{V2V_PORT}...")
try:
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client_socket.connect((V2V_HOST, V2V_PORT))
    hello_msg = {"type": "HELLO", "my_vid": car_b.vid}
    client_socket.send(json.dumps(hello_msg).encode())
    print(f"[+] Connected to Car A! Sent ID: {car_b.vid}")

    listener = threading.Thread(target=v2v_listener, args=(client_socket,), daemon=True)
    listener.start()
except Exception as e:
    print(f"[!] ERROR: {e}")
    exit()

# ==========================================
# PHASE 4: MAIN LOOP
# ==========================================
print("\n[+] Waiting for Car A to start test...")
print("    (Running headless. Data reporting to Car A's Unified Dashboard)")
print("    Press Ctrl+C to exit.\n")

try:
    while True:
        car_b.sensors.poll()
        car_b_pos = car_b.state['pos']
        vel_b = car_b.state.get('vel', [0,0,0])
        speed_b = math.sqrt(vel_b[0]**2 + vel_b[1]**2 + vel_b[2]**2)

        pos_msg = {
            "type": "POSITION",
            "pos": car_b_pos,
            "speed": vel_b,
            "status": car_b_status,
            "send_ts": time.time()
        }
        try:
            client_socket.send(json.dumps(pos_msg).encode())
            metrics.log_message_sent()
        except:
            pass

        if car_a_pos:
            dist = math.sqrt(
                (car_b_pos[0] - car_a_pos[0])**2 +
                (car_b_pos[1] - car_a_pos[1])**2
            )

            if pass_detection_active and not resumed:
                if dist < min_dist_recorded:
                    min_dist_recorded = dist
                
                # If distance starts increasing and is > 10m, pass is complete!
                if dist > min_dist_recorded + 1.0 and dist > 10.0:
                    resumed = True
                    car_b_status = "SUCCESS"
                    print(f"[+] ✅ Pass complete — Min clearance: {min_dist_recorded:.1f}m")
                    print(f"─── Run complete ───")
                    
                    # Accelerate back to normal speed and keep driving
                    car_b.ai_set_mode('random')
                    car_b.ai_drive_in_lane(True)
                    car_b.queue_lua_command('ai.driveInLane("on")')
                    car_b.ai_set_speed(12.5, mode='limit')
                    
                    # Finish run metrics
                    metrics.log_distance(min_dist_recorded)
                    metrics.log_full_stop(car_b_pos, speed_b)
                    metrics.end_run()

        time.sleep(0.05)

except KeyboardInterrupt:
    client_socket.close()
    exit()
