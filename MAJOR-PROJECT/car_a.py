"""
CAR A - V2V SCRIPT (Socket Server)
===================================
Runs V2V socket server, broadcasts position, sends warnings to Car B.
"""

import time
import math
import msvcrt
from v2v_metrics import MetricsCollector
from v2v_gui import V2VDashboard
import socket
import json
from beamngpy import BeamNGpy, Vehicle

# ==========================================
# CONFIG
# ==========================================
V2V_PORT = 5555
V2V_HOST = '127.0.0.1'
BNG_HOME = r"X:\Steam\steamapps\common\BeamNG.drive"

metrics = MetricsCollector(car_id="CAR_A", csv_filename="results/v2v_results.csv")

WARN_DISTANCE   = 30.0
SLOW_DISTANCE   = 20.0
STOP_DISTANCE   = 10.0

# ==========================================
# PHASE 1: CONNECT TO BEAMNG
# ==========================================
print("[-] Connecting to BeamNG.drive...")
bng = BeamNGpy('localhost', 64256, home=BNG_HOME, binary='BeamNG.drive.exe')

try:
    # Try attaching to a running instance first
    bng.open(launch=False)
    print("[+] Attached to existing BeamNG instance!")
except Exception:
    print("[-] No running instance found. Launching new BeamNG.drive...")
    bng.open(launch=True)
    input("\nPress ENTER when vehicles are spawned > ")

# ==========================================
# PHASE 2: IDENTIFY CAR A
# ==========================================
print("[-] Scanning vehicles...")
current_vehicles = bng.vehicles.get_current()
vehicle_list = list(current_vehicles.values())
car_a = None

print(f"\nVehicles found ({len(vehicle_list)}):")
for i, veh in enumerate(vehicle_list, 1):
    veh.connect(bng)
    try:
        veh.set_color((1, 0, 1, 1))  # Flash PINK
    except:
        pass
    print(f"  [{i}] {veh.vid}  (now PINK in-game)")
    time.sleep(0.5)

pick = input("Select CAR A (number) > ").strip()
try:
    car_a = vehicle_list[int(pick) - 1]
    car_a.set_color((0, 0, 1, 1))  # Blue
    car_a.queue_lua_command("controller.setShiftMode('arcade')")
    print(f"  -> CAR A = {car_a.vid} (Blue, Arcade)")
except (ValueError, IndexError):
    print("[!] Invalid selection!")
    bng.close()
    exit()

# ==========================================
# PHASE 3: START SOCKET SERVER
# ==========================================
print(f"[-] Starting V2V server on {V2V_HOST}:{V2V_PORT}...")
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
server_socket.bind((V2V_HOST, V2V_PORT))
server_socket.listen(1)

print("[*] Waiting for Car B to connect...")
conn, addr = server_socket.accept()
print(f"[+] Car B connected from {addr}")

car_b_vid = None
try:
    conn.settimeout(5.0)
    data = conn.recv(1024).decode()
    msg = json.loads(data)
    if msg.get('type') == 'HELLO':
        car_b_vid = msg.get('my_vid')
        print(f"[+] Car B vehicle ID: {car_b_vid}")
except:
    print("[!] Warning: Couldn't get Car B's vehicle ID")

# ==========================================
# PHASE 4: MAIN TEST LOOP
# ==========================================
run_count = 1

speed_input = input("\nBase speed for both cars (km/h) [default=45]: ").strip()
BASE_SPEED_KMH = float(speed_input) if speed_input else 45.0
BASE_SPEED_MPS = BASE_SPEED_KMH / 3.6
print(f"[+] Speed: {BASE_SPEED_KMH:.0f} km/h ({BASE_SPEED_MPS:.1f} m/s)")

# Launch GUI
gui = V2VDashboard("Unified V2V Dashboard")
gui.log(f"Base speed: {BASE_SPEED_KMH:.0f} km/h", "info")
gui.log(f"Car B connected: {car_b_vid}", "success")

# Initialize early status
gui.set_status('A', "IDLE")
gui.set_status('B', "IDLE")

while True:
    print(f"\n[Run #{run_count}] Press ENTER in this terminal to start > ", end='', flush=True)
    while True:
        gui.tick()
        if msvcrt.kbhit():
            key = msvcrt.getch()
            if key in (b'\r', b'\n'):
                print()  # Move to next line in terminal
                break
        time.sleep(0.01)

    metrics.start_run(run_count)
    gui.set_status('A', "DRIVING")
    gui.set_status('B', "DRIVING")
    gui.log(f"─── Run #{run_count} started ───", "info")

    # Enable AI for Car A — start with 'chase' to drive toward Car B
    # Once warning triggers, yield logic switches modes as needed.
    car_a.queue_lua_command("controller.setShiftMode('arcade')")
    car_a.control(throttle=0, brake=0, parkingbrake=0)
    time.sleep(0.2)
    car_a.ai_set_mode('chase')
    car_a.ai_drive_in_lane(True)
    car_a.queue_lua_command('ai.driveInLane("on")')
    car_a.ai_set_speed(BASE_SPEED_MPS, mode='limit')
    car_a.ai_set_aggression(0.3)

    # Send START to Car B
    start_msg = {"type": "START_RUN", "run": run_count, "target_vid": car_a.vid, "base_speed": BASE_SPEED_MPS}
    conn.send(json.dumps(start_msg).encode())
    gui.log(f"TX → START_RUN (target: {car_a.vid})", "info")

    warning_sent = False
    decision_made = False
    i_am_yielding = False
    resumed = False
    min_dist_recorded = float('inf')
    emergency_stop = False
    car_b_pos = None
    pre_warn_speed_a = None
    pre_warn_speed_b = None

    try:
        while True:
            car_a.sensors.poll()
            car_a_pos = car_a.state['pos']
            vel_a = car_a.state.get('vel', [0, 0, 0])
            speed_a = math.sqrt(vel_a[0]**2 + vel_a[1]**2 + vel_a[2]**2)
            gui.set_speed('A', speed_a)

            # Send position to Car B
            pos_msg = {
                "type": "POSITION",
                "pos": car_a_pos,
                "speed": vel_a,
                "send_ts": time.time()
            }
            try:
                conn.send(json.dumps(pos_msg).encode())
                metrics.log_message_sent()
                metrics.log_position_update()
            except (BrokenPipeError, ConnectionResetError, OSError):
                gui.log("Connection to Car B lost!", "stop")
                break

            # Receive Car B's position
            try:
                conn.settimeout(0.05)
                data = conn.recv(1024).decode()
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

                            if msg['type'] == 'POSITION':
                                car_b_pos = msg.get('pos')
                                metrics.log_message_received()

                                if not car_b_pos:
                                    continue
                                
                                # --- Update Car B GUI telemetry ---
                                car_b_vel = msg.get('speed', [0,0,0]) or [0,0,0]
                                speed_b_current = math.sqrt(car_b_vel[0]**2 + car_b_vel[1]**2 + car_b_vel[2]**2)
                                gui.set_speed('B', speed_b_current)
                                b_status = msg.get('status')
                                if b_status:
                                    gui.set_status('B', b_status)

                                dist = math.sqrt(
                                    (car_a_pos[0] - car_b_pos[0])**2 +
                                    (car_a_pos[1] - car_b_pos[1])**2
                                )
                                metrics.log_distance(dist)
                                gui.set_distance(dist)

                                # --- GRADUAL SLOWDOWN & NEGOTIATION ---
                                if dist < WARN_DISTANCE and not warning_sent:
                                    car_b_vel = msg.get('speed', [0,0,0]) or [0,0,0]
                                    speed_b = math.sqrt(car_b_vel[0]**2 + car_b_vel[1]**2 + car_b_vel[2]**2)
                                    pre_warn_speed_a = speed_a
                                    pre_warn_speed_b = speed_b
                                    metrics.log_warning_sent(dist, speed_a, other_speed=speed_b)

                                    warning_msg = {
                                        "type": "WARNING",
                                        "distance": dist,
                                        "message": "Oncoming vehicle ahead!",
                                        "send_ts": time.time()
                                    }
                                    try:
                                        conn.send(json.dumps(warning_msg).encode())
                                        metrics.log_message_sent()
                                    except:
                                        pass

                                    car_a.ai_set_speed(5.6, mode='limit')
                                    metrics.log_brake_applied(car_a_pos, speed_a)
                                    warning_sent = True

                                    gui.set_status('A', "WARNING")
                                    gui.log(f"⚠ WARNING sent — {dist:.1f}m", "warn", car='A')

                                # Tie-breaker negotiation at 35m (between WARN and SLOW zones)
                                if warning_sent and not decision_made and dist < 35.0:
                                    decision_made = True
                                    gui.log("Negotiating Right-of-Way...", "info")
                                    
                                    # Tie-breaker: Kinematic Energy (Speed) comparison
                                    # The slower vehicle yields (requires less braking distance/energy).
                                    # If exactly equal, fallback to ID tie-breaker.
                                    if pre_warn_speed_a < pre_warn_speed_b or (pre_warn_speed_a == pre_warn_speed_b and str(car_a.vid) < str(car_b_vid)):
                                        i_am_yielding = True
                                        gui.log(f"Spd: {pre_warn_speed_a*3.6:.1f} < {pre_warn_speed_b*3.6:.1f} km/h -> I WILL YIELD", "warn")
                                        
                                        yield_msg = {"type": "YIELDING", "distance": dist, "send_ts": time.time()}
                                        try:
                                            conn.send(json.dumps(yield_msg).encode())
                                            metrics.log_message_sent()
                                        except:
                                            pass
                                            
                                        # BeamNG treats speed limit '0' as "no limit", causing it to accelerate instead!
                                        # To gracefully halt in its lane, we must use the native 'stop' AI mode.
                                        car_a.ai_set_mode('stop')
                                        car_a.ai_drive_in_lane(True)
                                        car_a.queue_lua_command('ai.driveInLane("on")')
                                        gui.set_status('A', "STOPPED")
                                        gui.log("Yielding in lane.", "stop", car='A')
                                    
                                    else:
                                        i_am_yielding = False
                                        gui.log(f"Spd: {pre_warn_speed_a*3.6:.1f} > {pre_warn_speed_b*3.6:.1f} km/h -> I PROCEED", "success", car='A')

                                        # Tell Car B to yield and stop
                                        proceed_msg = {"type": "PROCEED", "distance": dist, "send_ts": time.time()}
                                        try:
                                            conn.send(json.dumps(proceed_msg).encode())
                                            metrics.log_message_sent()
                                        except:
                                            pass

                                        # Switch to 'random' to keep driving forward
                                        car_a.ai_set_mode('random')
                                        car_a.ai_drive_in_lane(True)
                                        car_a.queue_lua_command('ai.driveInLane("on")')
                                        car_a.ai_set_speed(5.6, mode='limit')  # Maintain 20 km/h safe passing speed
                                        gui.set_status('A', "DRIVING")

                                if decision_made:
                                    if dist < min_dist_recorded:
                                        min_dist_recorded = dist
                                    
                                    # Detect when the pass is complete (distance started increasing and is > 10m)
                                    if dist > min_dist_recorded + 1.0 and dist > 10.0 and not resumed:
                                        resumed = True
                                        gui.log("Pass complete. Resuming.", "success", car='A')
                                        gui.set_status('A', "SUCCESS")
                                        
                                        if i_am_yielding:
                                            # Release brakes and keep driving straight along the lane
                                            car_a.control(throttle=0, brake=0, parkingbrake=0)
                                            car_a.ai_set_mode('random')
                                            car_a.ai_drive_in_lane(True)
                                            car_a.queue_lua_command('ai.driveInLane("on")')
                                            car_a.ai_set_speed(BASE_SPEED_MPS, mode='limit')
                                            car_a.ai_set_aggression(0.5)
                                        
                                        # Finish run successfully
                                        metrics.log_full_stop(car_a_pos, speed_a)
                                        metrics.end_run()
                                        gui.log(f"─── Run #{run_count} complete ───", "info")
                                        time.sleep(2)
                                        break

                        except json.JSONDecodeError:
                            pass
            except socket.timeout:
                pass

            # --- Emergency Stop Fallback ---
            # (In case calculation fails and they actually hit each other)
            if dist < 3.0 and not emergency_stop:
                car_a.control(throttle=0, brake=1.0, parkingbrake=1.0)
                emergency_stop = True
                gui.log("EMERGENCY ABORT - TOO CLOSE!", "stop")

            gui.tick()
            time.sleep(0.05)

    except KeyboardInterrupt:
        gui.close()
        conn.close()
        server_socket.close()
        bng.close()
        exit()

    run_count += 1
