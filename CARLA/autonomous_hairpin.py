#!/usr/bin/env python

import carla
import time
import math
import csv
import random
import os
import weakref
import sys

# ---------------------------
# CONFIGURATION
# ---------------------------
SIMULATION_TIME_PER_HAIRPIN = 25
NUMBER_OF_BACKGROUND_VEHICLES = 30
TARGET_SPEED_KMH = 40.0
FOLLOW_DISTANCE = 5.0
CAMERA_SMOOTH_FACTOR = 0.1
HAIRPIN_MIN_YAW_DIFF = 85.0     # Lowered from 100 to find more turns
HAIRPIN_MAX_SEG_LENGTH = 60.0   # Max length of the road segment for the turn


# ================================================================
# SENSOR MANAGER
# ================================================================
class SensorManager(object):
    """
    Manages sensors for a given vehicle.
    Saves raw data to disk and holds other data for logging.
    """
    def __init__(self, parent_actor, output_prefix):
        self.parent = parent_actor
        self.world = parent_actor.get_world()
        self.bp_lib = self.world.get_blueprint_library()
        self.sensors = []
        self.output_prefix = output_prefix

        self.gps_data, self.imu_accel, self.imu_gyro = (0,0,0), (0,0,0), (0,0,0)
        self.radar_detections = 0
        self.cam_path, self.lidar_path = "", ""

        weak_self = weakref.ref(self)

        # --- RGB Camera ---
        cam_bp = self.bp_lib.find('sensor.camera.rgb')
        cam_bp.set_attribute('image_size_x', '800'); cam_bp.set_attribute('image_size_y', '600')
        camera = self.world.spawn_actor(cam_bp, carla.Transform(carla.Location(x=1.5, z=2.4)), attach_to=self.parent)
        camera.listen(lambda image: SensorManager._camera_callback(weak_self, image))
        self.sensors.append(camera)

        # --- GPS ---
        gps = self.world.spawn_actor(self.bp_lib.find('sensor.other.gnss'), carla.Transform(), attach_to=self.parent)
        gps.listen(lambda data: SensorManager._gps_callback(weak_self, data))
        self.sensors.append(gps)

        # --- IMU ---
        imu = self.world.spawn_actor(self.bp_lib.find('sensor.other.imu'), carla.Transform(), attach_to=self.parent)
        imu.listen(lambda data: SensorManager._imu_callback(weak_self, data))
        self.sensors.append(imu)

        # --- RADAR ---
        radar_bp = self.bp_lib.find('sensor.other.radar'); radar_bp.set_attribute('range', '50')
        radar = self.world.spawn_actor(radar_bp, carla.Transform(carla.Location(x=2.0, z=1.0)), attach_to=self.parent)
        radar.listen(lambda data: SensorManager._radar_callback(weak_self, data))
        self.sensors.append(radar)

        # --- LiDAR ---
        lidar_bp = self.bp_lib.find('sensor.lidar.ray_cast'); lidar_bp.set_attribute('range', '100')
        lidar = self.world.spawn_actor(lidar_bp, carla.Transform(carla.Location(x=0.0, z=2.4)), attach_to=self.parent)
        lidar.listen(lambda data: SensorManager._lidar_callback(weak_self, data))
        self.sensors.append(lidar)

    def destroy(self):
        """Stops and destroys all sensors managed by this instance."""
        for sensor in self.sensors:
            if sensor and sensor.is_alive:
                try: # Add try-except for robustness
                    sensor.stop()
                    sensor.destroy()
                except RuntimeError as e:
                    print(f"Warning: Error destroying sensor {sensor.id}: {e}")
        self.sensors.clear()

    # --- Static Callbacks ---
    @staticmethod
    def _camera_callback(weak_self, image):
        self = weak_self();
        if self: self.cam_path = f'{self.output_prefix}_cam/{image.frame}.png'; image.save_to_disk(self.cam_path)
    @staticmethod
    def _lidar_callback(weak_self, data):
        self = weak_self();
        if self: self.lidar_path = f'{self.output_prefix}_lidar/{data.frame}.ply'; data.save_to_disk(self.lidar_path)
    @staticmethod
    def _gps_callback(weak_self, data):
        self = weak_self();
        if self: self.gps_data = (data.latitude, data.longitude, data.altitude)
    @staticmethod
    def _imu_callback(weak_self, data):
        self = weak_self();
        if self: self.imu_accel = (data.accelerometer.x, data.accelerometer.y, data.accelerometer.z); self.imu_gyro = (data.gyroscope.x, data.gyroscope.y, data.gyroscope.z)
    @staticmethod
    def _radar_callback(weak_self, data):
        self = weak_self();
        if self: self.radar_detections = len(data)


# ================================================================
# MAIN SIMULATION
# ================================================================
def main():
    actor_list, sensor_managers = [], []
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)

    run_id = sys.argv[1] if len(sys.argv) > 1 else "run_0"
    output_dir = f"_v2v_multimodal_dataset_run_{run_id}"
    for i in [1, 2]:
        os.makedirs(f"{output_dir}/protagonist_{i}/cam", exist_ok=True)
        os.makedirs(f"{output_dir}/protagonist_{i}/lidar", exist_ok=True)
    csv_filename = f"{output_dir}/master_log_run_{run_id}.csv"

    is_left_hand_traffic = False # Default assumption

    try:
        # --- Apply left-hand traffic setting BEFORE loading the world ---
        current_world = client.get_world() # Get the current world (could be any map)
        settings = current_world.get_settings()
        if hasattr(settings, "left_handed_traffic"):
            settings.left_handed_traffic = True
            current_world.apply_settings(settings)
            print("🚦 Attempted to enable left-hand traffic mode.")
        else:
            print("⚠️ This CARLA version might predate the left_handed_traffic setting attribute.")


        # 1. SETUP: Load the desired mountain map
        world = client.load_world('Town04')

        # --- Verify the setting AFTER loading the target world ---
        current_settings = world.get_settings()
        if hasattr(current_settings, "left_handed_traffic") and current_settings.left_handed_traffic:
            print("🚗 Left-hand (Indian-style) traffic mode enabled and verified ✅")
            is_left_hand_traffic = True # Update flag
        else:
            print("⚠️ WARNING: Left-hand mode could not be verified! Check CARLA logs. Defaulting to right-hand traffic.")
            is_left_hand_traffic = False # Ensure flag is correct

        # --- Continue with the rest of the setup ---
        carla_map = world.get_map()
        blueprint_library = world.get_blueprint_library()
        spectator = world.get_spectator()

        traffic_manager = client.get_trafficmanager()
        traffic_manager.set_random_device_seed(int(run_id) if run_id.isdigit() else 42) # Seed TM differently each run
        traffic_manager.set_synchronous_mode(False) # Ensure TM is async if world is async
        traffic_manager.set_global_distance_to_leading_vehicle(FOLLOW_DISTANCE)
        # Apply potentially compatible TM settings
        try:
             traffic_manager.set_hybrid_physics_mode(True)
             traffic_manager.set_respawn_dormant_vehicles(True)
             traffic_manager.set_hybrid_physics_radius(70.0)
        except AttributeError:
             print("Note: Advanced TM settings (hybrid physics/respawn) might not be available in this CARLA version.")


        hairpins = find_hairpin_turns(carla_map, HAIRPIN_MIN_YAW_DIFF, HAIRPIN_MAX_SEG_LENGTH)
        print(f"✅ Found {len(hairpins)} potential hairpin locations.")
        if not hairpins:
            return

        fieldnames = ['timestamp', 'hairpin_id']
        for i in [1, 2]:
            fieldnames.extend([
                f'v{i}_x', f'v{i}_y', f'v{i}_z', f'v{i}_speed_kmh',
                f'v{i}_gps_lat', f'v{i}_gps_lon', f'v{i}_gps_alt',
                f'v{i}_imu_acc_x', f'v{i}_imu_acc_y', f'v{i}_imu_acc_z',
                f'v{i}_imu_gyro_x', f'v{i}_imu_gyro_y', f'v{i}_imu_gyro_z',
                f'v{i}_radar_detections', f'v{i}_cam_path', f'v{i}_lidar_path'
            ])

        with open(csv_filename, "w", newline="") as csvfile:
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            writer.writeheader()

            for idx, (wp_start, wp_end) in enumerate(hairpins):
                print(f"🔹 Simulating hairpin {idx + 1}/{len(hairpins)}")

                # Cleanup
                for manager in sensor_managers: manager.destroy()
                sensor_managers.clear()
                client.apply_batch([carla.command.DestroyActor(x) for x in actor_list])
                actor_list.clear()
                time.sleep(1)

                background_traffic = spawn_background_traffic(world, blueprint_library, traffic_manager, NUMBER_OF_BACKGROUND_VEHICLES)
                protagonist1 = spawn_protagonist(world, blueprint_library, wp_start)

                # --- Dynamic Opposing Lane Logic based on verified setting ---
                opposing_wp = None
                # In left-hand traffic, the opposing lane is usually physically to the right
                # In right-hand traffic, the opposing lane is usually physically to the left
                primary_candidate = wp_end.get_right_lane() if is_left_hand_traffic else wp_end.get_left_lane()
                fallback_candidate = wp_end.get_left_lane() if is_left_hand_traffic else wp_end.get_right_lane()

                if primary_candidate and primary_candidate.road_id == wp_end.road_id:
                     yaw_diff = abs(primary_candidate.transform.rotation.yaw - wp_end.transform.rotation.yaw)
                     if 150 <= yaw_diff % 360 <= 210: # Check if it's roughly opposite direction
                         opposing_wp = primary_candidate

                if opposing_wp is None and fallback_candidate and fallback_candidate.road_id == wp_end.road_id:
                     yaw_diff = abs(fallback_candidate.transform.rotation.yaw - wp_end.transform.rotation.yaw)
                     if 150 <= yaw_diff % 360 <= 210:
                         opposing_wp = fallback_candidate


                if opposing_wp is None:
                    print(f"❌ Could not find a valid opposing lane for hairpin {idx + 1} (Left-hand={is_left_hand_traffic}). Skipping.")
                    client.apply_batch([carla.command.DestroyActor(x) for x in background_traffic])
                    continue

                protagonist2 = spawn_protagonist(world, blueprint_library, opposing_wp)

                if not protagonist1 or not protagonist2:
                    print("❌ Failed to spawn one or both protagonist vehicles. Skipping.")
                    client.apply_batch([carla.command.DestroyActor(x) for x in background_traffic])
                    if protagonist1: protagonist1.destroy()
                    continue

                sensor_managers.append(SensorManager(protagonist1, f"{output_dir}/protagonist_1"))
                sensor_managers.append(SensorManager(protagonist2, f"{output_dir}/protagonist_2"))
                actor_list = [protagonist1, protagonist2] + background_traffic + sensor_managers[0].sensors + sensor_managers[1].sensors


                for v in [protagonist1, protagonist2]:
                    v.set_autopilot(True, traffic_manager.get_port())
                    traffic_manager.vehicle_percentage_speed_difference(v, 100 - (TARGET_SPEED_KMH / 50.0 * 100))
                    traffic_manager.auto_lane_change(v, False)

                start_time = time.time()
                previous_camera_location = None
                while time.time() - start_time < SIMULATION_TIME_PER_HAIRPIN:
                    # Pass the verified traffic mode to the camera function
                    previous_camera_location = set_spectator_transform(spectator, protagonist1, previous_camera_location, is_left_hand_traffic)

                    row = {'timestamp': time.time(), 'hairpin_id': idx}
                    protagonists = [protagonist1, protagonist2]
                    for i, (vehicle, manager) in enumerate(zip(protagonists, sensor_managers), 1):
                         if vehicle.is_alive:
                            loc, speed = get_vehicle_kinematics(vehicle)
                            row.update({
                                f'v{i}_x': loc.x, f'v{i}_y': loc.y, f'v{i}_z': loc.z, f'v{i}_speed_kmh': speed,
                                f'v{i}_gps_lat': manager.gps_data[0], f'v{i}_gps_lon': manager.gps_data[1], f'v{i}_gps_alt': manager.gps_data[2],
                                f'v{i}_imu_acc_x': manager.imu_accel[0], f'v{i}_imu_acc_y': manager.imu_accel[1], f'v{i}_imu_acc_z': manager.imu_accel[2],
                                f'v{i}_imu_gyro_x': manager.imu_gyro[0], f'v{i}_imu_gyro_y': manager.imu_gyro[1], f'v{i}_imu_gyro_z': manager.imu_gyro[2],
                                f'v{i}_radar_detections': manager.radar_detections,
                                f'v{i}_cam_path': manager.cam_path, f'v{i}_lidar_path': manager.lidar_path
                            })

                    writer.writerow(row)
                    time.sleep(0.1)

    except KeyboardInterrupt: print("\n⛔ Interrupted by user.")
    finally:
        print("\nCleaning up all spawned actors...")
        for manager in sensor_managers: manager.destroy()
        if 'client' in locals() and 'actor_list' in locals():
            destroyed_ids = set()
            batch = []
            for actor in actor_list:
                # Ensure actor is valid and not already added for destruction
                if actor and actor.is_alive and actor.id not in destroyed_ids:
                    # Check actor type before adding to batch (optional safety)
                    if isinstance(actor, carla.Actor):
                         batch.append(carla.command.DestroyActor(actor))
                         destroyed_ids.add(actor.id)
                    else:
                         print(f"Warning: actor_list contained non-actor object: {type(actor)}")
            if batch:
                try: client.apply_batch_sync(batch, True) # Use apply_batch_sync for better cleanup
                except RuntimeError as e: print(f"Error during final cleanup: {e}")

        # --- Attempt to reset traffic mode ---
        try:
            if 'client' in locals():
                 reset_world = client.get_world()
                 reset_settings = reset_world.get_settings()
                 if hasattr(reset_settings, "left_handed_traffic"):
                      reset_settings.left_handed_traffic = False # Attempt to set back to default
                      reset_world.apply_settings(reset_settings)
                      print("🚦 Attempted to reset traffic mode to right-hand.")
        except Exception as e:
            print(f"⚠️ Could not reset traffic settings: {e}")

        print(f"🎉 Simulation finished. Dataset saved to {csv_filename}")


# ================================================================
# HELPER FUNCTIONS (Keep these as before)
# ================================================================
def get_vehicle_kinematics(vehicle):
    loc = vehicle.get_location(); vel = vehicle.get_velocity()
    speed_kmh = 3.6 * math.sqrt(vel.x**2 + vel.y**2 + vel.z**2)
    return loc, speed_kmh

def find_hairpin_turns(carla_map, min_yaw_diff, max_seg_length):
    hairpins = []
    for seg_start, seg_end in carla_map.get_topology():
        dist = seg_start.transform.location.distance(seg_end.transform.location)
        if 0.1 < dist <= max_seg_length:
            yaw_diff = abs(seg_end.transform.rotation.yaw - seg_start.transform.rotation.yaw) % 360
            yaw_diff = min(yaw_diff, 360 - yaw_diff)
            if yaw_diff >= min_yaw_diff: hairpins.append((seg_start, seg_end))
    return hairpins

def spawn_background_traffic(world, bp_lib, tm, num_vehicles):
    spawn_points = world.get_map().get_spawn_points(); random.shuffle(spawn_points)
    traffic, car_bps = [], [bp for bp in bp_lib.filter('vehicle.*') if int(bp.get_attribute('number_of_wheels')) == 4]
    for _ in range(num_vehicles):
        if not spawn_points: break
        bp, spawn_point = random.choice(car_bps), spawn_points.pop()
        vehicle = world.try_spawn_actor(bp, spawn_point)
        if vehicle:
            vehicle.set_autopilot(True, tm.get_port()); tm.vehicle_percentage_speed_difference(vehicle, random.uniform(-20, 20))
            traffic.append(vehicle)
    return traffic

def spawn_protagonist(world, bp_lib, start_wp, look_behind=60.0):
    if start_wp is None: return None
    spawn_wp = start_wp
    for _ in range(int(look_behind)):
        try:
            prev_wps = spawn_wp.previous(1.0)
            if not prev_wps or prev_wps[0].is_junction: break
            spawn_wp = prev_wps[0]
        except RuntimeError: break
    spawn_transform = spawn_wp.transform; spawn_transform.location.z += 0.5
    return world.try_spawn_actor(bp_lib.find('vehicle.tesla.model3'), spawn_transform)

# --- Updated camera function with is_left_hand argument ---
def set_spectator_transform(spectator, vehicle, prev_loc, is_left_hand=False):
    """Makes the spectator camera smoothly follow a vehicle."""
    if not vehicle or not vehicle.is_alive: return prev_loc

    vehicle_transform = vehicle.get_transform()

    # Adjust Y offset based on traffic direction for better view over driver shoulder
    y_offset = -3.0 if is_left_hand else 3.0 # Shift right for LHT, left for RHT

    target_loc = vehicle_transform.location - 10 * vehicle_transform.get_forward_vector() + carla.Location(y=y_offset, z=4)

    if prev_loc is None:
        new_loc = target_loc
    else:
        new_loc = carla.Location(
            x=prev_loc.x + CAMERA_SMOOTH_FACTOR * (target_loc.x - prev_loc.x),
            y=prev_loc.y + CAMERA_SMOOTH_FACTOR * (target_loc.y - prev_loc.y),
            z=prev_loc.z + CAMERA_SMOOTH_FACTOR * (target_loc.z - prev_loc.z)
        )

    spectator.set_transform(carla.Transform(new_loc, carla.Rotation(pitch=-15, yaw=vehicle_transform.rotation.yaw)))
    return new_loc

# ================================================================
if __name__ == '__main__':
    main()