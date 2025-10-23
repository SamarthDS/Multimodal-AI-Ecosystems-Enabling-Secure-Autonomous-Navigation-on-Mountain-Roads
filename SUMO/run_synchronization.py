#!/usr/bin/env python

# Copyright (c) 2020 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
"""
Script to integrate CARLA and SUMO simulations
"""

# ==================================================================================================
# -- imports ---------------------------------------------------------------------------------------
# ==================================================================================================

import argparse
import logging
import os
import sys
import time
import carla # Make sure carla is imported

# ==================================================================================================
# -- find traci module -----------------------------------------------------------------------------
# ==================================================================================================

if 'SUMO_HOME' in os.environ:
    # Append tools directory from SUMO_HOME to path
    sys.path.append(os.path.join(os.environ['SUMO_HOME'], 'tools'))
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

# ==================================================================================================
# -- sumo integration imports ----------------------------------------------------------------------
# ==================================================================================================

from sumo_integration.bridge_helper import BridgeHelper  # pylint: disable=wrong-import-position
from sumo_integration.carla_simulation import CarlaSimulation  # pylint: disable=wrong-import-position
from sumo_integration.constants import INVALID_ACTOR_ID  # pylint: disable=wrong-import-position
from sumo_integration.sumo_simulation import SumoSimulation  # pylint: disable=wrong-import-position

# ==================================================================================================
# -- SimulationSynchronization Class ---------------------------------------------------------------
# ==================================================================================================


class SimulationSynchronization(object):
    """
    SimulationSynchronization class is responsible for the synchronization of sumo and carla
    simulations.
    """
    def __init__(self,
                 sumo_simulation,
                 carla_simulation,
                 tls_manager='none',
                 sync_vehicle_color=False,
                 sync_vehicle_lights=False):

        self.sumo = sumo_simulation
        self.carla = carla_simulation # CarlaSimulation object

        # --- NOTE: Map loading is now handled in synchronization_loop ---

        self.tls_manager = tls_manager
        self.sync_vehicle_color = sync_vehicle_color
        self.sync_vehicle_lights = sync_vehicle_lights

        if tls_manager == 'carla':
            self.sumo.switch_off_traffic_lights()
        elif tls_manager == 'sumo':
            # Check if world exists before trying to access it
            if self.carla.world:
                 self.carla.switch_off_traffic_lights()
            else:
                 logging.warning("CARLA world not available to switch off traffic lights.")


        # Mapped actor ids.
        self.sumo2carla_ids = {}  # Contains only actors controlled by sumo.
        self.carla2sumo_ids = {}  # Contains only actors controlled by carla.

        # Ensure world object exists before accessing attributes
        if self.carla.world:
             BridgeHelper.blueprint_library = self.carla.world.get_blueprint_library()
        else:
             logging.error("CARLA world object not initialized correctly. Cannot get blueprint library.")
             # Handle error appropriately, maybe exit or raise exception
             # sys.exit("Exiting due to CARLA world initialization failure.")

        BridgeHelper.offset = self.sumo.get_net_offset()

        # --- IMPORTANT: Keep synchronous mode settings commented out ---
        # --- This prevents the 'peer shutdown' error ---
        # if self.carla.world:
        #      settings = self.carla.world.get_settings()
        #      settings.synchronous_mode = True
        #      settings.fixed_delta_seconds = self.carla.step_length
        #      self.carla.world.apply_settings(settings)

        #      traffic_manager = self.carla.client.get_trafficmanager()
        #      traffic_manager.set_synchronous_mode(True)
        # else:
        #      logging.warning("CARLA world not available to apply synchronous settings.")
        # --- END OF COMMENTED OUT SYNC BLOCK ---

    def tick(self):
        """
        Tick to simulation synchronization
        """
        # Ensure world object is valid before ticking
        if not self.carla.world:
             logging.error("CARLA world is not valid during tick.")
             return # Skip tick if world is invalid

        # -----------------
        # sumo-->carla sync
        # -----------------
        self.sumo.tick()

        # Spawning new sumo actors in carla (i.e, not controlled by carla).
        sumo_spawned_actors = self.sumo.spawned_actors - set(self.carla2sumo_ids.values())
        for sumo_actor_id in sumo_spawned_actors:
            self.sumo.subscribe(sumo_actor_id)
            sumo_actor = self.sumo.get_actor(sumo_actor_id)

            carla_blueprint = BridgeHelper.get_carla_blueprint(sumo_actor, self.sync_vehicle_color)
            if carla_blueprint is not None:
                carla_transform = BridgeHelper.get_carla_transform(sumo_actor.transform,
                                                                   sumo_actor.extent)

                carla_actor_id = self.carla.spawn_actor(carla_blueprint, carla_transform)
                if carla_actor_id != INVALID_ACTOR_ID:
                    self.sumo2carla_ids[sumo_actor_id] = carla_actor_id
            else:
                self.sumo.unsubscribe(sumo_actor_id)

        # Destroying sumo arrived actors in carla.
        for sumo_actor_id in self.sumo.destroyed_actors:
            if sumo_actor_id in self.sumo2carla_ids:
                self.carla.destroy_actor(self.sumo2carla_ids.pop(sumo_actor_id))

        # Updating sumo actors in carla.
        for sumo_actor_id in self.sumo2carla_ids:
            carla_actor_id = self.sumo2carla_ids[sumo_actor_id]

            sumo_actor = self.sumo.get_actor(sumo_actor_id)
            carla_actor = self.carla.get_actor(carla_actor_id)

            # Ensure carla_actor is valid before proceeding
            if carla_actor is None:
                 logging.warning(f"Could not find CARLA actor {carla_actor_id} for SUMO actor {sumo_actor_id}. Skipping update.")
                 # Optionally remove from mapping if actor is unexpectedly gone
                 # self.sumo2carla_ids.pop(sumo_actor_id) 
                 continue


            carla_transform = BridgeHelper.get_carla_transform(sumo_actor.transform,
                                                               sumo_actor.extent)
            if self.sync_vehicle_lights:
                # Need to handle potential None carla_actor
                current_light_state = carla_actor.get_light_state() if carla_actor else carla.VehicleLightState.NONE
                carla_lights = BridgeHelper.get_carla_lights_state(current_light_state,
                                                                   sumo_actor.signals)
            else:
                carla_lights = None

            self.carla.synchronize_vehicle(carla_actor_id, carla_transform, carla_lights)

        # Updates traffic lights in carla based on sumo information.
        if self.tls_manager == 'sumo':
            common_landmarks = self.sumo.traffic_light_ids & self.carla.traffic_light_ids
            for landmark_id in common_landmarks:
                sumo_tl_state = self.sumo.get_traffic_light_state(landmark_id)
                carla_tl_state = BridgeHelper.get_carla_traffic_light_state(sumo_tl_state)

                self.carla.synchronize_traffic_light(landmark_id, carla_tl_state)

        # -----------------
        # carla-->sumo sync
        # -----------------
        self.carla.tick()

        # Spawning new carla actors (not controlled by sumo)
        carla_spawned_actors = self.carla.spawned_actors - set(self.sumo2carla_ids.values())
        for carla_actor_id in carla_spawned_actors:
            carla_actor = self.carla.get_actor(carla_actor_id)
            if carla_actor is None: continue # Skip if actor doesn't exist

            type_id = BridgeHelper.get_sumo_vtype(carla_actor)
            color = carla_actor.attributes.get('color', None) if self.sync_vehicle_color else None
            if type_id is not None:
                sumo_actor_id = self.sumo.spawn_actor(type_id, color)
                if sumo_actor_id != INVALID_ACTOR_ID:
                    self.carla2sumo_ids[carla_actor_id] = sumo_actor_id
                    self.sumo.subscribe(sumo_actor_id)

        # Destroying required carla actors in sumo.
        for carla_actor_id in self.carla.destroyed_actors:
            if carla_actor_id in self.carla2sumo_ids:
                self.sumo.destroy_actor(self.carla2sumo_ids.pop(carla_actor_id))

        # Updating carla actors in sumo.
        for carla_actor_id in self.carla2sumo_ids:
            sumo_actor_id = self.carla2sumo_ids[carla_actor_id]

            carla_actor = self.carla.get_actor(carla_actor_id)
            sumo_actor = self.sumo.get_actor(sumo_actor_id)
            
            # Ensure both actors exist before syncing
            if carla_actor is None or sumo_actor is None:
                 logging.warning(f"Could not find CARLA actor {carla_actor_id} or SUMO actor {sumo_actor_id}. Skipping update.")
                 continue


            sumo_transform = BridgeHelper.get_sumo_transform(carla_actor.get_transform(),
                                                             carla_actor.bounding_box.extent)
            if self.sync_vehicle_lights:
                carla_lights = self.carla.get_actor_light_state(carla_actor_id)
                if carla_lights is not None:
                    sumo_lights = BridgeHelper.get_sumo_lights_state(sumo_actor.signals,
                                                                     carla_lights)
                else:
                    sumo_lights = None
            else:
                sumo_lights = None

            self.sumo.synchronize_vehicle(sumo_actor_id, sumo_transform, sumo_lights)

        # Updates traffic lights in sumo based on carla information.
        if self.tls_manager == 'carla':
            common_landmarks = self.sumo.traffic_light_ids & self.carla.traffic_light_ids
            for landmark_id in common_landmarks:
                carla_tl_state = self.carla.get_traffic_light_state(landmark_id)
                sumo_tl_state = BridgeHelper.get_sumo_traffic_light_state(carla_tl_state)

                # Updates all the sumo links related to this landmark.
                self.sumo.synchronize_traffic_light(landmark_id, sumo_tl_state)

    def close(self):
        """
        Cleans synchronization.
        """
        # --- Ensure disabling sync mode only happens if world exists ---
        if self.carla.world:
             try:
                 settings = self.carla.world.get_settings()
                 settings.synchronous_mode = False
                 settings.fixed_delta_seconds = None
                 self.carla.world.apply_settings(settings)
             except RuntimeError as e:
                 logging.warning(f"Could not reset CARLA to async mode during close: {e}")
        else:
             logging.warning("CARLA world not available during close operation.")


        # Destroying synchronized actors.
        # Add checks to ensure actors exist before destroying
        carla_batch = []
        for carla_actor_id in self.sumo2carla_ids.values():
             actor = self.carla.get_actor(carla_actor_id)
             if actor and actor.is_alive:
                 carla_batch.append(carla.command.DestroyActor(actor))
        if carla_batch:
             try: self.carla.client.apply_batch_sync(carla_batch, True)
             except RuntimeError as e: logging.warning(f"Error destroying CARLA actors during close: {e}")


        for sumo_actor_id in self.carla2sumo_ids.values():
            # Sumo client likely handles check internally, but explicit check is safer
            try:
                 # Check if actor still exists in TraCI before trying to remove
                 if sumo_actor_id in self.sumo.get_state().keys():
                     self.sumo.destroy_actor(sumo_actor_id)
            except Exception as e: # Catch broader exceptions during cleanup
                 logging.warning(f"Error destroying SUMO actor {sumo_actor_id} during close: {e}")


        # Closing sumo and carla client.
        self.carla.close()
        self.sumo.close()

# ==================================================================================================
# -- synchronization_loop Function -----------------------------------------------------------------
# ==================================================================================================

def synchronization_loop(args):
    """
    Entry point for sumo-carla co-simulation.
    """
    sumo_simulation = SumoSimulation(args.sumo_cfg_file, args.step_length, args.sumo_host,
                                     args.sumo_port, args.sumo_gui, args.client_order)
    carla_simulation = CarlaSimulation(args.carla_host, args.carla_port, args.step_length)

    # --- ADDED: Set timeout BEFORE attempting map load ---
    try:
        carla_simulation.client.set_timeout(20.0) # Increased timeout
    except Exception as e:
         logging.error(f"Failed to set CARLA client timeout: {e}")
         sumo_simulation.close() # Clean up sumo connection if CARLA fails early
         return # Exit if we can't even set the timeout


    # --- ADDED: Explicitly load Town04 ---
    try:
        print("Attempting to load CARLA world: Town04...")
        carla_simulation.client.load_world('Town04')
        # Update the world object reference within carla_simulation
        carla_simulation.world = carla_simulation.client.get_world()
        print("CARLA world Town04 loaded successfully.")
        time.sleep(3) # Increased pause slightly
    except RuntimeError as e:
        print(f"Error loading Town04: {e}. Co-simulation might use the wrong map in CARLA.")
        # Decide if you want to exit or continue
        # sys.exit("Exiting due to map load failure.") # Uncomment to exit on failure
    except Exception as e: # Catch other potential client errors
        print(f"An unexpected error occurred during CARLA map load: {e}")
        sumo_simulation.close() # Clean up sumo
        return # Exit
    # --- END ADDED CODE ---


    # --- Check if world loaded successfully before proceeding ---
    if not carla_simulation.world:
         logging.error("CARLA world object failed to initialize after load attempt. Exiting.")
         sumo_simulation.close()
         return


    synchronization = SimulationSynchronization(sumo_simulation, carla_simulation, args.tls_manager,
                                                 args.sync_vehicle_color, args.sync_vehicle_lights)
    try:
        while True:
            start = time.time()

            synchronization.tick()

            end = time.time()
            elapsed = end - start
            if elapsed < args.step_length:
                time.sleep(args.step_length - elapsed)

    except KeyboardInterrupt:
        logging.info('Cancelled by user.')
    except Exception as e: # Catch unexpected errors during the loop
        logging.error(f"An error occurred during the synchronization loop: {e}")

    finally:
        logging.info('Cleaning synchronization')
        # Ensure synchronization object exists before closing
        if 'synchronization' in locals():
             synchronization.close()
        else: # Close individual simulations if synchronization object wasn't created
             if 'carla_simulation' in locals(): carla_simulation.close()
             if 'sumo_simulation' in locals(): sumo_simulation.close()

# ==================================================================================================
# -- main Function and Argument Parsing -------------------------------------------------------------
# ==================================================================================================

if __name__ == '__main__':
    argparser = argparse.ArgumentParser(description=__doc__)
    argparser.add_argument('sumo_cfg_file', type=str, help='sumo configuration file')
    argparser.add_argument('--carla-host',
                           metavar='H',
                           default='127.0.0.1',
                           help='IP of the carla host server (default: 127.0.0.1)')
    argparser.add_argument('--carla-port',
                           metavar='P',
                           default=2000,
                           type=int,
                           help='TCP port to listen to (default: 2000)')
    argparser.add_argument('--sumo-host',
                           metavar='H',
                           default=None, # Will default to 127.0.0.1 in SumoSimulation if None
                           help='IP of the sumo host server (default: 127.0.0.1)')
    argparser.add_argument('--sumo-port',
                           metavar='P',
                           default=None, # Will use traci.checkBinary to find port if None
                           type=int,
                           help='TCP port to listen to (default: auto-detect)')
    argparser.add_argument('--sumo-gui', action='store_true', help='run the gui version of sumo')
    argparser.add_argument('--step-length',
                           default=0.05,
                           type=float,
                           help='set fixed delta seconds (default: 0.05s)')
    argparser.add_argument('--client-order',
                           metavar='TRACI_CLIENT_ORDER',
                           default=1, # Default typically 0, but bridge might need higher if other clients connect
                           type=int,
                           help='client order number for the co-simulation TraCI connection (default: 1)')
    argparser.add_argument('--sync-vehicle-lights',
                           action='store_true',
                           help='synchronize vehicle lights state (default: False)')
    argparser.add_argument('--sync-vehicle-color',
                           action='store_true',
                           help='synchronize vehicle color (default: False)')
    argparser.add_argument('--sync-vehicle-all',
                           action='store_true',
                           help='synchronize all vehicle properties (default: False)')
    argparser.add_argument('--tls-manager',
                           type=str,
                           choices=['none', 'sumo', 'carla'],
                           help="select traffic light manager (default: none)",
                           default='none')
    argparser.add_argument('--debug', action='store_true', help='enable debug messages')
    arguments = argparser.parse_args()

    if arguments.sync_vehicle_all is True:
        arguments.sync_vehicle_lights = True
        arguments.sync_vehicle_color = True

    # Setup logging
    log_level = logging.DEBUG if arguments.debug else logging.INFO
    logging.basicConfig(format='%(levelname)s: %(message)s', level=log_level)

    # Execute simulation loop
    try:
        synchronization_loop(arguments)
    except Exception as e:
        logging.critical(f"Critical error in main execution: {e}")
    finally:
        logging.info("Simulation ended.")