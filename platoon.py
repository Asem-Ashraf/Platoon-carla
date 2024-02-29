#!/usr/bin/env python3.7

import carla
import time
import json
import math
from world import World
from arguments import parseArguments
import control


def main():
    args = parseArguments()

    # Carla initialization
    client = carla.Client(args.host, args.port)
    client.set_timeout(args.timeout)
    sim_world = World(client, args)

    arg_bp = sim_world.get_actor_blueprints(args.filter)
    if arg_bp is None:
        print(f"Vehicle {arg_bp} not found.\n Exiting...")
        exit(1)

    try:
        sim_world.spawn_platoon(arg_bp)

        # ======================================================================
        # CARLA ACTOR AND WORLD SNAPSHOT TO GET ACCESS TO THESE VALUES FROM THE
        # ACTOR'S ID WHILE THE SIMULATION KEEPS RUNNING
        # ======================================================================
        # world_snapshot = sim_world.world.get_snapshot()
        # actor_snapshot = world_snapshot.find(
        #     sim_world.leader_vehicle.sensor_instances[0].sensor_actor_instance.
        #     id)
        # actor_transform = actor_snapshot.get_transform()
        # actor_angular_velocity = actor_snapshot.get_angular_velocity()
        # actor_velocity = actor_snapshot.get_velocity()
        # actor_acceleration = actor_snapshot.get_acceleration()

        if not args.sync:
            while True:
                data = sim_world.get_data()

                leaderData = sim_world.leader_vehicle.get_data()

                controlInputs = control.get_control()

                sim_world.apply_control(controlInputs)

                # A delay so that the environment changes a little bit before
                # sending the next sensor readings. Otherwise, this script
                # flood the raspberry pi with sensors whenever it can.
                time.sleep(0.05)

        else:
            sim_world.settings.synchronous_mode = True
            # This delta seconds could also be given in the command-line.
            sim_world.settings.fixed_delta_seconds = 0.05
            sim_world.world.apply_settings(sim_world.settings)
            for i in range(100):
                # In this mode, a delay is not needed as the timing between
                # each tick can be set. This is the path to be taken for the
                # most accurate sensor reading.
                sim_world.world.tick()

    finally:
        sim_world.destroy_platoon()
        print('==================')
        print('Platoon Destroyed.')
        print('==================')


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')
    except RuntimeError as e:
        print(e)

