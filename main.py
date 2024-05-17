#!/usr/bin/env python3.7

import time
from world import World
import control as con
import ourmqtt

Ts = 0.05

def main():
    sim_world = World()

    try:
        sim_world.spawn_platoon()
        if not sim_world.args.sync:
            while True:
                data = sim_world.get_data()

                leaderData = sim_world.leader_vehicle.get_data()

                ourmqtt.send_data(data,leaderData)

                sim_world.apply_control(con.get_controls())

                # A delay so that the environment changes a little bit before
                # sending the next sensor readings. Otherwise, this script
                # flood the raspberry pi with sensors whenever it can.
                time.sleep(Ts)

        else:
            sim_world.settings.synchronous_mode = True
            # This delta seconds could also be given in the command-line.
            sim_world.settings.fixed_delta_seconds = Ts
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

