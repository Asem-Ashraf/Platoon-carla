#!/usr/bin/env python3.7

import carla
import time
import json
import math
import world
import arguments
import control

def main():
    args = arguments.parseArguments()

    # Carla initialization
    client = carla.Client(args.host, args.port)
    client.set_timeout(args.timeout)
    sim_world = world.World(client, args)

    try:
        arg_bp = sim_world.get_actor_blueprints(args.filter)
        if arg_bp is None:
            print("No vehicle found")
            exit(1)
        sim_world.spawn_platoon(arg_bp)

        # ======================================================================
        # -- This is something that is still being experimented with. The Idea
        # -- of world snapshot is not yet completely understood.
        # ======================================================================
        #print(sim_world.world.get_snapshot().find(sim_world.leader_vehicle.sensor_instances[0].sensor_actor_instance.id).get_transform())
        # ======================================================================

        if args.sync:
            sim_world.settings.synchronous_mode = True
            # This delta seconds could also be given in the command-line.
            sim_world.settings.fixed_delta_seconds = 0.05
            sim_world.world.apply_settings(sim_world.settings)
            for i in range(100):
                # In this mode, a delay is not needed as the timing between
                # each tick can be set. This is the path to be taken for the
                # most accurate sensor reading.
                sim_world.world.tick()

        else:
            while True:
                    data = sim_world.get_data()

                    leaderData = sim_world.leader_vehicle.get_data()
                    temp = leaderData['Velocity']
                    vl = math.sqrt(((temp.x)**2)+((temp.y)**2))

                    followerData = data[1]
                    temp = followerData['Velocity']
                    vf = math.sqrt(((temp.x)**2)+((temp.y)**2))

                    xl = 10000*float(leaderData['GNSS']['lat'])
                    yl = 10000*float(leaderData['GNSS']['lon'])
                    xf = 10000*float(followerData['GNSS']['lat'])
                    yf = 10000*float(followerData['GNSS']['lon'])

                    controlInputs = [0.5,0]

                    controlInputs[1] = control.get_control(xl,yl,xf,yf,vl,vf)

                    print("leader  :",xl,yl)
                    print("follower:",xf,yf)
                    print("speeds  :",vf,vl)
                    print(math.sqrt(((xf-xl)**2)+((yf-yl)**2)))
                    print(controlInputs)
                    sim_world.apply_control(controlInputs)

                    # A delay so that the environment changes a little bit before
                    # sending the next sensor readings. Otherwise, this script
                    # flood the raspberry pi with sensors whenever it can.
                    # time.sleep(0.05)
    finally:
        sim_world.destroy_platoon()
        print('==================')
        print('Platoon Destroyed.')
        print('==================')

if __name__ == '__main__':
    main()
