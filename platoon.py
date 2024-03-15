#!/usr/bin/env python3.7

import carla
import time
import numpy as np
import threading

import world
import arguments

import fusion as fu
import equations as eq
import control as con

test = 0
refs = []
N = 8
Ts = 0.07

def main():
    args = arguments.parseArguments()

    # Carla initialization
    client = carla.Client(args.host, args.port)
    client.set_timeout(args.timeout)

    sim_world = world.World(client, args)

    bp = sim_world.get_actor_blueprints(args.filter)

    spawns = sim_world.map.get_spawn_points()
    transform = spawns[222]
    np.set_printoptions(suppress=True,precision=4,linewidth=200)

    try:
        leader   = sim_world.world.spawn_actor(bp, transform)
        leader.set_autopilot()

        global refs
        for i in range(N):
            refs.append(fu.get_current_state(leader))
            time.sleep(Ts)

        follower = sim_world.world.spawn_actor(bp, transform)
        vehicle = con.MPC(eq.VehicleModel(follower),N,Ts)

        def move(leader,follower):
            while True:
                refs.pop(0)
                refs.append(fu.get_current_state(leader))
                refs[0] = fu.get_current_state(follower)
                time.sleep(Ts)

        t1 = threading.Thread(target=move,args=(leader,follower))
        t1.start()

        while True:
            start = time.time()
            acc, steer = vehicle.get_control(refs)
            end = time.time()
            if (acc < 0.0):
                brake = abs(acc)
                acc = 0.0
            else:
                brake = 0.0
            follower.apply_control( carla.VehicleControl(throttle=acc/6.0, steer=steer/np.deg2rad(70),brake = brake/6.0))
            print("-----------------------")
            print("a ",acc)
            print("d ",steer)
            print("b ",brake)
            print("-----------------------")
            print("time:",end-start)
            delay = Ts-(end-start)
            if(delay>0)
                time.sleep(delay)

    finally:
        leader.destroy()
        follower.destroy()
        print('==================')
        print('Platoon Destroyed.')
        print('==================')


if __name__ == '__main__':
    main()
            # print(follower.get_control())
            #
            # estimated_inputs_all = estimated_values[:n_controls*(N-1)]
            # all_states_init_val = estimated_values[(N-1)*n_controls:]
            # all_inputs_init_val[0] = acc
            # all_inputs_init_val[1] = steer
            # st = all_states_init_val.reshape(N,n_states)
            # u = estimated_inputs_all.reshape(N-1,n_controls)
            # print("where I got to           ",np.array(refs[0]))
            # print("apply this to go                 +",u[0])
            # print("where I want to go next  ",np.array(refs[1]))
            # print("I think I will get there ", st[1])
            # # sum = [0,0,0,0,0,0]
            # # for i in range(N-1):
            # #     print("ref ",i," ",np.array(refs[i]))
            # #     print("inp ",i,"    +  ",u[i])
            # #     print("sta ",i," ",st[i])
            # #     print("err ",i," ",st[i]-np.array(refs[i]))
            # #     print()
            #
            #     # sum += refs[i]-st[i]
            # # print(sum/N)
            # # print()

