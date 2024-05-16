#!/usr/bin/env python3.7

import carla
import time
import numpy as np
import threading

from world import World
from arguments import parseArguments

import fusion as fu
import equations as eq
import control as con

test = 0
refs = []
N = 8
Ts = 0.05

def main():
    np.set_printoptions(suppress=True,precision=4,linewidth=200)

    args = parseArguments()

    sim_world = World(args)

    bp = sim_world.bp

    spawns = sim_world.map.get_spawn_points()
    transform = spawns[222]

    cars = []

    try:

        leader   = sim_world.world.spawn_actor(bp, transform)
        cars.append(leader)
        leader.set_autopilot()

        global refs
        for i in range(N*8):
            refs.append(fu.get_current_state(leader))
            # print(refs[-1])
            time.sleep(Ts)

        follower = sim_world.world.spawn_actor(bp, transform)
        cars.append(follower)
        vehicle = con.MPC(eq.VehicleModel(),N,Ts)
        # vehicle = con.MPC(eq.VehicleModel(follower),N,Ts)

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
            referencesss = refs[:N+1]
            # if the distance between the 0th state and the Nth state is less than distance D
            # then the velocity should be all zeros in these states
            dis = 0
            for i in range(N):
                dis += np.sqrt(((referencesss[i][0]-referencesss[i+1][0])**2)+((referencesss[i][1]-referencesss[i+1][1])**2))
            print(dis)
            if(dis<=7):
                # print(referencesss)
                for i in range(N):
                    referencesss[i+1][3] = 0
            acc, steer = vehicle.get_control(referencesss)
            end = time.time()
            if (acc < 0.0):
                brake = abs(acc)
                acc = 0.0
            else:
                brake = 0.0
            follower.apply_control(carla.VehicleControl(throttle=acc, steer=steer,brake = brake))
            # print("-----------------------")
            # print("a ",acc)
            # print("d ",steer)
            # print("b ",brake)
            print("solver time: ",end-start)
            # print("-----------------------")
            delay = Ts-(end-start)
            if(delay>0):
                time.sleep(delay)

    finally:
        for car in cars:
            car.destroy()
        print('#############')
        print('# Destroyed #')
        print('#############')


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('\nCancelled by the user. Bye!')
    except RuntimeError as e:
        print(e)
# print("where I got to           ",np.array(refs[0]))
# print("apply this to go                 +",u[0])
# print("where I want to go next  ",np.array(refs[1]))
# print("I think I will get there ", st[1])
# sum = [0,0,0,0,0,0]
# for i in range(N-1):
#     print("ref ",i," ",np.array(refs[i]))
#     print("inp ",i,"    +  ",u[i])
#     print("sta ",i," ",st[i])
#     print("err ",i," ",st[i]-np.array(refs[i]))
#     print()
#     sum += refs[i]-st[i]
