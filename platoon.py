#!/usr/bin/env python3.7

import carla
import time
import world
import arguments
import control as con
import trajectory as tj
import equations as eq
import numpy as np
import os


def main():
    args = arguments.parseArguments()

    # Carla initialization
    client = carla.Client(args.host, args.port)
    client.set_timeout(args.timeout)
    sim_world = world.World(client, args)
    # bp = sim_world.get_actor_blueprints('vehicle.bh.crossbike')
    bp = sim_world.get_actor_blueprints(args.filter)

    spawns = sim_world.map.get_spawn_points()
    transform = spawns[222]

    # loc = transform.location
    # startwaypoint = sim_world.map.get_waypoint(loc, project_to_road=True, lane_type=carla.LaneType.Driving)

    try:
        instance = sim_world.world.spawn_actor(bp, transform)
        hz = 8
        # Load the initial states
        U1 = 0.0
        Ts = 0.02
        # x_dot = 10
        vehicle = eq.CarModel(instance)
        vehicle.constants['Ts'] = Ts

        instance.enable_constant_velocity(carla.Vector3D(10, 0, 0))
        time.sleep(0.1)
        instance.disable_constant_velocity()

        mpc = con.MPC()
        Ad, Bd, Cd, Dd = vehicle.get_discrete_state_space_matrices()
        Hdb, Fdbt, Cdb, Adc = mpc.mpc_simplification(Ad, Bd, Cd, Dd, hz)

        # trajectory = tj.get_trajectory(startpoint,spacing=Ts*x_dot, n=10000)
        # for i in trajectory:
        #     sim_world.world.debug.draw_point(i.transform.location,
        #                                      size=0.05,
        #                                      color=carla.Color(255, 0, 0, 255),
        #                                      life_time=100)
        # refSignals,xref = tj.simplify_trajectory(trajectory)
        # k = 0
        # outputs = 2
        bounding_box = instance.bounding_box
        states,px,v,py,theta = vehicle.get_next_state_carla()
        newtrans = spawns[223]
        while True:
            # for i in states:
            #     print(f'state: {i:.5f}')
            # print(f'v: {v:.5f}')
            # print(f'xref: {xref[int(k/outputs)]:.5f}')

            # if xref[int(k/outputs)]<=px:
            #     k = k + outputs
            #     if k + outputs * hz <= len(refSignals):
            #         r = refSignals[k:k + outputs * hz]
            #     else:
            #         r = refSignals[k:len(refSignals)]
            #         hz = hz - 1
            transform = newtrans
            loc = transform.location
            # loc.x = loc.x + bounding_box.extent.x + 1
            startwaypoint = sim_world.map.get_waypoint(loc, project_to_road=True, lane_type=carla.LaneType.Driving)
            # loc = startwaypoint.transform.location
            # loc.x = loc.x + bounding_box.extent.x + 1
            # startwaypoint = sim_world.map.get_waypoint(loc, project_to_road=True, lane_type=carla.LaneType.Driving)
            trajectory = tj.get_trajectory(startwaypoint,spacing=Ts*v, n=hz-2)
            newtrans = trajectory[1].transform
            for i in trajectory:
                sim_world.world.debug.draw_point(i.transform.location, size=0.05, color=carla.Color(255, 0, 0, 100), life_time=0.3)
            refSignals,xref = tj.simplify_trajectory(trajectory)
            r = refSignals
            os.system('clear')
            # for i in range(0,len(r),2):
            #     print(f'r{i}: {r[i]:.5f}')
            #     print(f'r{i+1}: {r[i+1]:.5f}')
            error = [r[1]-py,xref[0]-px]
            # print(f'error a: {error[0]:.6f}')
            print(f'error y: {error[0]:.6f}')
            print(f'error x: {error[1]:.6f}')
            # print('loca :', instance.get_transform().location)
            # Generate the augmented current state and the reference vector
            x_aug_t = np.transpose([np.concatenate((states, [U1]), axis=0)])
            # for i in states:
            #     print(f'state: {i:.5f}')
            # print(f'px: {px:.5f}')
            # print(f'xref: {xref[int(k/outputs)]:.5f}')
            # print('x_aug_t:', x_aug_t)
            ft = np.matmul( np.concatenate((np.transpose(x_aug_t)[0][0:len(x_aug_t)], r), axis=0), Fdbt)
            du = -np.matmul(np.linalg.inv(Hdb), np.transpose([ft]))
            U1 = U1 + du[0][0]
            # print('U1   :', U1)
            ster = U1 % 360
            if ster>= 180.0:
                ster = ster- 360.0
            if ster<= -180.0:
                ster = ster + 360.0
            # print('U1   :', ster)
            ster = max(min(ster,50.0),-50.0)/50.0
            U1 = ster*50.0
            # print('steer:', ster)
            # print('==================')
            # instance.apply_control(carla.VehicleControl(steer=ster))
            instance.apply_control(carla.VehicleControl(steer=ster))
            states,px,v,py,theta = vehicle.get_next_state_carla()
            instance.enable_constant_velocity(carla.Vector3D(10, 0, 0))
            time.sleep(0.1*Ts)
            instance.disable_constant_velocity()
            # Compute new states in the open loop system (interval: Ts/30)
            # time.sleep(Ts)
            # trajectory = tj.get_trajectory(sim_world.map.get_waypoint(
            #     instance.get_transform().location,
            #     project_to_road=True,
            #     lane_type=carla.LaneType.Driving),
            #                                n=hz)
            # print('trajectory:', trajectory[0].transform)
            # for i in trajectory:
            #     sim_world.world.debug.draw_point(i.transform.location,
            #                                      size=0.01,
            #                                      color=carla.Color(
            #                                          255, 0, 0, 255),
            #                                      life_time=0.1)
            # refSignals = tj.simplify_trajectory(trajectory)
        #     # time.sleep(1)

    finally:
        instance.destroy()
        print('==================')
        print('Platoon Destroyed.')
        print('==================')


if __name__ == '__main__':
    main()
