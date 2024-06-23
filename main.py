#!/usr/bin/env python3.7

import carla
import numpy as np
import cv2

from world import World
from arguments import parseArguments

import fusion as fu
import equations as eq
import control as con


def main():
    np.set_printoptions(suppress=True,precision=4,linewidth=200)

    args = parseArguments()

    sim_world = World(args)

    bp = sim_world.bp

    refs = []
    N = 80
    Ts = 0.02

    # spawns = sim_world.map.get_spawn_points()
    # transform = spawns[222]
    # transform = spawns[13]
    transform = carla.Transform(carla.Location(x=72.432770, y=-7.948537, z=0.1), carla.Rotation(pitch=1.276521, yaw=-62.710632, roll=0.036662))

    actors = []

    try:

        # spawn a leader vehicle
        leader   = sim_world.world.spawn_actor(bp, transform)
        actors.append(leader)

        # move the spectator to a top view above the leader (it does NOT move with the leader)
        spec_trans = carla.Transform()
        spec_trans.location = transform.location
        spec_trans.rotation = transform.rotation
        spec_trans.location.z += 30
        spec_trans.rotation.pitch-=90
        spec_trans.rotation.yaw+=90
        sim_world.spectator.set_transform(spec_trans)

        # a camera blueprint. This camera blueprint is used twice to record a video of the platoon from various angels.
        camera_bp = sim_world.blueprint_lib.filter('sensor.camera.rgb')[0]

        # FHD video
        height, width = 1080, 1920
        # 50 fps
        cameraFPS = 50
        frametime = 1/cameraFPS

        camera_bp.set_attribute('image_size_x', str(width))  # Set width
        camera_bp.set_attribute('image_size_y', str(height))   # Set height
        camera_bp.set_attribute('sensor_tick', str(frametime)) # Set the time in seconds between sensor captures

        # Front view angle transform
        frontViewTrans = carla.Transform(carla.Location(4.5,-2.5,1.5),carla.Rotation(0,20,0))

        # Top view angle transform
        topViewTrans = carla.Transform(carla.Location(-10,0,30),carla.Rotation(-30,0,90))

        # Attaching the camera to the leader
        frontViewCamActor = sim_world.world.spawn_actor(
                camera_bp,
                frontViewTrans,
                attach_to=leader,
                # SpringArm attachment for less shaky videos
                attachment_type=carla.AttachmentType.SpringArm)
        actors.append(frontViewCamActor)

        # Attaching the camera to the leader
        topViewCamActor = sim_world.world.spawn_actor(
                camera_bp,
                topViewTrans,
                attach_to=leader,
                # SpringArm attachment for less shaky videos
                attachment_type=carla.AttachmentType.SpringArm)
        actors.append(topViewCamActor)

        frontViewVideo = cv2.VideoWriter('FrontView.avi', cv2.VideoWriter_fourcc(*'XVID'), cameraFPS, (width, height))
        topViewVideo = cv2.VideoWriter('TopView.avi', cv2.VideoWriter_fourcc(*'XVID'), cameraFPS, (width, height))

        def addImageToVid(video,image):
            # Convert the raw image data to a numpy array
            array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
            array = np.reshape(array, (image.height, image.width, 4))
            # Convert from BGRA to BGR
            bgr_array = array[:, :, :3]
            # Store the image in the list
            video.write(bgr_array)
        # Set up the camera to call the process_image function for each captured frame

        frontViewCamActor.listen(lambda image: addImageToVid(frontViewVideo,image))
        topViewCamActor.listen(lambda image: addImageToVid(topViewVideo,image))

        leader.set_autopilot()




        global refs
        refs.append(fu.get_current_state(leader))
        print(sim_world.world.tick())
        for i in range(N):
            refs.append(fu.get_current_state(leader))
            print("before: ",refs[-1][2])
            if (refs[-2][2]-refs[-1][2])>np.pi:
                refs[-1][2]+=2*np.pi
            elif (refs[-2][2]-refs[-1][2])<-np.pi:
                refs[-1][2]-=2*np.pi
            print("after: ",refs[-1][2])
            print(sim_world.world.tick())


        follower = sim_world.world.spawn_actor(bp, transform)
        actors.append(follower)

        print(sim_world.world.tick())
        print(sim_world.world.tick())

        # vehicle = con.MPC(eq.VehicleModel(),N,Ts)
        vehicle = con.MPC(eq.VehicleModel(follower),N,Ts)
        index = 0
        while True:
            refs.append(fu.get_current_state(leader))

            differ =refs[-2][2]-refs[-1][2]
            times = round(abs(differ/(2*np.pi)))
            if (differ)>np.pi:
                refs[-1][2]+=2*np.pi*times
            elif (differ)<-np.pi:
                refs[-1][2]-=2*np.pi*times

            follower_list = refs[-N:]
            follower_list[0]=fu.get_current_state(follower)

            differ = follower_list[0][2]-follower_list[1][2]
            times = round(abs(differ/(2*np.pi)))
            if (differ)<-np.pi:
                follower_list[0][2]+=2*np.pi*times
            elif (differ)>np.pi:
                follower_list[0][2]-=2*np.pi*times

            acc, steer = vehicle.get_control(follower_list)
            if (acc < 0.0):
                brake = abs(acc)
                acc = 0.0
            else:
                brake = 0.0
            print("a ",acc)
            print("d ",steer)
            print("b ",brake)
            index += 1
            follower.apply_control(carla.VehicleControl(throttle=acc, steer=steer,brake = brake))
            sim_world.world.tick()

    finally:
        for car in actors:
            car.destroy()
        topViewVideo.release()
        frontViewVideo.release()
        print('\n#############')
        print('# Destroyed #')
        print('#############')


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('\nCancelled by the user. Bye!')
    except RuntimeError as e:
        print(e)
