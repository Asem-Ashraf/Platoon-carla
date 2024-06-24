#!/usr/bin/env python3.7

import carla
from vehicle import Vehicle
from arguments import parseArguments
import numpy as np
import cv2

# MAP = 'Town07_Opt'
MAP = 'Town06_Opt'
MAP_FULLNAME = 'Carla/Maps/' + MAP


class World:

    def __init__(self):
        self.args = parseArguments()

        self.client = carla.Client(self.args.host, self.args.port)
        self.client.set_timeout(self.args.timeout)

        self.world = self.client.get_world()
        self.tm = self.client.get_trafficmanager()

        if (self.world.get_map().name !=
                MAP_FULLNAME) | (self.args.reload_map):
            try:
                self.client.load_world(MAP)
            except:
                print("Map: Town06_Opt is not available")
                print("Have you downloaded it from additional maps?")
                print("exiting...")
                exit()

        self.map = self.world.get_map()
        self.spectator = self.world.get_spectator()
        self.settings = self.world.get_settings()
        self.blueprint_lib = self.world.get_blueprint_library()

        self.settings.fixed_delta_seconds = 1 / 50
        self.settings.synchronous_mode = True  # Enables synchronous mode
        self.settings.substepping = True
        self.settings.max_substep_delta_time = 0.005
        self.settings.max_substeps = 16

        self.tm.set_synchronous_mode(True)
        self.tm.set_random_device_seed(2)

        self.world.apply_settings(self.settings)
        # self.world.freeze_all_traffic_lights(True)

        self.leader_vehicle = None
        self.follower_vehicles = None
        bp = self.world.get_blueprint_library().filter(self.args.filter)

        if len(bp) < 1:
            print("Blueprint ", filter, " was not found")
            print("Refernce the blueprint library to find valid blueprints")
            print("exiting...")
            exit()
        self.bp = bp[0]
        self.records  = None

    def spawnPlatoon(self, record=False):
        '''
            spawns the leader and the followers along a specific road in the map
            then moves the spectator to view the platoon from above
        '''
        vehicle_length = self.get_vehicle_length()

        spawn_points = self.__myGet_spawn_points(
            self.args.count, self.args.spacing + vehicle_length)

        if self.follower_vehicles != None:
            print("Platoon already spawned")
            print("Exiting...")
            exit()
        self.follower_vehicles = []
        for _ in range(self.args.count - 1):
            self.follower_vehicles.append(
                Vehicle(_ + 1, self.world.spawn_actor(self.bp,
                                                      spawn_points[_])))

        self.leader_vehicle = Vehicle(
            0,
            self.world.spawn_actor(self.bp, spawn_points[self.args.count - 1]))
        if record:
            self.records = []
            # a camera blueprint. This camera blueprint is used twice to record a video of the platoon from various angels.
            camera_bp = self.blueprint_lib.filter('sensor.camera.rgb')[0]

            # FHD video
            height, width = 1080, 1920
            # 50 fps
            frametime = self.settings.fixed_delta_seconds
            cameraFPS = int(round(1/frametime))

            camera_bp.set_attribute('image_size_x', str(width))  # Set width
            camera_bp.set_attribute('image_size_y', str(height))   # Set height
            camera_bp.set_attribute('sensor_tick', str(frametime)) # Set the time in seconds between sensor captures

            # Front view angle transform
            frontViewTrans = carla.Transform(carla.Location(4.5,-2.5,1.5),carla.Rotation(0,20,0))

            # Top view angle transform
            topViewTrans = carla.Transform(carla.Location(-10,0,30),carla.Rotation(-30,0,90))

            # Attaching the camera to the leader
            frontViewCamActor = self.world.spawn_actor(
                    camera_bp,
                    frontViewTrans,
                    attach_to=self.leader_vehicle,
                    # SpringArm attachment for less shaky videos
                    attachment_type=carla.AttachmentType.SpringArm)
            self.follower_vehicles.append(frontViewCamActor)

            # Attaching the camera to the leader
            topViewCamActor = self.world.spawn_actor(
                    camera_bp,
                    topViewTrans,
                    attach_to=self.leader_vehicle,
                    # SpringArm attachment for less shaky videos
                    attachment_type=carla.AttachmentType.SpringArm)
            self.follower_vehicles.append(topViewCamActor)

            frontViewVideo = cv2.VideoWriter('FrontView.avi', cv2.VideoWriter_fourcc(*'XVID'), cameraFPS, (width, height))
            self.records.append(frontViewVideo)
            topViewVideo = cv2.VideoWriter('TopView.avi', cv2.VideoWriter_fourcc(*'XVID'), cameraFPS, (width, height))
            self.records.append(topViewVideo)

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

        # Special point for the spectator above the platoon's spawn location
        loc = spawn_points[self.args.count - 1].location
        loc.z += 50
        rot = carla.Rotation(pitch=90.0, yaw=90.0, roll=0.0)
        self.spectator.set_transform(carla.Transform(loc, rot))

    def destroyPlatoon(self):
        '''
            Destroys the leader and followers in the platoon
        '''
        if self.records != None:
            for vid in self.records:
                vid.release()
        for vehicle in self.follower_vehicles:
            vehicle.destroy()
        self.leader_vehicle.destroy()

    def getPlatoonData(self):
        '''
            returns the data of each vehicle in order in a list
        '''
        if self.leader_vehicle == None:
            print("Spawn the platoon first")
            print("Exiting...")
            exit()
        data = []
        data.append(self.leader_vehicle.getData())
        for vehicle in self.follower_vehicles:
            data.append(vehicle.getData())
        return data

    def applyPlatoonControls(self, controls):
        '''
            control: a list of controls for each follower in order
            applies the controls to the vehicles in order
        '''
        if self.leader_vehicle == None:
            print("Spawn the platoon first")
            print("Exiting...")
            exit()
        for i in range(len(self.follower_vehicles) - 1):
            self.follower_vehicles[i].applyControl(controls[i])

    def __myGet_spawn_points(self, n, vehicle_spacing):
        '''
            n: the number of points to return
            vehicle_spacing: the distance between each point
            returns a list of n points with vehicle_spacing between them
        '''
        if (self.map.name != MAP_FULLNAME) | (self.args.reload_map):
            self.client.load_world(MAP)
            self.spectator = self.world.get_spectator()
            self.map = self.world.get_map()
        # cherry pick points on the map on the longest straight road
        spawn_points = self.map.get_spawn_points()
        point1 = spawn_points[190]
        point2 = spawn_points[222]

        # Create an equation of a line from the two points, where
        # a = (y1 - y2) / (x1 - x2)
        a = (point1.location.y - point2.location.y) / (point1.location.x -
                                                       point2.location.x)
        # b = y - ax
        b = point2.location.y - (a * point2.location.x)

        # cherry picked start point for the line
        X = -330.0
        # Y = aX + b, this is deduced from the two points above
        Y = lambda x: ((a * x) + b)

        spawn_points = []

        for i in range(n):
            loc = carla.Location(x=X, y=Y(X), z=0.3)
            # preserving the rotation of the spawn point
            rot = point2.rotation
            transform = carla.Transform(loc, rot)
            spawn_points.append(transform)
            X += vehicle_spacing
        return spawn_points

    def __get_vehicle_length(self):
        v = self.world.spawn_actor(self.bp, self.map.get_spawn_points()[190])
        half_length = v.bounding_box.extent.x
        v.destroy()
        return half_length * 2
