#!/usr/bin/env python3.7

import carla
from vehicle import Vehicle
from arguments import parseArguments


class World:
    def __init__(self):
        self.args = parseArguments()

        client = carla.Client(self.args.host, self.args.port)
        client.set_timeout(self.args.timeout)
        carla_world = client.get_world()

        self.client = client
        self.world = carla_world

        self.leader_vehicle = None
        self.follower_vehicles = []
        bp = self.world.get_blueprint_library().filter(self.args.filter)

        if len(bp) < 1:
            print("Blueprint ", filter, " was not found")
            print("Refernce the blueprint library to find valid blueprints")
            print("exiting...")
            exit()
        self.bp = bp[0]

        if (self.world.get_map().name != 'Carla/Maps/Town06_Opt') | (self.args.reload_map):
            try:
                self.client.load_world('Town06_Opt')
            except:
                print("Map: Town06_Opt is not available")
                print("Have you downloaded it from additional maps?")
                print("exiting...")
                exit()
        self.settings = carla_world.get_settings()
        self.map = self.world.get_map()
        self.spectator = self.world.get_spectator()


    def spawn_platoon(self):
        '''
            spawns the leader and the followers along a specific road in the map
            then moves the spectator to view the platoon from above
        '''
        vehicle_length = self.get_vehicle_length()
        spawn_points = self.__myGet_spawn_points(
            self.args.count, self.args.spacing + vehicle_length)
        for _ in range(self.args.count - 1):
            self.follower_vehicles.append(
                Vehicle(_ + 1, self.world.spawn_actor(self.bp,
                                                      spawn_points[_])))
        self.leader_vehicle = Vehicle(
            0,
            self.world.spawn_actor(self.bp, spawn_points[self.args.count - 1]))
        # Special point for the spectator above the platoon's spawn location
        # loc = carla.Location(x=-293.839325, y=247.607788, z=48.441013)
        loc = spawn_points[self.args.count - 1].location
        loc.z += 50
        rot = carla.Rotation(pitch=90.0, yaw=90.0, roll=0.0)
        self.spectator.set_transform(carla.Transform(loc, rot))


    def destroy_platoon(self):
        '''
            Destroys the leader and followers in the platoon
        '''
        for vehicle in self.follower_vehicles:
            vehicle.destroy()
        self.leader_vehicle.destroy()

    def get_platoon_data(self):
        '''
            returns the data of each vehicle in a dictionary where each vehicle
            and its sensor readings are labeled
        '''
        data = {}
        for vehicle in self.follower_vehicles:
            data.update({vehicle.id: vehicle.get_data()})
        data.update({0: self.leader_vehicle.get_data()})
        return data

    def apply_platoon_control(self, control):
        '''
            control: a list of controls for each follower in order
            applies the controls to the vehicles in order
        '''
        for i in range(len(self.follower_vehicles) - 1):
            self.follower_vehicles[i].apply_control(control[i])

    def __myGet_spawn_points(self, n, vehicle_spacing):
        '''
            n: the number of points to return
            vehicle_spacing: the distance between each point
            returns a list of n points with vehicle_spacing between them
        '''
        if (self.map.name != 'Carla/Maps/Town06_Opt') | (self.args.reload_map):
            self.client.load_world('Town06_Opt')
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
    def get_vehicle_length(self):
        v = self.world.spawn_actor(self.bp, self.map.get_spawn_points()[190])
        half_length = v.bounding_box.extent.x
        v.destroy()
        return half_length * 2
