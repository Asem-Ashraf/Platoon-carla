#!/usr/bin/env python3.7

import carla
from vehicle import Vehicle


class World:

    def __init__(self, client, args):
        if (client.get_world().get_map().name !=
                'Carla/Maps/Town06_Opt') | (self.args.reload_map):
            client.load_world('Town06_Opt')
        self.world = client.get_world()
        self.map = self.world.get_map()
        self.settings = self.world.get_settings()
        # self.tm = carla_world.get_trafficmanager()
        self.args = args
        self.leader_vehicle = None
        self.followers_list = []
        self.spectator = self.world.get_spectator()
        # Special point for the spectator above the platoon's spawn location
        loc = carla.Location(x=-293.839325, y=247.607788, z=48.441013)
        rot = carla.Rotation(pitch=-88.925880, yaw=90.018211, roll=-0.001002)
        self.spectator.set_transform(carla.Transform(loc, rot))

    def __get_spawn_points(self, number_of_points, vehicle_spacing):
        # cherry pick points on the map on the longest straight road
        spawn_points = self.map.get_spawn_points()
        point1= spawn_points[190]
        point2= spawn_points[222]
        point1loc = point1.location
        point2loc = point2.location

        # Create an equation of a line from the two points, where
        # a = (y1 - y2) / (x1 - x2)
        a = (point1loc.y - point2loc.y) /
            (point1loc.x - point2loc.x)
        # print("a: ", a)
        # b = y - ax
        b = point2loc.y - (a * point2loc.x)
        # print("b: ", b)

        # cherry picked start point for the line
        X = -330.0
        # Y = aX + b, this is deduced from the two points above
        Y = lambda x: ((a * x) + b)

        spawn_points = []

        for i in range(number_of_points):
            loc = carla.Location(x=X, y=Y(X), z=0.3)
            # preserving the rotation of the spawn point
            ROT = point2.rotation
            transform = carla.Transform(loc, ROT)
            spawn_points.append(transform)
            X += vehicle_spacing
        return spawn_points

    def spawn_platoon(self, bp):
        # spawns a vehicle just to get the extent in x dimention
        temp_vehicle = Vehicle(0,
                          self.world.spawn_actor(bp,
                                                 self.map.get_spawn_points()[0]
                                                 )
                          )
        bias = temp_vehicle.carla_instance.bounding_box.extent.x
        temp_vehicle.destroy()
        spawn_points = self.__get_spawn_points(self.args.count,
                                               self.args.spacing + bias * 2,
                                               bp)
        for id in range(self.args.count - 1):
            self.followers_list.append(
                Vehicle(id + 1, self.world.spawn_actor(bp, spawn_points[id])))
        # leader spawns last, at the beginning of the platoon.
        self.leader_vehicle= Vehicle(
            0, self.world.spawn_actor(bp, spawn_points[self.args.count - 1]))

    def get_actor_blueprints(self, filter):
        bps = self.world.get_blueprint_library().filter(filter)
        if len(bps) == 1:
            return bps[0]
        else:
            return None

    def destroy_platoon(self):
        for vehicle in self.followers_list:
            vehicle.destroy()
        self.leader_vehicle.destroy()

    def get_edata(self):
        edata = []
        for vehicle in self.followers_list:
            edata.append(vehicle.get_edata())
        return edata

    def get_data(self):
        data = {}
        for vehicle in self.followers_list:
            data.update({vehicle.id: vehicle.get_data()})
        return data

    def apply_control(self, control_list):
        for i in range(len(self.followers_list) - 1):
            self.followers_list[i].apply_control(control_list[i])
