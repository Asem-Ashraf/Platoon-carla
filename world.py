#!/usr/bin/env python3.7

# ==============================================================================
# -- imports -------------------------------------------------------------------
# ==============================================================================
import carla
from vehicle import Vehicle

class World:
    def __init__(self, client, args):
        self.client = client
        carla_world = client.get_world()
        self.world = carla_world
        self.map = carla_world.get_map()
        self.settings = carla_world.get_settings()
        # self.tm = carla_world.get_trafficmanager()
        self.args = args
        self.leader_vehicle = None
        self.follower_vehicles = []

    def __get_spawn_points(self, number_of_points,vehicle_spacing,bp):
        if (self.map.name != 'Carla/Maps/Town06_Opt')|(self.args.reload_map):
            self.client.load_world('Town06_Opt')
            self.map = self.world.get_map()
        # cherry pick points on the map on the longest straight road
        spawn_points = self.map.get_spawn_points()
        point1 = spawn_points[190]
        point2 = spawn_points[222]

        # Create an equation of a line from the two points, where 
        # a = (y1 - y2) / (x1 - x2)
        a = (point1.location.y - point2.location.y) / (point1.location.x - point2.location.x)
        # print("a: ", a)
        # b = y - ax
        b = point2.location.y - (a * point2.location.x)
        # print("b: ", b)

        # cherry picked start point for the line
        X = -330.0
        # Y = aX + b, this is deduced from the two points above
        Y = lambda x: ((a*x) + b)

        spawn_points = []

        for i in range(number_of_points):
            loc = carla.Location(x=X, y=Y(X), z=0.3)
            # preserving the rotation of the spawn point
            ROT = point2.rotation
            transform = carla.Transform(loc, ROT)
            spawn_points.append(transform)
            X += vehicle_spacing
        return spawn_points


    def spawn_platoon(self,bp):
        self.leader_vehicle = Vehicle(0, self.world.spawn_actor(bp, self.map.get_spawn_points()[0]))
        bias = self.leader_vehicle.vehicle_actor_instance.bounding_box.extent.x
        self.leader_vehicle.destroy()
        spawn_points = self.__get_spawn_points(self.args.count, self.args.spacing+bias*2, bp)
        for _ in range(self.args.count-1):
            self.follower_vehicles.append(Vehicle(_+1, self.world.spawn_actor(bp, spawn_points[_])))
        self.leader_vehicle = Vehicle(0, self.world.spawn_actor(bp, spawn_points[self.args.count-1]))

    def get_actor_blueprints(self, filter):
        bps = self.world.get_blueprint_library().filter(filter)
        if len(bps) == 1:
            return bps[0]
        else:
            return None

    def destroy_platoon(self):
        for vehicle in self.follower_vehicles:
            vehicle.destroy()
        self.leader_vehicle.destroy()

    def get_edata(self):
        edata = []
        for vehicle in self.follower_vehicles:
            edata.append(vehicle.get_edata())
        return edata
    def get_data(self):
        data = {}
        for vehicle in self.follower_vehicles:
            data.update({vehicle.id : vehicle.get_data()})
        return data
    def apply_control(self, control):
        for i in range(len(self.follower_vehicles)-1):
            self.follower_vehicles[i].apply_control(control[i])
