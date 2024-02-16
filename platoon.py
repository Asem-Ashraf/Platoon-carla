#!/usr/bin/env python3.7

# ==============================================================================
# -- imports -------------------------------------------------------------------
# ==============================================================================
import carla
import weakref
import math
import numpy as np
import argparse
import time
import json

# ==============================================================================
# -- GnssSensor ----------------------------------------------------------------
# ==============================================================================
class GnssSensor(object):
    def __init__(self, parent_actor):
        self.sensor_actor_instance = None
        self._parent_actor_instance = parent_actor
        # Readings (Efficient)
        self.edata = []
        self.data = {"lat": "", "lon": ""}

        world = self._parent_actor_instance.get_world()
        bp = world.get_blueprint_library().find('sensor.other.gnss')
        self.sensor_actor_instance = world.spawn_actor(bp, carla.Transform(carla.Location(x=1.0, z=2.8)), attach_to=parent_actor)
        # We need to pass the lambda a weak reference to self to avoid circular reference.
        weak_self = weakref.ref(self)
        self.sensor_actor_instance.listen(lambda event: GnssSensor._on_gnss_event(weak_self, event))
    def get_edata(self):
        return self.edata;
    def get_data(self):
        return {"GNSS":self.data}
    def destroy(self):
        self.sensor_actor_instance.destroy()
    @staticmethod
    def _on_gnss_event(weak_self, event):
        self = weak_self()
        if not self:
            return
        self.edata = [event.latitude, event.longitude]
        self.data.update({"lat": str(event.latitude),
                          "lon": str(event.longitude)})

# ==============================================================================
# -- IMUSensor -----------------------------------------------------------------
# ==============================================================================
class IMUSensor(object):
    def __init__(self, parent_actor):
        self.sensor_actor_instance = None
        self._parent_actor_instance = parent_actor

        # Readings (Efficient)
        self.edata = []
        self.data = {}
        world = self._parent_actor_instance.get_world()
        bp = world.get_blueprint_library().find('sensor.other.imu')
        self.sensor_actor_instance = world.spawn_actor( bp, carla.Transform(), attach_to=parent_actor)
        # We need to pass the lambda a weak reference to self to avoid circular
        # reference.
        weak_self = weakref.ref(self)
        self.sensor_actor_instance.listen( lambda sensor_data: IMUSensor._IMU_callback(weak_self, sensor_data))
    def get_edata(self):
        return self.edata
    def get_data(self):
        return {"IMU":self.data}
    def destroy(self):
        self.sensor_actor_instance.destroy()
    @staticmethod
    def _IMU_callback(weak_self, sensor_data):
        self = weak_self()
        if not self:
            return
        # Readings (Efficient)
        self.edata=[
                [ sensor_data.accelerometer.x,
                 sensor_data.accelerometer.y,
                 sensor_data.accelerometer.z
                ],
                [ sensor_data.gyroscope.x,
                 sensor_data.gyroscope.y,
                 sensor_data.gyroscope.z
                ],
                math.degrees(sensor_data.compass)
                ]
        self.data.update({
            "accelerometer": {
                "x":str(sensor_data.accelerometer.x),
                "y":str(sensor_data.accelerometer.y),
                "z":str(sensor_data.accelerometer.z)
                },
            "gyroscope": {
                "x" : str(sensor_data.gyroscope.x),
                "y" : str(sensor_data.gyroscope.y),
                "z" : str(sensor_data.gyroscope.z)
                },
            "compass": str(sensor_data.compass)})

# ==============================================================================
# -- RadarSensor ---------------------------------------------------------------
# ==============================================================================
class RadarSensor(object):
    def __init__(self, parent_actor):
        self.sensor_actor_instance = None
        self._parent_actor_instance = parent_actor
        bound_x = 0.5 + self._parent_actor_instance.bounding_box.extent.x
        bound_y = 0.5 + self._parent_actor_instance.bounding_box.extent.y
        bound_z = 0.5 + self._parent_actor_instance.bounding_box.extent.z
        # Readings (Efficient)
        self.edata= []
        self.data = {
                    "points": {
                              "0": {
                                   "velocity": 0.0,
                                   "altitude": 0.0,
                                   "azimuth": 0.0,
                                   "depth": 0.0
                                   }
                              }
                    }
        world = self._parent_actor_instance.get_world()
        bp = world.get_blueprint_library().find('sensor.other.radar')
        bp.set_attribute('horizontal_fov', str(35))
        bp.set_attribute('vertical_fov', str(20))
        self.sensor_actor_instance = world.spawn_actor(
            bp,
            carla.Transform(
                carla.Location(x=bound_x + 0.05, z=bound_z+0.05),
                carla.Rotation(pitch=5)),
            attach_to=parent_actor)
        # We need a weak reference to self to avoid circular reference.
        weak_self = weakref.ref(self)
        self.sensor_actor_instance.listen(
            lambda radar_data: RadarSensor._Radar_callback(weak_self, radar_data))
    def get_edata(self):
        return self.edata
    def get_data(self):
        return {"RADAR":self.data}
    def destroy(self):
        self.sensor_actor_instance.destroy()
    @staticmethod
    def _Radar_callback(weak_self, radar_data):
        self = weak_self()
        if not self:
            return
        # To get a numpy [[vel, altitude, azimuth, depth],...[,,,]]:
        points = np.frombuffer(radar_data.raw_data, dtype=np.dtype('f4'))
        points = np.reshape(points, (len(radar_data), 4))
        self.edata=points.tolist()
        for i in range(len(radar_data)):
            self.data["points"].update( {str(i) : {"velocity" : str(points[i][0]),
                                                   "altitude" : str(points[i][1]),
                                                   "azimuth"  : str(points[i][2]),
                                                   "depth"    : str(points[i][3])
                                                   }
                                         })


class Vehicle:
    def __init__(self, id, vehicle_actor_instance):
        self.id = id
        self.vehicle_actor_instance = vehicle_actor_instance
        self.initial_transform = None
        self.sensor_instances = []
        self.sensor_instances.append(GnssSensor(vehicle_actor_instance))
        self.sensor_instances.append(IMUSensor(vehicle_actor_instance))
        self.sensor_instances.append(RadarSensor(vehicle_actor_instance))
        self.edata = []
        self.data = {"RADAR": {},
                     "IMU": {},
                     "GNSS": {},
                     "Velocity": {},
                     }
    def get_edata(self):
        self.edata.clear()
        for sensor in self.sensor_instances:
            self.edata.append(sensor.get_edata())
        v = self.vehicle_actor_instance.get_velocity()
        self.edata.append([ v.x, v.y, v.z ])
        return self.edata
    def get_data(self):
        for sensor in self.sensor_instances:
            self.data.update(sensor.get_data())
        v = self.vehicle_actor_instance.get_velocity()
        self.data["Velocity"].update({"x" : v.x, "y" : v.y, "z" : v.z})
        return self.data
    def destroy(self):
        for sensor in self.sensor_instances:
            sensor.destroy()
        self.vehicle_actor_instance.destroy()
    def apply_control(self, control):
        self.vehicle_actor_instance.apply_control(carla.VehicleControl(throttle=control))

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

# ==============================================================================

def parseArguments():
    argparser = argparse.ArgumentParser(
        description='CARLA Platoon')
    argparser.add_argument(
        '--host',
        metavar='H',
        default='127.0.0.1',
        help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port to listen to (default: 2000)')
    argparser.add_argument(
        '--count',
        default=2,
        type=int,
        help='Number of vehicles (including leader) (default: 2)')
    argparser.add_argument(
        '--spacing',
        default=15,
        type= int,
        help='Spacings between vehicles at spawn time (default: 15)')
    argparser.add_argument(
        '--timeout',
        default=10.0,
        type=int,
        help='Time to wait for the server before quitting (default: 10.0 seconds)')
    argparser.add_argument(
        '--reload-map',
        default=0,
        type=bool,
        help='Reload the map (default: False)')
    argparser.add_argument(
        '--filter',
        metavar='PATTERN',
        default='vehicle.tesla.cybertruck',
        help='actor filter (default: "vehicle.tesla.cybertruck")')
    argparser.add_argument(
        '--sync',
        action='store_true',
        help='Activate synchronous mode execution')
    args = argparser.parse_args()
    # This code looks ugly. Fix those 2 exception handlings.
    try:
        if args.spacing <0:
            raise Exception("spacing negative number")
    except Exception as e:
        print(str(e))
        exit()
    try:
        if args.timeout<0:
            raise Exception("timeout negative number")
    except Exception as e:
        print(str(e))
        exit()
    return args

def main():
    args = parseArguments()

    # Carla initialization
    client = carla.Client(args.host, args.port)
    client.set_timeout(args.timeout)
    sim_world = World(client, args)

    try:
        arg_bp = sim_world.get_actor_blueprints(args.filter)
        sim_world.spawn_platoon(arg_bp)

        # ======================================================================
        # -- This is something that is still being experimented with. The Idea
        # -- of world snapshot is not yet completely understood.
        # ======================================================================
        #print(sim_world.world.get_snapshot().find(sim_world.leader_vehicle.sensor_instances[0].sensor_actor_instance.id).get_transform())
        # ======================================================================



        # ======================================================================
        # -- This is the desired end code, this is not fully implemented yet.
        # ======================================================================
        # A command line option to enable synchronous mode.
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

                    somedata = { 
                                "xl" : str(xl),
                                "yl" : str(yl),
                                "xf" : str(xf),
                                "yf" : str(yf),
                                "vl" : str(vl),
                                "vf" : str(vf)
                            }
                    control = [0.5,0]
                    # function to calculate the control
                    def get_control(xl,yl,xf,yf,vl,vf):
                        kp = 0.025
                        c = -2.0 # Desired spacing
                        return max(0,min(1,kp * (math.sqrt(((xf-xl)**2)+((yf-yl)**2)) + c)))

                    control[1] = get_control(xl,yl,xf,yf,vl,vf)

                    print("leader  :",xl,yl)
                    print("follower:",xf,yf)
                    print("speeds  :",vf,vl)
                    print(math.sqrt(((xf-xl)**2)+((yf-yl)**2)))
                    print(control)
                    sim_world.apply_control(control)

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
