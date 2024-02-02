#!/usr/bin/env python3.7

# ==============================================================================
# -- imports -------------------------------------------------------------------
# ==============================================================================


import carla
import paho.mqtt.client as mqtt
import weakref
import math
import numpy as np
import argparse
import time
import json

# ==============================================================================
# -- PahoMQTT ------------------------------------------------------------------
# ==============================================================================

# @brief MQTT client class
# @details This class is used to connect to an MQTT broker and publish/subscribe to topics
class MqttClient(object):
    # @brief Initialize the MQTT client
    # @param broker_address The IP address of the MQTT broker
    # @param broker_port The port of the MQTT broker
    # @return None
    def __init__(self, broker_address, broker_port):
        self.client = mqtt.Client()
        self.client.connect(broker_address, broker_port, 60)
        self.client.loop_start()

    # @brief Publish a message to a topic
    # @param topic The topic to publish to
    # @param message The message to publish
    # @return None
    def publish(self, topic, message):
        self.client.publish(topic, message)

    # @brief Subscribe to a topic
    # @param topic The topic to subscribe to
    # @return None
    def subscribe(self, topic):
        self.client.subscribe(topic)

    # @brief Unsubscribe from a topic
    # @param topic The topic to unsubscribe from
    # @return None
    def unsubscribe(self, topic):
        self.client.unsubscribe(topic)

    # @brief Set the callback for when a message is received
    # @param callback The callback function
    # @return None
    def set_on_message_callback(self, callback):
        self.client.on_message = callback

    # @brief Set the callback for when a message is published
    # @param callback The callback function
    # @return None
    def set_on_publish_callback(self, callback):
        self.client.on_publish = callback

    # @brief Disconnect from the broker
    # @return None
    def disconnect(self):
        self.client.disconnect()


# ==============================================================================
# -- GnssSensor ----------------------------------------------------------------
# ==============================================================================


class GnssSensor(object):
    def __init__(self, parent_actor):
        self.sensor_actor_instance = None
        self._parent_actor_instance = parent_actor

        # Readings (Efficient)
        # self.lat = 0.0
        # self.lon = 0.0

        self.data = {"lat": 0.0,
                     "lon": 0.0}

        world = self._parent_actor_instance.get_world()
        bp = world.get_blueprint_library().find('sensor.other.gnss')
        self.sensor_actor_instance = world.spawn_actor(bp,
                                                       carla.Transform(carla.Location(x=1.0, z=2.8)),
                                                       attach_to=parent_actor)
        # We need to pass the lambda a weak reference to self to avoid circular
        # reference.
        weak_self = weakref.ref(self)
        self.sensor_actor_instance.listen(lambda event: GnssSensor._on_gnss_event(weak_self, event))

    def get_data(self):
        # efficient
        # return [self.lat, self.lon]
        # for the json file
        return {"GNSS":self.data}


    def destroy(self):
        self.sensor_actor_instance.destroy()

    @staticmethod
    def _on_gnss_event(weak_self, event):
        self = weak_self()
        if not self:
            return
        self.data.update({"lat": str(event.latitude),
                          "lon": str(event.longitude)})

        # # Readings (Efficient)
        # self.lat = event.latitude
        # self.lon = event.longitude


# ==============================================================================
# -- IMUSensor -----------------------------------------------------------------
# ==============================================================================


class IMUSensor(object):
    def __init__(self, parent_actor):
        self.sensor_actor_instance = None
        self._parent_actor_instance = parent_actor

        # # Readings (Efficient)
        # self.accelerometer = (0.0, 0.0, 0.0)
        # self.gyroscope = (0.0, 0.0, 0.0)
        # self.compass = 0.0

        self.data = {}
        world = self._parent_actor_instance.get_world()
        bp = world.get_blueprint_library().find('sensor.other.imu')
        self.sensor_actor_instance = world.spawn_actor( bp, carla.Transform(), attach_to=parent_actor)
        # We need to pass the lambda a weak reference to self to avoid circular
        # reference.
        weak_self = weakref.ref(self)
        self.sensor_actor_instance.listen( lambda sensor_data: IMUSensor._IMU_callback(weak_self, sensor_data))

    def get_data(self):
        return {"IMU":self.data}

    def destroy(self):
        self.sensor_actor_instance.destroy()

    @staticmethod
    def _IMU_callback(weak_self, sensor_data):
        self = weak_self()
        if not self:
            return
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

        # # Readings (Efficient)
        # self.accelerostr(meter = sensor_data.accelerometer
        # self.gyroscope = sensor_data.gyroscope
        # self.compass = math.degrees(sensor_data.compass)

        # # Readings (Ranged from -99.9 to 99.9)
        # ranging the data from a min of -99.9 to a max of 99.9
        # limits = (-99.9, 99.9)
        # self.accelerometer = (
        #     max(limits[0], min(limits[1], sensor_data.accelerometer.x)),
        #     max(limits[0], min(limits[1], sensor_data.accelerometer.y)),
        #     max(limits[0], min(limits[1], sensor_data.accelerometer.z)))
        # self.gyroscope = (
        #     max(limits[0], min(limits[1], math.degrees(sensor_data.gyroscope.x))),
        #     max(limits[0], min(limits[1], math.degrees(sensor_data.gyroscope.y))),
        #     max(limits[0], min(limits[1], math.degrees(sensor_data.gyroscope.z))))
        # self.compass = math.degrees(sensor_data.compass)


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

        # Readings
        self.data = {"points": {
            "0": {"velocity": 0.0, "altitude": 0.0, "azimuth": 0.0, "depth": 0.0}
            }}

        # # Readings (Efficient)
        # self.points = []

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
        for i in range(len(radar_data)):
            self.data["points"].update( {str(i) : {"velocity" : str(points[i][0]),
                                                   "altitude" : str(points[i][1]),
                                                   "azimuth"  : str(points[i][2]),
                                                   "depth"    : str(points[i][3])
                                                   }
                                         })
            # # Readings (Efficient)
            # self.points = points


class Vehicle:
    def __init__(self, id, vehicle_actor_instance):
        self.id = id
        self.vehicle_actor_instance = vehicle_actor_instance
        self.initial_transform = None
        self.sensor_instances = []
        self.sensor_instances.append(GnssSensor(vehicle_actor_instance))
        self.sensor_instances.append(IMUSensor(vehicle_actor_instance))
        self.sensor_instances.append(RadarSensor(vehicle_actor_instance))
        self.data = {"RADAR": {},
                     "IMU": {},
                     "GNSS": {},
                     "Velocity": "",
                     }

    def get_data(self):
        for sensor in self.sensor_instances:
            self.data.update(sensor.get_data())
        self.data.update({"Velocity" : self.vehicle_actor_instance.get_velocity()})
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

    def get_data(self):
        data = {}
        # data.update({self.leader_vehicle.id : self.leader_vehicle.get_data()})
        for vehicle in self.follower_vehicles:
            data.update({vehicle.id : vehicle.get_data()})
        return data

    def apply_control(self, control):
        self.leader_vehicle.apply_control(control[0])
        for i in range(len(self.follower_vehicles)):
            self.follower_vehicles[i].apply_control(control[i+1])

# ==============================================================================

# MQTT broker details
broker_address = "localhost"
broker_port = 1883

# Define topics
publish_topic = "carla/sensors"
subscribe_topic = "carla/actions"

control = [0.5,0]

# Callback when a message is published
def on_publish(client, userdata, mid):
    print(f"Message {mid} published to topic {publish_topic}")

# Callback when a message is received from the subscribed topic
def on_message(client, userdata, msg):
    print(f"Received message: {msg.payload.decode()} from topic {msg.topic}")
    # try :
    control[1] = float(json.loads(msg.payload.decode())['af'])
    # except:
    #     pass


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

    # MQTT initialization
    mqtt_client = MqttClient(broker_address, broker_port)
    mqtt_client.set_on_publish_callback(on_publish)
    mqtt_client.set_on_message_callback(on_message)
    mqtt_client.subscribe(subscribe_topic)

    # Carla initialization
    client = carla.Client(args.host, args.port)
    client.set_timeout(args.timeout)
    sim_world = World(client, args)

    try:
        arg_bp = sim_world.get_actor_blueprints(args.filter)
        sim_world.spawn_platoon(arg_bp)
        # time.sleep(10)

        # ======================================================================
        # -- This is something that is still being experimented with. The Idea
        # -- of world snapshot is not yet completely understood.
        # ======================================================================
        #print(sim_world.world.get_snapshot().find(sim_world.leader_vehicle.sensor_instances[0].sensor_actor_instance.id).get_transform())
        # ======================================================================

        # A command line option to enable synchronous mode.
        if args.sync:
            sim_world.settings.synchronous_mode = True
            # This delta seconds could also be given in the command-line.
            sim_world.settings.fixed_delta_seconds = 0.05
            sim_world.world.apply_settings(sim_world.settings)

        # ======================================================================
        # -- This is the desired end code, this is not fully implemented yet.
        # ======================================================================
        if args.sync:
            for i in range(100):
                # In this mode, a delay is not needed as the timing between
                # each tick can be set. This is the path to be taken for the
                # most accurate sensor reading.
                sim_world.world.tick()

                # mqtt_client.publish(publish_topic, json.dumps(sim_world.get_data())) 
                # How to make an explicit function call to get_control() so 
                # that I can apply it here?
                # sim_world.apply_control(get_control())
        else:
            while True:
                    data = sim_world.get_data()
                    temp = sim_world.leader_vehicle.get_data()['Velocity']
                    vl = math.sqrt(((temp.x)**2)+((temp.y)**2))
                    temp = data[1]['Velocity']
                    vf = math.sqrt(((temp.x)**2)+((temp.y)**2))
                    xl = 10000*float(sim_world.leader_vehicle.get_data()['GNSS']['lat'])
                    yl = 10000*float(sim_world.leader_vehicle.get_data()['GNSS']['lon'])
                    xf = 10000*float(data[1]['GNSS']['lat'])
                    yf = 10000*float(data[1]['GNSS']['lon'])
                    somedata = { 
                                "xl" : str(xl),
                                "yl" : str(yl),
                                "xf" : str(xf),
                                "yf" : str(yf),
                                "vl" : str(vl),
                                "vf" : str(vf)
                            }
                    mqtt_client.publish(publish_topic, json.dumps(somedata)) 
                    print("leader  :",xl,yl)
                    print("follower:",xf,yf)
                    print("speeds  :",vf,vl)
                    print(math.sqrt(((xf-xl)**2)+((yf-yl)**2)))
                    print(control)
                    sim_world.apply_control(control)

                # A delay so that the environment changes a little bit before
                # sending the next sensor readings. Otherwise, this script
                # flood the raspberry pi with sensors whenever it can.
                    time.sleep(0.05)
        # ======================================================================
    finally:
        # Disconnect when the script is interrupted
        mqtt_client.disconnect()
        sim_world.destroy_platoon()
        print('==================')
        print('Platoon Destroyed.')
        print('==================')

if __name__ == '__main__':
    main()
                    # function to calculate the control
                    # def get_control(xl,yl,xf,yf,vl,vf):
                    #     # This is a dummy control function. This is where the
                    #     # control algorithm should be implemented.
                    #     kp = 0.025
                    #     kd = 0.01
                    #     c = -15
                    #     return math.tanh(kp * (math.sqrt(((xf-xl)**2)+((yf-yl)**2))) + c - kd*(vl-vf))+1
                    # control[1] = get_control(xl,yl,xf,yf,vl,vf)
                    #
