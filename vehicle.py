#!/usr/bin/env python3.7

import sensors
import carla

class Vehicle:
    def __init__(self, id, vehicle_actor_instance):
        self.id = id
        self.vehicle_actor_instance = vehicle_actor_instance
        self.initial_transform = None
        self.sensor_instances = []
        self.sensor_instances.append(sensors.GnssSensor(vehicle_actor_instance))
        self.sensor_instances.append(sensors.IMUSensor(vehicle_actor_instance))
        self.sensor_instances.append(sensors.RadarSensor(vehicle_actor_instance))
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
