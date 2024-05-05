#!/usr/bin/env python3.7

import sensors
import carla


class Vehicle:

    def __init__(self, id, vehicle_actor_instance):
        self.id = id
        self.instance = vehicle_actor_instance
        self.initial_transform = None
        self.sensors = []
        self.sensors.append( sensors.GnssSensor(vehicle_actor_instance))
        self.sensors.append( sensors.IMUSensor(vehicle_actor_instance))
        self.sensors.append( sensors.RadarSensor(vehicle_actor_instance))
        self.edata = []
        self.data = {
            "RADAR": {},
            "IMU": {},
            "GNSS": {},
            "Velocity": {},
        }

    def get_edata(self):
        self.edata.clear()
        for sensor in self.sensors:
            self.edata.append(sensor.get_edata())
        # velocity in the global frame and not the body frame of the vehicle
        v = self.instance.get_velocity()
        self.edata.append([v.x, v.y, v.z])
        return self.edata

    def get_data(self):
        for sensor in self.sensors:
            self.data.update(sensor.get_data())
        # velocity in the global frame and not the body frame of the vehicle
        v = self.instance.get_velocity()
        self.data["Velocity"].update({"x": v.x, "y": v.y, "z": v.z})
        return self.data

    def destroy(self):
        for sensor in self.sensors:
            sensor.destroy()
        self.instance.destroy()

    def apply_control(self, throttle=0,steer=0,brake=0):
        self.instance.apply_control(
            carla.VehicleControl(throttle=throttle,steer=steer,brake=brake))
