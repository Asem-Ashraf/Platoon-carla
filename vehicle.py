#!/usr/bin/env python3.7

import carla
import sensors


class Vehicle:

    def __init__(self, id, vehicle_actor_instance):
        self.id = id
        self.carla_instance = vehicle_actor_instance
        self.sensor_instances = [
            sensors.GnssSensor(vehicle_actor_instance),
            sensors.IMUSensor(vehicle_actor_instance),
            sensors.RadarSensor(vehicle_actor_instance)
        ]
        self.edata = []
        self.data = {
            "RADAR": {},
            "IMU": {},
            "GNSS": {},
            "Velocity": {},
        }

    def get_edata(self):
        self.edata.clear()
        for sensor in self.sensor_instances:
            self.edata.append(sensor.get_edata())
        velocity_vector = self.carla_instance.get_velocity()
        self.edata.append(
            [velocity_vector.x, velocity_vector.y, velocity_vector.z])
        return self.edata

    def get_data(self):
        for sensor in self.sensor_instances:
            self.data.update(sensor.get_data())
        velocity_vector = self.carla_instance.get_velocity()
        self.data["Velocity"].update({
            "x": velocity_vector.x,
            "y": velocity_vector.y,
            "z": velocity_vector.z
        })
        return self.data

    def destroy(self):
        for sensor in self.sensor_instances:
            sensor.destroy()
        self.carla_instance.destroy()

    def apply_control(self, control):
        self.carla_instance.apply_control(control)
