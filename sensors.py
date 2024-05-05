#!/usr/bin/env python3.7

import weakref
import math
import numpy as np
import carla

# ==============================================================================
# -- GnssSensor ----------------------------------------------------------------
# ==============================================================================
class GnssSensor(object):
    def __init__(self, parent_actor):
        self._parent_actor_instance = parent_actor

        # Readings (Efficient)
        self.edata = []

        self.data = {"lat": "", "lon": ""}

        world = self._parent_actor_instance.get_world()

        bp = world.get_blueprint_library().find('sensor.other.gnss')

        self.instance = world.spawn_actor(bp, carla.Transform(carla.Location(x=1.0, z=2.8)), attach_to=parent_actor)
        # We need to pass the lambda a weak reference to self to avoid circular reference.
        weak_self = weakref.ref(self)
        self.instance.listen(lambda event: GnssSensor._on_gnss_event(weak_self, event))
    def get_edata(self):
        return self.edata;
    def get_data(self):
        return {"GNSS":self.data}
    def destroy(self):
        self.instance.destroy()
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
        self._parent_actor_instance = parent_actor

        # Readings (Efficient)
        self.edata = []
        self.data = {}
        world = self._parent_actor_instance.get_world()
        bp = world.get_blueprint_library().find('sensor.other.imu')
        self.instance = world.spawn_actor( bp, carla.Transform(), attach_to=parent_actor)
        # We need to pass the lambda a weak reference to self to avoid circular
        # reference.
        weak_self = weakref.ref(self)
        self.instance.listen( lambda sensor_data: IMUSensor._IMU_callback(weak_self, sensor_data))
    def get_edata(self):
        return self.edata
    def get_data(self):
        return {"IMU":self.data}
    def destroy(self):
        self.instance.destroy()
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
        self.instance = world.spawn_actor(
            bp,
            carla.Transform(
                carla.Location(x=bound_x + 0.05, z=bound_z+0.05),
                carla.Rotation(pitch=5)),
            attach_to=parent_actor)
        # We need a weak reference to self to avoid circular reference.
        weak_self = weakref.ref(self)
        self.instance.listen(
            lambda radar_data: RadarSensor._Radar_callback(weak_self, radar_data))
    def get_edata(self):
        return self.edata
    def get_data(self):
        return {"RADAR":self.data}
    def destroy(self):
        self.instance.destroy()
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
