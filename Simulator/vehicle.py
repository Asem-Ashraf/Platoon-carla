#!/usr/bin/env python3.7

import fusion as fu


class Vehicle:

    def __init__(self, id, vehicle_actor_instance):
        self.id = id
        self.carla_instance = vehicle_actor_instance

    def getData(self):
        return fu.get_current_state(self.carla_instance)

    def destroy(self):
        self.carla_instance.destroy()

    def applyControl(self, control):
        throttle = control[0]
        steer = control[1]
        brake = control[3]
        self.carla_instance.apply_control(throttle=throttle,
                                          steer=steer,
                                          brake=brake)
