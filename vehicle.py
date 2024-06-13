#!/usr/bin/env python3.7

import fusion as fu

class Vehicle:

    def __init__(self, id, vehicle_actor_instance):
        self.id = id
        self.carla_instance = vehicle_actor_instance

    def get_data(self):
        return fu.get_current_state(self.carla_instance)

    def destroy(self):
        self.carla_instance.destroy()

    def apply_control(self, control):
        self.carla_instance.apply_control(control)
