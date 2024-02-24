#!/usr/bin/env python3.7

import numpy as np
import carla

class CarModel():
    def __init__(self, instance, x_dot=10.0):
        self.instance = instance
        physics_control = instance.get_physics_control()
        center_of_mass = physics_control.center_of_mass
        bounding_box = instance.bounding_box
        self.constants = {}
        self.constants['m'] = physics_control.mass
        self.constants['Iz'] = physics_control.moi
        self.constants['Ts'] = 1
        self.constants['lf'] = bounding_box.extent.x - center_of_mass.x
        self.constants['lr'] = bounding_box.extent.x + center_of_mass.x
        self.constants['Caf'] = (physics_control.wheels[0].lat_stiff_value+
                                 physics_control.wheels[1].lat_stiff_value) / 2
        self.constants['Car'] = (physics_control.wheels[2].lat_stiff_value+
                                 physics_control.wheels[3].lat_stiff_value) / 2
        m=self.constants['m']
        Iz=self.constants['Iz']
        Caf=self.constants['Caf']
        Car=self.constants['Car']
        lf=self.constants['lf']
        lr=self.constants['lr']

        # Get the state space matrices for the control
        A1=-(2*Caf+2*Car)/(m*x_dot)
        A2=-x_dot-(2*Caf*lf-2*Car*lr)/(m*x_dot)
        A3=-(2*lf*Caf-2*lr*Car)/(Iz*x_dot)
        A4=-(2*lf**2*Caf+2*lr**2*Car)/(Iz*x_dot)

        self.A=np.array([[A1, 0, A2, 0],
                    [0, 0, 1, 0],
                    [A3, 0, A4, 0],
                    [1, x_dot, 0, 0]
                   ])
        self.B=np.array([[2*Caf/m],
                    [0],
                    [2*lf*Caf/Iz],
                    [0]
                   ])
        self.C=np.array([[0, 1, 0, 0],
                    [0, 0, 0, 1]
                   ])
        self.D=0

    def get_discrete_state_space_matrices(self):
        Ts=self.constants['Ts']
        Ad = np.identity(np.size(self.A,1))+Ts*self.A
        Bd = Ts*self.B
        Cd = self.C
        Dd = self.D
        return Ad, Bd, Cd, Dd

    def get_next_state_carla(self):
        '''
        px - x position
        py - y position
        theta - heading
        v - velocity
        omega - angular velocity
        return: state - the state at the next time step
        '''
        transform = self.instance.get_transform()
        px = transform.location.x
        py = transform.location.y
        theta = transform.rotation.yaw%360
        if theta >= 180.0:
            theta = theta - 360.0
        if theta < -180.0:
            theta = theta + 360.0
        # vx = self.instance.get_velocity().x
        v = self.instance.get_velocity()
        vy = v.y
        v = np.sqrt(v.x**2 + v.y**2)
        omega = self.instance.get_angular_velocity().z

        next_state = [vy, theta, omega, py]

        return next_state, px,v,py,theta
