#!/usr/bin/env python3.7

import numpy as np

def get_current_state(instance):
    '''
    X     - Global x position (meter)
    Y     - Global y position (meter)
    psi   - Heading angle (radians)
    Vx    - Longitudinal velocity (m/s)
    Vy    - Lateral velocity (m/s)
    omega - Angular velocity (rad/s)

    return: state - the state at the next time step
    '''
    transform = instance.get_transform()
    # position in Global coordinates
    X = transform.location.x
    Y = transform.location.y

    # yaw in degrees
    psi = transform.rotation.yaw % 360
    # Limit between ]-pi,pi]
    if psi > 180.0:
        psi = psi - 360.0
    if psi <= -180.0:
        psi = psi + 360.0
    psi = np.deg2rad(psi)

    # velocity in global coordinates
    v = instance.get_velocity()

    # velocities in vehicle coordinates
    Vx = v.x*np.cos(psi) + v.y*np.sin(psi)
    Vy = -v.x*np.sin(psi) + v.y*np.cos(psi)

    omega = np.deg2rad(instance.get_angular_velocity().z)

    next_state = [
        X,
        Y,
        psi,
        Vx,
        Vy,
        omega
    ]

    return np.array(next_state)
