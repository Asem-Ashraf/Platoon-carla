#!/usr/bin/env python3.7

import carla

def get_trajectory(startwaypoint, spacing=15, n=5):
    '''
        startpoint: type carla.transform
        spacing: in meters
        n: number of waypoints to generate,
        returns a list of (n) waypoints with (spacing) between them in the
        forward direction of the closest lane to (startpoint). Each waypoint has
        the transform attributes location and rotation.
    '''
    # Build the trajectory from scratch
    trajectory = [startwaypoint]
    while n >= 0:
        nextwaypoint = trajectory[-1].next(spacing)[0]
        if nextwaypoint is None:
            print('Error: No more waypoints. Exiting.')
            exit()
        trajectory.append(nextwaypoint)
        n -= 1
    return trajectory

def shift_trajectory(trajectory, shift=1, spacing=10.0):
        trajectory.pop(0)
        trajectory.append(trajectory[-1].next(spacing)[0])
        return trajectory

def simplify_trajectory(trajectory):
    '''
        trajectory: list of waypoints
        # returns the reference vaules in the form 
        # [[x1, y1, yaw1], [x2, y2, yaw2], ...]
    '''
    simplified = []
    x = []
    for i in range(len(trajectory)):
        # simplified.append(trajectory[i].location.x)
        simplified.append(trajectory[i].transform.rotation.yaw%360)
        if simplified[-1] >= 180.0:
            simplified[-1] = simplified[-1] - 360.0
        if simplified[-1] < -180.0:
            simplified[-1] = simplified[-1] + 360.0
        simplified.append(trajectory[i].transform.location.y)
        x.append(trajectory[i].transform.location.x)
    return simplified,x
