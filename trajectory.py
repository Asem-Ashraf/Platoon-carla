#!/usr/bin/env python3.7

import numpy as np

class Trajectory():
    def __init__(self,startwaypoint,defaultSpacing = 2):
        self.defaultSpacing = defaultSpacing
        self.startwaypoint  = startwaypoint

    def get_trajectory(self, spacing, n=5):
        '''
            spacing:    distance between each waypoint in meters
            n:          number of waypoints in the trajectory

            returns a list of (n) waypoints with (spacing) between them in the
            forward direction of the closest lane to (startpoint). Each waypoint
            has the transform attributes which contains its location and
            supposed rotation of the vehicle at that waypoint.
        '''
        # Build the trajectory from scratch
        trajectory = [self.startwaypoint]
        for i in range(n):
            try:
                nextwaypoint = trajectory[-1].next(spacing)[0]
            except:
                nextwaypoint = trajectory[-1].next(self.defaultSpacing)[0]
            if nextwaypoint is None:
                print('Error: No more waypoints. Exiting.')
                exit()
            trajectory.append(nextwaypoint)
        return trajectory

    def shift_trajectory(self, trajectory, spacing, shift=1):
        for _ in range(shift):
            trajectory.pop(0)
            try:
                trajectory.append(trajectory[-1].next(spacing)[0])
            except:
                trajectory.append(trajectory[-1].next(self.defaultSpacing)[0])
        return trajectory

    def simplify_trajectory(self, trajectory):
        '''
            trajectory: list of waypoints
            returns the reference values in the form 
            [
                [x1, y1, yaw1, vx1, vy1, omega1],
                [x2, y2, yaw2, vx2, vy2, omega2],
                ...
            ]
        '''
        simplified = []
        for i in range(len(trajectory)):
            simplified.append(trajectory[i].transform.location.x)
            simplified.append(trajectory[i].transform.location.y)
            psi = trajectory[i].transform.rotation.yaw%360
            if psi >= 180.0:
                psi = psi - 360.0
            if psi < -180.0:
                psi = psi + 360.0
            simplified.append(np.deg2rad(psi))
            vx = 5 #m/s
            vy = 0
            omega = 0
            simplified.append(vx)
            simplified.append(vy)
            simplified.append(np.deg2rad(omega))
        return simplified
