import Trajectory.Fusion.fusion as fu
import numpy as np


class ReferenceTrajectory():

    def __init__(self, N = 100, ID = 1):
        # An empty list with the reference trajectory.
        # this is a list of a lists of 6 floats
        # [[f,f,f,f,f,f],[f,f,f,f,f,f],[f,f,f,f,f,f],...]
        self.refs = []

        # The vehicle ID. 1 is the first follower after the leader and so on.
        # This is used in calculating which n points of the reference
        # trajectory will be given to the MPC controller
        self.ID = ID

        # The number of points in a trajectory. This is set in the simulation script so there is a one source of truth.
        self.N = N

        # Build the empty list with the received leader states.
        # At least (N * ID) -1 states needs to be received to form a vehicle's
        # trajectory.
        for i in range(self.N * self.ID - 1):
            self.refs.append(fu.getLeaderStates())

        # The first element of this list of references is always the current
        # vehicle states.
        self.refs.insert(0, fu.getMyCurrentStates())

        # The last element is not used by the MPC controller, but it is used to
        # calculate the distance between two consecutive vehicles for
        # safe-distance keeping.
        self.refs.append(fu.getFrontVehicleStates(self.ID))

    def updateTrjaectory(self, shift=1):
        for _ in range(shift):
            # Remove the vehicle's old states as it should have been passed by now.
            self.refs.pop(0)

            # The first element should be the vehicle's current states, so it
            # overwrites an old leader states element that is not needed anymore.
            self.refs[0] = fu.getMyCurrentStates()

            # Something that I do that I will explain later.
            difference = self.refs[0][2] - self.refs[1][2]
            floorDifference = round(abs(difference / (2 * np.pi)))
            if (difference) < -np.pi:
                self.refs[0][2] += 2 * np.pi * floorDifference
            elif (difference) > np.pi:
                self.refs[0][2] -= 2 * np.pi * floorDifference

            # Replace the old front vehicle states element with the latest leader states.
            self.refs[-1] = fu.getLeaderStates()

            # Something that I do that I will explain later.
            differ = self.refs[-2][2] - self.refs[-1][2]
            floorDifference = round(abs(differ / (2 * np.pi)))
            if (differ) > np.pi:
                self.refs[-1][2] += 2 * np.pi * floorDifference
            elif (differ) < -np.pi:
                self.refs[-1][2] -= 2 * np.pi * floorDifference

            # Then, append the front vehicle states to the end.
            self.refs.append(fu.getFrontVehicleStates(self.ID))
