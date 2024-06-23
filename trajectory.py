import fusion as fu


class ReferenceTrajectory():

    def __init__(self, N):
        self.refs = []
        self.ID = fu.getMyPlatoonPosition()
        for i in range(N * self.ID - 1):
            self.refs.append(fu.getLeaderStates())
        self.refs.insert(0, fu.getMyCurrentStates())
        self.refs.append(fu.getFrontVehicleStates(self.ID))
        return self, self.refs

    def getUpdatedTrjaectory(self, shift=1):
        for _ in range(shift):
            self.refs.pop(0)
            self.refs[0] = fu.getCurrentStates()
            self.refs[-1] = fu.getLeaderStates()
            self.refs.append(fu.getFrontVehicleStates(self.ID))
        return self.refs
