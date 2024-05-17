import fusion as fu

def get_init_trajectory(N):
    refs = []
    refs.append(fu.getCurrentStates())
    for i in range(N):
        refs.append(fu.getNextPoint())
    return refs

def shift_trajectory(trajectory, shift=1):
    for _ in range(shift):
        trajectory.pop(0)
        trajectory[0] = fu.getCurrentStates()
        trajectory.append(fu.getNextPoint())
    return trajectory
