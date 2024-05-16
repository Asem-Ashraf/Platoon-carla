def shift_trajectory(trajectory, shift=1):
    for _ in range(shift):
        trajectory.pop(0)
        trajectory.append(getNextPoint())
    return trajectory
