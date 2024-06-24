from MPC.control import MPC
from Trajectory.trajectory import ReferenceTrajectory
import Communication.ourmqtt as carla
# import time


def main():
    reftraj = ReferenceTrajectory()
    controller = MPC(reftraj.N, reftraj.Ts)
    while True:
        # start = time.time()
        carla.sendControls(controller.get_control(reftraj.refs))
        # end = time.time()
        # print(end-start)
        reftraj.updateTrjaectory()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('\nCancelled by the user. Bye!')
    except RuntimeError as e:
        print(e)
