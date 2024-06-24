from control import MPC
from trajectory import ReferenceTrajectory
import ourmqtt
# import time


def main():
    reftraj = ReferenceTrajectory()
    controller = MPC(reftraj.N, reftraj.Ts)
    while True:
        # start = time.time()
        ourmqtt.sendControls(controller.get_control(reftraj.refs))
        # end = time.time()
        # print(end-start)
        reftraj.getUpdatedTrjaectory()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('\nCancelled by the user. Bye!')
    except RuntimeError as e:
        print(e)
