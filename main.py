from equations import VehicleModel
from control import MPC
from trajectory import ReferenceTrajectory
import ourmqtt
# import time

N = 8
Ts = 0.05


def main():
    controller = MPC(VehicleModel(), N, Ts)
    reftraj, referenceTrajectory = ReferenceTrajectory(N)
    while True:
        # start = time.time()
        ourmqtt.sendControls(controller.get_control(referenceTrajectory))
        # end = time.time()
        # print(end-start)
        referenceTrajectory = reftraj.getUpdatedTrjaectory()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('\nCancelled by the user. Bye!')
    except RuntimeError as e:
        print(e)
