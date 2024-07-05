from MPC.control import MPC

from Trajectory.trajectory import ReferenceTrajectory

from Helpers.arguments import parseArguments

import Communication.ourmqtt as CARLA

# import time


def main():
    args = parseArguments()
    ourclient = CARLA.initComms(args.host, args.port, args.id)
    reftraj = ReferenceTrajectory(ID=args.id, N=args.N)
    ourclient.before = 4
    controller = MPC(args.N, args.T)
    while True:
        # start = time.time()
        CARLA.sendControls(controller.get_control(reftraj.refs))
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
