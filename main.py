import equations as eq
import control as con
from trajectory import shift_trajectory
import time

references = []
N = 8
Ts = 0.05 # 50ms

def main():
    vehicle = con.MPC(eq.VehicleModel(),N,Ts)
    while True:
        start = time.time()
        acc, steer = vehicle.get_control(shift_trajectory(references))
        # send the values
        end = time.time()
        delay = Ts-(end-start)
        if(delay>0):
            time.sleep(delay)

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('\nCancelled by the user. Bye!')
    except RuntimeError as e:
        print(e)
