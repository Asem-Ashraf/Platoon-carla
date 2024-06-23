import equations as eq
import control as con
import trajectory as tj
import ourmqtt
import time

references = []
N = 8
Ts = 0.05 # 50ms

def main():
    vehicle = con.MPC(eq.VehicleModel(),N,Ts)
    trajectory = tj.get_init_trajectory(N)
    while True:
        start = time.time()
        ourmqtt.sendControls(vehicle.get_control(tj.shift_trajectory(trajectory)))
        end = time.time()
        print(end-start)

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('\nCancelled by the user. Bye!')
    except RuntimeError as e:
        print(e)
