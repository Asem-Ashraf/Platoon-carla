# Truck Platooning
These scripts are made with the purpose of creating a truck platooning
environment in CARLA simulator where sensor fusion, trajectory planning and
control algorithms are used.  
A proof-of-concept of V2V platooning using embedded linux on a raspberry pi is
the end goal.

### ```arguments.py``` Command-line arguments
- parse command line arguments into a dictionary using the `argparse` module.
- TODO: convert to a config file



### ```sensors.py``` Sensor classes
- contains classes for IMU, Radar, and GNSS sensors with their APIs for data
collection.
- APIs
    - `get_edata()`:  gets sensor data in a list (compact). Refer to `reading.example`.
    - `get_data()`:  gets sensor data in a dictionary (readable). Refer to `reading.example`.
    - `destroy()`:  Destroys a sensor actor.
### `fusion.py` States at a point
- Gets the states of a vehicle at a specific point in time.
- Points are used in making a trajectory.
- A vehicle can then follow that trajectory
- Currently, all the states are generated from CARLA's APIs.
- **TODO**
    - Explore real sensor fusion.
### ```vehicle.py``` Vehicle class
- Wrapper class for a vehicle.
- APIs collects the data of all sensors.
    - `get_edata()`: gets all sensors data in a list (compact). Refer to `reading.example`.
    - `get_data()`: gets all sensors data in a dictionary (readable). Refer to `reading.example`.
    - `destroy()`: destroys all sensors then the vehicle.
    - `apply_control()`: applies controls to the vehicle.


### ```control.py``` MPC controller
- Cost function definition.
- Bounds for each state.
- Solver for the NLP problem.
- **TODO**
    - Know more about solvers to know which solver suits our purposes. IPopt,
    qpOASES, ...
### ```equations.py``` Vehicle model
- Equations of motion implementation using casadi library.
- Gets information about the vehicle through CARLA's APIs.
- **TODO**
    - Revise the tyre model.
    - fix wrong predictions at low speeds.
    - translate the heading angle parameter into a relative error.


### ```world.py``` World wrapper
- Initializes CARLA's environment by loading the correct map.
- APIs:
    - `spawn_platon()`: Spawns the platoon on a cherry-picked road.
    - `destroy_platon()`: destroys all the vehicles in the platoon.
    - `get_platoon_data()`: gets the followers' sensors data in a dictionary (readable). Refer to `reading.example`.
    - `get_platoon_edata()`: gets the followers' sensors data in a list (compact). Refer to `reading.example`.
    - `apply_platoon_control()`: given a list of the control actions for each
    follower, applies the controls.
