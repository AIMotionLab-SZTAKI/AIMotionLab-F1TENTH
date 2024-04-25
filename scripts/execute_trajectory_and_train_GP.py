import os
from aimotion_f1tenth_utils.F1Client import F1Client
from aimotion_f1tenth_utils.utils import CONTROLLER_MODE
from aimotion_f1tenth_utils.Trajectory import Trajectory
import matplotlib as mpl
import os
import matplotlib.pyplot as plt
import numpy as np


mpl.rcParams["text.usetex"] = False 

# Design / load the trajectory
train_velocities = [0.5, 1, 1.5]

train_trajectories = [Trajectory(f"training_v_{v}") for v in train_velocities]
t = np.linspace(0, 2*np.pi, 9)
a =1.5
b = 3
x = a * np.sin(t)
y = b * np.sin(2*t)
training_path=np.vstack((x,y)).T
for train_trajectory,v in zip(train_trajectories, train_velocities): 
    train_trajectory.build_from_points_const_speed(path_points=training_path,
                                                   path_smoothing=0.001, 
                                                   path_degree=5, 
                                                   const_speed=v)
    
# connect to the vehicle
car_1 = F1Client("192.168.2.62", 8069)

# select the controller
car_1.select_controller("GP_LPV_LQR")
car_1.reset_state_logger()

for traj in train_trajectories:
    print(f"Executing {traj.trajectory_ID}")
    # execute_trajectory
    car_1.execute_trajectory(trajectory=traj)

    # block the script until the execution is finished
    car_1.wait_while_running()

print("Exections finished training GPs")
training_data = car_1.GP_train()

lat_x = training_data[0][0]
lat_y = training_data[0][1]
long_x = training_data[0][0]
long_y = training_data[0][1]

# TODO: plot the training data: see ModularLPBLQR class

car_1.reset_controller()

# TODO:
# run a test on a test trajectory


