import os
from aimotion_f1tenth_utils.F1Client import F1Client
from aimotion_f1tenth_utils.utils import CONTROLLER_MODE
from aimotion_f1tenth_utils.Trajectory import Trajectory
import matplotlib as mpl
import os
import matplotlib.pyplot as plt

mpl.rcParams["text.usetex"] = False 

# Design / load the trajectory
traj_ID = "traj_1"
traj = Trajectory(trajectory_ID=traj_ID) # create the trajetory object
traj.load(os.path.dirname(__file__)+"/"+traj_ID+".traj")
x_r , y_r = traj.get_trajectory()

# connect to the vehicle
car_1 = F1Client("192.168.2.62", 8069)

# select the controller
car_1.select_controller("GP_LPV_LQR")
car_1.reset_state_logger()

# execute_trajectory
car_1.execute_trajectory(trajectory=traj)

# block the script until the execution is finished
car_1.wait_while_running()

car_1.GP_train()