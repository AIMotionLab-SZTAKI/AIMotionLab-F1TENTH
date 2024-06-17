from aimotion_f1tenth_utils.F1Client import F1Client
from aimotion_f1tenth_utils.Trajectory import Trajectory
from aimotion_f1tenth_utils.utils import CONTROLLER_MODE
import time
import numpy as np
import yaml
import os


parent_dir = os.path.dirname(os.path.dirname(__file__))

file_name = os.path.join(parent_dir, "configs/JoeBush1.yaml")

with open(file_name) as f:
    full_params = yaml.full_load(f)

    MPCC_params = full_params["parameter_server"]["ros__parameters"]["controllers"]["MPCC"]

car_1 = F1Client("192.168.2.62", 8069)
print(f"Connected to {car_1.car_ID}")

# select the controller

#car_1.select_controller("MPCC")


print(car_1.get_MPCC_params())

#print(car_1.set_MPCC_params(MPCC_params))


input("Press enter to start trajectory execution")

parent_dir = os.path.dirname(os.path.dirname(__file__))

traj_name = "paperclip.traj"
traj_file = os.path.join(parent_dir, "scripts", traj_name)
traj = Trajectory("traj_1")

traj.load(traj_file)

traj.plot_trajectory()

car_1.set_mode(mode= CONTROLLER_MODE.IDLE)
car_1.execute_trajectory(traj)


if car_1.get_mode() == CONTROLLER_MODE.RUNNING:
    #print(car_1.get_MPCC_horizon())
    time.sleep(1/10)
