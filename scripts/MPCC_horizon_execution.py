from aimotion_f1tenth_utils.F1Client import F1Client
from aimotion_f1tenth_utils.Trajectory import Trajectory
from aimotion_f1tenth_utils.utils import CONTROLLER_MODE
from trajectory_generators import null_infty, eight, null_paperclip, train8
from scipy.interpolate import splev
import time
import numpy as np
import yaml
import os
import argparse
from threading import Thread
from MPCC_plotter import MPCC_plotter
import matplotlib.pyplot as plt
import numpy as np





parent_dir = os.path.dirname(os.path.dirname(__file__))

traj_name = "paperclip.traj"
traj_file = os.path.join(parent_dir, "trajectories", traj_name)
traj = Trajectory("traj_1")

path, v = null_infty()
traj.build_from_waypoints(path, v, 0, 5)



parser = argparse.ArgumentParser(description='--horizon: set to 1 if plot is needed')
parser.add_argument("--horizon", type=int, default=0)

args = parser.parse_args()
plotter = MPCC_plotter()




def horizon_call_spin(car: F1Client):
    
    while True:
        if car.get_mode() == CONTROLLER_MODE.RUNNING:
           try:
               x = np.array(car.get_MPCC_horizon())


               (x_ref,y_ref) = splev(x[6,:], traj.pos_tck)
               plotter.set_ref_point(x_ref,y_ref)
               plotter.update_plot(x[0,:], x[1, :])
           except Exception as e:
               print(e)
           time.sleep(1/60)




t_eval=np.linspace(0, traj.t_end, 100)
        
s=splev(t_eval, traj.evol_tck)
v=splev(t_eval, traj.evol_tck, der=1)
(x,y)=splev(s, traj.pos_tck)

plotter.set_ref_traj(x,y)



parent_dir = os.path.dirname(os.path.dirname(__file__))

file_name = os.path.join(parent_dir, "configs/JoeBush1.yaml")

with open(file_name) as f:
    full_params = yaml.full_load(f)

    MPCC_params = full_params["parameter_server"]["ros__parameters"]["controllers"]["MPCC"]

car_1 = F1Client("JoeBush1")
print(f"Connected to {car_1.car_ID}")

# select the controller


car_1.select_controller("MPCC")


print(car_1.get_MPCC_params())

print(car_1.set_MPCC_params(MPCC_params))

print(car_1.get_MPCC_params())

input("Press enter to start trajectory execution")
car_1.execute_trajectory(trajectory=traj)



if args.horizon == 1:

    plotter.show()
else:
    exit()

horizon_call_spin(car=car_1)




plt.close()