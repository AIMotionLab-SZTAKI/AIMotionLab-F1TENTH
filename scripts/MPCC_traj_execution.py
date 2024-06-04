import os
from aimotion_f1tenth_utils.F1Client import F1Client
from aimotion_f1tenth_utils.utils import CONTROLLER_MODE
from trajectory_generators import null_infty, eight, null_paperclip, train8
from aimotion_f1tenth_utils.Trajectory import Trajectory
import matplotlib as mpl
import os
import matplotlib.pyplot as plt
import numpy as np
import pickle
mpl.rcParams["text.usetex"] = False 

# Design / load the trajectory
traj_ID = "traj_1"
traj = Trajectory("traj1")
points = np.array([[0, -1.5],[0, 0],[0, 1.5],[0,2]])
traj.build_from_points_const_speed(points, 0.0001, 3, 0.5)
#traj.load("paperclip.traj")
traj.plot_trajectory()
path, v = null_paperclip()
#traj.plot_trajectory()
x_r, y_r, *_ = traj.get_trajectory()

# connect to the vehicle
car_1 = F1Client("192.168.2.62", 8069)
print(f"Connected to {car_1.car_ID}")


# select the controller
car_1.select_controller("MPCC")
#car_1.reinit_GP_LPV_LQR(vehicle_params=vehicle_params, GP_LPV_LQR_params=GP_LPV_LQR_params)
#car_1.reset_state_logger()
#car_1.GP_reset()
#car_1.GP_to_online()

car_1.set_mode(CONTROLLER_MODE.IDLE)
#car_1.reset_controller()
#car_1.reset_state_logger()

# execute_trajectory
car_1.execute_trajectory(trajectory=traj)

# block the script until the execution is finished
car_1.wait_while_running()

states1, inputs1, c1, errors1 = car_1.get_logs()

plt.figure()

plt.plot(x_r, y_r)
plt.plot(states1[:,0], states1[:,1])
plt.legend(["Reference", "Measurement1"])
plt.axis('equal')
plt.figure()
plt.plot(errors1)
plt.legend(["lateral", "heading", "long", "velocity"])

plt.figure()
plt.plot(inputs1)

plt.show()

res = {
    "states": states1,
    "inputs": inputs1,
    "errors": errors1, 
    "ref": [x_r, y_r]
}


with open("result_good.pickle", "wb") as f:
    pickle.dump(res, f)