import os
from aimotion_f1tenth_utils.F1Client import F1Client
from aimotion_f1tenth_utils.utils import CONTROLLER_MODE
from trajectory_generators import null_infty, eight, null_paperclip, train8
from aimotion_f1tenth_utils.Trajectory import Trajectory
import matplotlib as mpl
import os
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
import numpy as np
import pickle
import yaml

mpl.rcParams["text.usetex"] = False 

# Design / load the trajectory

#traj_ID = "traj_1"
#traj = Trajectory("traj1")
#points = np.array([[0, -1.5],[0, 0],[0, 1.5],[0,2]])
#traj.build_from_points_const_speed(points, 0.0001, 3, 0.5)
##traj.load("paperclip.traj")
#traj.plot_trajectory()
#path, v = null_paperclip()
##traj.plot_trajectory()


parent_dir = os.path.dirname(os.path.dirname(__file__))

traj_name = "paperclip.traj"
traj_file = os.path.join(parent_dir, "trajectories", traj_name)
traj = Trajectory("traj_1")

traj.load(traj_file)
points = np.array([[0, -1.5],[0, 0],[0, 1.5],[0,2]])
#traj.build_from_points_const_speed(points, 0.0001, 3, 0.5)

path, v = null_paperclip()
traj.build_from_waypoints(path, v, 0, 5)


traj.plot_trajectory()

x_r, y_r, *_ = traj.get_trajectory()



# connect to the vehicle
car_1 = F1Client("JoeBush1")
print(f"Connected to {car_1.car_ID}")


parent_dir = os.path.dirname(os.path.dirname(__file__))

file_name = os.path.join(parent_dir, "configs/JoeBush1.yaml")

with open(file_name) as f:
    full_params = yaml.full_load(f)

    MPCC_params = full_params["parameter_server"]["ros__parameters"]["controllers"]["MPCC"]


car_1.set_MPCC_params(MPCC_params)
# select the controller
car_1.select_controller("MPCC")
#car_1.reinit_GP_LPV_LQR(vehicle_params=vehicle_params, GP_LPV_LQR_params=GP_LPV_LQR_params)
car_1.reset_state_logger()
#car_1.GP_reset()
#car_1.GP_to_online()

car_1.set_mode(CONTROLLER_MODE.IDLE)
#car_1.reset_controller()
#car_1.reset_state_logger()

# execute_trajectory
car_1.execute_trajectory(trajectory=traj)

# block the script until the execution is finished
car_1.wait_while_running()

states1, inputs1, c1, errors1 = car_1.get_logs() #errors1: {"lateral": lat_error, "heading": theta, "long": long_error, "velocity": computing_time}


#Path following figure:
plt.figure()
plt.title("Real life results")
plt.plot(x_r, y_r, label = "reference trajectory")

#Create continious colorbarmapping

points = np.array([states1[:,0], states1[:,1]]).T.reshape(-1,1,2)
segments = np.concatenate([points[:-1], points[1:]], axis = 1)

norm = plt.Normalize(vmin =0, vmax=5)
lc = LineCollection(segments=segments, cmap = "turbo", norm=norm)
lc.set_array(states1[:,3])
lc.set_linewidth(2)

plt.gca().add_collection(lc)
plt.xlabel("x[m]")
plt.ylabel("y[m]")
cbar = plt.colorbar(lc, label = 'v_xi')
plt.axis('equal')




#Computing time figure
fig, axs = plt.subplots(2,1, figsize = (10,6))

axs[0].title.set_text("Computing time histogram")
try:
    axs[0].hist(errors1[:,3]*1000)
    axs[0].axvline(x = MPCC_params["Tf"]/MPCC_params["N"]*1000, color = 'r', label = 'sampling time [ms]')
    axs[1].plot(np.arange(np.shape(errors1[:,3])[0]), errors1[:,3]*1000)
    axs[1].axhline(y = MPCC_params["Tf"]/MPCC_params["N"]*1000, color = 'r', label = 'sampling time [ms]')
    print(errors1[:,3])
    print(np.shape(errors1))
except Exception as e:
    print(e)

    
plt.show()


"""
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
    
    """
