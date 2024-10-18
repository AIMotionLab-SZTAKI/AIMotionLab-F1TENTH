import os
from aimotion_f1tenth_utils.F1Client import F1Client
from aimotion_f1tenth_utils.utils import CONTROLLER_MODE
from trajectory_generators import null_infty, eight, null_paperclip, train8
from complex_trajectories import paperclip_forward, paperclip_backward
from aimotion_f1tenth_utils.Trajectory import Trajectory
import matplotlib as mpl
import os
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
import numpy as np
import pickle
import yaml

mpl.rcParams["text.usetex"] = False 


parent_dir = os.path.dirname(os.path.dirname(__file__))

traj_name = "paperclip.traj"
traj_file = os.path.join(parent_dir, "trajectories", traj_name)
traj = Trajectory("traj_1")

traj.load(traj_file)
points = np.array([[0, -1.5],[0, 0],[0, 1.5],[0,2]])
#traj.build_from_points_const_speed(points, 0.0001, 3, 0.5)
traj.plot_trajectory()
path, v = paperclip_forward(r = 1.1)
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
    MPCC_reverse_params = full_params["parameter_server"]["ros__parameters"]["controllers"]["MPCC_reverse"]
car_1.reset_state_logger()


car_1.select_controller("MPCC")
car_1.set_controller_parameters({"MPCC": MPCC_params, "MPCC_reverse":MPCC_reverse_params})
car_1.set_trajectory(trajectory=traj, generate_solver= True)
car_1.start_controller()

# block the script until the execution is finished
car_1.wait_while_running()

#Reversing

car_1.select_controller("MPCC_reverse")
path, v = paperclip_backward(r = 1.1)
traj.build_from_waypoints(path, v, 0, 5)
car_1.set_trajectory(trajectory=traj, generate_solver= True)
car_1.start_controller()

car_1.wait_while_running()


path, v = paperclip_forward(r=1.1, mirror= True)
traj.build_from_waypoints(path, v, 0, 5)
car_1.select_controller("MPCC")
car_1.reset_controller()
car_1.set_trajectory(trajectory=traj, generate_solver= True)
car_1.start_controller()

car_1.wait_while_running()

car_1.select_controller("MPCC_reverse")
car_1.reset_controller()
path, v = paperclip_backward(r = 1.1, mirror= True)
traj.build_from_waypoints(path, v, 0, 5)
car_1.set_trajectory(trajectory=traj, generate_solver= True)
car_1.start_controller()

car_1.wait_while_running()





#Plotting results
states1, inputs1, c1, errors1 = car_1.get_logs() #errors1: {"contouring": lat_error, "heading": theta, "long": long_error, "velocity": computing_time}


#Path following figure:
plt.figure()

plt.plot(x_r, y_r, label = "reference trajectory")

#Create continious colorbarmapping

points = np.array([states1[:,0], states1[:,1]]).T.reshape(-1,1,2)
segments = np.concatenate([points[:-1], points[1:]], axis = 1)

norm = plt.Normalize(vmin =0, vmax=3.5)
lc = LineCollection(segments=segments, cmap = "turbo", norm=norm)
lc.set_array(states1[:,3])
lc.set_linewidth(2)

plt.gca().add_collection(lc)
plt.xlabel("x[m]")
plt.ylabel("y[m]")
cbar = plt.colorbar(lc, label = '$v_{\\xi}$')
plt.axis('equal')

plt.grid(True)


#Computing time figure
fig, axs = plt.subplots(2,1, figsize = (10,6))

axs[0].title.set_text("Computing time histogram")
try:
    axs[0].hist(errors1[:,3]*1000)
    axs[0].axvline(x = MPCC_params["Tf"]/MPCC_params["N"]*1000, color = 'r', label = 'sampling time [ms]')
    axs[0].set_ylabel("Iteration [-]")
    axs[0].set_xlabel("Computing time [ms]")
    axs[0].legend()

    axs[1].plot(np.arange(np.shape(errors1[:,3])[0]), errors1[:,3]*1000)
    axs[1].axhline(y = MPCC_params["Tf"]/MPCC_params["N"]*1000, color = 'r', label = 'sampling time [ms]')
    axs[1].set_ylabel("Computing time [ms]")
    axs[1].set_xlabel("Iteration [-]")
    axs[1].legend()
    print(errors1[:,3])
    print(np.shape(errors1))
except Exception as e:
    print(e)

print(inputs1)
for ax in axs:
    ax.grid(True)

fig, axs = plt.subplots(3,1, figsize = (10,6))

axs[0].set_title("Motor reference")
axs[0].plot(np.arange(np.shape(inputs1[:,0])[0]),inputs1[:,0])
axs[0].set_xlabel("Iteration [-]")
axs[0].set_ylabel("d [-]")



axs[1].set_title("Steering servo reference")
axs[1].plot(np.arange(np.shape(inputs1[:,1])[0]),inputs1[:,1])
axs[1].set_xlabel("Iteration [-]")
axs[1].set_ylabel("$\\delta$ [-]")



axs[2].set_title("Errors")
axs[2].plot(np.arange(np.shape(errors1[:,0])[0]),errors1[:,0], label = "$e_\\mathrm{f}$ [m]")
axs[2].plot(np.arange(np.shape(errors1[:,0])[0]),errors1[:,3], label = "$e_\\mathrm{l}$ [m]")
axs[2].set_xlabel("Iteration [-]")
axs[2].set_ylabel("errors [m]")
axs[2].legend()

plt.tight_layout()
for ax in axs:
    ax.grid(True)
plt.ion()
plt.show()

input("Press enter to close...")
