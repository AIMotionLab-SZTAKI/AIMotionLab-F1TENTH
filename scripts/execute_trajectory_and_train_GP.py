import os
from aimotion_f1tenth_utils.F1Client import F1Client
from aimotion_f1tenth_utils.utils import CONTROLLER_MODE
from aimotion_f1tenth_utils.Trajectory import Trajectory
from trajectory_generators import train8
import matplotlib as mpl
import os
import matplotlib.pyplot as plt
import numpy as np


mpl.rcParams["text.usetex"] = False 

# Design / load the trajectory used for the training
train_velocities = [0.75, 1.1, 1.5]

train_trajectories = [Trajectory(f"training_v_{v}") for v in train_velocities]
t = np.linspace(0, 2*np.pi, 9)
a = 2.8
b = 1.4
x = a * np.sin(t)
y = b * np.sin(2*t)
training_path=np.flip(np.vstack((x,y)).T, axis=1)
test_path, _ = train8() 
for train_trajectory,v in zip(train_trajectories, train_velocities): 
    train_trajectory.build_from_points_const_speed(path_points=training_path,
                                                   path_smoothing=0.001, 
                                                   path_degree=5, 
                                                   const_speed=v)
# Design the validation trajectory
test_trajectory = Trajectory("test")
test_trajectory.build_from_points_const_speed(path_points=test_path,
                                              path_smoothing=0.001, 
                                                path_degree=5,
                                                const_speed=1)
#test_trajectory.load(os.path.join(os.path.dirname(__file__), "..", "trajectories/paperclip.traj"))   

# connect to the vehicle
car_1 = F1Client("JoeBush1")

# select the controller
#car_1.reinit_LPV_LQR_from_yaml(os.path.join(os.path.dirname(__file__), "GP_control_params.yaml"))
car_1.select_controller("GP_LPV_LQR")
car_1.reset_state_logger()
car_1.GP_reset()

# nominal round
car_1.execute_trajectory(trajectory=test_trajectory)
car_1.wait_while_running()

n_states, n_inputs, n_c, n_errors = car_1.get_logs()
car_1.reset_controller()
car_1.reset_state_logger()

for traj in train_trajectories:
    print(f"Executing {traj.trajectory_ID}")
    # execute_trajectory
    car_1.execute_trajectory(trajectory=traj)

    # block the script until the execution is finished
    car_1.wait_while_running()

print("Training trajectory executions finished finished training GPs")
training_data = car_1.GP_train(retrieve_training_data = True)

lat_x = training_data[0][0]
lat_y = training_data[0][1]
long_x = training_data[1][0]
long_y = training_data[1][1]

fig, axs1 = plt.subplots(2, 1)
axs1[0].plot(lat_x)
axs1[0].legend(["v_xi","v_eta", "omega"])
axs1[1].plot(lat_y)
axs1[1].legend(["train_y"])

fig, axs2 = plt.subplots(2, 1)
axs2[0].plot(long_x)
axs2[0].legend(["v_xi","v_eta", "omega"])
axs2[1].plot(long_y)
axs2[1].legend(["train_y"])

plt.show()

car_1.reset_controller()
car_1.reset_state_logger()

car_1.execute_trajectory(trajectory=test_trajectory)
car_1.wait_while_running()

states, inputs, c, errors = car_1.get_logs()

plt.figure()

x_r, y_r, *rst = test_trajectory.get_trajectory()
plt.plot(x_r, y_r)
plt.plot(n_states[:,0], n_states[:,1])
plt.plot(states[:,0], states[:,1])
plt.legend(["Reference", "Nominal", "Adaptive"])

plt.figure()
plt.plot(states)
plt.legend(["x", "y", "heading", "v_xi", "v_eta", "omega"])

plt.figure()
plt.plot(inputs)
plt.legend(["d", "delta"])

plt.figure()
plt.plot(errors)
plt.legend(["lateral", "heading", "long", "velocity", "q"])
plt.show()

