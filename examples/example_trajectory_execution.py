import os
import matplotlib.pyplot as plt
import pickle

from aimotion_f1tenth_utils.F1Client import F1Client
from aimotion_f1tenth_utils.utils import CONTROLLER_MODE
from aimotion_f1tenth_utils.Trajectory import Trajectory


# Design / load the trajectory
traj_ID = "traj_1"
traj = Trajectory(trajectory_ID=traj_ID) # create the trajetory object
traj.load(os.path.join(os.path.dirname(__file__) , traj_ID+".traj"))

# display the trajectory in a figure
traj.plot_trajectory(block=False)

# connect to the vehicle
car_1 = F1Client(host = "192.168.2.62", port = 8069)
print(f"Connected to {car_1.car_ID}")


# select the controller
car_1.select_controller("GP_LPV_LQR")

# reset the internal state of the controller
car_1.reset_controller()

# reset the state logger
car_1.reset_state_logger()

# execute_trajectory
car_1.execute_trajectory(trajectory=traj)

# block the script until the execution is finished
car_1.wait_while_running()


# retrieve logs
states1, inputs1, c1, errors1 = car_1.get_logs()

# plot the logs
plt.figure()
x_r, y_r, *_ = traj.get_trajectory()
plt.plot(x_r, y_r)
plt.plot(states1[:,0], states1[:,1])
plt.legend(["Reference", "Measurement1"])

plt.figure()
plt.plot(errors1)
plt.legend(["Lateral", "Heading", "Longitudinal", "Velocity"])

plt.figure()
plt.plot(inputs1)
plt.legend(["d", "detla"])

# display
plt.show()