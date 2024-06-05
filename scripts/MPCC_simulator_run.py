from aimotion_f1tenth_system.src.vehicle_control.vehicle_control.controllers.MPCC_controller import MPCC_Controller
from aimotion_f1tenth_utils.Trajectory import Trajectory
from aimotion_f1tenth_system.src.vehicle_control.vehicle_control.MPCC.trajectory import Spline_2D
import yaml
import os
import numpy as np
from scipy.interpolate import splev
import matplotlib.pyplot as plt
parent_dir = os.path.dirname(os.path.dirname(__file__))

file_name = os.path.join(parent_dir, "configs/JoeBush1.yaml")
with open(file_name) as file:
    params = yaml.full_load(file)

args = {}

args["vehicle_params"] = params["parameter_server"]["ros__parameters"]["vehicle_params"]
args["MPCC_params"] = params["parameter_server"]["ros__parameters"]["controllers"]["MPCC"]


controller = MPCC_Controller(vehicle_params= args["vehicle_params"], MPCC_params= args["MPCC_params"])



traj_name = "paperclip.traj"
traj_file = os.path.join(parent_dir, "scripts", traj_name)
traj = Trajectory("traj_1")

traj.load(traj_file)
points = np.array([[0, -1.5],[0, 0],[0, 1.5],[0,2]])
#traj.build_from_points_const_speed(points, 0.0001, 3, 0.5)


traj.plot_trajectory()

theta_start = 0.15
(x,y) = (splev(theta_start-0.1, traj.pos_tck))
phi = -1.51
x0 = np.array([x+0.01,y-0.01,phi, 0.01, 0.0,0.0])

print(x0)

controller.set_trajectory(pos_tck = traj.pos_tck,
                        evol_tck = traj.evol_tck,
                        x0 = x0,
                        theta_start = theta_start) #The class will convert the tck into its own trajectory format



iteration = 300
Tf = args["MPCC_params"]["Tf"]
N = args["MPCC_params"]["N"]

dt = Tf/N

dt = 1/60
print(f"Integration time: {dt}")
input("Press enter to run sim")
x_sim = np.array(np.reshape(x0, (-1,1)))

errors = np.array(np.zeros((5,1)))

print(errors)
for i in range(iteration):
    u, error = controller.compute_control(x0=x0)
    x0,t= controller.simulate(x0, u, dt)

    x_sim = np.append(x_sim, np.reshape(x0, (-1,1)), axis= 1)

    errors = np.append(errors, np.reshape(error, (-1,1)), axis = 1)

s = np.linspace(0, controller.trajectory.L)

plt.figure()
plt.plot(controller.trajectory.spl_sx(s), controller.trajectory.spl_sy(s))
plt.plot(x_sim[0,:], x_sim[1,:])
plt.axis("equal")

plt.figure()

plt.plot(np.arange(len(x_sim[0,:])),errors[0,:])
plt.show()