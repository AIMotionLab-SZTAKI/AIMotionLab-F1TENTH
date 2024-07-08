from aimotion_f1tenth_system.src.vehicle_control.vehicle_control.controllers.MPCC_controller import MPCC_Controller
from aimotion_f1tenth_utils.Trajectory import Trajectory
from aimotion_f1tenth_system.src.vehicle_control.vehicle_control.MPCC.trajectory import Spline_2D
import yaml
import os
import time
import numpy as np
from scipy.interpolate import splev
import matplotlib.pyplot as plt
from trajectory_generators import null_infty, eight, null_paperclip, train8
from MPCC_plotter import MPCC_plotter
parent_dir = os.path.dirname(os.path.dirname(__file__))

file_name = os.path.join(parent_dir, "configs/Simulator_config.yaml")
with open(file_name) as file:
    params = yaml.full_load(file)

args = {}

args["vehicle_params"] = params["parameter_server"]["ros__parameters"]["vehicle_params"]
args["MPCC_params"] = params["parameter_server"]["ros__parameters"]["controllers"]["MPCC"]
args["drive_bridge"] = params["parameter_server"]["ros__parameters"]["drive_bridge"]
args["crazy_observer"] = params["parameter_server"]["ros__parameters"]["crazy_observer"]
MOTOR_LIMIT = args["drive_bridge"]["MOTOR_LIMIT"]

controller = MPCC_Controller(vehicle_params= args["vehicle_params"], mute = False, MPCC_params= args["MPCC_params"])



#Silverstone trajectory

traj = Trajectory("Silverstone")
traj.load("trajectories/Silverstone.traj")

# Spielberg trajectory

traj = Trajectory("Spielberg")
traj.load("trajectories/Spielberg.traj")

#traj.plot_trajectory()
theta_start = 0.10


# Normal trajectory

traj_name = "paperclip.traj"
traj_file = os.path.join(parent_dir, "trajectories", traj_name)
traj = Trajectory("traj_1")

traj.load(traj_file)
points = np.array([[0, -1.5],[0, 0],[0, 1.5],[0,2]])
#traj.build_from_points_const_speed(points, 0.0001, 3, 0.5)

path, v = null_infty()
traj.build_from_waypoints(path, v, 0, 5)

(x,y) = (splev(theta_start-0.1, traj.pos_tck))

phi = 0.84 #normal

#phi = -2.87969 #Spielberg

#phi =1.19 #Silverstone
x0 = np.array([x-0.05,y+0.05,phi, 0, 0.0,0.0])


controller.set_trajectory(pos_tck = traj.pos_tck,
                        evol_tck = traj.evol_tck,
                        x0 = x0,
                        theta_start = theta_start) #The class will convert the tck into its own trajectory format


#print(controller.trajectory.get_path_parameters_ang(theta_start))
#input("Press enter to start sim")

iteration = 1400
Tf = args["MPCC_params"]["Tf"]
N = args["MPCC_params"]["N"]

dt = Tf/N
dt = 1/args["crazy_observer"]["FREQUENCY"]
#input("Press enter to run sim")
x_sim = np.array(np.reshape(x0, (-1,1)))

errors = np.array(np.zeros((5,1)))


freq = np.array([0])

controller.muted = False
u_sim = np.zeros((2,1))
theta_sim = np.array([0])


plotter = MPCC_plotter()

s = np.linspace(0, controller.trajectory.L,10000)

plotter.set_ref(np.array(controller.trajectory.spl_sx(s)), np.array(controller.trajectory.spl_sy(s)))

plotter.show()



for i in range(iteration):
    t_s = time.time()
    u, error, finished = controller.compute_control(x0=x0)

    if u[0] > MOTOR_LIMIT: #Limit the output to the drivebridge limit
        u[0] = MOTOR_LIMIT  


    freq = np.append(freq, 1/(time.time()-t_s))
    
    
    x0,t= controller.simulate(x0, u, dt)

    #Simulating noise: 
    x0[3] = x0[3]*np.random.normal(1,0.02)
    x0[0] = x0[0]+np.random.normal(0,1)*0.005
    x0[1] = x0[1]+np.random.normal(0,1)*0.005
    
    x_sim = np.append(x_sim, np.reshape(x0, (-1,1)), axis= 1)

    errors = np.append(errors, np.reshape(error, (-1,1)), axis = 1)
    u_sim = np.append(u_sim, np.reshape(u,(-1,1)), axis = 1)
    theta_sim = np.append(theta_sim, controller.theta)
    if finished:
        break


    horizon = np.array(np.reshape(controller.ocp_solver.get(0, 'x'),(-1,1)))
    for i in range(controller.parameters.N-1):
        x   = controller.ocp_solver.get(i+1, 'x')
        x = np.reshape(x, (-1,1))
        horizon = np.append(horizon, x, axis = 1)
    plotter.update_plot(new_x = horizon[0,:], new_y = horizon[1,:])
    time.sleep(dt)
    plotter.ax.set_title(f"Current speed: {x_sim[3,-1]:.3f}m/s")

s = np.linspace(0, controller.trajectory.L,1000)


plt.figure()
plt.title("Trajectory")
plt.plot(controller.trajectory.spl_sx(s), controller.trajectory.spl_sy(s))
plt.plot(x_sim[0,:], x_sim[1,:])
plt.scatter(x_sim[0,:], x_sim[1,:], c = x_sim[3,:],s = 15,cmap = "Reds")
plt.axis("equal")

plt.xlabel("x[m]")
plt.ylabel("y[m]")


plt.figure()
plt.title("Contouring error")
plt.plot(theta_sim,errors[0,:])
plt.xlabel("theta[m]")
plt.ylabel("Error[m]")


plt.figure()
plt.title("Controller Frequency")

plt.xlabel("theta[m]")
plt.ylabel("Frequency [Hz]")
plt.plot(theta_sim[1:-1],freq[1:-1])


plt.figure()
plt.subplot(2,1,1)

plt.title("Motor reference (d)")

plt.xlabel("theta[m]")
plt.ylabel("d[-]")

plt.plot(theta_sim[1:], u_sim[0,1:])

plt.subplot(2,1,2)

plt.title("Speed")

plt.xlabel("theta[m]")
plt.ylabel("speed[m/s]")

plt.plot(theta_sim, x_sim[3,:])



plt.show()

print()
input("Press enter to exit...")