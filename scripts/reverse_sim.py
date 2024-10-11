from aimotion_f1tenth_system.src.vehicle_control.vehicle_control.controllers.MPCC_reverse_controller import MPCC_reverse_controller
from F1TENTH_sim import F1TENTH_sim
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
from matplotlib.collections import LineCollection

sim_speed = 100

parent_dir = os.path.dirname(os.path.dirname(__file__))

file_name = os.path.join(parent_dir, "configs/JoeBush1.yaml")
with open(file_name) as file:
    params = yaml.full_load(file)

args = {}

args["vehicle_params"] = params["parameter_server"]["ros__parameters"]["vehicle_params"]
args["MPCC_params"] = params["parameter_server"]["ros__parameters"]["controllers"]["MPCC_reverse"]
args["drive_bridge"] = params["parameter_server"]["ros__parameters"]["drive_bridge"]
args["crazy_observer"] = params["parameter_server"]["ros__parameters"]["crazy_observer"]
MOTOR_LIMIT = args["drive_bridge"]["MOTOR_LIMIT"]

controller = MPCC_reverse_controller(vehicle_params= args["vehicle_params"], mute = False, MPCC_params= args["MPCC_params"])

simulator = F1TENTH_sim(vehicle_params=args["vehicle_params"])
#Silverstone trajectory

traj = Trajectory("trajectory")

#traj.plot_trajectory()
theta_start = 0.1


# Normal trajectory


path, v = null_infty()
traj.build_from_waypoints(path, v, 0, 5)



(x,y) = (splev(theta_start, traj.pos_tck))

phi = 0.84 #normal

#phi = -2.87969 #Spielberg

#phi =1.19 #Silverstone
x0 = np.array([x,y,phi+np.pi, 0, 0.0,0.0])


controller.set_trajectory(pos_tck = traj.pos_tck,
                        evol_tck = traj.evol_tck,
                        generate_solver=True) #The class will convert the tck into its own trajectory format

controller.init_controller(x0 = x0)

#print(controller.trajectory.get_path_parameters_ang(theta_start))
#input("Press enter to start sim")

iteration = 1400
Tf = args["MPCC_params"]["Tf"]
N = args["MPCC_params"]["N"]

dt = Tf/N

#input("Press enter to run sim")
x_sim = np.array(np.reshape(x0, (-1,1)))

errors = np.array(np.zeros((4,1)))


c_t = np.array([])

controller.muted = False
u_sim = np.zeros((2,1))
theta_sim = np.array([0])


plotter = MPCC_plotter()

s = np.linspace(0, controller.trajectory.L,10000)

plotter.set_ref_traj(np.array(controller.trajectory.spl_sx(s)), np.array(controller.trajectory.spl_sy(s)))

plotter.show()



for i in range(iteration):
    u, error, finished = controller.compute_control(x0=x0)

    if u[0] > MOTOR_LIMIT: #Limit the output to the drivebridge limit
        u[0] = MOTOR_LIMIT  


    c_t = np.append(c_t, controller.c_t)
    
    
    x0,t= simulator.simulate(x0, u, dt)

    #Simulating noise: 
    #x0[3] = x0[3]*np.random.normal(1,0.05)
    #x0[2] = x0[2]*np.random.normal(1,0.001)
    #x0[0] = x0[0]+np.random.normal(0,1)*0.005
    #x0[1] = x0[1]+np.random.normal(0,1)*0.005
    
    x_sim = np.append(x_sim, np.reshape(x0, (-1,1)), axis= 1)
    errors = np.append(errors, np.reshape(error, (-1,1)), axis = 1)
    u_sim = np.append(u_sim, np.reshape(u,(-1,1)), axis = 1)
    theta_sim = np.append(theta_sim, controller.theta)
    if finished:
        break


    x_ref, y_ref = (controller.trajectory.spl_sx(controller.theta),controller.trajectory.spl_sy(controller.theta))
    plotter.set_ref_point(np.array(float(x_ref)), np.array(float(y_ref)))

    horizon = np.array(np.reshape(controller.ocp_solver.get(0, 'x'),(-1,1)))
    for i in range(controller.parameters.N-1):
        x   = controller.ocp_solver.get(i+1, 'x')
        x = np.reshape(x, (-1,1))
        horizon = np.append(horizon, x, axis = 1)
    plotter.update_plot(new_x = horizon[0,:], new_y = horizon[1,:])
    time.sleep(dt/sim_speed)
    plotter.ax.set_title(f"Current speed: {x_sim[3,-1]:.3f}m/s")

    #input(f"x:{x_sim[0][-1]} y:{x_sim[1][-1]} phi:{x_sim[2][-1]} v_xi:{x_sim[3][-1]} v_eta:{x_sim[4][-1]} omega: {x_sim[5][-1]}")

#Creating simulation result plots
s = np.linspace(0, controller.trajectory.L,1000)


plt.figure()
plt.title("Trajectory")
plt.plot(controller.trajectory.spl_sx(s), controller.trajectory.spl_sy(s))
#plt.plot(x_sim[0,:], x_sim[1,:])
#plt.scatter(x_sim[0,:], x_sim[1,:], c = x_sim[3,:],s = 15,cmap = "Reds")
#plt.axis("equal")
#Create continious colorbarmapping

points = np.array([x_sim[0,:], x_sim[1,:]]).T.reshape(-1,1,2)
segments = np.concatenate([points[:-1], points[1:]], axis = 1)

norm = plt.Normalize(vmin =0, vmax=5)
lc = LineCollection(segments=segments, cmap = "turbo", norm=norm)
lc.set_array(-x_sim[3,:])
lc.set_linewidth(2)

plt.gca().add_collection(lc)
plt.xlabel("x[m]")
plt.ylabel("y[m]")
cbar = plt.colorbar(lc, label = 'v_xi')
plt.axis('equal')



plt.xlabel("x[m]")
plt.ylabel("y[m]")


fig, axs = plt.subplots(2,1, figsize = (10,6))

axs[0].title.set_text("Computing time historgram")
axs[0].hist(errors[3,1:-1]*1000)
axs[0].axvline(x = controller.MPCC_params["Tf"]/controller.MPCC_params["N"]*1000, color = 'r', label = 'sampling time [ms]')
axs[0].legend()
axs[0].set_xlabel("Computing time [ms]")
axs[0].set_xlim(left = 0)

axs[1].title.set_text("Computing time")
axs[1].set_xlabel("Iteration [-]")
axs[1].set_ylabel("c_t [ms]")
axs[1].plot(np.arange(np.shape(errors[3,1:-1])[0]),errors[3,1:-1]*1000 , label = "computing time [ms]")
axs[1].axhline(y = controller.MPCC_params["Tf"]/controller.MPCC_params["N"]*1000, color = 'r', label = 'sampling time [ms]')
axs[1].legend()

plt.tight_layout()


fig, axs = plt.subplots(3,1, figsize = (10,6))


axs[0].title.set_text("Contouring error")
axs[0].set_xlabel("Iteration [-]")
axs[0].set_ylabel("e_c [m]")
axs[0].plot(np.arange(np.shape(errors[0,:-1])[0]),errors[0,:-1] )

axs[1].title.set_text("Longitinal error")
axs[1].set_xlabel("Iteration [-]")
axs[1].set_ylabel("e_l [m]")
axs[1].plot(np.arange(np.shape(errors[0,:-1])[0]),errors[1,:-1] )


axs[2].title.set_text("Progress")
axs[2].set_xlabel("Iteration [-]")
axs[2].set_ylabel("θ [m]")
axs[2].plot(np.arange(np.shape(errors[0,:-1])[0]),errors[2,:-1] )

plt.tight_layout()

fig, axs = plt.subplots(2,1, figsize = (10,6))

axs[0].title.set_text("Motor reference")
axs[0].set_xlabel("Iteration [-]")
axs[0].set_ylabel("d [-]")
axs[0].plot(np.arange(np.shape(u_sim[0,1:-1])[0]),u_sim[0,1:-1] )


axs[1].title.set_text("Steering servo reference")
axs[1].set_xlabel("Iteration [-]")
axs[1].set_ylabel("δ [-]")
axs[1].plot(np.arange(np.shape(u_sim[1,1:-1])[0]),u_sim[1,1:-1] )

plt.tight_layout()

"""
plt.figure()
plt.title("Contouring error")
plt.plot(theta_sim[:-1],errors[0,1:])
plt.xlabel("theta[m]")
plt.ylabel("Error[m]")


plt.figure()
plt.title("Lateral error")
plt.plot(theta_sim[:-1],errors[2,1:])
plt.xlabel("theta[m]")
plt.ylabel("Error[m]")


plt.figure()
plt.title("Controller computing time")

plt.xlabel("theta[m]")
plt.ylabel("Computing time [s]")

#plt.plot(theta_sim[1:-1],c_t[:-1])


plt.hist(c_t, bins=8, edgecolor='black', label = "computing times")  
plt.axvline(x=dt, color='red', linestyle='--', linewidth=2, label = "sampling time")
#plt.xlim(left = 0)
#plt.xscale("log")
plt.legend()

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
"""


plt.show()

print()
input("Press enter to exit...")