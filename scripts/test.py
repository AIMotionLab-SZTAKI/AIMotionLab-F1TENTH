from aimotion_f1tenth_utils.Trajectory import Trajectory
from aimotion_f1tenth_system.src.vehicle_control.vehicle_control.MPCC.trajectory import Spline_2D
import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import splev
import casadi as cs
traj = Trajectory("traj_1.traj")

traj.load("example_1.traj")

traj.plot_trajectory()
t_end = traj.evol_tck[0][-1]

print(traj.evol_tck[0][-1])

t_eval=np.linspace(0, t_end, 1000)

s=splev(t_eval, traj.evol_tck)

(x,y) = splev(s, traj.pos_tck)

points_list = []


for i in range(len(x)):
    
    points_list.append([i, x[i], y[i]])

trajectory = Spline_2D(np.array([[0,0,0],[1,1,1],[2,2,2]]))

trajectory.spl_sx = cs.interpolant("traj", "bspline", [s], x)
trajectory.spl_sy = cs.interpolant("traj", "bspline", [s], y)

plt.plot(trajectory.spl_sx(s[:-1]), trajectory.spl_sy(s[:-1]))

plt.show()