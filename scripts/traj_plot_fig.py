from scipy.interpolate import splev
import matplotlib.pyplot as plt
from aimotion_f1tenth_utils.Trajectory import Trajectory
import os
import numpy as np
from trajectory_generators import null_infty, eight, null_paperclip, train8


parent_dir = os.path.dirname(os.path.dirname(__file__))

traj_name = "paperclip.traj"
traj_file = os.path.join(parent_dir, "trajectories", traj_name)
traj = Trajectory("traj_1")


traj.load(traj_file)
points = np.array([[0, -1.5],[0, 0],[0, 1.5],[0,2]])
#traj.build_from_points_const_speed(points, 0.0001, 3, 0.5)

path, v = null_paperclip()
traj.build_from_waypoints(path, v, 0, 5)


t_eval=np.linspace(0, traj.t_end, 100)
        
s=splev(t_eval, traj.evol_tck)
v=splev(t_eval, traj.evol_tck, der=1)
(x,y)=splev(s, traj.pos_tck)

plt.axis('equal')
plt.plot(x,y)
plt.show()