from aimotion_f1tenth_utils.F1Client import F1Client
from aimotion_f1tenth_utils.Trajectory import Trajectory
from aimotion_f1tenth_utils.utils import CONTROLLER_MODE
from trajectory_generators import null_infty, eight, null_paperclip, train8
from scipy.interpolate import splev
import time
import numpy as np
import yaml
import os


import matplotlib.pyplot as plt
import numpy as np


# Create a figure and axis
fig, ax = plt.subplots()
ax.set_xlim(-5, 5)
ax.set_ylim(-5, 5)

# Initialize a line object, that will be updated
line, = ax.plot([], [], 'ro')  # 'ro' means red color, circle markers
ref_line, = ax.plot([],[], 'b')


# Data container
data = {'x': [], 'y': []}
def update_plot(new_x, new_y):
    """Update the plot with new point, removing the previous one."""
    # Clear previous points
    data['x'].clear()
    data['y'].clear()
    
    # Add new point
    data['x'].append(new_x)
    data['y'].append(new_y)
    
    # Update line data
    line.set_data(data['x'], data['y'])
    
    # Redraw the figure
    fig.canvas.draw_idle()
    fig.canvas.flush_events()
    # Show the plot in interactive mode



parent_dir = os.path.dirname(os.path.dirname(__file__))

traj_name = "paperclip.traj"
traj_file = os.path.join(parent_dir, "trajectories", traj_name)
traj = Trajectory("traj_1")

path, v = null_paperclip()
traj.build_from_waypoints(path, v, 0, 5)
#traj.plot_trajectory()

t_eval=np.linspace(0, traj.t_end, 100)
        
s=splev(t_eval, traj.evol_tck)
v=splev(t_eval, traj.evol_tck, der=1)
(x,y)=splev(s, traj.pos_tck)

ref_line.set_data(x, y)



parent_dir = os.path.dirname(os.path.dirname(__file__))

file_name = os.path.join(parent_dir, "configs/JoeBush1.yaml")

with open(file_name) as f:
    full_params = yaml.full_load(f)

    MPCC_params = full_params["parameter_server"]["ros__parameters"]["controllers"]["MPCC"]

car_1 = F1Client("JoeBush1")
print(f"Connected to {car_1.car_ID}")

# select the controller


car_1.select_controller("MPCC")


print(car_1.get_MPCC_params())

print(car_1.set_MPCC_params(MPCC_params))

print(car_1.get_MPCC_params())

input("Press enter to start trajectory execution")
car_1.execute_trajectory(trajectory=traj)

plt.ion()
plt.show()
while True:
    if car_1.get_mode() == CONTROLLER_MODE.RUNNING:
        try:
            x = np.array(car_1.get_MPCC_horizon())
            update_plot(x[0,:], x[1, :])
        except Exception as e:
            print(e)
        time.sleep(1/10)
