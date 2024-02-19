from aimotion_f1tenth_utils.Trajectory import Trajectory
from aimotion_f1tenth_utils.F1Client import Connection

import matplotlib as mpl
mpl.rcParams["text.usetex"] = False 

# 1, design the trajectory 
traj_ID = "example_1"
traj = Trajectory(trajectory_ID=traj_ID) # create the trajetory object
traj.draw_from_waypoints() # place waypoints on the matplotlib graph

# 2, Create connection the the manager
host="192.168.2.65"
port=8000
conn = Connection(host=host, port=port)

# 3, Check if the traj_ID is unique
stored_trajectories = conn.list_trajectories()
assert stored_trajectories[0], stored_trajectories[1]
assert traj_ID+".traj" not in stored_trajectories[1], f"Trajectory with ID {traj_ID} already exists on the server!"

# 4, Upload the trajectory
assert conn.upload_trajectory(traj), "Trajectory upload failed"