import numpy as np
import time
import os
from aimotion_f1tenth_utils.F1Client import F1TENTH, Connection
from aimotion_f1tenth_utils.Trajectory import Trajectory

# Create a connection to the fleet manager
server = "192.168.2.65"#"127.0.1.1" #"localhost"
port = 8000
conn = Connection(server, port) 
# connection is used to send non vehicle specific messages, i.e individual trajectories and actions to upload to the server

if not os.path.exists("logs"):
    os.mkdir("logs")


# design previously known trajectories
traj1=Trajectory("traj_1")
traj1.build_from_points_const_speed(np.flip(np.array(
    [
        [0, 0],
        [1, 1],
        [2, 2],
        [3, 2],
        [4, 1],
        [4.5, 0],
        [4, -1],
        [3, -2],
        [2, -2],
        [1, -1],
        [0, 0],
        [-1, 1],
        [-2, 2],
        [-3, 2],
        [-4, 1],
        [-4.5, 0],
        [-4, -2.1],
        [-3, -2.3],
        [-2, -2],
        [-1, -1],
        [0, 0],
    ]
),axis=1)*.5, 0.001, 5, .8)

# upload trajectories to the server
conn.upload_trajectory(traj1)

print(conn.send({"command": "reload"})) #Reoding the manager
print(conn.send({"command": "list_trajectories"})) # getting the trajectories

# create a vehicle object
car1 = F1TENTH("JoeBush1", conn)
print(car1.toggle_radio_active(True)) # rádió indítás-> rádio
print(car1.execute_trajectory("traj_1"))

"""
 # execute a previously uploaded trajectory
time.sleep(5) # wait a bit
car1.execute_trajectory("traj_2") # execute another trajectory
time.sleep(5) # wait a bit
car1.execute_trajectory("traj_3") # execute another trajectory
time.sleep(5) # wait a bit
car1.execute_trajectory("traj_4") # execute another trajectory
"""
