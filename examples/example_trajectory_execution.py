import numpy as np
import time
import os
from aimotion_f1tenth_utils.F1Client import F1TENTH, Connection
from aimotion_f1tenth_utils.Trajectory import Trajectory

# Create a connection to the fleet manager
server = "192.168.2.65"#"127.0.1.1" #"localhost"
port = 8001
conn = Connection(server, port) 
# connection is used to send non vehicle specific messages, i.e individual trajectories and actions to upload to the server

if not os.path.exists("logs"):
    os.mkdir("logs")
# design prevoiusly known trajectories
traj1=Trajectory("traj_1")
traj1.build_from_points_const_speed(np.array([[0,0],[1,0],[1,1],[0,1],[0,0]]), 0.001, 3, 1)
traj2=Trajectory("traj_2")
traj2.build_from_points_const_speed(np.array([[0,0],[1,1], [2,0], [3,-1], [4,0], [3,1], [2,0], [1,-1], [0,0]]), 0.001, 3, 3)
traj3=Trajectory("traj_3")
traj3.build_from_points_const_speed(np.array([[0,0],[1,0],[2,0],[3,0],[4,0]]), 0.001, 3, 1)
traj4=Trajectory("traj_4")

# upload trajectories to the server




conn.upload_trajectory(traj1)
conn.upload_trajectory(traj2)
#conn.upload_trajectory(traj3)

conn.upload_trajectory(traj3)

print(conn.send({"command": "reload"}))
"""
# create a vehicle object

print(conn.send({"command": "list_trajectories"}))


print("activating:")
print(car1.toggle_active(True))
print("getting state:")
print(car1.get_state())
print("starting logging")
car1.toggle_logging(True)
time.sleep(5)
car1.toggle_save()
print(car1.toggle_active(False))

"""

car1 = F1TENTH("JoeBush1", conn)

car1.get_logs()

car1.connection.client.close()
#car1.execute_trajectory("traj_1")
# the vehicle abject is used to send/recieve vehicle specific messages, 
# i.e state, trajectory execution and action execution commands


# get the state of the vehicle


#res = car1.get_state()

### Trajectory execution example


"""
 # execute a previously uploaded trajectory
time.sleep(5) # wait a bit
car1.execute_trajectory("traj_2") # execute another trajectory
time.sleep(5) # wait a bit
car1.execute_trajectory("traj_3") # execute another trajectory
time.sleep(5) # wait a bit
car1.execute_trajectory("traj_4") # execute another trajectory
"""
