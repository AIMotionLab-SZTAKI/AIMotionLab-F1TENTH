import numpy as np
import time
import os
from aimotion_f1tenth_utils.F1Client import F1TENTH, Connection
from aimotion_f1tenth_utils.Trajectory import Trajectory

# Create a connection to the fleet manager
server = "192.168.2.65" #"127.0.1.1" #"localhost"
port = 8000
conn = Connection(server, port) 
# connection is used to send non vehicle specific messages, i.e individual trajectories and actions to upload to the server


print(conn.send({"command": "list_trajectories"})) # getting the trajectories


car1 = F1TENTH("JoeBush1", conn)


res = car1.execute_trajectory("traj_1", block= True)
#res = car1.wait_until_done()
print(res)
car1.connection.client.close()


# the vehicle object is used to send/recieve vehicle specific messages, 
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
