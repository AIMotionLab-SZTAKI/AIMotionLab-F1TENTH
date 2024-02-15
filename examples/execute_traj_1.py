"""Simple script that executes a trajectory that has been previously uploaded to the server."""

from aimotion_f1tenth_utils.F1Client import F1TENTH, Connection

# establish connetion
server = "192.168.2.65"
port = 8000
conn = Connection(server, port) 

# check trajectory
available_trajectories = conn.list_trajectories()
assert available_trajectories[0], available_trajectories[1]
assert "traj_1.traj" in available_trajectories[1], "traj_1.traj has not been uploaded to the server!"

# create car object 
car1 = F1TENTH("JoeBush1", conn)

# execute trajectory
res = car1.execute_trajectory("traj_1", block= True)

assert res, "The execution of the trajectory has failed!"
car1.connection.client.close()