from scipy.interpolate import splprep, splrep, splev
import numpy as np
import matplotlib.pyplot as plt
from aimotion_f1tenth_utils.Trajectory import Trajectory
from aimotion_f1tenth_utils.F1Client import F1Client
from aimotion_f1tenth_utils.utils import CONTROLLER_MODE
import time
import socket
import os
import pickle
import struct

# load trajectories
traj_folder = os.path.join(os.path.dirname(os.path.dirname(__file__)), "trajectories", "city_demo")
traj_IDs = ["path_1.traj", "path_2.traj", "path_3.traj", "path_4.traj", "path_5.traj", "path_6.traj"]

# load trajectories
trajectories = []
for traj_ID in traj_IDs:
    traj = Trajectory(trajectory_ID=traj_ID)
    traj.load(os.path.join(traj_folder, traj_ID))
    trajectories.append(traj)

# car connection
car = F1Client("JoeBush1")
car.set_mode(CONTROLLER_MODE.IDLE)

1


GP_LPV_LQR_params = {
        "GP_type": "SGP", # GRAD_SGP, RLS_SGP,
        "frequency": 60.0,
        "num_of_inducing": 20,
        "forgetting_factor": 20,
        "confidence_level": 0.9,
        "retrain_iter": 5,
        "batch_size": 5,
        "lat_gains" : {
            'k1': [0.00266,-0.0168,0.0368,0.0357],
            'k2':  [0.0424, -0.268, 0.588, 0.57],
            'k3': [0.00952, -0.109,0.469, 0.0322],
            'k1_r': [-0.0008*1, 0.0442*1, -1.2247*1],
            'k2_r': [-0.0002*1,0.0191*1,-0.9531*1]
            },

        "long_gains" :{
            'k1': [0.0001, -0.0014, 0.0908],
            'k2': [-0.0025, 0.0298, 0.0095]
          }
        }
    

vehicle_params = {
        'm': 2.9,
        'C_f': 41.7372,
        'C_r': 29.4662,
        'l_f': 0.163,
        'l_r': 0.22,
        'C_m1': 52.4282,
        'C_m2': 5.2465,
        'C_m3': 2.9194, # increase for comparison
}


car.reinit_GP_LPV_LQR(vehicle_params=vehicle_params,
                        GP_LPV_LQR_params=GP_LPV_LQR_params)

# SKYBRUSH TCP connections
drone_server_ip="192.168.2.77"
dummy_server=7001   
skybrush_server = 6001
start_delay = 0
warning_time = 2.0
try:
    Socket =  socket.socket()
    Socket.connect((drone_server_ip, skybrush_server))
    Socket.settimeout(None)
    print(f"Connected to SKYBRUSH server.")
except ConnectionRefusedError:
    try:
        Socket = socket.socket()
        Socket.connect((drone_server_ip, dummy_server))
        Socket.settimeout(None)
        print(f"Connected to DUMMY server.")
    except ConnectionRefusedError:
        Socket = None
if Socket is not None:
    start_delay = struct.unpack("f", Socket.recv(1024).strip())[0]
else:
    start_delay = float(input("Specify demo countdown: "))
# sleep for the predefined time
print(f"Waiting {start_delay} to launch!")
time.sleep(start_delay)
print("Trajectory execution started!")

# start execution
for i, trajectory in enumerate(trajectories):
    # send trajectory index in demo mode
    if Socket is not None:
        serialized=pickle.dumps((warning_time, trajectory.export_to_skybrush()))
        assert len(serialized)< 65536
        Socket.sendall(serialized)

    # sleep for warning_time
    time.sleep(warning_time)

    # execute trajectory
    #car.reset_controller()
    car.execute_trajectory(trajectory)
    car.wait_while_running()

if Socket is not None:
#    Socket.sendall(b'-1')
    Socket.close()
