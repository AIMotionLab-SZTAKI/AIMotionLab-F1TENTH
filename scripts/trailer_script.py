from aimotion_f1tenth_utils.F1Client import F1Client
from aimotion_f1tenth_utils.Trajectory import Trajectory
import os
import socket
import time
import motioncapture
import csv
from aimotion_f1tenth_utils.utils import CONTROLLER_MODE
import numpy as np


def quat_2_yaw(quat):
    x = quat[0]
    y = quat[1]
    z = quat[2]
    w = quat[3]
    siny_cosp = 2*(w*z + x*y)
    cosy_cosp = 1-2*(y*y + z*z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)
    return yaw


## TODO: load the trajectory into traj
traj = Trajectory("paperclip")
traj.load(os.path.join(os.path.dirname(__file__), "paperclip.traj"))

# SKYBRUSH PARAMS
skybrush_ip = "192.168.2.77"
skybrush_port = 6001

# establish connetion to manger

# create car object 
car_1 = F1Client("192.168.2.62", 8069)
print(f"Connected to {car_1.car_ID}")

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
            'k1_r': [-0.0008, 0.0442, -1.2247],
            'k2_r': [-0.0002,0.0191,-0.9531]
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
        'l_r': 0.168,
        'C_m1': 52.4282,
        'C_m2': 5.2465,
        'C_m3': 1.7,# increase for comparison
}

car_1.reinit_GP_LPV_LQR(vehicle_params=vehicle_params,
                        GP_LPV_LQR_params=GP_LPV_LQR_params)

# select the controller
car_1.select_controller("GP_LPV_LQR")
car_1.reset_state_logger()

# setup logging
mc = motioncapture.MotionCaptureOptitrack("192.168.2.141")
# Creating the log writer etc.
log_names = ['time_stamp_sec', 'car_x', 'car_y', 'car_heading', 'payload_x','payload_y', 'payload_heading','trailer_x','trailer_y',
             'trailer_heading']
log_files = os.listdir(os.path.join(os.path.dirname(__file__), "..", "logs"))
if not len(log_files):
    log_num = 1
else:
    log_num = max([int(cur_file.strip("log.csv")) for cur_file in log_files]) + 1
csv_file = open(os.path.join(os.path.dirname(__file__), "..", "logs", f"log{log_num}.csv"), 'w')
writer = csv.DictWriter(csv_file, fieldnames=log_names)
writer.writeheader()

# wait for skybrush signal
#try: # try to open demo port
#    skybrush_client_socket=socket.socket()
#    skybrush_client_socket.connect((skybrush_ip, skybrush_port))
#except Exception as e:
#    print(e)
#    break
#print("Waiting for Skybrush signal...")
#
#msg=skybrush_client_socket.recv(1024)
#skybrush_client_socket.close()
#
#print(f"Waiting {float(msg)} to launch!")
#time.sleep(float(msg))
#print("Trajectory execution started!")


# execute trajectory
res = car_1.execute_trajectory(trajectory=traj)

start_time = time.time()
print(f"Trajectory started at time {start_time:.3f}")

while car_1.get_mode() == CONTROLLER_MODE.RUNNING:
    # log
    mc.waitForNextFrame()
    payload  = mc.rigidBodies.get("payload2")
    if payload is not None:
        payload_heading = quat_2_yaw([payload.rotation.x, payload.rotation.y, payload.rotation.z, payload.rotation.w])
        payload_position = [payload.position[0], payload.position[1]]
    else:
        payload_heading = np.nan
        payload_position = [np.nan, np.nan]

    trailer  = mc.rigidBodies.get("trailer")
    if trailer is not None:
        trailer_heading = quat_2_yaw([trailer.rotation.x, trailer.rotation.y, trailer.rotation.z, trailer.rotation.w])
        trailer_position = [trailer.position[0], trailer.position[1]]
    else:
        trailer_heading = np.nan
        trailer_position = [np.nan, np.nan]

    car  = mc.rigidBodies.get("JoeBush1")
    if car is not None:
        car_heading = quat_2_yaw([car.rotation.x, car.rotation.y, car.rotation.z, car.rotation.w])
        car_position = [car.position[0], car.position[1]]
    else:
        car_heading = np.nan
        car_position = [np.nan, np.nan]
    
    writer.writerow({
        'time_stamp_sec': time.time()-start_time,
        'car_x': car_position[0],
        'car_y': car_position[1],
        'car_heading': car_heading,
        'payload_x':  payload_position[0],
        'payload_y':  payload_position[1],
        'payload_heading': payload_heading,
        'trailer_x':  trailer_position[0],
        'trailer_y':  trailer_position[1],
        'trailer_heading': trailer_heading
        })

csv_file.close()

print("Goodbye!")