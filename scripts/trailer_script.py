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

def paperclip():
    focus_x = [0, 0]
    focus_y = [-0.7, 0.7]
    r = 0.8
    len_straight = focus_y[1] - focus_y[0]
    len_turn = r * np.pi
    ppm = 6
    num_straight = int(len_straight * ppm)
    num_turn = int(len_turn * ppm)
    x = np.hstack((np.linspace(focus_x[0] + r, focus_x[1] + r, num_straight),
                   focus_x[1] + r * np.cos(np.linspace(0, np.pi, num_turn)),
                   np.linspace(focus_x[1] - r, focus_x[0] - r, num_straight),
                   focus_x[0] + r * np.cos(np.linspace(np.pi, 2*np.pi, num_turn))
                   ))
    y = np.hstack((np.linspace(focus_y[0], focus_y[1], num_straight),
                   focus_y[1] + r * np.sin(np.linspace(0, np.pi, num_turn)),
                   np.linspace(focus_y[1], focus_y[0], num_straight),
                   focus_y[0] + r * np.sin(np.linspace(np.pi, 2*np.pi, num_turn))
                   ))
    x = np.roll(x, 6)
    y = np.roll(y, 6)
    points = np.array([[x_, y_] for x_, y_ in zip(x, y)])
    delete_idx = []
    for i, point in enumerate(points):
        if i > 0:
            if np.linalg.norm(point - points[i-1, :]) < 0.1:
                delete_idx += [i]
    points = np.delete(points, delete_idx, 0)
    return points

def dented_paperclip():
    focus_x = [0, 0]
    focus_y = [-1, 1]
    r = 0.8
    len_straight = focus_y[1] - focus_y[0]
    len_turn = r * np.pi
    r_dent = 0.2
    len_dent = cosine_arc_length(r_dent, 2*np.pi/len_straight, 0, len_straight)
    ppm = 4
    num_straight = int(len_straight * ppm)
    num_turn = int(len_turn * ppm)
    num_dent = int(len_dent * ppm)
    x = np.hstack((np.linspace(focus_x[1] + r, focus_x[0] + r, num_straight),
                   focus_x[1] + r * np.cos(np.linspace(0, np.pi, num_turn)),
                   -r + r_dent - r_dent * np.cos(np.linspace(0, 2*np.pi, num_dent)),
                   focus_x[0] + r * np.cos(np.linspace(np.pi, 2*np.pi, num_turn))
                   ))
    y = np.hstack((np.linspace(focus_y[0], focus_y[1], num_dent),
                   focus_y[1] + r * np.sin(np.linspace(0, np.pi, num_turn)),
                   np.linspace(focus_y[1], focus_y[0], num_straight),
                   focus_y[0] + r * np.sin(np.linspace(np.pi, 2*np.pi, num_turn))
                   ))
    x = np.roll(x, -15)
    y = np.roll(y, -15)
    points = np.array([[x_, y_] for x_, y_ in zip(x, y)])
    delete_idx = []
    for i, point in enumerate(points):
        if i > 0:
            if np.linalg.norm(point - points[i-1, :]) < 0.1:
                delete_idx += [i]
    points = np.delete(points, delete_idx, 0)
    return points

def cosine_arc_length(amplitude, frequency, start, end):
    # Define the derivative of the cosine function
    def derivative_cos(x):
        return -amplitude * frequency * np.sin(frequency * x)

    # Define the integrand
    def integrand(x):
        return np.sqrt(1 + derivative_cos(x) ** 2)

    # Integrate the integrand function using scipy's quad function
    from scipy.integrate import quad
    arc_length, _ = quad(integrand, start, end)

    return arc_length


traj = Trajectory("paperclip")
traj.load(os.path.join(os.path.dirname(__file__), "paperclip.traj"))
#traj.plot_trajectory(True)
""" # Generate and save new trajectory
traj = Trajectory("paperclip_reversed")
path_points = np.roll(paperclip(), shift=13, axis=0)[::-1, :]
traj.build_from_points_const_speed(path_points=path_points, path_smoothing=1e-4, path_degree=5, const_speed=0.6)
traj.save("")
# traj.plot_trajectory(True)
# exit()
"""

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

#car_1.reinit_GP_LPV_LQR(vehicle_params=vehicle_params,
#                        GP_LPV_LQR_params=GP_LPV_LQR_params)

car_1.reinit_LPV_LQR_from_yaml(os.path.join(os.path.dirname(__file__), "trailer_control_params.yaml"))


# select the controller
car_1.select_controller("GP_LPV_LQR")
car_1.reset_state_logger()

# setup logging
mc = motioncapture.MotionCaptureOptitrack("192.168.2.141")
# Creating the log writer etc.
log_names = ['time_stamp_sec', 'car_x', 'car_y', 'car_heading', 'payload_x','payload_y', 'payload_heading','trailer_x','trailer_y',
             'trailer_heading', 'car_vx', 'car_vy', 'car_ang_vel', 'payload_vx', 'payload_vy', 'payload_ang_vel', 
             'trailer_vx', 'trailer_vy', 'trailer_ang_vel']
log_files = os.listdir(os.path.join(os.path.dirname(__file__), "..", "logs"))
if not len(log_files):
    log_num = 1
else:
    log_num = max([int(cur_file.strip("log.csv")) for cur_file in log_files]) + 1
csv_file = open(os.path.join(os.path.dirname(__file__), "..", "logs", f"log{log_num}.csv"), 'w')
writer = csv.DictWriter(csv_file, fieldnames=log_names)
writer.writeheader()

# Initialize velocity variables and moving average coefficient
state_estimator_alpha = 0.3

payload_velocity = [0.0, 0.0]
trailer_velocity = [0.0, 0.0]
car_velocity = [0.0, 0.0]
payload_ang_vel = 0.0
trailer_ang_vel = 0.0
car_ang_vel = 0.0

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

# execute trajectory
res = car_1.execute_trajectory(trajectory=traj)

start_time = time.time()
timestamp = None

while car_1.get_mode() == CONTROLLER_MODE.RUNNING:
    # log
    mc.waitForNextFrame()
    timestamp_new = time.time() - start_time

    payload = mc.rigidBodies.get("payload2")
    if payload is not None:
        payload_heading_new = quat_2_yaw([payload.rotation.x, payload.rotation.y, payload.rotation.z, payload.rotation.w])
        payload_position_new = [payload.position[0], payload.position[1]]
    else:
        payload_heading_new = np.nan
        payload_position_new = [np.nan, np.nan]

    trailer = mc.rigidBodies.get("trailer")
    if trailer is not None:
        trailer_heading_new = quat_2_yaw([trailer.rotation.x, trailer.rotation.y, trailer.rotation.z, trailer.rotation.w])
        trailer_position_new = [trailer.position[0], trailer.position[1]]
    else:
        trailer_heading_new = np.nan
        trailer_position_new = [np.nan, np.nan]

    car = mc.rigidBodies.get("JoeBush1")
    if car is not None:
        car_heading_new = quat_2_yaw([car.rotation.x, car.rotation.y, car.rotation.z, car.rotation.w])
        car_position_new = [car.position[0], car.position[1]]
    else:
        car_heading_new = np.nan
        car_position_new = [np.nan, np.nan]

    if timestamp is not None:
        # Update velicities only if positions have been updated at least once
        dt = timestamp_new - timestamp
        heading_diff = payload_heading_new - payload_heading
        while heading_diff > np.pi:
            heading_diff -= 2 * np.pi
        while heading_diff < -np.pi:
            heading_diff += 2 * np.pi
        payload_ang_vel += state_estimator_alpha * (heading_diff / dt - payload_ang_vel)

        heading_diff = trailer_heading_new - trailer_heading
        while heading_diff > np.pi:
            heading_diff -= 2 * np.pi
        while heading_diff < -np.pi:
            heading_diff += 2 * np.pi
        trailer_ang_vel += state_estimator_alpha * (heading_diff / dt - trailer_ang_vel)

        heading_diff = car_heading_new - car_heading
        while heading_diff > np.pi:
            heading_diff -= 2 * np.pi
        while heading_diff < -np.pi:
            heading_diff += 2 * np.pi
        car_ang_vel += state_estimator_alpha * (heading_diff / dt - car_ang_vel)

        payload_velocity = [vel + state_estimator_alpha * ((pos_new - pos) / dt - vel) for 
                            vel, pos_new, pos in zip(payload_velocity, payload_position_new, payload_position)]
        trailer_velocity = [vel + state_estimator_alpha * ((pos_new - pos) / dt - vel) for 
                            vel, pos_new, pos in zip(trailer_velocity, trailer_position_new, trailer_position)]
        car_velocity = [vel + state_estimator_alpha * ((pos_new - pos) / dt - vel) for 
                            vel, pos_new, pos in zip(car_velocity, car_position_new, car_position)]
    
    payload_position = payload_position_new.copy()
    trailer_position = trailer_position_new.copy()
    car_position = car_position_new.copy()
    payload_heading = payload_heading_new
    trailer_heading = trailer_heading_new
    car_heading = car_heading_new
    timestamp = timestamp_new

    writer.writerow({
        'time_stamp_sec': timestamp,
        'car_x': car_position[0],
        'car_y': car_position[1],
        'car_heading': car_heading,
        'payload_x':  payload_position[0],
        'payload_y':  payload_position[1],
        'payload_heading': payload_heading,
        'trailer_x':  trailer_position[0],
        'trailer_y':  trailer_position[1],
        'trailer_heading': trailer_heading,
        'car_vx': car_velocity[0],
        'car_vy': car_velocity[1],
        'car_ang_vel': car_ang_vel,
        'payload_vx': payload_velocity[0],
        'payload_vy': payload_velocity[1],
        'payload_ang_vel': payload_ang_vel,
        'trailer_vx': trailer_velocity[0],
        'trailer_vy': trailer_velocity[1],
        'trailer_ang_vel': trailer_ang_vel
        })

csv_file.close()

print(f"Log saved to log{log_num}.csv")