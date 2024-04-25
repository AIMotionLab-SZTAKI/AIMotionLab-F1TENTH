from aimotion_f1tenth_utils.F1Client import F1Client
from aimotion_f1tenth_utils.Trajectory import Trajectory
import os
import socket
import time


## TODO: load the trajectory into traj
traj = Trajectory("paperclip")
traj.load(os.path.dirname(__file__)+"/"+"paperclip.traj")

# SKYBRUSH PARAMS
skyrush_ip = "192.168.2.77"
skybrush_port = 6001

# establish connetion to manger

# create car object 
car_1 = F1Client("192.168.2.62", 8069)
print(f"Connected to {car_1.car_ID}")


# select the controller
car_1.select_controller("GP_LPV_LQR")
car_1.reset_state_logger()

while True:

    # wait for skybrush signal
    #try: # try to open demo port
    #    skybrush_client_socket=socket.socket()
    #    skybrush_client_socket.connect((skyrush_ip, skybrush_port))
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
    car_1.wait_while_running()

    # ???
    break

print("Goodbye!")