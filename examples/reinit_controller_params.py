import os
from aimotion_f1tenth_utils.F1Client import F1Client
from aimotion_f1tenth_utils.utils import CONTROLLER_MODE
from aimotion_f1tenth_utils.Trajectory import Trajectory
import matplotlib as mpl
import os
import matplotlib.pyplot as plt

mpl.rcParams["text.usetex"] = False 

# Design / load the trajectory
traj_ID = "traj_1"
traj = Trajectory(trajectory_ID=traj_ID) # create the trajetory object
traj.load(os.path.dirname(__file__)+"/"+traj_ID+".traj")
x_r , y_r = traj.get_trajectory()

# connect to the vehicle
car_1 = F1Client("192.168.2.62", 8069)
print(f"Connected to {car_1.car_ID}")

# select the controller
car_1.select_controller("GP_LPV_LQR")
car_1.reset_state_logger()

# execute_trajectory
car_1.execute_trajectory(trajectory=traj)

# block the script until the execution is finished
car_1.wait_while_running()

states1, inputs1, c1, errors1 = car_1.get_logs()

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
        'C_m3': 2.9194,# increase for comparison
}


car_1.reinit_GP_LPV_LQR(vehicle_params=vehicle_params,
                        GP_LPV_LQR_params=GP_LPV_LQR_params)

# select the controller
car_1.select_controller("GP_LPV_LQR")
car_1.reset_state_logger()

# execute_trajectory
car_1.execute_trajectory(trajectory=traj)

# block the script until the execution is finished
car_1.wait_while_running()

states2, inputs2, c2, errors2 = car_1.get_logs()

plt.figure()

plt.plot(x_r, y_r)
plt.plot(states1[:,0], states1[:,1])
plt.plot(states2[:,0], states2[:,1])
plt.legend(["Reference", "Measurement1", "Measurement2"])

fig, axs = plt.subplots(2, 1)
axs[0].plot(errors1)
axs[1].plot(errors2)
plt.legend(["lateral", "heading", "long", "velocity"])
plt.show()