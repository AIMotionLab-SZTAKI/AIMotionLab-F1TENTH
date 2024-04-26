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

#traj.plot_trajectory()


# connect to the vehicle
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
            'k1': [0.00127, -0.00864, 0.0192, 0.0159],
            'k2':  [0.172, -1.17, 2.59, 3.14],
            'k3':[0.00423, 0.0948, 0.463, 0.00936],
            'k1_r': [-0.0008, 0.0442, -1.2247],
            'k2_r': [-0.0002,0.0191,-0.9531]
            },
        "long_gains" :{
            'k1': [0.0001, -0.0014, 0.1908],
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
        'C_m3': 1.1194, # increase for comparison
}

# select the controller
#car_1.select_controller("GP_LPV_LQR")
car_1.reset_state_logger()

# execute_trajectory
car_1.execute_trajectory(trajectory=traj)

# block the script until the execution is finished
car_1.wait_while_running()

states1, inputs1, c1, errors1 = car_1.get_logs()

plt.figure()

plt.plot(x_r, y_r)
plt.plot(states1[:,0], states1[:,1])
plt.legend(["Reference", "Measurement1"])

fig, axs = plt.subplots(2, 1)
axs[0].plot(errors1)
plt.legend(["lateral", "heading", "long", "velocity"])
plt.show()