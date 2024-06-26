# Control the vehicles by the Python API

This page outlines the TCP-based communication protocol and the Python API that can be used for controlling the vehicles.

## Communication protocol

The onboard stack of the F1TENTH vehicles runs a TCP server that listens for incoming connections on port 8000.
The server expects to recieve packets of the following format:

- 4 bytes: The length prefix that indicates the length of the payload. The prefix can be encoded as
  > ```python
  > struct.pack('I', len(payload))
  > ```
- N bytes: The payload, which is a serialized Python dictonary which can be constructed as
  > ```python
  > payload = {'key1': value1, 'key2': value2, ...}
  > payload = pickle.dumps(payload)
  > ```

The server will respond with a serialized Python dictionary that contains the response to the request. The standard format of a request sent by the client is:

> ```python
> request = {'command': 'command_name', "arg1": value1, "arg2": value2, ...}
> ```

where the command_name is the name of the command that the client wants to execute.
The full list of available commands can be found in the automatically generated code documentation of the F1Client class of the API.
The server will respond with a dictionary that contains the key status with a boolean value which indicates the status of the command execution.
If the command failed, the value of the status key will be False and the dictionary will contain an additional key error that contains the error message.
The comminication interface is implemented in the [aimotion_f1tenth_utils.communicaton package](modules/aimotion_f1tenth_utils.communicaton.md) module.

## The Python API

The Python API of the aimotion_f1tenth_utils package provides a high-level interface for controlling the vehicles.
It automatically handles the communication with the vehicle (i.e serializes the messages, retrives and handles the responses)
and provides a set of functions that can be used to control the vehicle. The automatically generated documentation of the API can be found [aimotion_f1tenth_utils package](modules/aimotion_f1tenth_utils.md)

## Examples

Finally, the following examples present the main functionalities of the API. The examples assume that the vehicles are running and listening for incoming connections on port 8000.
The scripts can also be found in the examples directory of the package.

### Installing the onboard stack of the vehicles

```python
from aimotion_f1tenth_utils.install import install_onboard_stack

# define the ID of the car
car_ID = 'JoeBush1'

# before running make sure that the vehicle
# is turned on and connected to the network
install_onboard_stack(car_ID)
```

### Manual control by the keyboard

```python
from aimotion_f1tenth_utils.F1Client import F1Client
from aimotion_f1tenth_utils.utils import CONTROLLER_MODE


# connect to the vehicle
car_1 = F1Client(car_ID="JoeBush1") # (host="192.168.2.62", port=8069) is also possible

# set the mode to manual
car_1.set_mode(mode = CONTROLLER_MODE.MANUAL)

# control the vehicle using the keyboard
car_1.keyboard_control(d_max=.2, delta_max=.4)

# reset mode
car_1.set_mode(mode = CONTROLLER_MODE.IDLE)
```

### Execute a presaved trajectory

```python
import os
import matplotlib.pyplot as plt
import pickle

from aimotion_f1tenth_utils.F1Client import F1Client
from aimotion_f1tenth_utils.utils import CONTROLLER_MODE
from aimotion_f1tenth_utils.Trajectory import Trajectory


# Design / load the trajectory
traj_ID = "traj_1"
traj = Trajectory(trajectory_ID=traj_ID) # create the trajetory object
traj.load(os.path.join(os.path.dirname(__file__) , traj_ID+".traj"))

# display the trajectory in a figure
traj.plot_trajectory(block=False)

# connect to the vehicle
car_1 = F1Client(host = "192.168.2.62", port = 8069)
print(f"Connected to {car_1.car_ID}")


# select the controller
car_1.select_controller("GP_LPV_LQR")

# reset the internal state of the controller
car_1.reset_controller()

# reset the state logger
car_1.reset_state_logger()

# execute_trajectory
car_1.execute_trajectory(trajectory=traj)

# block the script until the execution is finished
car_1.wait_while_running()


# retrieve logs
states1, inputs1, c1, errors1 = car_1.get_logs()

# plot the logs
plt.figure()
x_r, y_r, *_ = traj.get_trajectory()
plt.plot(x_r, y_r)
plt.plot(states1[:,0], states1[:,1])
plt.legend(["Reference", "Measurement1"])

plt.figure()
plt.plot(errors1)
plt.legend(["Lateral", "Heading", "Longitudinal", "Velocity"])

plt.figure()
plt.plot(inputs1)
plt.legend(["d", "detla"])

# display
plt.show()
```
