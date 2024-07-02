# aimotion_f1tenth_utils package

## Subpackages

* [aimotion_f1tenth_utils.communicaton package](aimotion_f1tenth_utils.communicaton.md)
  * [Submodules](aimotion_f1tenth_utils.communicaton.md#submodules)
  * [aimotion_f1tenth_utils.communicaton.TCPClient module](aimotion_f1tenth_utils.communicaton.md#module-aimotion_f1tenth_utils.communicaton.TCPClient)
    * [`TCPClient`](aimotion_f1tenth_utils.communicaton.md#aimotion_f1tenth_utils.communicaton.TCPClient.TCPClient)
      * [`TCPClient.close()`](aimotion_f1tenth_utils.communicaton.md#aimotion_f1tenth_utils.communicaton.TCPClient.TCPClient.close)
      * [`TCPClient.connect()`](aimotion_f1tenth_utils.communicaton.md#aimotion_f1tenth_utils.communicaton.TCPClient.TCPClient.connect)
      * [`TCPClient.send()`](aimotion_f1tenth_utils.communicaton.md#aimotion_f1tenth_utils.communicaton.TCPClient.TCPClient.send)
  * [aimotion_f1tenth_utils.communicaton.TCPServer module](aimotion_f1tenth_utils.communicaton.md#module-aimotion_f1tenth_utils.communicaton.TCPServer)
    * [`TCPServer`](aimotion_f1tenth_utils.communicaton.md#aimotion_f1tenth_utils.communicaton.TCPServer.TCPServer)
      * [`TCPServer.close_connection()`](aimotion_f1tenth_utils.communicaton.md#aimotion_f1tenth_utils.communicaton.TCPServer.TCPServer.close_connection)
      * [`TCPServer.handle_connection()`](aimotion_f1tenth_utils.communicaton.md#aimotion_f1tenth_utils.communicaton.TCPServer.TCPServer.handle_connection)
      * [`TCPServer.start()`](aimotion_f1tenth_utils.communicaton.md#aimotion_f1tenth_utils.communicaton.TCPServer.TCPServer.start)
      * [`TCPServer.stop()`](aimotion_f1tenth_utils.communicaton.md#aimotion_f1tenth_utils.communicaton.TCPServer.TCPServer.stop)
  * [Module contents](aimotion_f1tenth_utils.communicaton.md#module-aimotion_f1tenth_utils.communicaton)

## Submodules

## aimotion_f1tenth_utils.F1Client module

### *class* aimotion_f1tenth_utils.F1Client.F1Client(car_ID=None, host=None, port=8069)

Bases: `object`

#### GP_reset()

Reset the GP components of the GP-LPV-LQR controller

* **Raises:**
  **Exception** – If the reset failed

#### GP_to_online()

Switch the GP-LPV-LQR controller to online mode

* **Raises:**
  **Exception** – If the switch to online mode failed

#### GP_train(states=None, inputs=None, c=None, errors=None, random_seed=None, retrieve_training_data=False)

Train the GP components of the GP-LPV-LQR controller

* **Parameters:**
  * **states** (*np.ndarray*) – The states of the vehicle to be used for training (default is None to use the onboard logs)
  * **inputs** (*np.ndarray*) – The inputs of the vehicle to be used for training (default is None to use the onboard logs)
  * **c** (*np.ndarray*) – The curvature of the vehicle to be used for training (default is None to use the onboard logs)
  * **errors** (*np.ndarray*) – The errors of the vehicle to be used for training (default is None to use the onboard logs)
  * **random_seed** (*int*) – The random seed for the training (default is None)
  * **retrieve_training_data** (*bool*) – If the training data should be returned (default is False)

#### emergency_stop()

Immediately stops the vehicle, by interrupting the trajectory execution and setting the control inputs to zero.

* **Raises:**
  **Exception** – If the vehicle could not be stopped

#### execute_trajectory(trajectory: [Trajectory](#aimotion_f1tenth_utils.Trajectory.Trajectory))

Execute a trajectory on the vehicle

* **Parameters:**
  **trajectory** ([*Trajectory*](#aimotion_f1tenth_utils.Trajectory.Trajectory)) – The trajectory to be executed
* **Raises:**
  **Exception** – If the trajectory could not be executed

#### get_MPCC_horizon()

Get current solution through the optimization horizon
:return: states

#### get_MPCC_params()

Get the current parameter list of the acados optimiser.
:type: dict
:return: Current parameter dict

#### get_controllers()

Retrieves the available controllers of the vehicle

* **Returns:**
  The available controllers
* **Return type:**
  list[str]
* **Raises:**
  **Exception** – If the controllers could not be retrieved

#### get_latest_inputs()

Retrieve the latest inputs from the vehicle

* **Returns:**
  The latest inputs [d, delta]

#### get_logs()

Retrieve the logs from the vehicle

* **Returns:**
  The states, inputs, c (curvature) and errors
* **Return type:**
  tuple

#### get_mode()

Retrieves the current mode of the vehicle

* **Returns:**
  The current mode of the vehicle
* **Return type:**
  [CONTROLLER_MODE](#aimotion_f1tenth_utils.utils.CONTROLLER_MODE)
* **Raises:**
  **Exception** – If the mode could not be retrieved

#### keyboard_control(d_max: float = 0.075, delta_max: float = 0.5, frequency: float = 40)

Control the vehicle with the keyboard. This method is blocking as it opens a pygame window,
where WASD keys can be used to control the vehicle. Press SHIFT to increase the throttle.
The vehicle must be in MANUAL mode to accept manual control commands.
The onboard drive bridge will clamp the commands is necessary.

* **Parameters:**
  * **d_max** (*float*) – The maximum throttle command (default is 0.075)
  * **delta_max** (*float*) – The maximum steering command in radians (default is 0.5)
  * **frequency** (*float*) – The frequency of the control loop in Hz (default is 40)

#### manual_control(d: float, delta: float)

Sends manual control commands to the vehicle.
The vehicle must be in MANUAL mode to accept manual control commands.
The onboard drive bridge will clamp the commands is necessary.

* **Parameters:**
  * **d** (*float*) – The throttle command (-1: full reverse to 1:full forward, 0 is neutral)
  * **delta** (*float*) – The steering command in radians (-.5: full left to .5: full right, 0 is neutral)

#### reinit_GP_LPV_LQR(vehicle_params, GP_LPV_LQR_params)

Reinitialize the GP-LPV-LQR controller with new parameters

* **Parameters:**
  * **vehicle_params** (*dict*) – The vehicle parameters
  * **GP_LPV_LQR_params** (*dict*) – The GP-LPV-LQR parameters
* **Raises:**
  **Exception** – If the reinitialization failed

#### reinit_LPV_LQR_from_yaml(yaml_path)

Reinitialize the GP-LPV-LQR controller with parameters from a yaml file

* **Parameters:**
  **yaml_path** (*str*) – The path to the yaml file
* **Raises:**
  **Exception** – If the reinitialization failed

#### reset_controller()

Resets the internal state of the controller.

* **Raises:**
  **Exception** – If the controller could not be reset

#### reset_state_logger()

Resets the state logger on the vehicle

* **Raises:**
  **Exception** – If the logger could not be reset

#### select_controller(controller: str)

Secects the controller to be used

* **Parameters:**
  **controller** (*str*) – The name of the controller
* **Raises:**
  **Exception** – If the controller could not be set

#### set_MPCC_params(params: dict)

Set the parameters for the acados and casadi solvers.
:param params: A dictionary of the parameters

#### set_mode(mode: [CONTROLLER_MODE](#aimotion_f1tenth_utils.utils.CONTROLLER_MODE))

Set the mode of the vehicle defined by the CONTROLLER_MODE enum.
The modes are:
- RUNNING: the vehicle is executing a trajectory
- IDLE: controllers are ready and configured, but no trajectory is being executed
- STOP: interrupts all control and stops the car, then returns to idle mode
- PROCESSING: onboard calculation in progress, no control possible
- MANUAL: controllers are bypassed and the car can be controlled manually with manual_control() method

* **Parameters:**
  **mode** ([*CONTROLLER_MODE*](#aimotion_f1tenth_utils.utils.CONTROLLER_MODE)) – The mode to be set
* **Raises:**
  **Exception** – If the mode could not be set

#### wait_while_running()

Blocks the script while the vehicle is in RUNNING mode

## aimotion_f1tenth_utils.Trajectory module

### *class* aimotion_f1tenth_utils.Trajectory.Trajectory(trajectory_ID)

Bases: `object`

#### build_from_points_const_speed(path_points: ndarray, path_smoothing: float, path_degree: int, const_speed: float)

Builds a trajectory using a set of reference waypoints and a constant referecne velocity

* **Parameters:**
  * **path_points** (*np.ndarray*) – Reference points of the trajectory
  * **path_smoothing** (*float*) – Smoothing factor used for the spline interpolation
  * **path_degree** (*int*) – Degree of the fitted Spline
  * **const_speed** (*float*) – Constant speed of the vehicle

#### build_from_points_smooth_const_speed(path_points: ndarray, path_smoothing: float, path_degree: int, virtual_speed: float)

Builds a trajectory using a set of reference waypoints and a virtual reference velocity with smoothed start and end reference

* **Parameters:**
  * **path_points** (*np.ndarray*) – Reference points of the trajectory
  * **path_smoothing** (*float*) – Smoothing factor used for the spline interpolation
  * **path_degree** (*int*) – Degree of the fitted Spline
  * **virtual_speed** – The virtual constant speed for the vehicle. In practice the designer smoothens the start and end of the trajectory

#### build_from_waypoints(path_points: ndarray, speed_points: ndarray, path_smoothing: float, path_degree: int, dt: float = 0.01)

Builds a trajectory using a set of reference waypoints and a speed profile

* **Parameters:**
  * **path_points** (*np.ndarray*) – Reference points of the trajectory
  * **speed_points** (*np.ndarray*) – Speed profile of the vehicle
  * **path_smoothing** (*float*) – Smoothing factor used for the spline interpolation
  * **path_degree** (*int*) – Degree of the fitted Spline

#### draw_from_waypoints(x_lims=[-2.5, 2.5], y_lims=[-3, 3])

Opens a plot window and allows the user to draw a trajectory by clicking on the plot

* **Parameters:**
  * **x_lims** (*list*) – X-axis limits of the plot
  * **y_lims** (*list*) – Y-axis limits of the plot

#### evaluate(state, i, time, control_step)

Evaluates the trajectory based on the vehicle state & time

* **Parameters:**
  * **state** (*dict*) – Vehicle state
  * **i** (*int*) – Iterator valiable only used by the simulator
  * **time** (*float*) – Current time
  * **control_step** – he step of the controller

#### export_to_skybrush()

Exports the trajectory to a time dependent representation, i.e. x,y = Path(t)

Note: This representation is used for the Skybrush interface.

* **Returns:**
  The time dependent representation of the trajectory
* **Return type:**
  tuple

#### export_to_time_dependent()

Exports the trajectory to a time dependent representation, i.e. x,y = Path(t)

Note: This representation is used for the Skybrush interface.

* **Returns:**
  The time dependent representation of the trajectory
* **Return type:**
  tuple

#### get_trajectory()

Returns the evaluated trajectory data:
x, y : The x and y coordinates of the vehicle
v : The reference velocity
c : The curvature of the path

* **Returns:**
  The trajectory data (x,y,v,c)
* **Return type:**
  tuple

#### load(file_path)

Loads the trajectory class from .traj files

* **Parameters:**
  **file_path** (*str*) – The path to the file

#### plot_trajectory(block=True)

Plots, the defined path of the trajectory in the X-Y plane. Nota, that this function interrupts the main thread and the simulator!

* **Parameters:**
  **block** – If True, the function blocks the main thread until the plot is closed

#### save(dir_path)

Saves the trajectory file in a binary format at the predefined location
:param file_path: the path to sve the file
:type file_path: str

#### to_send()

Returns the trajectory data in a format that can be sent to the server

* **Returns:**
  The trajectory data
* **Return type:**
  tuple

## aimotion_f1tenth_utils.install module

### *class* aimotion_f1tenth_utils.install.FileTransporterSFTPClient(sock)

Bases: `SFTPClient`

Subclass of SFTPClient to achieve directory transport

#### mkdir(path, mode=511, ignore_existing=False)

Makes a directory on the remote host

* **Parameters:**
  * **path** (*str*) – The path of the directory to create
  * **mode** (*int*) – The permissions to set on the directory
  * **ignore_existing** (*bool*) – Whether to ignore if the directory already exists

#### put_dir(source, target)

Recursively upload a full directory structure

#### rmall(path)

Recursively remove a directory tree on the remote host

* **Parameters:**
  **path** (*str*) – The path of the directory to remove

### aimotion_f1tenth_utils.install.create_clients(IP_ADRESS, USERNAME, PASSWORD)

Helper function that creates the SSH and SFTP clients

* **Parameters:**
  * **IP_ADRESS** (*str*) – The IP adress of the machine
  * **USERNAME** (*str*) – The username of the machine
  * **PASSWORD** (*str*) – The password of the machine

### aimotion_f1tenth_utils.install.create_environment(ROS_MASTER_URI, IP_ADRESS, path)

Helper function that creates a unique env.sh file for every machine during installation.
The env.sh is later used to source the environment on remote launches.

* **Parameters:**
  * **ROS_MASTER_URI** (*str*) – The ROS master URI
  * **IP_ADRESS** (*str*) – The IP adress of the machine
  * **path** (*str*) – The path to the env.sh file

### aimotion_f1tenth_utils.install.install_onboard_stack(car_ID)

Installs the onboard stack on the vehicle

* **Parameters:**
  **car_ID** (*str*) – The ID of the vehicle

## aimotion_f1tenth_utils.logger module

### aimotion_f1tenth_utils.logger.get_logger()

## aimotion_f1tenth_utils.utils module

### *class* aimotion_f1tenth_utils.utils.CONTROLLER_MODE(value)

Bases: `Enum`

An enumeration.

#### IDLE *= 2*

#### MANUAL *= 5*

#### PROCESSING *= 4*

#### RUNNING *= 1*

#### STOP *= 3*

### *class* aimotion_f1tenth_utils.utils.TrajectoryBase

Bases: `ABC`

#### *abstract* evaluate(state: ndarray, t: float)

#### *abstract* reset_trajectory()

#### *abstract* set_trajectory(pos_tck: tuple, evol_tck: tuple, reversed: bool)

## Module contents
