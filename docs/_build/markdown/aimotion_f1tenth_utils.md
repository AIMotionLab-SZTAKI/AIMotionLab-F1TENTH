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

## aimotion_f1tenth_utils.F1Client module

### *class* aimotion_f1tenth_utils.F1Client.Connection(host, port)

Bases: `object`

#### send(message)

Function that sends a message to the server and returns the recieved response

* **Parameters:**
  **message** (*dict*) – Message to be sent to the server. The message should be encoded in a dictinary format
* **Returns:**
  Response from the server
* **Return type:**
  dict

#### upload_choreography(choreography)

Uploads a choreography to the server

* **Parameters:**
  **choreography** (*Choreography*) – Choreography object to be uploaded
* **Returns:**
  Status of the upload
* **Return type:**
  bool

#### upload_trajectory(trajectory: [Trajectory](#aimotion_f1tenth_utils.Trajectory.Trajectory))

Uploads a trajectory to the server

* **Parameters:**
  **trajectory** ([*Trajectory*](#aimotion_f1tenth_utils.Trajectory.Trajectory)) – Trajectory object to be uploaded
* **Returns:**
  Status of the upload
* **Return type:**
  bool

#### verify_vehicle(car_ID)

Verifies if a vehicle exists in the server side application with the given car_ID

* **Parameters:**
  **car_ID** (*str*) – ID of the vehicle
* **Returns:**
  Verification status
* **Return type:**
  bool

### *class* aimotion_f1tenth_utils.F1Client.F1TENTH(car_ID, connection: [Connection](#aimotion_f1tenth_utils.F1Client.Connection))

Bases: `object`

#### execute_trajectory(trajectory_ID, trajectory_data=None, block: bool = False)

Executes a trajectory on the vehicle.

* **Parameters:**
  * **trajectory_ID** (*str*) – ID of the trajectory to be executed
  * **trajectory_data** (*list* *,* *optional*) – Optional trajectory data to be executed. If the trajectory is already saved on the server, this parameter can be omitted.
  * **block** (*bool* *,* *optional*) – Wait until the execution is done
* **Returns:**
  Status of the execution
* **Return type:**
  bool

#### get_current_progress()

Gets the current progress of the trajectory execution

* **Returns:**
  Current progress value in percentage
* **Return type:**
  int

#### get_logs(target_path=None)

Gets the log files for the vehicle and saves to the dedicated location

* **Parameters:**
  **target_path** (*str*) – Path to save the log files
* **Returns:**
  Status of the log retrieval

#### get_state()

Requests the current state of the vehicle from the server

* **Returns:**
  State of the vehicle
* **Return type:**
  dict
* **Raises:**
  **Exception** – If the state of the vehicle could not be retrieved

#### toggle_logging(start: bool)

Switches the ros2 node’s logging status variable from True to False or the other way

* **Parameters:**
  **start** (*bool*) – Status of the logging
* **Returns:**
  Status of the logging

#### toggle_radio_active(ON: bool)

Turns on the vehicles radio communication to stream mocap data for state estimation

* **Parameters:**
  **ON** (*bool*) – Status of the radio communication
* **Returns:**
  Status of the activation

#### toggle_save()

Saves current log file for the vehicle and starts new log file
Turns off logging for vehicle=> logging status must be set back to True

* **Returns:**
  Status of the save

#### wait_until_done()

## aimotion_f1tenth_utils.Trajectory module

### *class* aimotion_f1tenth_utils.Trajectory.ScheduledTrajectory(trajectory_ID, t_start)

Bases: [`Trajectory`](#aimotion_f1tenth_utils.Trajectory.Trajectory)

### *class* aimotion_f1tenth_utils.Trajectory.Trajectory(trajectory_ID)

Bases: `object`

#### build_from_points_const_speed(path_points: ndarray, path_smoothing: float, path_degree: int, const_speed: float)

Object responsible for storing the reference trajectory data.

* **Parameters:**
  * **path_points** (*np.ndarray*) – Reference points of the trajectory
  * **path_smoothing** (*float*) – Smoothing factor used for the spline interpolation
  * **path_degree** (*int*) – Degree of the fitted Spline
  * **const_speed** (*float*) – Constant speed of the vehicle

#### evaluate(state, i, time, control_step)

Evaluates the trajectory based on the vehicle state & time

* **Parameters:**
  * **state** (*dict*) – Vehicle state
  * **i** (*int*) – Iterator valiable only used by the simulator
  * **time** (*float*) – Current time
  * **control_step** – he step of the controller

#### export_to_time_dependent()

Exports the trajectory to a time dependent representation

#### plot_trajectory(block=True)

Plots, the defined path of the trajectory in the X-Y plane. Nota, that this function interrupts the main thread and the simulator!

* **Parameters:**
  **block** – If True, the function blocks the main thread until the plot is closed

#### to_send()

Returns the trajectory data in a format that can be sent to the server

## aimotion_f1tenth_utils.logger module

### aimotion_f1tenth_utils.logger.get_logger()
