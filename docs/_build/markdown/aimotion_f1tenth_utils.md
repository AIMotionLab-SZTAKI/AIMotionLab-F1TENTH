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

#### upload_action(action)

#### upload_trajectory(trajectory: [Trajectory](#aimotion_f1tenth_utils.Trajectory.Trajectory))

#### verify_vehicle(car_ID)

### *class* aimotion_f1tenth_utils.F1Client.F1TENTH(car_ID, connection: [Connection](#aimotion_f1tenth_utils.F1Client.Connection))

Bases: `object`

#### execute_trajectory(trajectory_ID, trajectory_data=None)

#### get_logs(target_path=None)

sends the command with the car_ID
receives all the logs of a certain vehicle
saves them to /logs

#### get_state()

#### toggle_logging(start: bool)

Switches the ros2 node’s logging status variable from True to False or the other way
Args:
- start:

> - True: turns logging on
> - False: turns logging off

#### toggle_radio_active(ON: bool)

Turns on the vehicle’s radio and echo function in the main_UI
Args: 
- On:

> - True-> turn vehicle on
> - False-> turn vehicle off

#### toggle_save()

Saves current log file for the vehicle
Starts new log file
Turns off logging for vehicle=> logging status must be set back to True

## aimotion_f1tenth_utils.Trajectory module

### *class* aimotion_f1tenth_utils.Trajectory.Trajectory(trajectory_ID)

Bases: `object`

#### build_from_points_const_speed(path_points: ndarray, path_smoothing: float, path_degree: int, const_speed: float)

Object responsible for storing the reference trajectory data.

Args:
: path_points (numpy.ndarray): Reference points of the trajectory
  smoothing (float): Smoothing factor used for the spline interpolation
  degree (int): Degree of the fitted Spline

#### evaluate(state, i, time, control_step)

Evaluates the trajectory based on the vehicle state & time

#### export_to_time_dependent()

Exports the trajectory to a time dependent representation

#### plot_trajectory(block=True)

Plots, the defined path of the trajectory in the X-Y plane. Nota, that this function interrupts the main thread and the simulator!

#### to_send()

## aimotion_f1tenth_utils.logger module

### aimotion_f1tenth_utils.logger.get_logger()
