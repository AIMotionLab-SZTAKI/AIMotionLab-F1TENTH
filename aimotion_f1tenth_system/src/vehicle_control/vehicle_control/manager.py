from rclpy.node import Node
from vehicle_state_msgs.msg import VehicleStateStamped
from drive_bridge_msg.msg import InputValues
import numpy as np
import torch
import threading
from .utils import Controller, init_GP_LPV_LQR, CONTROLLER_MODE, Trajectory, state2array, StateLogger
from .controllers.utils import normalize, Controller
from .TCPServer import TCPServer

class ControlManager(Node):
    def __init__(self, car_ID: str, TCP_params: dict, **kwargs):
        """ControlManager class to manage the controllers and the communication
        
        :param car_ID: the ID of the car
        :type car_ID: str
        :param TCP_params: dict containing the TCP server parameters
        :type TCP_params: dict
        """
        super().__init__(car_ID + "_control_manager")
        self.car_ID : str = car_ID
        self.controllers : dict[Controller] = {}
        self.MODE = CONTROLLER_MODE.PROCESSING

        # setup tcp server for communication
        self.TCP_server = TCPServer(TCP_params["host"], TCP_params["port"], self._TCP_callback)
        self.TCP_thread = threading.Thread(target=self.TCP_server.start)
        self.TCP_thread.start()

        # check kwargs and initialize controllers
        if "GP_LPV_LQR_params" in kwargs:
            controller = init_GP_LPV_LQR(vehicle_params = kwargs["vehicle_params"],
                                         GP_LPV_LQR_params = kwargs["GP_LPV_LQR_params"])
            self.controllers["GP_LPV_LQR"] = controller

        # TODO: implement other comtroller initializations in utils and call it here

        # declare controllers
        self.active_controller: Controller = None
        self.current_trajectory = Trajectory()

        # logger
        self.state_logger = StateLogger()

        # init ros topics
        self._init_ros_topics()

        # finalize
        print(f"ControlManager initialized with controllers: {list(self.controllers.keys())}")
        self.MODE = CONTROLLER_MODE.IDLE

    def _TCP_callback(self, message: dict):
        """Callback function to handle incoming TCP messages
        
        :param message: Dictionary containing the message information
        :type message: dict
        :return: Dictionary containing the response
        :rtype: dict
        """
        # commands to handle
        # - select controller
        # - execute trajectory
        # - stop controller
        # - reset controller
        # - GP related: collect_data, train, mode: (online, offline, reset)
        cmd = message["command"]

        # identification
        if cmd == "?":
            return {"status": True, "car_ID": self.car_ID}

        # controller selection
        elif cmd == "select_controller":
            controller = message["controller"]
            if controller in self.controllers.keys():
                self.active_controller: Controller = self.controllers[controller]
                self.MODE = CONTROLLER_MODE.IDLE
                self.get_logger().info(f"Selected {controller}, waiting in IDLE mode!")
                return {"status": True}
            else:
                return {"status": False, "error": "Controller not found!"}
        
        # get available controllers
        elif cmd == "get_controllers":
            controllers = list(self.controllers.keys())
            return {"status": True, "controllers": controllers}

    
        
        # mode selection
        elif cmd == "set_mode":
            # cannot accept command if controller is processing
            if self.MODE == CONTROLLER_MODE.PROCESSING:
                return {"status": False, "error": "Controller is processing!"}
            
            mode = message["mode"]
            try:
                self.MODE = CONTROLLER_MODE(mode)
                self.get_logger().info(f"Switched to {self.MODE.name}")
                return {"status": True}
            except:
                return {"status": False, "error": "Invalid mode!"}
        
        # get mode
        elif cmd == "get_mode":
            return {"status": True, "mode": self.MODE.value}

        # manual control
        elif cmd == "manual_control":
            if self.MODE == CONTROLLER_MODE.MANUAL:
                # send control values
                self._manual_control(message["d"], message["delta"])
                return {"status": True}
            else: 
                return {"status": False, "error": "Not in manual mode!"}
        
        # execute trajectory
        elif cmd == "execute_trajectory":
            if self.MODE == CONTROLLER_MODE.IDLE:
                # execute the trajectory
                res, info = self._execute_trajectory(message["trajectory"])
                if res:
                    return {"status": True}
                else:
                    return {"status": False, "error": f"Trajectory execution failed: {info}"}
            else:
                return {"status": False, "error": "Not in idle mode!"}

        # reset controller
        elif cmd == "reset_controller":
            self.active_controller.reset()
            self.get_logger().info(f"Controller has been reset!")
            return {"status": True}
        
        # stop controller
        elif cmd == "stop_controller":
            self._stop()
            return {"status": True}
        
        # reset logger
        elif cmd == "reset_logger":
            self.state_logger.reset()
            self.get_logger().info("State logger has been reset!")
            return {"status": True}
        
        # get the logs
        elif cmd == "get_logs":
            self.get_logger().info("State logs requested!")
            return {"status": True,
                    "states": self.state_logger.get_states(),
                    "inputs": self.state_logger.get_inputs(),
                    "c": self.state_logger.get_c(),
                    "errors": self.state_logger.get_errors()
                    }
        
        # train the GP based on the dataset available in the stat logger
        elif cmd == "GP_train":
            self.MODE = CONTROLLER_MODE.PROCESSING
            try:
                if "seed" in message.keys():
                    torch.manual_seed(message["seed"])
                if message["data"] == None:
                    # retrieve the logs 
                    train_data = self.active_controller.train_GP_controllers(states=self.state_logger.get_states(),
                                                 inputs=self.state_logger.get_inputs(),
                                                 errors=self.state_logger.get_errors(), 
                                                 c=self.state_logger.get_c(),
                                                 plot_training_data=False)
                else:
                    # use the provided data
                    train_data = self.active_controller.train_GP_controllers(states=message["data"]["states"],
                                                 inputs=message["data"]["inputs"],
                                                 errors=message["data"]["errors"], 
                                                 c=message["data"]["c"],
                                                 plot_training_data=False)
                self.MODE = CONTROLLER_MODE.IDLE
                self.get_logger().info("GPs have been trained!")
                if message["return_training_data"]:
                    self.get_logger().info("Returning training dataset!")
                    return {"status": True, "training_data": train_data}
                else: return {"status": True}
            except Exception as e:
                self.MODE = CONTROLLER_MODE.IDLE
                return {"status": False, "error": f"Training failed: {e}"}
        
        # set the GP mode
        elif cmd == "reinit_GP_LPV_LQR":
            if self.MODE != CONTROLLER_MODE.IDLE:
                return {"status": False, "error": "Not in idle mode!"}
            try:
                self.controllers["GP_LPV_LQR"] = init_GP_LPV_LQR(vehicle_params = message["vehicle_params"],
                                                                 GP_LPV_LQR_params = message["GP_LPV_LQR_params"])
                self.active_controller = self.controllers["GP_LPV_LQR"]
                self.MODE = CONTROLLER_MODE.IDLE
                self.get_logger().info("GP LPV LQR controller reinitialized!")               
                return {"status": True}
            except Exception as e:
                return {"status": False, "error": f"Reinitialization failed: {e}"}

        elif cmd == "GP_to_online":
            try:
                self.active_controller.to_online()
                return {"status": True}
            except Exception as e:
                return {"status": False, "error": f"GP to online failed: {e}"}
            
        elif cmd == "GP_reset":
            try:
                self.active_controller.reset(GP_reset=True)
                self.get_logger().info("GPs hae been reset!")
                return {"status": True}
            except Exception as e:
                return {"status": False, "error": f"GP reset failed: {e}"}

        # get the latest input logs
        elif cmd == "get_latest_inputs":
            self.get_logger().info("Latest input logs requested!")
            return {"status": True,
                    "inputs": self.state_logger.get_latest_inputs()
                    }

        # unkown commands
        else:
            return {"status": False, "error": "Invalid command"}

    def _manual_control(self, d: float, delta: float):
        """Sends manual control commands to the drive bridge
        
        :param d: motor PWM value
        :type d: float
        :param delta: steering angle
        :type delta: float
        """
        self.pub.publish(InputValues(d = float(d), delta = float(delta)))

    def _init_ros_topics(self):
        """Initializes the ROS topics"""
        self.state_subscriber = self.create_subscription(VehicleStateStamped, self.car_ID+'_state', self._state_callback, 1)
        self.pub = self.create_publisher(InputValues, self.car_ID+'_control', 1)
        self.current_state = None

    def _stop(self):
        """Stops the controller and returns to idle mode"""
        self.MODE = CONTROLLER_MODE.STOP
        self.get_logger().info("Controller stop requested!")
        self.pub.publish(InputValues(d = 0.0, delta = 0.0))
        self.current_trajectory.reset_trajectory()
        try:
            self.active_controller.reset()
        except:
            pass # might be beneficial to give feedback
        self.MODE = CONTROLLER_MODE.IDLE

    def _execute_trajectory(self, trajectory: dict):
        """Executes the given trajectory if the controller is in idle mode
        
        :param trajectory: dict containing the trajectory information
        :return: (bool, str) indicating the success of the execution and an error message if any
        """

        self.get_logger().info("Trajectory execution requested!")
        if self.current_state is None:
            self.get_logger().warning("State estimation is not available, execution terminated!")
            return False, "No state estimation available!"

        # set trajectory
        self.current_trajectory.set_trajectory(trajectory["pos_tck"],
                                               trajectory["evol_tck"],
                                               trajectory["reversed"])

        # check if the trajectory is valid
        setpoint = self.current_trajectory.evaluate(self.current_state, 0)
        s0 = setpoint["s0"]
        z0 = setpoint["z0"]
        v = setpoint["v_ref"]
        ref_pos = setpoint["ref_pos"]

        current_pos = self.current_state[:2]
        theta_p = normalize(np.arctan2(s0[1], s0[0]))
        
        # if reversing motion is required invert heading
        if v < 0:
            current_heading = self.current_state[2] + np.pi
        else:
            current_heading = self.current_state[2]

        if abs(np.dot(current_pos-ref_pos, z0))>0.5 or abs(normalize(theta_p-normalize(current_heading)))>0.5:
            self.get_logger().warning("Too much deviation from trajectory reference, execution terminalted!")
            self.get_logger().info("Current position:   {0}, {1}".format(current_pos[0],current_pos[1]))
            self.get_logger().info("Reference position: {0}, {1}".format(ref_pos[0],ref_pos[1]))
            self.get_logger().info("Heading:            {0}".format(normalize(current_heading)))
            self.get_logger().info("Reference Heading:  {0}".format(theta_p))
            self.get_logger().info("Lateral error:      {0},  Heading error: {1}".format(abs(np.dot(current_pos-ref_pos, z0)),
                                                                                         normalize(theta_p-normalize(current_heading))))
            return False, "Too much deviation from trajectory reference!"
       
        if self.active_controller == None:
            self.active_controller = self.controllers["GP_LPV_LQR"]
            self.get_logger().info(f"No controller selected, defaulting to GP_LPV_LQR!") # TODO: this should not be hard coded

        self.t0 = 0 # get the current time
        self.t0 = self._get_time()
        #self.state_logger.reset()

        self.MODE = CONTROLLER_MODE.RUNNING
        self.get_logger().info("Prestart check passed, execution started!")
        return True, None


    def _is_running(self):
        """Check if the controller is running"""
        return self.MODE == CONTROLLER_MODE.RUNNING

    def _state_callback(self, data):
        #try:
        self.current_state = state2array(data)

        if not self._is_running(): return

        t = self._get_time()

        # get setpoint from the trajectory & evaluate the controller
        
        setpoint = self.current_trajectory.evaluate(self.current_state, t)
    
        # check if the goal is reached
        if not setpoint["running"]:
            self._stop()
            return
        try:
            u, errors, finished = self.active_controller.compute_control(self.current_state, setpoint)
            if finished:
                self._stop()
            self.state_logger.log_state(t,self.current_state, setpoint, errors, u)
            if self._is_running():
            # publish the control input
                self.pub.publish(InputValues(d = float(u[0]), delta = float(u[1])))
        
            print(f"e_lat: {errors[0]}, heading: {errors[1]}, position: {errors[2]}, q: {errors[4]}") 
    
        except Exception as e:
            self._logger.warning(str(e))
            self._stop()


        
    def _get_time(self):
        """Get the current time in seconds that is elapsed since t0"""
        t_tuple = self.get_clock().now().seconds_nanoseconds()
        time = float(t_tuple[0]) + float(t_tuple[1]) / 10**9 - self.t0
        return time