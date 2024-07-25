from aimotion_f1tenth_utils.Trajectory import Trajectory
from aimotion_f1tenth_utils.communicaton.TCPClient import TCPClient
from aimotion_f1tenth_utils.logger import get_logger
from aimotion_f1tenth_utils.utils import CONTROLLER_MODE

import pygame
import yaml
import time


import os
os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = "hide"  # Disable annoying welcome message :@



class F1Client:
    def __init__(self, car_ID= None, host = None, port = 8069) -> None:
        """TCP-based client for commuication with an F1TENTH vehicle
        
        :param car_ID: The ID of the vehicle
        :param host: IP address of the car
        :type host: str
        :param port: Port of the car
        :type port: int

        :raises Exception: If the connection to the car could not be established
        :raises Exception: If neither the ID nor the host and port are provided

        """
        if car_ID is None and (host is None or port is None):
            raise Exception("Either the car_ID or the host and port must be provided!")

        if car_ID is not None and host is None: # the car_ID is provided, read the host from the config file
            self.car_ID = car_ID
            config_folder = os.path.join(os.path.dirname(os.path.dirname(__file__)), "configs")
            login_config_path = os.path.join(config_folder, f"{car_ID}_login.yaml")

            with open(login_config_path, "r") as f:
                config_data = yaml.safe_load(f)
                self.host = config_data["IP"]

        elif car_ID is None and host is not None: # the host is provided, read the car_ID from the config file
            self.host = host

        else:
            raise Exception("Either the car_ID or the host and port must be provided!")
        
        
        self.port = port
        self.client = TCPClient()

        if not self.client.connect(self.host, self.port):
            raise Exception(f"Could not connect to {self.host}:{self.port}")
        
        if car_ID is None:
            self.car_ID = self._get_ID() # TODO: chak if the IDs match
       
        
    def _get_ID(self) -> str:
        """Get the ID of the vehicle
        
        :return: The ID of the vehicle
        :rtype: str
        :raises Exception: If the vehicle cannot be identified
        """
        info = self.client.send({"command": "?"})
        try:
            if info["status"] == True:
                return info["car_ID"]
            else: raise Exception(f"{info['error']}")
        except:
            raise Exception("Vehicle cannot be identified! Are you sure that you are trying to connect to an F1TENTH vehicle?")
        
    def select_controller(self, controller: str) -> None:
        """Secects the controller to be used
        
        :param controller: The name of the controller
        :type controller: str
        
        :raises Exception: If the controller could not be set
        """

        res = self.client.send({"command": "select_controller", "controller": controller})
        if res["status"] == False:
            raise Exception(f"Could not set controller: {res['error']}") 
        

    def get_state(self):
        """Get current state of the vehicle
        :return: state: np.array"""

        res = self.client.send({"command": "get_state"})

        if res["status"] == False:
            raise Exception(f"Could not get current state {res['error']}")
        return res["state"]


    def get_MPCC_horizon(self):
        """Get current solution through the optimization horizon
        :return: states"""
        
        res = self.client.send({"command": "MPCC_get_current_horizon"})

        
        if res["status"] == False:
            raise Exception(f"Could not get current solution horizon {res['error']}")
        return res["states"]

    def get_MPCC_params(self)->dict:
        """Get the current parameter list of the acados optimiser.
        :type: dict
        :return: Current parameter dict"""
        res = self.client.send({"command": "MPCC_param_get"})
        if res["status"] == False:
            raise Exception(f"Could not get the current parameters: {res['error']}")
        return res['MPCC_params']

    def set_MPCC_params(self, params: dict):
        """Set the parameters for the acados and casadi solvers.
        :param params: A dictionary of the parameters"""
        
        res = self.client.send({"command": "MPCC_param_update", "MPCC_params": params})

        if res["status"] == False:
            raise Exception(f"Could not set parameters: {res['error']}")

    def get_controllers(self) -> list[str]:
        """Retrieves the available controllers of the vehicle
        
        :return: The available controllers
        :rtype: list[str]
        :raises Exception: If the controllers could not be retrieved"""
        res = self.client.send({"command": "get_controllers"})
        if res["status"] == False:
            raise Exception("Could not set controller")
        return res["controllers"]
        
        
    def set_mode(self, mode: CONTROLLER_MODE) -> None:
        """Set the mode of the vehicle defined by the CONTROLLER_MODE enum.
        The modes are:
        - RUNNING: the vehicle is executing a trajectory
        - IDLE: controllers are ready and configured, but no trajectory is being executed
        - STOP: interrupts all control and stops the car, then returns to idle mode
        - PROCESSING: onboard calculation in progress, no control possible
        - MANUAL: controllers are bypassed and the car can be controlled manually with manual_control() method

        :param mode: The mode to be set
        :type mode: CONTROLLER_MODE
        :raises Exception: If the mode could not be set"""
        res = self.client.send({"command": "set_mode", "mode": mode.value})
        if res["status"] == False:
            raise Exception(f"Could not set mode: {res['error']}")


    def manual_control(self, d: float, delta: float) -> None:
        """Sends manual control commands to the vehicle.
        The vehicle must be in MANUAL mode to accept manual control commands.
        The onboard drive bridge will clamp the commands is necessary.
        
        :param d: The throttle command (-1: full reverse to 1:full forward, 0 is neutral)
        :type d: float
        :param delta: The steering command in radians (-.5: full left to .5: full right, 0 is neutral)
        :type delta: float
        """
        res = self.client.send({"command": "manual_control", "d": d, "delta": delta})
        if res["status"] == False:
            raise Exception(f"Could not set manual control: {res['error']}")

    def keyboard_control(self,
                         d_max: float = 0.075,
                         delta_max: float = 0.5,
                         frequency: float = 40) -> None:
        """Control the vehicle with the keyboard. This method is blocking as it opens a pygame window, 
        where WASD keys can be used to control the vehicle. Press SHIFT to increase the throttle.
        The vehicle must be in MANUAL mode to accept manual control commands.
        The onboard drive bridge will clamp the commands is necessary.

        :param d_max: The maximum throttle command (default is 0.075)
        :type d_max: float
        :param delta_max: The maximum steering command in radians (default is 0.5)
        :type delta_max: float
        :param frequency: The frequency of the control loop in Hz (default is 40)
        :type frequency: float
        """

        
        pygame.init()
        screen = pygame.display.set_mode((200, 20))
        pygame.display.set_caption("F1TENTH-control")
        font = pygame.font.SysFont(None, 16)  # You can choose any font and size you like
        text = font.render("Use WASD keys to control the car!", True, (0, 0, 0))  # Rendering text with black color

        running = True
        while running:

            d=0
            delta = 0
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False

            keys = pygame.key.get_pressed()
            if keys[pygame.K_ESCAPE]: break
            if keys[pygame.K_w]: d += d_max
            if keys[pygame.K_s]: d -= d_max
            if keys[pygame.K_a]: delta += delta_max
            if keys[pygame.K_d]: delta -= delta_max
            if keys[pygame.K_RSHIFT] or keys[pygame.K_LSHIFT]: d *= 1.5
            time.sleep(1/frequency)

            screen.fill((255, 255, 255))
            text_rect = text.get_rect()
            text_rect.center = (100, 10)
            screen.blit(text, text_rect)
            pygame.display.flip()
            pygame.event.pump()

            res = self.client.send({"command": "manual_control", "d": d, "delta": delta})
            if res["status"] == False:
                raise Exception(f"Could not set manual control: {res['error']}")
        
        pygame.quit()

    def execute_trajectory(self, trajectory: Trajectory) -> None:
        """Execute a trajectory on the vehicle

        :param trajectory: The trajectory to be executed
        :type trajectory: Trajectory
        :raises Exception: If the trajectory could not be executed
        """

        message = {"command": "execute_trajectory",
                   "trajectory":{
                       "pos_tck": trajectory.pos_tck,
                        "evol_tck": trajectory.evol_tck,
                        "reversed": trajectory.reversed}}
        
        res = self.client.send(message)
        if res["status"] == False:
            raise Exception(f"Could not execute trajectory: {res['error']}")

    def reset_controller(self) -> None:
        """Resets the internal state of the controller.

        :raises Exception: If the controller could not be reset 
        """
        res = self.client.send({"command": "reset_controller"})
        if res["status"] == False:
            raise Exception(f"Could not reset controller: {res['error']}")

    def emergency_stop(self):
        """
        Immediately stops the vehicle, by interrupting the trajectory execution and setting the control inputs to zero.

        :raises Exception: If the vehicle could not be stopped
        """
        res = self.client.send({"command": "stop_controller"})
        if res["status"] == False:
            raise Exception(f"Could not stop vehicle: {res['error']}")

    def get_logs(self) -> tuple:
        """Retrieve the logs from the vehicle
        
        :return: The states, inputs, c (curvature) and errors
        :rtype: tuple
        """
        res = self.client.send({"command": "get_logs"})
        if res["status"] == False:
            raise Exception(f"Cannot retrieve logs: {res['error']}")
        states = res["states"]
        inputs = res["inputs"]
        c = res["c"]
        errors = res["errors"]
        return states, inputs, c, errors
    
    def get_latest_inputs(self) -> list:
        """Retrieve the latest inputs from the vehicle
    
        :return: The latest inputs [d, delta]
        """
        res = self.client.send({"command": "get_latest_inputs"})
        if res["status"] == False:
            raise Exception(f"Cannot retrieve logs: {res['error']}")
        inputs = res["inputs"]
        return inputs
    
    def reset_state_logger(self) -> None:
        """Resets the state logger on the vehicle
        
        :raises Exception: If the logger could not be reset
        """
        res = self.client.send({"command": "reset_logger"})
        if res["status"] == False:
            raise Exception(f"Cannnot reset logger: {res['error']}")


    def get_mode(self) -> CONTROLLER_MODE:
        """
        Retrieves the current mode of the vehicle
        
        :return: The current mode of the vehicle
        :rtype: CONTROLLER_MODE

        :raises Exception: If the mode could not be retrieved
        """
        res = self.client.send({"command": "get_mode"})
        if res["status"] == False:
            raise Exception(f"Cannnot get mode: {res['error']}")
        return CONTROLLER_MODE(res["mode"])


    def wait_while_running(self) -> None:
        """
        Blocks the script while the vehicle is in RUNNING mode
        """
        running = True
        while running:
            if self.get_mode() == CONTROLLER_MODE.RUNNING:
                time.sleep(1/10)
            else:
                running = False


    def reinit_GP_LPV_LQR(self, vehicle_params, GP_LPV_LQR_params) -> None:
        """Reinitialize the GP-LPV-LQR controller with new parameters
        
        :param vehicle_params: The vehicle parameters
        :type vehicle_params: dict
        :param GP_LPV_LQR_params: The GP-LPV-LQR parameters
        :type GP_LPV_LQR_params: dict
        
        :raises Exception: If the reinitialization failed
        """
        res = self.client.send({"command": "reinit_GP_LPV_LQR", 
                                "GP_LPV_LQR_params": GP_LPV_LQR_params,
                                "vehicle_params": vehicle_params})
        if res["status"] == False:
            raise Exception(f"Reinitialization failed: {res['error']}")


    def reinit_LPV_LQR_from_yaml(self, yaml_path):
        """Reinitialize the GP-LPV-LQR controller with parameters from a yaml file

        :param yaml_path: The path to the yaml file
        :type yaml_path: str

        :raises Exception: If the reinitialization failed
        """

        # load yaml into dict
        with open(yaml_path, "r") as f:
            params = yaml.safe_load(f)
            res = self.client.send({"command": "reinit_GP_LPV_LQR",
                                "GP_LPV_LQR_params": params["GP_LPV_LQR_params"],
                                "vehicle_params": params["vehicle_params"]})
        if res["status"] == False:
            raise Exception(f"Reinitialization failed: {res['error']}")



    def GP_train(self,
                 states = None,
                 inputs = None,
                 c = None,
                 errors = None,
                 random_seed = None,
                 retrieve_training_data = False) -> tuple:
        """Train the GP components of the GP-LPV-LQR controller
        
        :param states: The states of the vehicle to be used for training (default is None to use the onboard logs)
        :type states: np.ndarray
        :param inputs: The inputs of the vehicle to be used for training (default is None to use the onboard logs)
        :type inputs: np.ndarray
        :param c: The curvature of the vehicle to be used for training (default is None to use the onboard logs)
        :type c: np.ndarray
        :param errors: The errors of the vehicle to be used for training (default is None to use the onboard logs)
        :type errors: np.ndarray
        :param random_seed: The random seed for the training (default is None)
        :type random_seed: int
        :param retrieve_training_data: If the training data should be returned (default is False)
        :type retrieve_training_data: bool
        """
        if states is not None and inputs is not None and c is not None and errors is not None:
            # TODO: check if all arrays have the same length
            res = self.client.send({"command": "GP_train", 
                                    "data": {"states": states, 
                                             "inputs": inputs,
                                             "c": c,
                                             "errors": errors},
                                    "seed": random_seed, 
                                    "return_training_data": retrieve_training_data})
        else:
            res = self.client.send({"command": "GP_train",
                                    "data": None,
                                    "random_seed": random_seed,
                                    "return_training_data": retrieve_training_data})
        if res["status"] == False:
            raise Exception(f"Training failed: {res['error']}")
        
        if retrieve_training_data:
            return res["training_data"] # ((lat_x, lat_y), (long_x, long_y))
        else: 
            return None
        
    def GP_reset(self) -> None:
        """Reset the GP components of the GP-LPV-LQR controller
        
        :raises Exception: If the reset failed
        """
        res = self.client.send({"command": "GP_reset"})
        if res["status"] == False:
            raise Exception(f"Could not reset GP: {res['error']}")
        
    def GP_to_online(self) -> None:
        """Switch the GP-LPV-LQR controller to online mode
        
        :raises Exception: If the switch to online mode failed"""
        res = self.client.send({"command": "GP_to_online"})
        if res["status"] == False:
            raise Exception(f"Could not switch to online mode: {res['error']}")