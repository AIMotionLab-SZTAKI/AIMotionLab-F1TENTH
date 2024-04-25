from aimotion_f1tenth_utils.Trajectory import Trajectory
from aimotion_f1tenth_utils.communicaton.TCPClient import TCPClient
from aimotion_f1tenth_utils.logger import get_logger
from aimotion_f1tenth_utils.utils import CONTROLLER_MODE

import pygame
import socket
import pickle
import struct
import yaml

import os
import time


class F1Client:
    def __init__(self, host, port) -> None:
        """TCP-based client for commuication with an F1TENTH vehicle
        
        :param host: IP address of the car
        :type host: str
        :param port: Port of the car
        :type port: int

        :raises Exception: If the connection to the car could not be established

        """
        self.host = host
        self.port = port
        self.client = TCPClient()

        if not self.client.connect(host, port):
            raise Exception(f"Could not connect to {host}:{port}")
        
        self.car_ID = self._get_ID()
       
        
    def _get_ID(self):
        info = self.client.send({"command": "?"})
        try:
            if info["status"] == True:
                return info["car_ID"]
            else: raise Exception(f"{info['error']}")
        except:
            raise Exception("Vehicle cannot be identified! Are you sure that you are trying to connect to an F1TENTH vehicle?")
        
    def select_controller(self, controller: str):
        res = self.client.send({"command": "select_controller", "controller": controller})
        if res["status"] == False:
            raise Exception(f"Could not set controller: {res['error']}") 
        

    def get_controllers(self):
        res = self.client.send({"command": "get_controllers"})
        if res["status"] == False:
            raise Exception("Could not set controller")
        return res["controllers"]
        
        
    def set_mode(self, mode: CONTROLLER_MODE):
        res = self.client.send({"command": "set_mode", "mode": mode.value})
        if res["status"] == False:
            raise Exception(f"Could not set mode: {res['error']}")


    def manual_control(self, d: float, delta: float):
        res = self.client.send({"command": "manual_control", "d": d, "delta": delta})
        if res["status"] == False:
            raise Exception(f"Could not set manual control: {res['error']}")

    def keyboard_control(self, d_max: float = 0.75, delta_max: float = 0.5, frequency: float = 40):
        
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

    def execute_trajectory(self, trajectory: Trajectory):
        message = {"command": "execute_trajectory",
                   "trajectory":{
                       "pos_tck": trajectory.pos_tck,
                        "evol_tck": trajectory.evol_tck,
                        "reversed": trajectory.reversed}}
        
        res = self.client.send(message)
        if res["status"] == False:
            raise Exception(f"Could not execute trajectory: {res['error']}")

    def reset_controller(self):
        res = self.client.send({"command": "reset_controller"})
        if res["status"] == False:
            raise Exception(f"Could not reset controller: {res['error']}")

    def emergency_stop(self):
        res = self.client.send({"command": "stop_controller"})
        if res["status"] == False:
            raise Exception(f"Could not stop vehicle: {res['error']}")

    def get_logs(self):
        res = self.client.send({"command": "get_logs"})
        if res["status"] == False:
            raise Exception(f"Cannot retrieve logs: {res['error']}")
        states = res["states"]
        inputs = res["inputs"]
        c = res["c"]
        errors = res["errors"]
        return states, inputs, c, errors
    
    def reset_state_logger(self):
        res = self.client.send({"command": "reset_logger"})
        if res["status"] == False:
            raise Exception(f"Cannnot reset logger: {res['error']}")


    def get_mode(self):
        res = self.client.send({"command": "get_mode"})
        if res["status"] == False:
            raise Exception(f"Cannnot get mode: {res['error']}")
        return CONTROLLER_MODE(res["mode"])


    def wait_while_running(self):
        running = True
        while running:
            if self.get_mode() == CONTROLLER_MODE.RUNNING:
                time.sleep(1/10)
            else:
                running = False


    def reinit_GP_LPV_LQR(self, vehicle_params, GP_LPV_LQR_params):
        res = self.client.send({"command": "reinit_GP_LPV_LQR", 
                                "GP_LPV_LQR_params": GP_LPV_LQR_params,
                                "vehicle_params": vehicle_params})
        if res["status"] == False:
            raise Exception(f"Reinitialization failed: {res['error']}")


    def reinit_LPV_LQR_from_yaml(self, yaml_path):
        # load yaml into dict
        params = yaml.load(open(yaml_path, "r"), Loader=yaml.FullLoader)
        res = self.client.send({"command": "reinit_LPV_LQR",
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
                 retrieve_training_data = False):
        """Train the GPs"""
        if states is not None and inputs is not None and c is not None and errors is not None:
            # TODO: check if all arrays have the same length
            res = self.client.send({"command": "GP_train", 
                                    "data": {"states": states, 
                                             "inputs": inputs,
                                             "c": c,
                                             "errors": errors},
                                    "seed": random_seed, 
                                    "retrieve_training_data": retrieve_training_data})
        else:
            res = self.client.send({"command": "GP_train",
                                    "data": None,
                                    "random_seed": random_seed,
                                    "retrieve_training_data": retrieve_training_data})
        if res["status"] == False:
            raise Exception(f"Training failed: {res['error']}")
        
        if retrieve_training_data:
            return res["training_data"] # ((lat_x, lat_y), (long_x, long_y))
        
    def GP_reset(self):
        res = self.client.send({"command": "GP_reset"})
        if res["status"] == False:
            raise Exception(f"Could not reset GP: {res['error']}")
        
    def GP_to_online(self):
        res = self.client.send({"command": "GP_to_online"})
        if res["status"] == False:
            raise Exception(f"Could not switch to online mode: {res['error']}")