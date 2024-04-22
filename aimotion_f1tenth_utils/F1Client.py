from aimotion_f1tenth_utils.Trajectory import Trajectory
from aimotion_f1tenth_utils.communicaton.TCPClient import TCPClient
from aimotion_f1tenth_utils.logger import get_logger

import socket
import pickle
import struct

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
        
        car_ID = self._get_ID()
        if car_ID is None:
            raise ValueError("Vehicle cannot be identified! Are you sure that you are trying to connect to an F1TENTH vehicle?")
        self.car_ID = car_ID
        
    def _get_ID(self):
        info = self.client.send({"command": "?"})

        if info["status"] == True:
            return info["car_ID"]

        else:
            return None
        
    def select_controller(self):
        pass

    def set_mode(self):
        pass

    def manual_control(self):
        pass

    def keyboard_control(self):
        pass

    def execute_trajectory(self):
        pass

    def reset_controller(self):
        pass

    def emergency_stop(self):
        pass

    def set_GP_type(self):
        pass

    def set_GP_mode(self):
        pass

    def GP_train(self):
        pass

    def GP_finetune(self):
        pass

    def GP_reset(self):
        pass

    def get_logs(self):
        pass





