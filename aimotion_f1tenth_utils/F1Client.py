from aimotion_f1tenth_utils.communicaton.TCPClient import TCPClient
from aimotion_f1tenth_utils.logger import get_logger

from aimotion_f1tenth_utils.Trajectory import Trajectory
import socket
import pickle
import struct

import os

class Connection:
    def __init__(self, host, port) -> None:
        self.host = host
        self.port = port
        self.client = TCPClient()

        if not self.client.connect(host, port):
            raise Exception(f"Could not connect to {host}:{port}")

    def verify_vehicle(self, car_ID):
        result = self.client.send({"car_ID": car_ID, "command": "verify_vehicle"})
        if result["status"] == True and result["car_ID"] == car_ID:
            return True
        else:
            return False
            
        


    def upload_trajectory(self, trajectory: Trajectory):
        message = {"command": "upload_trajectory",
                   "trajectory_ID": trajectory.trajectory_ID,
                   "pos_tck": trajectory.pos_tck,
                   "evol_tck": trajectory.evol_tck,
                   "reversed": trajectory.reversed}
        
        result = self.send(message)
        return result["status"]
    

    def upload_action(self, action):
        message = {"command": "upload_action",
                   "action_ID": action.action_ID,
                   "action_data": action}
        
        result = self.connection.send(message)
        return result["status"]

        
    def send(self, message):
        return self.client.send(message)


class F1TENTH:
    def __init__(self, car_ID, connection: Connection) -> None:
        self.car_ID = car_ID
        
        # establish TCP connection with the manager
        self.connection = connection
        if not self.connection.verify_vehicle(self.car_ID):
            raise Exception(f"Vehicle with ID {self.car_ID} could not be verified by the manager!")
    def toggle_save(self):
        """
        Saves current log file for the vehicle
        Starts new log file
        Turns off logging for vehicle=> logging status must be set back to True
        """

        result = self.connection.send({"car_ID": self.car_ID, "command": "new_log"})
        return result
    

    def toggle_radio_active(self, ON: bool):
        """
        Turns on the vehicle's radio and echo function in the main_UI
        Args: 
        - On: 
            - True-> turn vehicle on
            - False-> turn vehicle off
        """


        result = self.connection.send({"car_ID": self.car_ID, "command": "activate", "ON": ON})
       
        return result
        #TODO manage result
    def toggle_logging(self, start: bool):
        """
        Switches the ros2 node's logging status variable from True to False or the other way
        Args:
        - start:
            - True: turns logging on
            - False: turns logging off

        """



        result = self.connection.send({"car_ID": self.car_ID, "command": "logging", "ON": start})

        return result
    def get_logs(self, target_path=None):
        """
        sends the command with the car_ID
        receives all the logs of a certain vehicle
        saves them to /logs
        """
        
        message = {"car_ID": self.car_ID, "command": "get_logs"}
        
        result = self.connection.send(message)

        for i in result["files"]:
            file_name = i
        
            if target_path is None:
                file_path = os.path.join("logs", file_name)
            else:
                file_path = os.path.join(file_path, "logs", file_name)

            with open(file_path, "bw") as file:
                file.write(result[i])

        return result


    def get_state(self):
        result = self.connection.send({"car_ID": self.car_ID, "command": "get_state"})
        if result["status"] == True:
            return result["state"]
        else:
            raise Exception(f"Could not get state of vehicle with ID {self.car_ID}; error message: {result['error']}")


    def execute_trajectory(self, trajectory_ID, trajectory_data = None):
        
        message = {"car_ID": self.car_ID,
                   "command": "execute_trajectory",
                   "trajectory_ID": trajectory_ID}
        
        if trajectory_data is not None:
            message["pos_tck"] = trajectory_data[0]
            message["evol_tck"] = trajectory_data[1]
        
        result = self.connection.send(message) # this method should be blocking, i.e only return the result if the execution is finished or failed
        return result["status"]


    
if __name__=="__main__":
    # Server details
    server_ip = '127.0.1.1'  # Replace with the server IP address
    server_port = 8000  # Replace with the server port
    connection = Connection(server_ip, server_port)

    car_ID = "JOEBUSH1"
    f1= F1TENTH(car_ID, connection)