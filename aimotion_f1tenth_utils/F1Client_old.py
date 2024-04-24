from aimotion_f1tenth_utils.communicaton.TCPClient import TCPClient
from aimotion_f1tenth_utils.logger import get_logger

from aimotion_f1tenth_utils.Trajectory import Trajectory
import socket
import pickle
import struct

import os
import time

class Connection:
    def __init__(self, host, port) -> None:
        """Class implementation of a connection to the fleet manager
        
        :param host: IP address of the server
        :type host: str
        :param port: Port of the server
        :type port: int

        :raises Exception: If the connection to the server could not be established

        """
        self.host = host
        self.port = port
        self.client = TCPClient()

        if not self.client.connect(host, port):
            raise Exception(f"Could not connect to {host}:{port}")

    def verify_vehicle(self, car_ID):
        """Verifies if a vehicle exists in the server side application with the given car_ID
        
        :param car_ID: ID of the vehicle
        :type car_ID: str
        :return: Verification status
        :rtype: bool
        """
        result = self.client.send({"car_ID": car_ID, "command": "verify_vehicle"})
        if result["status"] == True and result["car_ID"] == car_ID:
            return True
        else:
            return False
            
        
    def upload_trajectory(self, trajectory: Trajectory):
        """Uploads a trajectory to the server
        
        :param trajectory: Trajectory object to be uploaded
        :type trajectory: Trajectory
        :return: Status of the upload
        :rtype: bool"""


        message = {"command": "upload_trajectory",
                   "trajectory_ID": trajectory.trajectory_ID,
                   "pos_tck": trajectory.pos_tck,
                   "evol_tck": trajectory.evol_tck,
                   "reversed": trajectory.reversed}
        
        result = self.send(message)
        return result["status"]
    

    def list_trajectories(self):
        """List the trajectories stored by the fleet manager server
        
        :return: Tuple containing the status flag (bool) and a list of the trajectory IDs (list[str]) if the request has been successfull else the error message
        :rtype: tuple
        """
        resp = self.client.send({"command": "list_trajectories"})
        if resp["status"]:
            res = (True, resp["trajectories"])
        else:
            res = (False, resp["error"])
        return res
    
    def reload_manager(self):
        """Reloads the fleet_manager GUI
        
        :return: Status flag of the command
        :rtype: bool
        """
        
        res = self.client.send({"command": "reload"})

        return res["status"] 


    def upload_choreography(self, choreography):
        """Uploads a choreography to the server

        :param choreography: Choreography object to be uploaded
        :type choreography: Choreography
        :return: Status of the upload
        :rtype: bool
        """


        message = {"command": "upload_action",
                   "choreography_ID": choreography.choreography_ID,
                   "choreography_data": choreography.data}
        
        result = self.send(message)
        return result["status"]

        
    def send(self, message):
        """Function that sends a message to the server and returns the recieved response
        
        :param message: Message to be sent to the server. The message should be encoded in a dictinary format
        :type message: dict

        :return: Response from the server
        :rtype: dict
        """
        return self.client.send(message)


class F1TENTH:
    def __init__(self, car_ID, connection: Connection) -> None:
        """Class implementation of a client object to handle an F1TENTH vehicle remotely
        
        :param car_ID: ID of the vehicle
        :type car_ID: str

        :param connection: Connection object to the fleet manager
        :type connection: Connection

        :raises Exception: If the vehicle with the given car_ID could not be verified by the manager

        """
        self.car_ID = car_ID
        
        # establish TCP connection with the manager
        self.connection = connection
        if not self.connection.verify_vehicle(self.car_ID):
            raise Exception(f"Vehicle with ID {self.car_ID} could not be verified by the manager!")
        

    def toggle_save(self):
        """
        Saves current log file for the vehicle and starts new log file
        Turns off logging for vehicle=> logging status must be set back to True

        :return: Status of the save
        """

        result = self.connection.send({"car_ID": self.car_ID, "command": "new_log"})
        return result
    

    def toggle_radio_active(self, ON: bool):
        """
        Turns on the vehicles radio communication to stream mocap data for state estimation

        :param ON: Status of the radio communication
        :type ON: bool
        :return: Status of the activation
        """


        result = self.connection.send({"car_ID": self.car_ID, "command": "activate", "ON": ON})
       
        return result
        #TODO manage result
    

    def toggle_logging(self, start: bool):
        """
        Switches the ros2 node's logging status variable from True to False or the other way
        
        :param start: Status of the logging
        :type start: bool
        :return: Status of the logging
        """


        result = self.connection.send({"car_ID": self.car_ID, "command": "logging", "ON": start})

        return result
    
    def get_logs(self, target_path=None):
        """
        Gets the log files for the vehicle and saves to the dedicated location

        :param target_path: Path to save the log files
        :type target_path: str
        :return: Status of the log retrieval
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

        return result["status"]

    def get_current_progress(self):
        """Gets the current progress of the trajectory execution
        
        :return: Current progress value in percentage
        :rtype: int
        """

        message = {"command":"get_progress",
                   "car_ID": self.car_ID}
        result = self.connection.send(message)

        if result["status"]:
            return result["progress"] #If no error occures then return the progress
        else:
            return -1 #If there is error during the request then return -1
 
    def get_state(self):
        """Requests the current state of the vehicle from the server
        
        :return: State of the vehicle
        :rtype: dict

        :raises Exception: If the state of the vehicle could not be retrieved
        """
        result = self.connection.send({"car_ID": self.car_ID, "command": "get_state"})
        if result["status"] == True:
            return result["state"]
        else:
            raise Exception(f"Could not get state of vehicle with ID {self.car_ID}; error message: {result['error']}")


    def execute_trajectory(self, trajectory_ID, trajectory_data = None, block: bool  = False):
        """Executes a trajectory on the vehicle.
        
        :param trajectory_ID: ID of the trajectory to be executed
        :type trajectory_ID: str
        :param trajectory_data: Optional trajectory data to be executed. If the trajectory is already saved on the server, this parameter can be omitted.
        :type trajectory_data: list, optional
        :param block: Wait until the execution is done
        :type block: bool, optional
        :return: Status of the execution
        :rtype: bool
        """

        self.toggle_radio_active(True)
        message = {"car_ID": self.car_ID,
                   "command": "execute_trajectory",
                   "trajectory_ID": trajectory_ID}
        
        if trajectory_data is not None:
            message["pos_tck"] = trajectory_data[0]
            message["evol_tck"] = trajectory_data[1]
        
        result = self.connection.send(message) # this method should be blocking, i.e only return the result if the execution is finished or failed
        
        if block:
            return self.wait_until_done()
        else:
            return result["status"]

    def wait_until_done(self):
        """
        Blocks thread until the execution of the current trajectory execution is finished
        """
        
        message = {"command": "get_progress",
                   "car_ID": self.car_ID}   
        response = {"progress" : 0}
        while response["progress"] != 100 and response["progress"] != -1:
            time.sleep(1/40)
            response = self.connection.send(message)


        if response["progress"] == 100:
            return True
        else:
            return False
        

    
if __name__=="__main__":
    # Server details
    server_ip = '127.0.1.1'  # Replace with the server IP address
    server_port = 8000  # Replace with the server port
    connection = Connection(server_ip, server_port)

    car_ID = "JOEBUSH1"
    f1= F1TENTH(car_ID, connection)