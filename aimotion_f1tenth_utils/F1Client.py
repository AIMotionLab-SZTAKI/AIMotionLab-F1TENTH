from aimotion_f1tenth_utils.communicaton.TCPClient import TCPClient
from aimotion_f1tenth_utils.logger import get_logger


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
    
    def upload_trajectory(self, trajectory):
        message = {"command": "upload_trajectory",
                   "trajectory_ID": trajectory.trajectory_ID,
                   "pos_tck": trajectory.pos_tck,
                   "evol_tck": trajectory.evol_tck}
        
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
    server_ip = '127.0.0.1'  # Replace with the server IP address
    server_port = 8000  # Replace with the server port
    connection = Connection(server_ip, server_port)

    car_ID = "JOEBUSH1"
    f1= F1TENTH(car_ID, connection)