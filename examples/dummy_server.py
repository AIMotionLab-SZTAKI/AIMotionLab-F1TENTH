from aimotion_f1tenth_utils.communicaton.TCPServer import TCPServer
import time


def callback(message):
    """Dummy callback function to test the F1Client scripting utilities"""
    # NOTE: This is only to test the basic fucntionalities, not intended for production usage

    # Create a dictionary for the vehicels that can be verified, i.e they exist inn the manager
    vehicles_dict = {"AI_car_01", "JoeBush1"}
    recieved_trajectories = []
    recieved_actions = []

    match message["command"]:
        # vehicle verification
        case "verify_vehicle":
            if message["car_ID"] in vehicles_dict:
                message["status"] = True
            else:
                message["status"] = False
            return message
        
        # get state
        case "get_state":
            response={}
            response["status"] = True
            response["state"] = {"pos_x": 0,
                                "pos_y": 0,
                                "head_angle": 0,
                                "long_vel": 0,
                                "lat_vel": 0,
                                "yaw_rate": 0}
            return response
        
        # send trajectory
        case "upload_trajectory":
            recieved_trajectories.append(message["trajectory_ID"])
            return {"status": True}
        
        case "execute_trajectory":
            if message["trajectory_ID"] in recieved_trajectories:
                time.sleep(5) # wait to emulate execution
                return {"status": True}
            else:
                return {"status": False, "error": "Trajectory not found!"}
            
        case "upload_action":
            recieved_actions.append(message["action_ID"])
            return {"status": True}
        
        case "execute_action": 
            if message["action_ID"] in recieved_actions:
                time.sleep(10) # wait to emulate execution
                return {"status": True}
            else:
                return {"status": False, "error": "Action not found!"}

        case _:
            return {"status": False, "error": "Invalid command!"}
    return message



if __name__=="__main__":
    # Server details
    host = "localhost"
    port = 8000

    # Create a TCP server
    server = TCPServer(host, port, callback)
    server.start()