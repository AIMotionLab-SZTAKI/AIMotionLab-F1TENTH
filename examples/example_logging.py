# Simple script, that:
    #gets the current state of a vehicle
    #starts the logging process for the vehicle
    #stop the logging process after 5 seconds
    #requests the created log file from the server and saves it
from aimotion_f1tenth_utils.F1Client import F1TENTH, Connection
import os, time

# Create a connection to the fleet manager
server = "192.168.2.65"#"127.0.1.1" #"localhost"
port = 8000
conn = Connection(server, port) 
# connection is used to send non vehicle specific messages, i.e individual trajectories and actions to upload to the server

#Creating the log folder if it doesn't exist already
if not os.path.exists("logs"):
    os.mkdir("logs")

conn = Connection(server, port)

car1 = F1TENTH("JoeBush1", conn)

car1.toggle_radio_active(True) #Turning on radio broadcast for the vehicle

print("Current state of the vehicle:"+car1.get_state()) #returns the current state

#starting the logging process

car1.toggle_logging(True) 

time.sleep(5) #Wait for 5 seconds

#Saving the log file, we don't need to call toggle_logging(False)
car1.toggle_save() #toggle_save() automatically turns of logging

car1.get_logs() #default target for saving the log files: logs/

car1.connection.client.close()








