import socket
import time
import datetime
import struct
import pickle

# Server details
server_ip = '127.0.0.1'  # Replace with the server IP address
server_port = 8000  # Replace with the server port

# Create a TCP socket
client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# Connect to the server
client_socket.connect((server_ip, server_port))

message = {"text": "Hello from the client!",
           "number": 123,
           "float": 123.456,
           "boolean": True,
           "list": [1, 2, 3, 4, 5],
           "dict": {"key": "value"}}

long_message = [message for i in range(1000000)]

serialized_message = pickle.dumps(long_message)

print("Message length:", len(serialized_message))

# Send the length prefix
length_prefix = struct.pack("!I", len(serialized_message))
client_socket.sendall(length_prefix)

# Send the current date to the server
client_socket.sendall(serialized_message)
client_socket.send("heble".encode())

time.sleep(10)

# Close the connection
client_socket.close()
