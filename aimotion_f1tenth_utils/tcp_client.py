import socket
import time
import datetime
import struct
import pickle
from aimotion_f1tenth_utils.logger import get_logger



class TCPClient:
    def __init__(self) -> None:
        """Class implementation for the TCP client"""
        self.client_socket = None
        self.logger = get_logger()

    def connect(self, host, port):
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            self.client_socket.connect((host, port))
            return True
        except ConnectionRefusedError:
            self.logger.error(f"Connection refused by {host}:{port}")
            return False

    def send(self, message):
        serialized_message = pickle.dumps(message)
        length_prefix = struct.pack("!I", len(serialized_message))
        try:
            self.client_socket.sendall(length_prefix)
            self.client_socket.sendall(serialized_message)
            return True
        except BrokenPipeError:
            self.logger.error("Broken pipe error!")
            return False


    def close(self):
        self.client_socket.close()
        self.logger.info("Connection closed")


# test the client
if __name__ == "__main__":
    # Server details
    server_ip = '127.0.0.1'  # Replace with the server IP address
    server_port = 8000  # Replace with the server port


    client = TCPClient()
    client.connect(server_ip, server_port)

    message = {"text": "Hello from the client!",
           "number": 123,
           "float": 123.456,
           "boolean": True,
           "list": [1, 2, 3, 4, 5],
           "dict": {"key": "value"}}

    long_message = [message for i in range(1000000)]

    client.send(long_message)
    client.close()
