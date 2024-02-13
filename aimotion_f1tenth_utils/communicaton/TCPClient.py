import socket
import time
import datetime
import struct
import pickle
import random
import numpy as np
from aimotion_f1tenth_utils.logger import get_logger



class TCPClient:
    def __init__(self) -> None:
        """TCP Client implementation used to communicate with the F1TENTH fleet manager"""
        self.client_socket = None
        self.logger = get_logger()

    def connect(self, host, port):
        """Establish connection to the TCP server with the given host and port
        
        
        :param host: IP address of the server
        :type host: str
        :param port: Port of the server
        :type port: int

        :return: Status of the connection
        :rtype: bool
        """


        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            self.client_socket.connect((host, port))
            self.logger.info(f"Connected to {host}:{port}")
            return True
        except ConnectionRefusedError:
            self.logger.error(f"Connection refused by {host}:{port}")
            return False

    def send(self, message):
        """Function that sends a message to the server and returns the recieved response
        
        :param message: Message to be sent to the server. The message should be encoded in a dictinary format
        :type message: dict
        """

        serialized_message = pickle.dumps(message)
        length_prefix = struct.pack("!I", len(serialized_message))
        try:
            self.client_socket.sendall(length_prefix)
            self.client_socket.sendall(serialized_message)
        except BrokenPipeError:
            self.logger.error("Broken pipe error!")
            return {}
        
        length_prefix = self.client_socket.recv(4)
            
        if not length_prefix:
            self.logger.info(f"No response received from server!")
            return {}
        
        # Unpack the length prefix
        message_length = struct.unpack("!I", length_prefix)[0]

        # Receive the serialized data in chunks
        data = b""
        while len(data) < message_length:
            chunk = self.client_socket.recv(min(4096, message_length - len(data)))
            if not chunk:
                self.logger.info(f"No data received from server!")
                return {}
            data += chunk
            

        return pickle.loads(data)

    def close(self):
        """Close the connection to the server"""
        self.client_socket.close()
        self.logger.info("Connection closed")


# test the client
if __name__ == "__main__":
    X=random.randint(0,100)

    # Server details
    server_ip = '127.0.0.1'  # Replace with the server IP address
    server_port = 8000  # Replace with the server port


    client = TCPClient()
    client.connect(server_ip, server_port)

    message = {"car_ID": "JOEBUSH1",
           "array": np.random.rand(100,100)*X}

    res=client.send(message)
    print(res)

    client.close()