import socket
import threading
import struct
import pickle
from aimotion_f1tenth_utils.logger import get_logger

class TCPServer:
    def __init__(self, host, port, message_callback=None):
        """
        TCP server class implementation for the communication between the fleet manager and the vehicles
        
        :param host: IP address of the server
        :param port: Port of the server
        :param message_callback: Callback function that is called when a valid message is received
        """
        self.host = host
        self.port = port
        self.message_callback = message_callback

        self.server_socket = None
        self.num_of_connections = 0
        self.connections = []
        self.running = True
        self.logger = get_logger()

    def start(self):
        # Create a TCP socket
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.bind((self.host, self.port))
        self.server_socket.listen(5)
        self.logger.info(f"Server started on {self.host}:{self.port}")

        # Wait for connections and accept them by threads
        while self.running:
            # accept connection
            client_socket, client_address = self.server_socket.accept()
            
            # increment connection number and assign con_ID
            self.num_of_connections += 1
            con_ID = self.num_of_connections

            # start connection thread
            connection_thread = threading.Thread(target=self.handle_connection, args=(client_socket, con_ID,))
            connection_thread.start()

            # add connection to list
            self.connections.append((self.num_of_connections, client_socket, connection_thread))
            self.logger.info(f"New connection from {client_address[0]}:{client_address[1]}, connection ID {self.num_of_connections}")
            

    def handle_connection(self, client_socket, con_ID):
        """Function that handles the connection with a single connected client!"""

        # start infinite loop until the server is shut down
        while self.running:

            # receive the length prefix of the transfered data
            length_prefix = client_socket.recv(4)
            
            if not length_prefix:
                self.logger.info("No data received, closing connection with ID {self.num_of_connections}")
                break
        
            # Unpack the length prefix
            message_length = struct.unpack("!I", length_prefix)[0]

            # Receive the serialized data in chunks
            data = b""
            while len(data) < message_length:
                chunk = client_socket.recv(min(4096, message_length - len(data)))
                if not chunk:
                    self.logger.info(f"No data received, closing connection with ID {con_ID}")
                    data = b"" # reset data as the connection has been terminated during transfer
                    break
                data += chunk
            
            # check if the connection has been terminated during transfer
            if data == b"": 
                break

            self.message_callback(data)

        self.close_connection(con_ID)
        

    def close_connection(self, con_ID):
        """Function that closes the connection with the given ID
        
        :param con_ID: ID of the connection to be closed
        """
        connection = next((connection for connection in self.connections if connection[0] == con_ID), None)
        if connection is not None:
            connection[1].close()
            self.connections.remove(connection)
            self.logger.info(f"Connection with ID {con_ID} closed!")

    def stop(self):
        """Function that stops the server forces all the connections to close"""
        
        for connection in self.connections:
            connection[1].shutdown(socket.SHUT_RDWR)
            connection[2].join()
            
        self.server_socket.close()
        print("Server has been terminated!")

# Usage example
server = TCPServer('localhost', 8000)
server.start()
