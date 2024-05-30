import socket
import threading
import struct
import pickle
# from aimotion_f1tenth_utils.logger import get_logger

class TCPServer:
    def __init__(self, host, port, message_callback=None):
        """
        TCP server class implementation for the fleet manager.
        
        :param host: IP address of the server
        :type host: str
        :param port: Port of the server
        :type port: int
        :param message_callback: Callback function that is called when a valid message is received
        :type message_callback: function
        """
        self.host = host
        self.port = port
        self.message_callback = message_callback

        self.server_socket = None
        self.num_of_connections = 0
        self.connections = []
        self.running = True

    def start(self):
        """Start the TCP server, waits for connections and establishes communication channels with the clients"""
        # Create a TCP socket
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.bind((self.host, self.port))
        self.server_socket.listen(5)
        #self.logger.info(f"Server started on {self.host}:{self.port}")
        print(f"TCP-Server started on {self.host}:{self.port}, awaiting commands!")

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
            self.connections.append((con_ID, client_socket, connection_thread))
            #self.logger.info(f"New connection from {client_address[0]}:{client_address[1]}, connection ID {con_ID}")
            

    def handle_connection(self, client_socket, con_ID):
        """Function that handles the connection with a single connected client This function is called in a thread for each connected client
        
        :param client_socket: Socket object of the connected client
        :type client_socket: socket
        :param con_ID: ID of the connection
        :type con_ID: int
        """

        # start infinite loop until the server is shut down
        while self.running:

            # receive the length prefix of the transfered data
            try:
                length_prefix = client_socket.recv(4)
            except: 
                #self.logger.info(f"No data received, closing connection with ID {con_ID}")
                break
            
            if not length_prefix:
                #self.logger.info(f"No data received, closing connection with ID {con_ID}")
                break
        
            # Unpack the length prefix
            message_length = struct.unpack("!I", length_prefix)[0]

            # Receive the serialized data in chunks
            data = b""
            while len(data) < message_length:
                chunk = client_socket.recv(min(4096, message_length - len(data)))
                if not chunk:
                    #self.logger.info(f"No data received, closing connection with ID {con_ID}")
                    data = b"" # reset data as the connection has been terminated during transfer
                    break
                data += chunk
            
            # check if the connection has been terminated during transfer
            if data == b"": 
                break
            
            # handle callback
            result = self.message_callback(pickle.loads(data))

            # return the result
            result_serialized = pickle.dumps(result)
            length_prefix = struct.pack("!I", len(result_serialized))
            try:
                client_socket.sendall(length_prefix)
                client_socket.sendall(result_serialized)
            except BrokenPipeError:
                #self.logger.error("Broken pipe error!")
                break


        self.close_connection(con_ID)
        

    def close_connection(self, con_ID):
        """Function that closes the connection with the given ID
        
        :param con_ID: ID of the connection to be closed
        :type con_ID: int
        """
        connection = next((connection for connection in self.connections if connection[0] == con_ID), None)
        if connection is not None:
            connection[1].close()
            self.connections.remove(connection)
            #self.logger.info(f"Connection with ID {con_ID} closed!")

    def stop(self):
        """Function that stops the server forces all the connections to close"""
        
        for connection in self.connections:
            connection[1].shutdown(socket.SHUT_RDWR)
            connection[2].join()
            
        self.server_socket.close()
        print("Server has been terminated!")


if __name__=="__main__":
    # Usage example

    def callback(message):
        return {"status": True}
    server = TCPServer('localhost', 8000, callback)
    server.start()
