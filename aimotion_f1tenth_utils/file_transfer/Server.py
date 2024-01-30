import socket
import os

class tcp_file_server:
    def __init__(self, ip_address, target_path):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        #self.socket.bind((ip_address, 8010))
        self.socket.bind((ip_address, 8010))
        self.socket.listen(2)
        self.target_path = target_path


    def receive_logs(self):
        client_socket, address = self.socket.accept()
        num_of_files = client_socket.recv(1024).decode()
        if not num_of_files:
            return
        num_of_files = int(num_of_files)

        for i in range(num_of_files):
            
            file_name = str(client_socket.recv(1024).decode())
            
            file_size = int(client_socket.recv(1024).decode())

            received_data = b''

            while len(received_data) < file_size:
                chunk = client_socket.recv(1024)
                if not chunk:
                    break
                
                received_data += chunk
            file_path = os.path.join(self.target_path, file_name)
            with open(file_path, "bw") as file:
                file.write(received_data)
    def destroy(self):
        self.socket.close()