import socket
import os
import time

class tcp_file_client:
    def __init__(self, IP):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.connect((IP, 8010))
        #self.socket.connect(("192.168.2.65", 8010))

    
    def sendfiles(self, files: list):
        num_of_files = len(files)

        self.socket.sendall(str(num_of_files).encode())

        for log_file in files:


            file_name = os.path.basename(log_file)

            time.sleep(0.01)
            self.socket.sendall(str(file_name).encode())
            time.sleep(0.01)

            
            file_size = os.path.getsize(log_file)

            self.socket.sendall(str(file_size).encode())
            time.sleep(0.01)

            with open(log_file, "rb") as f:
                data = f.read(1024)
                while data:
                    self.socket.sendall(data)
                    data = f.read(1024)

    def destroy(self):
        self.socket.close()
