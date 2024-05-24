import paramiko
import os
from aimotion_fleet_manager.utils.install_utils import create_clients
import time
from pathlib import Path
import yaml
import PyQt5
from ament_index_python.packages import get_package_share_directory
from PyQt5.QtWidgets import *
import shutil

import os
from PyQt5.QtCore import QThread



import os
import paramiko

class FileTransporterSFTPClient(paramiko.SFTPClient):
    """
    Subclass of SFTPClient to achieve directory transport
    """
    def put_dir(self, source, target):
        
        for item in os.listdir(source):
            if item in ["install", "build", "log"]:
                continue
            if os.path.isfile(os.path.join(source, item)):
                self.put(os.path.join(source, item), '%s/%s' % (target, item))
            else:
                self.mkdir('%s/%s' % (target, item), ignore_existing=True)
                self.put_dir(os.path.join(source, item), '%s/%s' % (target, item))

    def mkdir(self, path, mode=511, ignore_existing=False):
        try:
            super(FileTransporterSFTPClient, self).mkdir(path, mode)
        except IOError:
            if ignore_existing:
                pass
            else:
                raise
    def rmall(self,path):
        try:
            files = self.listdir(path)
        except IOError:
            return
        for f in files:
            filepath = os.path.join(path, f)
            try:
                self.remove(filepath)
            except IOError:
                self.rmall(filepath)
        self.rmdir(path)




def create_clients(IP_ADRESS, USERNAME, PASSWORD):
    """
    Helper function that creates the SSH and SFTP clients
    
    Arguments:
        - IP_ADRESS(str): Remote computer adress
        - USERNAME(str): Remote computer login name
        - PASSWORD(str): Remote login password
    """
    SSH_client=paramiko.SSHClient()
    SSH_client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    SSH_client.connect(IP_ADRESS, username=USERNAME, password=PASSWORD, timeout=10)
    # get transport & open 

    transport=SSH_client.get_transport()
    SFTP_client=FileTransporterSFTPClient.from_transport(transport)
    return SSH_client, SFTP_client

def create_environment(ROS_MASTER_URI, IP_ADRESS, path):
    """
    Helper function that creates a unique env.sh file for every machine during installation.
    The env.sh is later used to source the environment on remote launches.

    Arguments:
        - ROS_MASTER_URI: The IP adress & port of the ROS master
        - IP_ADRESS: The IP adress of the machine where the environment will be sourced
        - path: env.sh file path    
    """

    with open(path, 'w') as f:
        # source environment/workspace
        f.write('#! /usr/bin/env bash\nsource /opt/ros/melodic/setup.bash\nsource $( cd -- "$( dirname --"${BASH_SOURCE[0]}" )" &> /dev/null && pwd )/aimotion-f1tenth-system/devel/setup.bash\n')
        # add environment variables
        f.write(f'export ROS_MASTER_URI="{ROS_MASTER_URI}"\nexport ROS_IP={IP_ADRESS}\nexec "$@"')
        f.close()




def install_onboard_stack(car_ID):
    """TODO: description"""

    config_folder = os.path.join(os.path.dirname(os.path.dirname(__file__)), "configs")
    target_path = os.path.join(os.path.join(os.path.join(os.path.join(os.path.join(os.path.dirname(os.path.dirname(__file__)), 
                                              "aimotion_f1tenth_system"), 
                                              "src"),
                                              "param_server"),
                                              "config"),
                                              "param.yaml")
    
    login_config_path = os.path.join(config_folder, f"{car_ID}_login.yaml")
    config_path = os.path.join(config_folder, f"{car_ID}.yaml")

    with open(login_config_path, "r") as f:
        config_data = yaml.safe_load(f)
        username = config_data["Username"]
        host = config_data["IP"]
        password = config_data["Password"]

    shutil.copy(config_path, target_path)

    print(f"Connecting to {username}@{host} with password: {password}")

    SSH_client, SFTP_client = create_clients(host, username, password)
    
    print("Deleting existing workspace...")

    _stdin, stdout, stderr = SSH_client.exec_command("rm -rf aimotion-f1tenth-system") #removing previous package
       
    time.sleep(5)
    SFTP_client.mkdir("aimotion-f1tenth-system", ignore_existing=False)

    print("Transfering files onto the vehicle...")
    #_stdin, stdout, stderr = SSH_client.exec_command("cd aimotion-f1tenth-system; python3 -m venv venv; source venv/bin/activate; pip install numpy scipy pyyaml torch gpytorch pynumdiff") #removing previous package
        
    sys_dir = os.path.join(Path(os.path.dirname(os.path.dirname(__file__))),"aimotion_f1tenth_system")
    SFTP_client.put_dir(sys_dir, "aimotion-f1tenth-system")

    print("Building ROS2 workspace...")       

    _stdin, stdout, stderr = SSH_client.exec_command('bash --login -c "source /opt/ros/foxy/local_setup.bash ;cd aimotion-f1tenth-system/; colcon build"')
    try:
        for line in iter(stdout.readline, ""):
                print(line)
        if stdout.channel.recv_exit_status():
            print("Failed to build ROS workspace!")
        else:
            print("Successfully installed aimotion-f1tenth-system on vehicle")
    except Exception as e:
        print("Unexpected error!")
        print(e)
            
    SSH_client.close()
    SFTP_client.close()


# install_onboard_stack("JoeBush1")

        