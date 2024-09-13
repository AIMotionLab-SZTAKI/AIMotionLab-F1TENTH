
from rclpy.node import Node
from ..aimotion_f1tenth_system.src.vehicle_state_msgs.msg import VehicleState

class State_Listener(Node):

    def __init__(self):
        super().__init__('state_listener')
